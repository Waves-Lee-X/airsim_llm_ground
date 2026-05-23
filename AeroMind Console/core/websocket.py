from __future__ import annotations

import base64
import hashlib
import json
import logging
import struct
import time
from typing import Any, Callable

logger = logging.getLogger(__name__)

WS_MAGIC = b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
_FIN_TEXT = 0x81
_FIN_CLOSE = 0x88
_FIN_PONG = 0x8A


def ws_accept_key(client_key: str) -> str:
    digest = hashlib.sha1(client_key.encode() + WS_MAGIC).digest()
    return base64.b64encode(digest).decode()


def _masked(data: bytes, mask_key: bytes) -> bytes:
    return bytes(b ^ mask_key[i % 4] for i, b in enumerate(data))


def ws_send_text(wfile: Any, text: str) -> None:
    payload = text.encode("utf-8")
    frame = bytearray()
    frame.append(_FIN_TEXT)
    n = len(payload)
    if n < 126:
        frame.append(n)
    elif n < 65536:
        frame.append(126)
        frame.extend(struct.pack(">H", n))
    else:
        frame.append(127)
        frame.extend(struct.pack(">Q", n))
    frame.extend(payload)
    wfile.write(bytes(frame))
    wfile.flush()


def ws_send_close(wfile: Any, code: int = 1000) -> None:
    payload = struct.pack(">H", code)
    frame = bytearray([_FIN_CLOSE, len(payload)])
    frame.extend(payload)
    try:
        wfile.write(bytes(frame))
        wfile.flush()
    except OSError:
        pass


def ws_send_pong(wfile: Any, data: bytes = b"") -> None:
    frame = bytearray([_FIN_PONG, len(data)])
    frame.extend(data)
    wfile.write(bytes(frame))
    wfile.flush()


def ws_recv_frame(rfile: Any) -> tuple[int, bytes]:
    b1 = rfile.read(1)
    if not b1:
        raise EOFError("WebSocket closed by peer")
    opcode = b1[0] & 0x0F
    b2 = rfile.read(1)[0]
    masked = bool(b2 & 0x80)
    length = b2 & 0x7F
    if length == 126:
        length = struct.unpack(">H", rfile.read(2))[0]
    elif length == 127:
        length = struct.unpack(">Q", rfile.read(8))[0]
    mask_key = rfile.read(4) if masked else b""
    data = rfile.read(length)
    if masked and mask_key:
        data = _masked(data, mask_key)
    return opcode, data


def ws_serve(
    wfile: Any,
    rfile: Any,
    on_message: Callable[[str], None] | None = None,
    heartbeat_s: float = 15.0,
) -> None:
    """Blocking read-loop: handles ping/pong/close; passes text frames to on_message."""
    last_read = time.monotonic()
    try:
        while True:
            try:
                opcode, data = ws_recv_frame(rfile)
                last_read = time.monotonic()
            except EOFError:
                return
            if opcode == 0x8:
                ws_send_close(wfile, 1000)
                return
            if opcode == 0x9:
                ws_send_pong(wfile, data)
            elif opcode == 0x1 and on_message is not None:
                try:
                    on_message(data.decode("utf-8"))
                except Exception:
                    logger.debug("WebSocket message handler error", exc_info=True)
            if heartbeat_s > 0 and time.monotonic() - last_read > heartbeat_s:
                ws_send_pong(wfile)
                last_read = time.monotonic()
    except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
        pass


def ws_handshake(headers: dict[str, str]) -> str | None:
    """Validate WebSocket upgrade request and return accept key, or None if invalid."""
    if headers.get("Upgrade", "").lower() != "websocket":
        return None
    client_key = headers.get("Sec-WebSocket-Key", "")
    if not client_key:
        return None
    return ws_accept_key(client_key)
