"""AirSim / msgpackrpc compatibility patches.

Applied once at import time. Each patch is guarded so failures are non-fatal.
"""

from __future__ import annotations

import logging
import sys

logger = logging.getLogger(__name__)

_PATCHES_APPLIED = False


def apply_all() -> None:
    """Apply all compatibility patches. Idempotent — safe to call multiple times."""
    global _PATCHES_APPLIED
    if _PATCHES_APPLIED:
        return
    _patch_msgpackrpc_ioloop()
    _patch_msgpack_encoding()
    _PATCHES_APPLIED = True


def _patch_msgpackrpc_ioloop() -> None:
    """Fix msgpackrpc IOLoop conflict with tornado.

    msgpackrpc's step_timeout calls stop() + start() inside a tornado IOLoop
    callback. tornado forbids recursive start() on a running IOLoop.
    Remove the extra start() — let Future.join() manage the IOLoop lifecycle.
    """
    try:
        import msgpackrpc.session
        from msgpackrpc.error import TimeoutError as _MsgpackTimeoutError
        from msgpackrpc.compat import iteritems as _msgpack_iteritems

        _orig_step_timeout = msgpackrpc.session.Session.step_timeout

        def _patched_step_timeout(self: msgpackrpc.session.Session) -> None:
            timeouts = []
            for msgid, future in _msgpack_iteritems(self._request_table):
                if future.step_timeout():
                    timeouts.append(msgid)
            if not timeouts:
                return
            self._loop.stop()
            for to_msgid in timeouts:
                to_future = self._request_table.pop(to_msgid)
                to_future.set_error(_MsgpackTimeoutError("Request timed out"))

        msgpackrpc.session.Session.step_timeout = _patched_step_timeout
        logger.info("msgpackrpc IOLoop compatibility patch applied")
    except Exception:
        logger.debug("msgpackrpc IOLoop patch skipped", exc_info=True)


def _patch_msgpack_encoding() -> None:
    """Fix AirSim msgpack encoding crash.

    AirSim C++ server sends non-UTF-8 bytes (image/sensor binary data) in RPC
    responses. msgpack 0.5.6 Unpacker with unicode_errors='strict' throws
    UnicodeDecodeError, which tears down the tornado RPC connection.
    Patch to use 'replace' — invalid bytes become U+FFFD instead of crashing.
    """
    try:
        import msgpack as _msgpack_lib
        import msgpackrpc.transport.tcp as _tcp_transport

        # Version detection — only patch msgpack < 1.0 (pre-raw mode)
        _msgpack_version = tuple(int(x) for x in _msgpack_lib.__version__.split(".")[:2])
        if _msgpack_version >= (1, 0):
            logger.info("msgpack >= 1.0 detected — encoding patch not needed")
            return

        _orig_base_socket_init = _tcp_transport.BaseSocket.__init__

        def _patched_base_socket_init(self, stream, encodings):
            self._stream = stream
            self._packer = _msgpack_lib.Packer(
                encoding=encodings[0], default=lambda x: x.to_msgpack(),
            )
            self._unpacker = _msgpack_lib.Unpacker(
                encoding=encodings[1], unicode_errors='replace',
            )

        _tcp_transport.BaseSocket.__init__ = _patched_base_socket_init
        logger.info(
            "AirSim msgpack encoding patch applied (unicode_errors=replace, version=%s)",
            _msgpack_lib.__version__,
        )
    except Exception:
        logger.debug("msgpack encoding patch skipped", exc_info=True)
