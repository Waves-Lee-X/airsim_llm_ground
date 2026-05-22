from __future__ import annotations

from pathlib import Path
from dataclasses import dataclass

try:
    import pywintypes  # type: ignore
    import win32api  # type: ignore
    import win32con  # type: ignore
    import win32gui  # type: ignore
    import win32process  # type: ignore
except Exception:  # pragma: no cover - pywin32 is Windows-only and optional at import time.
    pywintypes = None
    win32api = None
    win32con = None
    win32gui = None
    win32process = None


DEFAULT_AIRSIM_WINDOW_KEYWORDS = (
    "AirSim",
    "Blocks",
    "Unreal",
    "WindowsNoEditor",
    "Kitbash3d_Warzone"
)

DEFAULT_AIRSIM_PROCESS_KEYWORDS = (
    "airsim",
    "blocks",
    "unreal",
    "unrealeditor",
    "ue4editor",
    "windowsnoeditor",
    "win64-shipping",
)

BLOCKED_PROCESS_NAMES = (
    "code.exe",
    "cursor.exe",
    "devenv.exe",
    "explorer.exe",
    "powershell.exe",
    "windowsterminal.exe",
    "cmd.exe",
    "python.exe",
    "pythonw.exe",
    "chrome.exe",
    "msedge.exe",
)


@dataclass(frozen=True)
class WindowMatch:
    hwnd: int
    title: str
    process_id: int = 0
    process_name: str = ""


def _require_pywin32() -> None:
    if win32gui is None or win32con is None or win32process is None:
        raise RuntimeError("pywin32 is required for UE/AirSim window embedding. Install it with: pip install pywin32")


def _window_process_info(hwnd: int) -> tuple[int, str]:
    _thread_id, process_id = win32process.GetWindowThreadProcessId(hwnd)
    process_name = ""
    if process_id <= 0:
        return 0, process_name

    process_handle = None
    try:
        access = win32con.PROCESS_QUERY_INFORMATION | win32con.PROCESS_VM_READ
        process_handle = win32api.OpenProcess(access, False, process_id)
        process_path = win32process.GetModuleFileNameEx(process_handle, 0)
        process_name = Path(process_path).name
    except Exception:
        process_name = ""
    finally:
        if process_handle is not None:
            try:
                win32api.CloseHandle(process_handle)
            except Exception:
                pass
    return int(process_id), process_name


def list_visible_windows() -> list[WindowMatch]:
    _require_pywin32()
    windows: list[WindowMatch] = []

    def _enum_proc(hwnd: int, _extra: object) -> bool:
        if not win32gui.IsWindowVisible(hwnd):
            return True
        title = win32gui.GetWindowText(hwnd).strip()
        if title:
            process_id, process_name = _window_process_info(hwnd)
            windows.append(
                WindowMatch(
                    hwnd=hwnd,
                    title=title,
                    process_id=process_id,
                    process_name=process_name,
                )
            )
        return True

    win32gui.EnumWindows(_enum_proc, None)
    return windows


def find_window_by_keywords(
    keywords: tuple[str, ...] | list[str] | None = None,
    process_keywords: tuple[str, ...] | list[str] | None = None,
    exclude_hwnds: set[int] | None = None,
    require_process_match: bool = True,
) -> WindowMatch | None:
    title_terms = tuple(keywords or DEFAULT_AIRSIM_WINDOW_KEYWORDS)
    process_terms = tuple(process_keywords or DEFAULT_AIRSIM_PROCESS_KEYWORDS)
    lowered_title_terms = tuple(term.lower() for term in title_terms if term.strip())
    lowered_process_terms = tuple(term.lower() for term in process_terms if term.strip())
    if not lowered_title_terms:
        return None

    excluded = exclude_hwnds or set()
    for window in list_visible_windows():
        if window.hwnd in excluded:
            continue
        lowered_title = window.title.lower()
        title_matches = any(term in lowered_title for term in lowered_title_terms)
        if not title_matches:
            continue

        lowered_process_name = window.process_name.lower()
        if lowered_process_name in BLOCKED_PROCESS_NAMES:
            continue
        process_matches = bool(lowered_process_name) and any(
            term in lowered_process_name for term in lowered_process_terms
        )
        if process_matches:
            return window

        if not require_process_match:
            return window
    return None


def describe_visible_windows() -> list[str]:
    return [
        f"{window.hwnd} | pid={window.process_id} | process={window.process_name or '-'} | title={window.title}"
        for window in list_visible_windows()
    ]


def restore_window_if_needed(child_hwnd: int) -> None:
    _require_pywin32()
    if not win32gui.IsWindow(child_hwnd):
        raise RuntimeError(f"Window handle is no longer valid: {child_hwnd}")
    if win32gui.IsIconic(child_hwnd):
        win32gui.ShowWindow(child_hwnd, win32con.SW_RESTORE)


def embed_external_window(child_hwnd: int, parent_hwnd: int) -> None:
    _require_pywin32()
    if not win32gui.IsWindow(child_hwnd):
        raise RuntimeError(f"Child window handle is invalid or closed: {child_hwnd}")
    if not win32gui.IsWindow(parent_hwnd):
        raise RuntimeError(f"Parent container handle is invalid: {parent_hwnd}")

    restore_window_if_needed(child_hwnd)
    style = win32gui.GetWindowLong(child_hwnd, win32con.GWL_STYLE)
    style &= ~(
        win32con.WS_CAPTION
        | win32con.WS_THICKFRAME
        | win32con.WS_MINIMIZEBOX
        | win32con.WS_MAXIMIZEBOX
        | win32con.WS_SYSMENU
    )
    style |= win32con.WS_CHILD | win32con.WS_VISIBLE
    win32gui.SetWindowLong(child_hwnd, win32con.GWL_STYLE, style)

    previous_parent = win32gui.SetParent(child_hwnd, parent_hwnd)
    if previous_parent == 0:
        last_error = win32api.GetLastError() if win32api is not None else 0
        if last_error:
            raise RuntimeError(f"SetParent failed with Win32 error: {last_error}")

    win32gui.ShowWindow(child_hwnd, win32con.SW_SHOW)
    win32gui.SetWindowPos(
        child_hwnd,
        None,
        0,
        0,
        0,
        0,
        win32con.SWP_NOMOVE | win32con.SWP_NOSIZE | win32con.SWP_NOZORDER | win32con.SWP_FRAMECHANGED,
    )


def resize_embedded_window(child_hwnd: int, width: int, height: int) -> None:
    _require_pywin32()
    if not win32gui.IsWindow(child_hwnd):
        raise RuntimeError(f"Embedded window has been closed: {child_hwnd}")
    win32gui.MoveWindow(child_hwnd, 0, 0, max(1, int(width)), max(1, int(height)), True)


def detach_external_window(child_hwnd: int) -> None:
    _require_pywin32()
    if not win32gui.IsWindow(child_hwnd):
        return
    style = win32gui.GetWindowLong(child_hwnd, win32con.GWL_STYLE)
    style &= ~win32con.WS_CHILD
    style |= win32con.WS_OVERLAPPEDWINDOW | win32con.WS_VISIBLE
    win32gui.SetWindowLong(child_hwnd, win32con.GWL_STYLE, style)
    win32gui.SetParent(child_hwnd, 0)
    win32gui.SetWindowPos(
        child_hwnd,
        None,
        80,
        80,
        1280,
        720,
        win32con.SWP_NOZORDER | win32con.SWP_FRAMECHANGED,
    )
    win32gui.ShowWindow(child_hwnd, win32con.SW_SHOW)


def is_window_alive(hwnd: int | None) -> bool:
    if not hwnd or win32gui is None:
        return False
    return bool(win32gui.IsWindow(hwnd))
