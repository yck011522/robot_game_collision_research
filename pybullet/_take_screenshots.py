"""Take Tk-window screenshots at fixed offsets while another process runs.

Usage:
    python pybullet/_take_screenshots.py --title-substr "UR10e keyboard" \
        --offsets 2 5 8 11 --out-dir pybullet/_shots
"""

from __future__ import annotations

import argparse
import os
import sys
import time

try:
    from PIL import ImageGrab  # type: ignore
except Exception:
    print("Pillow required:  pip install Pillow", file=sys.stderr)
    raise

try:
    import ctypes
    from ctypes import wintypes
except Exception:
    ctypes = None  # type: ignore


def find_window_rect(title_substr: str):
    """Walk top-level windows on Windows, return (left, top, right, bottom)."""
    if ctypes is None:
        return None
    user32 = ctypes.windll.user32
    EnumWindowsProc = ctypes.WINFUNCTYPE(ctypes.c_bool, wintypes.HWND, wintypes.LPARAM)
    found = {"hwnd": None}

    def cb(hwnd, _):
        if not user32.IsWindowVisible(hwnd):
            return True
        length = user32.GetWindowTextLengthW(hwnd)
        if length == 0:
            return True
        buf = ctypes.create_unicode_buffer(length + 1)
        user32.GetWindowTextW(hwnd, buf, length + 1)
        if title_substr.lower() in buf.value.lower():
            found["hwnd"] = hwnd
            return False
        return True

    user32.EnumWindows(EnumWindowsProc(cb), 0)
    hwnd = found["hwnd"]
    if hwnd is None:
        return None
    rect = wintypes.RECT()
    user32.GetWindowRect(hwnd, ctypes.byref(rect))
    return (rect.left, rect.top, rect.right, rect.bottom)


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--title-substr", required=True)
    p.add_argument("--offsets", type=float, nargs="+", required=True)
    p.add_argument("--out-dir", required=True)
    args = p.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    t0 = time.perf_counter()
    for off in args.offsets:
        # Sleep until offset
        while True:
            now = time.perf_counter() - t0
            if now >= off:
                break
            time.sleep(min(0.05, off - now))
        bbox = find_window_rect(args.title_substr)
        path = os.path.join(args.out_dir, "shot_t{:05.1f}s.png".format(off))
        if bbox is None:
            print("Window not found at t={:.1f}s ({})".format(off, args.title_substr))
            continue
        try:
            img = ImageGrab.grab(bbox=bbox)
            img.save(path, "PNG")
            print("Saved {}  bbox={}".format(path, bbox))
        except Exception as exc:
            print("Capture failed at t={:.1f}s: {}".format(off, exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
