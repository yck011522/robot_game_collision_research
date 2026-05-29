"""Smoke test: open a small Tk window for ~5s, save a PNG, then exit.

Run:
    python pybullet/_tk_smoke.py
"""

from __future__ import annotations

import os
import sys
import time
import tkinter as tk


def main() -> int:
    root = tk.Tk()
    root.title("Tk Smoke Test")
    root.geometry("360x180+200+200")

    label = tk.Label(
        root,
        text="Tk smoke test\n(will auto-close in 5s)",
        font=("Segoe UI", 14),
        fg="#202020",
    )
    label.pack(expand=True, fill="both", padx=12, pady=12)

    countdown = tk.Label(root, text="5", font=("Segoe UI", 24, "bold"), fg="#a02020")
    countdown.pack(pady=8)

    start = time.perf_counter()
    deadline = start + 5.0

    out_dir = os.path.dirname(os.path.abspath(__file__))
    screenshot_path = os.path.join(out_dir, "_tk_smoke_screenshot.png")
    log_path = os.path.join(out_dir, "_tk_smoke_result.txt")

    state = {"shot_taken": False, "shot_ok": False, "shot_err": ""}

    def take_screenshot() -> None:
        # Try PIL ImageGrab (Windows-friendly, captures the window region).
        try:
            from PIL import ImageGrab  # type: ignore

            root.update_idletasks()
            x = root.winfo_rootx()
            y = root.winfo_rooty()
            w = root.winfo_width()
            h = root.winfo_height()
            bbox = (x, y, x + w, y + h)
            img = ImageGrab.grab(bbox=bbox)
            img.save(screenshot_path, "PNG")
            state["shot_ok"] = True
        except Exception as e:  # noqa: BLE001
            state["shot_err"] = "{}: {}".format(type(e).__name__, e)
        finally:
            state["shot_taken"] = True

    def tick() -> None:
        now = time.perf_counter()
        remaining = max(0, int(deadline - now + 0.5))
        countdown.config(text=str(remaining))
        # Take screenshot ~1s in (after window has had time to render).
        if not state["shot_taken"] and (now - start) >= 1.0:
            take_screenshot()
        if now >= deadline:
            root.destroy()
            return
        root.after(100, tick)

    root.after(100, tick)
    try:
        root.mainloop()
    except Exception as e:  # noqa: BLE001
        with open(log_path, "w", encoding="utf-8") as f:
            f.write("mainloop_error: {}: {}\n".format(type(e).__name__, e))
        return 2

    with open(log_path, "w", encoding="utf-8") as f:
        f.write("mainloop_completed: True\n")
        f.write("screenshot_attempted: {}\n".format(state["shot_taken"]))
        f.write("screenshot_ok: {}\n".format(state["shot_ok"]))
        f.write("screenshot_path: {}\n".format(screenshot_path))
        f.write("screenshot_error: {}\n".format(state["shot_err"]))
        f.write("python: {}\n".format(sys.version.replace("\n", " ")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
