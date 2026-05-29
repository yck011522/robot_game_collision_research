"""Interactive collision explorer with per-joint local-gradient visualization.

A small Tkinter app driving:
  - one GUI PyBullet instance (visualises the robot at the current config and
    reports whether the current pose is in collision)
  - six headless PyBullet worker processes that, every UI tick, each test 20
    nearby joint variations (their assigned joint, +/- 10 degrees at 1 deg
    spacing) and return collision results
  - a coloured "horizon" bar per joint showing the local collision landscape
    around the current value (green = free, red = collision)

================================================================
ENGINEERING DECISIONS
================================================================

1. UI FRAMEWORK: Tkinter.  Built into Python, no extra dependency.  All UI
   work happens on the main thread; PyBullet GUI also runs on the main
   thread (PyBullet requires this on Windows).

2. SLIDER RANGE: -180..+180 deg as requested.  Slider values are clamped
   to the URDF joint limits before being sent to PyBullet.  Elbow has a
   tighter limit (+/- 180 deg) so its slider can hit the edges; other
   joints have wider URDF limits (+/- 360 deg) and the slider is the
   limiting factor.

3. PROBE LAYOUT: per joint we test 10 offsets on each side at 1 deg
   spacing  (offsets = -10..-1 then +1..+10, 20 total).  Total work per
   tick = 6 joints x 20 = 120 collision checks.  At ~5 ms/check on this
   machine, sequential cost would be ~600 ms.  Distributed 1 chunk per
   joint across 6 worker processes the elapsed time is roughly the
   slowest single worker ~ 100 ms plus IPC, comfortably below the human
   perceptual threshold for slider feedback.

4. WORKER POOL: 6 persistent processes built once at startup with
   ProcessPoolExecutor(initializer=_proc_initializer).  Each worker has
   its own PyBullet engine, robot cell and warmup check.  Setup cost is
   paid once at app launch (~ 2 s) and excluded from per-tick latency.

5. STATIC CHUNKING (one chunk per joint): matches the workload exactly --
   6 chunks for 6 workers, perfectly balanced.  Dynamic distribution is
   not useful here because the chunks are equal-size and the per-joint
   work is similar.

6. THROTTLING: a single 'pending_futures' list.  A new batch is dispatched
   only when (a) the slider state is dirty AND (b) the previous batch has
   fully finished.  Dragging a slider fast queues up nothing; the UI
   just shows results based on the most recently completed batch.  This
   keeps the UI responsive and prevents work pile-up.

7. CURRENT-POSE CHECK: the GUI PyBullet instance (also a planner) does
   the single collision check for the current slider position.  Workers
   only do the +/- variations.  This keeps the visualisation and the
   "in collision now?" status decoupled from worker latency.

8. VISUALISATION BAR: one Canvas per joint, drawn to scale across the
   full +/- 180 deg slider range.  At the marker position 21 cells are
   drawn (the 20 probes + the current cell).  The marker triangle is
   coloured by the current-pose collision result (green or red).  Cells
   outside the probed region are left grey.

================================================================

Usage
-----
    conda activate game
    python pybullet/bullet_collision_interactive_explorer.py
"""

from __future__ import annotations

import math
import os
import sys
import time
import tkinter as tk
from tkinter import ttk
from concurrent.futures import ProcessPoolExecutor

# CRITICAL: compas_fab before any code that could trigger a bare `import pybullet`.
from compas.data import json_load
from compas_fab.backends import PyBulletClient, PyBulletPlanner
from compas_fab.backends.exceptions import CollisionCheckError

HERE = os.path.dirname(os.path.abspath(__file__))
JSON_PATH = os.path.join(HERE, "robot_cell_and_state.json")

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Probe layout
PROBE_HALF = 7  # degrees on each side
PROBE_STEP_DEG = 1.0  # spacing (degrees)
PROBE_OFFSETS_DEG = list(range(-PROBE_HALF, 0)) + list(
    range(1, PROBE_HALF + 1)
)  # 20 entries, excludes 0
PROBE_OFFSETS_RAD = [math.radians(d) for d in PROBE_OFFSETS_DEG]

# UI tick
TICK_MS = 33  # ~30 Hz UI refresh
SLIDER_MIN_DEG = -180.0
SLIDER_MAX_DEG = 180.0

# Visual
BAR_WIDTH_PX = 760
BAR_HEIGHT_PX = 50
COLOR_BG = "#dddddd"
COLOR_FREE = "#3cb371"
COLOR_COLL = "#dc4040"
COLOR_UNKNOWN = "#bbbbbb"
COLOR_MARKER_FREE = "#1e8e4a"
COLOR_MARKER_COLL = "#a02020"
COLOR_MARKER_UNKNOWN = "#444444"


# ---------------------------------------------------------------------------
# Scene loading -- shared between GUI client and workers
# ---------------------------------------------------------------------------


def load_scene():
    data = json_load(JSON_PATH)
    robot_cell = data["robot_cell"]
    robot_cell_state = data["robot_cell_state"]
    robot_cell.robot_model.attr.pop("transmission", None)
    joints = {j.name: j for j in robot_cell.robot_model.get_configurable_joints()}
    lower = [
        joints[n].limit.lower if joints[n].limit else -math.pi for n in JOINT_NAMES
    ]
    upper = [joints[n].limit.upper if joints[n].limit else math.pi for n in JOINT_NAMES]
    return robot_cell, robot_cell_state, lower, upper


# ---------------------------------------------------------------------------
# Worker -- one persistent PyBullet client per process
# ---------------------------------------------------------------------------

_PROC_STATE: dict = {}


def _proc_initializer() -> None:
    class _NS:
        pass

    ns = _NS()
    robot_cell, robot_cell_state, _, _ = load_scene()
    client = PyBulletClient(connection_type="direct", verbose=False)
    client.__enter__()
    planner = PyBulletPlanner(client)
    planner.set_robot_cell(robot_cell)
    planner.set_robot_cell_state(robot_cell_state)
    ns.client = client
    ns.planner = planner
    ns.robot_cell_state = robot_cell_state
    ns.cfg = robot_cell_state.robot_configuration.copy()
    try:
        planner.check_collision(robot_cell_state, options={"verbose": False})
    except CollisionCheckError:
        pass
    _PROC_STATE["ns"] = ns


def _proc_ping(_):
    return os.getpid()


def _proc_check_variation(args):
    """Run 20 collision checks varying a single joint around a base config.

    Args:
        (base_rad, joint_idx, offsets_rad)
    Returns:
        list[bool] of length len(offsets_rad).  True = collision.
    """
    base_rad, joint_idx, offsets_rad = args
    ns = _PROC_STATE["ns"]
    planner = ns.planner
    rcs = ns.robot_cell_state
    cfg = ns.cfg

    results = []
    base = list(base_rad)
    for off in offsets_rad:
        vals = list(base)
        vals[joint_idx] = vals[joint_idx] + off
        cfg.joint_values = vals
        rcs.robot_configuration = cfg
        try:
            planner.check_collision(rcs, options={"verbose": False})
            results.append(False)
        except CollisionCheckError:
            results.append(True)
    return results


# ---------------------------------------------------------------------------
# Tkinter application
# ---------------------------------------------------------------------------


class ExplorerApp:
    def __init__(
        self,
        root: tk.Tk,
        executor: ProcessPoolExecutor,
        gui_planner,
        gui_robot_cell,
        gui_robot_cell_state,
        joint_limits_rad,
    ):
        self.root = root
        self.executor = executor
        self.gui_planner = gui_planner
        self.robot_cell = gui_robot_cell
        self.robot_cell_state = gui_robot_cell_state
        self.cfg = gui_robot_cell_state.robot_configuration.copy()
        self.joint_limits = joint_limits_rad  # list of (lo, hi) in rad

        # State
        initial_deg = [
            math.degrees(v)
            for v in gui_robot_cell_state.robot_configuration.joint_values
        ]
        # Clamp initial to slider range
        initial_deg = [max(SLIDER_MIN_DEG, min(SLIDER_MAX_DEG, d)) for d in initial_deg]
        self.slider_vars = [tk.DoubleVar(value=d) for d in initial_deg]
        self.value_labels: list[ttk.Label] = []
        self.bar_canvases: list[tk.Canvas] = []
        self.dirty = True
        self.pending_futures: list = []
        self.last_results: list = [[None] * len(PROBE_OFFSETS_DEG) for _ in range(6)]
        self.current_in_coll: bool | None = None
        self.last_batch_ms: float = 0.0

        self._build_ui()

        # Wire slider callbacks AFTER UI is built (avoid firing during setup)
        for var in self.slider_vars:
            var.trace_add("write", lambda *_: self._on_slider_change())

        # Kick off the first update
        self._tick()

    # ----- UI construction -----

    def _build_ui(self) -> None:
        self.root.title("UR10e collision explorer")
        self.root.geometry("760x520")

        # Status bar at top
        status_frame = ttk.Frame(self.root, padding=(10, 8))
        status_frame.pack(side=tk.TOP, fill=tk.X)
        ttk.Label(status_frame, text="Current pose:", font=("Segoe UI", 11)).pack(
            side=tk.LEFT
        )
        self.status_label = tk.Label(
            status_frame,
            text="(checking...)",
            font=("Segoe UI", 11, "bold"),
            fg="white",
            bg=COLOR_MARKER_UNKNOWN,
            width=12,
            anchor="center",
        )
        self.status_label.pack(side=tk.LEFT, padx=(8, 16))
        self.latency_label = ttk.Label(status_frame, text="", font=("Consolas", 9))
        self.latency_label.pack(side=tk.LEFT)

        # Legend
        legend_frame = ttk.Frame(self.root, padding=(10, 0))
        legend_frame.pack(side=tk.TOP, fill=tk.X)
        ttk.Label(
            legend_frame,
            text="Bar shows local collision landscape (+/- 10 deg around current, 1 deg steps):",
            font=("Segoe UI", 9),
        ).pack(side=tk.LEFT)
        for label, color in [
            (" free ", COLOR_FREE),
            (" collision ", COLOR_COLL),
            (" unknown ", COLOR_UNKNOWN),
        ]:
            tk.Label(legend_frame, text=label, bg=color, font=("Segoe UI", 8)).pack(
                side=tk.LEFT, padx=2
            )

        # One row per joint
        body = ttk.Frame(self.root, padding=(10, 6))
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        for i, name in enumerate(JOINT_NAMES):
            row = ttk.Frame(body)
            row.pack(side=tk.TOP, fill=tk.X, pady=4)

            # Joint label
            ttk.Label(
                row, text="J{}: {}".format(i, name), width=24, font=("Consolas", 9)
            ).pack(side=tk.LEFT)

            # Slider
            scale = ttk.Scale(
                row,
                from_=SLIDER_MIN_DEG,
                to=SLIDER_MAX_DEG,
                orient=tk.HORIZONTAL,
                variable=self.slider_vars[i],
                length=180,
            )
            scale.pack(side=tk.LEFT, padx=(0, 6))

            # Numeric value
            lbl = ttk.Label(row, text="0.0", width=7, font=("Consolas", 9), anchor="e")
            lbl.pack(side=tk.LEFT, padx=(0, 8))
            self.value_labels.append(lbl)

            # Bar canvas
            canvas = tk.Canvas(
                row,
                width=BAR_WIDTH_PX,
                height=BAR_HEIGHT_PX,
                bg=COLOR_BG,
                highlightthickness=1,
                highlightbackground="#888888",
            )
            canvas.pack(side=tk.LEFT)
            self.bar_canvases.append(canvas)

        # Buttons row at the bottom
        btn_frame = ttk.Frame(self.root, padding=(10, 8))
        btn_frame.pack(side=tk.BOTTOM, fill=tk.X)
        ttk.Button(
            btn_frame, text="Reset to loaded pose", command=self._reset_to_initial
        ).pack(side=tk.LEFT)
        ttk.Button(btn_frame, text="Zero all", command=self._zero_all).pack(
            side=tk.LEFT, padx=(8, 0)
        )
        ttk.Label(
            btn_frame,
            text="(workers will refresh after each move)",
            font=("Segoe UI", 8),
            foreground="#666666",
        ).pack(side=tk.RIGHT)

    # ----- Slider helpers -----

    def _on_slider_change(self) -> None:
        # Just mark dirty; the tick decides when to dispatch.
        self.dirty = True
        # Update numeric label and force-clear stale bar results immediately
        for i, var in enumerate(self.slider_vars):
            self.value_labels[i].config(text="{:+.1f} deg".format(var.get()))

    def _reset_to_initial(self) -> None:
        for var, v in zip(
            self.slider_vars, self.robot_cell_state.robot_configuration.joint_values
        ):
            var.set(max(SLIDER_MIN_DEG, min(SLIDER_MAX_DEG, math.degrees(v))))

    def _zero_all(self) -> None:
        for var in self.slider_vars:
            var.set(0.0)

    # ----- Main tick -----

    def _tick(self) -> None:
        try:
            # 1) Harvest completed worker batch
            if self.pending_futures and all(f.done() for f in self.pending_futures):
                try:
                    self.last_results = [f.result() for f in self.pending_futures]
                except Exception as exc:
                    print("Worker error:", exc, file=sys.stderr)
                    self.last_results = [
                        [None] * len(PROBE_OFFSETS_DEG) for _ in range(6)
                    ]
                self.pending_futures = []
                self.last_batch_ms = (
                    time.perf_counter() - self._batch_start_t
                ) * 1000.0
                self._redraw_all_bars()
                self._update_latency_label()

            # 2) If dirty and worker pool is free, dispatch a new batch
            if self.dirty and not self.pending_futures:
                self.dirty = False
                base_rad = [math.radians(v.get()) for v in self.slider_vars]
                # Clamp to URDF joint limits before sending
                base_rad = [
                    max(lo, min(hi, v))
                    for v, (lo, hi) in zip(base_rad, self.joint_limits)
                ]

                # Update the GUI client + current-pose collision status
                self._update_gui_pybullet(base_rad)

                # Submit one chunk per joint
                chunks = [(tuple(base_rad), i, PROBE_OFFSETS_RAD) for i in range(6)]
                self._batch_start_t = time.perf_counter()
                self.pending_futures = [
                    self.executor.submit(_proc_check_variation, c) for c in chunks
                ]
        finally:
            self.root.after(TICK_MS, self._tick)

    # ----- GUI PyBullet update -----

    def _update_gui_pybullet(self, joint_values_rad: list) -> None:
        self.cfg.joint_values = list(joint_values_rad)
        self.robot_cell_state.robot_configuration = self.cfg
        try:
            self.gui_planner.check_collision(
                self.robot_cell_state, options={"verbose": False}
            )
            self.current_in_coll = False
        except CollisionCheckError:
            self.current_in_coll = True

        # Update status badge
        if self.current_in_coll:
            self.status_label.config(text="COLLISION", bg=COLOR_MARKER_COLL)
        else:
            self.status_label.config(text="FREE", bg=COLOR_MARKER_FREE)

    def _update_latency_label(self) -> None:
        self.latency_label.config(
            text="last batch: {:5.1f} ms  (120 checks across 6 workers)".format(
                self.last_batch_ms
            )
        )

    # ----- Bar drawing -----

    def _redraw_all_bars(self) -> None:
        for i in range(6):
            self._draw_bar(i)

    def _draw_bar(self, idx: int) -> None:
        canvas = self.bar_canvases[idx]
        canvas.delete("all")
        w = BAR_WIDTH_PX
        h = BAR_HEIGHT_PX

        def deg_to_x(d: float) -> float:
            frac = (d - SLIDER_MIN_DEG) / (SLIDER_MAX_DEG - SLIDER_MIN_DEG)
            return frac * w

        cur_deg = self.slider_vars[idx].get()

        # Background tick marks every 30 deg
        for d in range(-180, 181, 30):
            x = deg_to_x(d)
            canvas.create_line(x, h - 4, x, h, fill="#888888")
        # Zero centre line
        x0 = deg_to_x(0)
        canvas.create_line(x0, 0, x0, h, fill="#aaaaaa", dash=(2, 3))

        # Probe cells
        results = self.last_results[idx]
        for off_deg, result in zip(PROBE_OFFSETS_DEG, results):
            d = cur_deg + off_deg
            xa = deg_to_x(d - 0.5 * PROBE_STEP_DEG)
            xb = deg_to_x(d + 0.5 * PROBE_STEP_DEG)
            if result is None:
                color = COLOR_UNKNOWN
            elif result:
                color = COLOR_COLL
            else:
                color = COLOR_FREE
            canvas.create_rectangle(xa, 4, xb, h - 5, fill=color, outline="")

        # Centre cell (current pose) -- coloured by overall status if known
        d = cur_deg
        xa = deg_to_x(d - 0.5 * PROBE_STEP_DEG)
        xb = deg_to_x(d + 0.5 * PROBE_STEP_DEG)
        if self.current_in_coll is None:
            color = COLOR_UNKNOWN
        elif self.current_in_coll:
            color = COLOR_COLL
        else:
            color = COLOR_FREE
        canvas.create_rectangle(xa, 4, xb, h - 5, fill=color, outline="black")

        # Triangle marker on top, pointing down
        cx = deg_to_x(cur_deg)
        if self.current_in_coll is None:
            mc = COLOR_MARKER_UNKNOWN
        elif self.current_in_coll:
            mc = COLOR_MARKER_COLL
        else:
            mc = COLOR_MARKER_FREE
        canvas.create_polygon(
            cx - 5,
            -1,
            cx + 5,
            -1,
            cx,
            7,
            fill=mc,
            outline="black",
        )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def _wait_for_workers(executor: ProcessPoolExecutor, n: int) -> None:
    """Force all worker processes to finish their initializer before we start."""
    _ = list(executor.map(_proc_ping, range(n * 3)))


def main() -> None:
    print("Loading scene for GUI PyBullet instance ...")
    robot_cell, robot_cell_state, lower, upper = load_scene()
    joint_limits = list(zip(lower, upper))

    print("Starting GUI PyBullet client ...")
    gui_client = PyBulletClient(connection_type="gui", verbose=False)
    gui_client.__enter__()
    gui_planner = PyBulletPlanner(gui_client)
    gui_planner.set_robot_cell(robot_cell)
    gui_planner.set_robot_cell_state(robot_cell_state)
    # Warmup
    try:
        gui_planner.check_collision(robot_cell_state, options={"verbose": False})
    except CollisionCheckError:
        pass

    n_workers = 6
    print("Spawning {} worker processes (one per joint) ...".format(n_workers))
    executor = ProcessPoolExecutor(max_workers=n_workers, initializer=_proc_initializer)

    print("Warming up workers ...")
    _wait_for_workers(executor, n_workers)
    print("Workers ready.")

    print("Launching UI ...")
    root = tk.Tk()
    app = ExplorerApp(
        root,
        executor,
        gui_planner,
        robot_cell,
        robot_cell_state,
        joint_limits,
    )

    def on_close():
        print("Shutting down ...")
        try:
            executor.shutdown(wait=False, cancel_futures=True)
        except Exception:
            pass
        try:
            gui_client.__exit__(None, None, None)
        except Exception:
            pass
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)

    try:
        root.mainloop()
    finally:
        try:
            executor.shutdown(wait=False, cancel_futures=True)
        except Exception:
            pass
        try:
            gui_client.__exit__(None, None, None)
        except Exception:
            pass


if __name__ == "__main__":
    # Required on Windows for ProcessPoolExecutor inside a __main__ guard.
    main()
