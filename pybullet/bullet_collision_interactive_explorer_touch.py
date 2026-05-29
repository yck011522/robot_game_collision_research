"""Interactive collision explorer using discovered touch lists.

Variant of ``bullet_collision_interactive_explorer.py`` that loads
``bullet_collision_pair_discovery.json`` and patches every
``RigidBodyState.touch_links`` / ``touch_bodies`` (and the tool states)
with the pairs the discovery run never observed colliding. Both the GUI
PyBullet client and every worker process apply the same patch, so all
collision checks skip the never-observed pairs.

See ``bullet_collision_interactive_explorer.py`` for the design notes;
this file only diverges in scene loading.

Usage
-----
    conda activate game
    python pybullet/bullet_collision_interactive_explorer_touch.py
"""

from __future__ import annotations

import json
import math
import os
import sys
import time
import tkinter as tk
from tkinter import ttk
from concurrent.futures import ProcessPoolExecutor

from compas.data import json_load
from compas_fab.backends import PyBulletClient, PyBulletPlanner
from compas_fab.backends.exceptions import CollisionCheckError

HERE = os.path.dirname(os.path.abspath(__file__))
JSON_PATH = os.path.join(HERE, "robot_cell_and_state.json")
DISCOVERY_PATH = os.path.join(HERE, "bullet_collision_pair_discovery.json")

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

PROBE_HALF = 12
PROBE_STEP_DEG = 1.0
PROBE_OFFSETS_DEG = list(range(-PROBE_HALF, 0)) + list(range(1, PROBE_HALF + 1))
PROBE_OFFSETS_RAD = [math.radians(d) for d in PROBE_OFFSETS_DEG]

TICK_MS = 33
SLIDER_MIN_DEG = -180.0
SLIDER_MAX_DEG = 180.0

BAR_WIDTH_PX = 720
BAR_HEIGHT_PX = 50
COLOR_BG = "#dddddd"
COLOR_FREE = "#3cb371"
COLOR_COLL = "#dc4040"
COLOR_UNKNOWN = "#bbbbbb"
COLOR_MARKER_FREE = "#1e8e4a"
COLOR_MARKER_COLL = "#a02020"
COLOR_MARKER_UNKNOWN = "#444444"


# ---------------------------------------------------------------------------
# Scene loading + touch-list patching
# ---------------------------------------------------------------------------


def _load_discovery():
    with open(DISCOVERY_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


def _apply_touch_lists(robot_cell_state, discovery: dict) -> dict:
    """Mutate robot_cell_state in place using discovered never-colliding pairs."""
    per_body = discovery.get("per_rigid_body", {})
    per_tool = discovery.get("per_tool", {})

    n_bodies = 0
    n_tools = 0
    total_links = 0
    total_bodies = 0

    for key, info in per_body.items():
        state = robot_cell_state.rigid_body_states.get(key)
        if state is None:
            continue
        tl = list(info.get("touch_links_candidates", []))
        tb = list(info.get("touch_bodies_candidates", []))
        state.touch_links = tl
        state.touch_bodies = tb
        n_bodies += 1
        total_links += len(tl)
        total_bodies += len(tb)

    if hasattr(robot_cell_state, "tool_states"):
        for key, info in per_tool.items():
            state = robot_cell_state.tool_states.get(key)
            if state is None:
                continue
            tl = list(info.get("touch_links_candidates", []))
            tb = list(info.get("touch_bodies_candidates", []))
            if hasattr(state, "touch_links"):
                state.touch_links = tl
            if hasattr(state, "touch_bodies"):
                state.touch_bodies = tb
            n_tools += 1
            total_links += len(tl)
            total_bodies += len(tb)

    return {
        "n_bodies_patched": n_bodies,
        "n_tools_patched": n_tools,
        "total_touch_links": total_links,
        "total_touch_bodies": total_bodies,
    }


def load_scene(apply_touch: bool = True):
    data = json_load(JSON_PATH)
    robot_cell = data["robot_cell"]
    robot_cell_state = data["robot_cell_state"]
    robot_cell.robot_model.attr.pop("transmission", None)
    if apply_touch:
        discovery = _load_discovery()
        _apply_touch_lists(robot_cell_state, discovery)
    joints = {j.name: j for j in robot_cell.robot_model.get_configurable_joints()}
    lower = [
        joints[n].limit.lower if joints[n].limit else -math.pi for n in JOINT_NAMES
    ]
    upper = [joints[n].limit.upper if joints[n].limit else math.pi for n in JOINT_NAMES]
    return robot_cell, robot_cell_state, lower, upper


# ---------------------------------------------------------------------------
# Worker
# ---------------------------------------------------------------------------

_PROC_STATE: dict = {}


def _proc_initializer() -> None:
    class _NS:
        pass

    ns = _NS()
    robot_cell, robot_cell_state, _, _ = load_scene(apply_touch=True)
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
        patch_stats: dict,
    ):
        self.root = root
        self.executor = executor
        self.gui_planner = gui_planner
        self.robot_cell = gui_robot_cell
        self.robot_cell_state = gui_robot_cell_state
        self.cfg = gui_robot_cell_state.robot_configuration.copy()
        self.joint_limits = joint_limits_rad
        self.patch_stats = patch_stats

        initial_deg = [
            math.degrees(v)
            for v in gui_robot_cell_state.robot_configuration.joint_values
        ]
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

        for var in self.slider_vars:
            var.trace_add("write", lambda *_: self._on_slider_change())

        self._tick()

    def _build_ui(self) -> None:
        self.root.title("UR10e collision explorer (touch lists)")
        self.root.geometry("1300x560")

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

        # Touch-list info bar
        info_frame = ttk.Frame(self.root, padding=(10, 0))
        info_frame.pack(side=tk.TOP, fill=tk.X)
        ttk.Label(
            info_frame,
            text=(
                "Touch lists active: {nb} bodies + {nt} tools patched, "
                "{tl} touch_links + {tb} touch_bodies skipped per check  "
                "(source: {f})".format(
                    nb=self.patch_stats["n_bodies_patched"],
                    nt=self.patch_stats["n_tools_patched"],
                    tl=self.patch_stats["total_touch_links"],
                    tb=self.patch_stats["total_touch_bodies"],
                    f=os.path.basename(DISCOVERY_PATH),
                )
            ),
            font=("Segoe UI", 9),
            foreground="#2a6f3a",
        ).pack(side=tk.LEFT)

        legend_frame = ttk.Frame(self.root, padding=(10, 0))
        legend_frame.pack(side=tk.TOP, fill=tk.X)
        ttk.Label(
            legend_frame,
            text="Bar shows local collision landscape (+/- 12 deg around current, 1 deg steps):",
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

        body = ttk.Frame(self.root, padding=(10, 6))
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        for i, name in enumerate(JOINT_NAMES):
            row = ttk.Frame(body)
            row.pack(side=tk.TOP, fill=tk.X, pady=4)

            ttk.Label(
                row, text="J{}: {}".format(i, name), width=24, font=("Consolas", 9)
            ).pack(side=tk.LEFT)

            scale = ttk.Scale(
                row,
                from_=SLIDER_MIN_DEG,
                to=SLIDER_MAX_DEG,
                orient=tk.HORIZONTAL,
                variable=self.slider_vars[i],
                length=180,
            )
            scale.pack(side=tk.LEFT, padx=(0, 6))

            lbl = ttk.Label(row, text="0.0", width=7, font=("Consolas", 9), anchor="e")
            lbl.pack(side=tk.LEFT, padx=(0, 8))
            self.value_labels.append(lbl)

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

    def _on_slider_change(self) -> None:
        self.dirty = True
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

    def _tick(self) -> None:
        try:
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

            if self.dirty and not self.pending_futures:
                self.dirty = False
                base_rad = [math.radians(v.get()) for v in self.slider_vars]
                base_rad = [
                    max(lo, min(hi, v))
                    for v, (lo, hi) in zip(base_rad, self.joint_limits)
                ]
                self._update_gui_pybullet(base_rad)
                chunks = [(tuple(base_rad), i, PROBE_OFFSETS_RAD) for i in range(6)]
                self._batch_start_t = time.perf_counter()
                self.pending_futures = [
                    self.executor.submit(_proc_check_variation, c) for c in chunks
                ]
        finally:
            self.root.after(TICK_MS, self._tick)

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

        if self.current_in_coll:
            self.status_label.config(text="COLLISION", bg=COLOR_MARKER_COLL)
        else:
            self.status_label.config(text="FREE", bg=COLOR_MARKER_FREE)

    def _update_latency_label(self) -> None:
        self.latency_label.config(
            text="last batch: {:5.1f} ms  (120 checks across 6 workers, touch lists ON)".format(
                self.last_batch_ms
            )
        )

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

        for d in range(-180, 181, 30):
            x = deg_to_x(d)
            canvas.create_line(x, h - 4, x, h, fill="#888888")
        x0 = deg_to_x(0)
        canvas.create_line(x0, 0, x0, h, fill="#aaaaaa", dash=(2, 3))

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

        cx = deg_to_x(cur_deg)
        if self.current_in_coll is None:
            mc = COLOR_MARKER_UNKNOWN
        elif self.current_in_coll:
            mc = COLOR_MARKER_COLL
        else:
            mc = COLOR_MARKER_FREE
        canvas.create_polygon(
            cx - 5, -1, cx + 5, -1, cx, 7, fill=mc, outline="black",
        )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def _wait_for_workers(executor: ProcessPoolExecutor, n: int) -> None:
    _ = list(executor.map(_proc_ping, range(n * 3)))


def main() -> None:
    if not os.path.exists(DISCOVERY_PATH):
        raise SystemExit(
            "Discovery JSON not found: {}\n  Run bullet_collision_pair_discovery.py first.".format(
                DISCOVERY_PATH
            )
        )

    print("Loading scene + applying touch lists for GUI PyBullet instance ...")
    robot_cell, robot_cell_state, lower, upper = load_scene(apply_touch=True)
    joint_limits = list(zip(lower, upper))

    # Recompute patch stats for the info banner (the discovery + state we loaded).
    discovery = _load_discovery()
    # Build a fresh state purely to count what gets patched (counts are
    # independent of which copy we use, but using a throwaway avoids
    # double-counting on already-patched lists if entries grow).
    _, rcs_stats, _, _ = load_scene(apply_touch=False)
    patch_stats = _apply_touch_lists(rcs_stats, discovery)
    print(
        "  touch lists applied: {nb} bodies, {nt} tools, "
        "{tl} touch_links, {tb} touch_bodies".format(
            nb=patch_stats["n_bodies_patched"],
            nt=patch_stats["n_tools_patched"],
            tl=patch_stats["total_touch_links"],
            tb=patch_stats["total_touch_bodies"],
        )
    )

    print("Starting GUI PyBullet client ...")
    gui_client = PyBulletClient(connection_type="gui", verbose=False)
    gui_client.__enter__()
    gui_planner = PyBulletPlanner(gui_client)
    gui_planner.set_robot_cell(robot_cell)
    gui_planner.set_robot_cell_state(robot_cell_state)
    try:
        gui_planner.check_collision(robot_cell_state, options={"verbose": False})
    except CollisionCheckError:
        pass

    n_workers = 6
    print("Spawning {} worker processes (one per joint, touch lists ON) ...".format(n_workers))
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
        patch_stats,
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
    main()
