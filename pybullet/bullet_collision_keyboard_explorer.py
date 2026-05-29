"""Keyboard-driven interactive collision explorer.

A Tkinter app where six robot joints are jogged simultaneously by held keys
on a US keyboard:

    1 2 3 4 5 6   -> fast positive  (configurable, default +20 deg/s)
    q w e r t y   -> slow positive  (configurable, default +10 deg/s)
    a s d f g h   -> slow negative  (configurable, default -10 deg/s)
    z x c v b n   -> fast negative  (configurable, default -20 deg/s)

Held keys are combined ALGEBRAICALLY per axis. Holding 1+q on J0 gives
+30 deg/s desired; q+a on the same axis cancels to 0.

Each tick we:
  1. Read held-keys -> desired joint velocity vector v_des (rad/s).
  2. Acceleration-clamp current_v toward v_des.
  3. Velocity-clamp current_v to +/- max_vel.
  4. SYNCHRONOUSLY run the forward-path collision check for v_cmd's
     unit direction (12 steps x FORWARD_STEP_DEG, deterministically
     split across `--forward-workers` chunks). Wait for ALL workers
     before continuing. No cached result is ever reused. This is the
     SAFETY GATE.
  5. ASYNCHRONOUSLY dispatch the +/-PROBE_HALF_DEG proximity probes
     (6 axes split into `--prox-workers` chunks). Fire-and-forget on a
     SEPARATE process pool so it cannot block the safety gate. The
     freshest completed batch is used for the proximity soft slow-down;
     a slightly stale (~1 tick) batch is acceptable because proximity
     is never a hard safety gate.
  6. Compute path-clamp scalar (safety, can go to 0) and proximity
     scalar (soft slow-down, never below `prox_floor`).
  7. v_out = current_v * min(path_scalar, prox_scalar).
  8. Integrate position with dt; push to GUI PyBullet client.

Clamps are GLOBAL (single scalar applied to all axes) so the velocity
vector direction is preserved -- the direction the workers checked is
exactly the direction the robot moves in.

Workers
-------
Two independent ProcessPoolExecutors (default 6 + 6 = 12 processes).
This pairs naturally with a 10-thread production target by running
`--forward-workers 3 --prox-workers 2` per robot, so two robots fit
inside 10 worker processes. The chunking is recomputed from the worker
counts but the partition is fixed at startup -- no load balancing at
runtime.

  - 1 GUI PyBullet (main thread, visualises current pose)
  - `--forward-workers` synchronous safety-gate workers
  - `--prox-workers` asynchronous soft-slowdown workers

All workers patch in the touch-lists from
``bullet_collision_pair_discovery.json``.

Usage
-----
    conda activate game
    python pybullet/bullet_collision_keyboard_explorer.py
    # production-equivalent allocation per robot (5 workers / robot):
    python pybullet/bullet_collision_keyboard_explorer.py \
        --forward-workers 3 --prox-workers 2
"""

from __future__ import annotations

import datetime
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
LOG_DIR = os.path.join(HERE, "explorer_logs")


def _pack_bits(bits) -> int:
    """Pack a sequence of bools into a single integer (bit i = bits[i])."""
    n = 0
    for i, b in enumerate(bits):
        if b:
            n |= 1 << i
    return n


def unpack_bits(value: int, length: int) -> list[bool]:
    """Inverse of _pack_bits. Public so the replay tool can import it."""
    return [bool((value >> i) & 1) for i in range(length)]


JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

INITIAL_POS_DEG = [0.0, -90.0, 90.0, 0.0, 0.0, 0.0]

# Keyboard rows (US layout). Index 0..5 -> joints J0..J5.
FAST_POS_KEYS = ["1", "2", "3", "4", "5", "6"]
SLOW_POS_KEYS = ["q", "w", "e", "r", "t", "y"]
SLOW_NEG_KEYS = ["a", "s", "d", "f", "g", "h"]
FAST_NEG_KEYS = ["z", "x", "c", "v", "b", "n"]

# Probe layout (proximity)
PROBE_HALF_DEG = 10
PROBE_OFFSETS_DEG = list(range(-PROBE_HALF_DEG, 0)) + list(range(1, PROBE_HALF_DEG + 1))
PROBE_OFFSETS_RAD = [math.radians(d) for d in PROBE_OFFSETS_DEG]

# Forward-trajectory layout (FIXED JOINT-SPACE DISTANCE spacing, not time)
# We step N_FORWARD_STEPS along the unit direction of v_cmd, each step is
# FORWARD_STEP_DEG degrees in 6D joint space (L2 norm). The path-clamp scalar
# is therefore proportional to actual distance-to-collision, independent of
# the current speed.
#
# DETERMINISTIC DISPATCH: forward steps are partitioned into N_FORWARD_WORKERS
# fixed contiguous chunks (one chunk per worker). Proximity axes are likewise
# partitioned into N_PROX_WORKERS fixed contiguous chunks of joints. The
# partitions never change at runtime; the pools do no load balancing.
N_FORWARD_STEPS = 12
FORWARD_STEP_DEG = 1.0
FORWARD_HORIZON_DEG = N_FORWARD_STEPS * FORWARD_STEP_DEG  # 12 deg
DEFAULT_FORWARD_WORKERS = 6  # CLI overridable; production target is 3 per robot
DEFAULT_PROX_WORKERS = 6  # CLI overridable; production target is 2 per robot


def _partition(items, n_chunks):
    """Split a sequence into n_chunks contiguous tuples, as equal as possible.

    Deterministic: no load balancing. Sizes differ by at most 1.
    """
    items = list(items)
    base, rem = divmod(len(items), n_chunks)
    out = []
    i = 0
    for k in range(n_chunks):
        size = base + (1 if k < rem else 0)
        out.append(tuple(items[i : i + size]))
        i += size
    return tuple(out)

# UI defaults
DEFAULT_FPS = 30
# Per-axis defaults. First three joints (big arm) get conservative limits;
# wrist joints (last three) can move faster.
DEFAULT_MAX_VEL_DPS = [20.0, 20.0, 20.0, 30.0, 30.0, 30.0]
DEFAULT_MAX_ACCEL_DPS2 = [50.0, 50.0, 50.0, 80.0, 80.0, 80.0]
DEFAULT_SLOW_DPS = 10.0
DEFAULT_FAST_DPS = 30.0
DEFAULT_PROX_FLOOR_PCT = 50.0
DEFAULT_PATH_CUTOFF_DEG = 3.0  # path-clamp scale = 0 if obstacle within this distance

# Drawing
COLOR_BG = "#dddddd"
COLOR_FREE = "#3cb371"
COLOR_COLL = "#dc4040"
COLOR_UNKNOWN = "#bbbbbb"
COLOR_MARKER_FREE = "#1e8e4a"
COLOR_MARKER_COLL = "#a02020"
COLOR_MARKER_UNKNOWN = "#444444"
COLOR_VEL_FILL = "#4f9fd6"
COLOR_DESIRED = "#1f78ff"
COLOR_AFTER_PATH = "#ff9020"
COLOR_AFTER_PROX = "#222222"

PROX_BAR_W = 620
PROX_BAR_H = 38
FWD_BAR_W = 700
FWD_BAR_H = 44
VEL_BAR_W = 240
VEL_BAR_H = 38
SLIDER_MIN_DEG = -180.0
SLIDER_MAX_DEG = 180.0


# ---------------------------------------------------------------------------
# Scene + touch lists
# ---------------------------------------------------------------------------


def _load_discovery():
    with open(DISCOVERY_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


def _apply_touch_lists(robot_cell_state, discovery: dict) -> dict:
    per_body = discovery.get("per_rigid_body", {})
    per_tool = discovery.get("per_tool", {})
    n_b = n_t = tl_total = tb_total = 0
    for key, info in per_body.items():
        state = robot_cell_state.rigid_body_states.get(key)
        if state is None:
            continue
        tl = list(info.get("touch_links_candidates", []))
        tb = list(info.get("touch_bodies_candidates", []))
        state.touch_links = tl
        state.touch_bodies = tb
        n_b += 1
        tl_total += len(tl)
        tb_total += len(tb)
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
            n_t += 1
            tl_total += len(tl)
            tb_total += len(tb)
    return {
        "n_bodies_patched": n_b,
        "n_tools_patched": n_t,
        "total_touch_links": tl_total,
        "total_touch_bodies": tb_total,
    }


def load_scene(apply_touch: bool = True):
    data = json_load(JSON_PATH)
    robot_cell = data["robot_cell"]
    robot_cell_state = data["robot_cell_state"]
    robot_cell.robot_model.attr.pop("transmission", None)
    if apply_touch:
        _apply_touch_lists(robot_cell_state, _load_discovery())
    joints = {j.name: j for j in robot_cell.robot_model.get_configurable_joints()}
    lower = [
        joints[n].limit.lower if joints[n].limit else -math.pi for n in JOINT_NAMES
    ]
    upper = [joints[n].limit.upper if joints[n].limit else math.pi for n in JOINT_NAMES]
    return robot_cell, robot_cell_state, lower, upper


# ---------------------------------------------------------------------------
# Worker process state
# ---------------------------------------------------------------------------

_W: dict = {}


def _proc_init() -> None:
    robot_cell, robot_cell_state, _, _ = load_scene(apply_touch=True)
    client = PyBulletClient(connection_type="direct", verbose=False)
    client.__enter__()
    planner = PyBulletPlanner(client)
    planner.set_robot_cell(robot_cell)
    planner.set_robot_cell_state(robot_cell_state)
    try:
        planner.check_collision(robot_cell_state, options={"verbose": False})
    except CollisionCheckError:
        pass
    _W["client"] = client
    _W["planner"] = planner
    _W["rcs"] = robot_cell_state
    _W["cfg"] = robot_cell_state.robot_configuration.copy()


def _proc_ping(_):
    return os.getpid()


def _proc_proximity(args):
    """Check 20 (or 2*PROBE_HALF) offsets on a single joint.

    args = (base_rad_tuple, joint_idx, offsets_rad_tuple)
    returns list[bool] (True = collision).
    """
    base_rad, joint_idx, offsets_rad = args
    planner = _W["planner"]
    rcs = _W["rcs"]
    cfg = _W["cfg"]
    out = []
    base = list(base_rad)
    for off in offsets_rad:
        vals = list(base)
        vals[joint_idx] = vals[joint_idx] + off
        cfg.joint_values = vals
        rcs.robot_configuration = cfg
        try:
            planner.check_collision(rcs, options={"verbose": False})
            out.append(False)
        except CollisionCheckError:
            out.append(True)
    return out


def _proc_proximity_chunk(args):
    """Run proximity probes for a deterministic subset of joint axes.

    args = (base_rad_tuple, axes_tuple, offsets_rad_tuple)
    returns dict[int, list[bool]] keyed by axis index.
    """
    base_rad, axes, offsets_rad = args
    planner = _W["planner"]
    rcs = _W["rcs"]
    cfg = _W["cfg"]
    base = list(base_rad)
    out = {}
    for axis in axes:
        axis_out = []
        for off in offsets_rad:
            vals = list(base)
            vals[axis] = vals[axis] + off
            cfg.joint_values = vals
            rcs.robot_configuration = cfg
            try:
                planner.check_collision(rcs, options={"verbose": False})
                axis_out.append(False)
            except CollisionCheckError:
                axis_out.append(True)
        out[axis] = axis_out
    return out


def _proc_forward_chunk(args):
    """Check a deterministic subset of forward-trajectory step indices.

    args = (base_rad_tuple, step_vec_rad_tuple, step_indices_tuple)
        step_vec is the per-step joint-space offset (radians) along the unit
            direction of v_cmd, scaled to FORWARD_STEP_DEG.
        step_indices is the 1-based set of step numbers this worker is
            responsible for (e.g. (1, 2) -> test base + step_vec*1 and
            base + step_vec*2). The split is fixed at the call site, so the
            worker pool has no load balancing role; each invocation does
            exactly len(step_indices) collision checks.
    returns list[bool] aligned to step_indices (True = collision).
    """
    base_rad, step_vec, indices = args
    planner = _W["planner"]
    rcs = _W["rcs"]
    cfg = _W["cfg"]
    base = list(base_rad)
    out = []
    for k in indices:
        vals = [base[i] + step_vec[i] * k for i in range(6)]
        cfg.joint_values = vals
        rcs.robot_configuration = cfg
        try:
            planner.check_collision(rcs, options={"verbose": False})
            out.append(False)
        except CollisionCheckError:
            out.append(True)
    return out


# ---------------------------------------------------------------------------
# Tk application
# ---------------------------------------------------------------------------


class KeyboardExplorer:
    def __init__(
        self,
        root: tk.Tk,
        fwd_executor: ProcessPoolExecutor,
        prox_executor: ProcessPoolExecutor,
        gui_planner,
        robot_cell_state,
        joint_limits_rad,
        patch_stats: dict,
        n_forward_workers: int = DEFAULT_FORWARD_WORKERS,
        n_prox_workers: int = DEFAULT_PROX_WORKERS,
    ):
        self.root = root
        self.fwd_executor = fwd_executor
        self.prox_executor = prox_executor
        self.gui_planner = gui_planner
        self.robot_cell_state = robot_cell_state
        self.cfg = robot_cell_state.robot_configuration.copy()
        self.joint_limits = joint_limits_rad
        self.patch_stats = patch_stats
        # Deterministic partitions derived from worker counts
        self.n_forward_workers = n_forward_workers
        self.n_prox_workers = n_prox_workers
        self.forward_chunks = _partition(
            range(1, N_FORWARD_STEPS + 1), n_forward_workers
        )
        self.prox_axis_chunks = _partition(range(6), n_prox_workers)

        # State
        self.pos_rad = [math.radians(d) for d in INITIAL_POS_DEG]
        self.vel_rad = [0.0] * 6  # current actual velocity
        self.v_des_rad = [0.0] * 6
        self.v_cmd_rad = [0.0] * 6  # after accel + max_vel clamp, before safety
        self.v_after_path_rad = [0.0] * 6
        self.v_out_rad = [0.0] * 6
        self.current_in_coll: bool | None = None

        # Worker results.
        #   Forward: written fresh every tick by a synchronous dispatch.
        #   Proximity: written by an asynchronous dispatch -- the most
        #     recent completed result is reused while a newer one is in
        #     flight. Proximity staleness is measured (`prox_age_s`) and
        #     logged. This is safe because proximity is a *soft* slow-down,
        #     never a hard safety gate; the forward check is the gate.
        self.prox_results: list[list[bool]] = [
            [False] * len(PROBE_OFFSETS_DEG) for _ in range(6)
        ]
        self.fwd_result: list[bool] = [False] * N_FORWARD_STEPS
        self.fwd_step_deg_used: float = FORWARD_STEP_DEG  # spacing the worker used
        # Async proximity state
        self.prox_future = None
        self.prox_in_flight_t: float = 0.0  # perf_counter() when dispatched
        self.prox_last_harvest_t: float = time.perf_counter()
        self.prox_age_s: float = 0.0  # age of the data we just used for clamps
        self.prox_pipeline_ms: float = 0.0  # last measured dispatch->harvest wall time

        # Last computed clamp diagnostics (for the readout panel)
        self.prox_nearest_deg: float | None = None
        self.path_nearest_deg: float | None = None
        self.last_path_scalar: float = 1.0
        self.last_prox_scalar: float = 1.0

        # Held keys
        self.pressed: set[str] = set()

        # Timing
        self.last_tick_t = time.perf_counter()
        self.last_tick_dt = 0.0
        self.fps_ema = 0.0
        self.fps_alpha = 0.1
        self.target_fps_var = tk.DoubleVar(value=DEFAULT_FPS)

        # Tunables
        self.max_vel_vars = [tk.DoubleVar(value=v) for v in DEFAULT_MAX_VEL_DPS]
        self.max_accel_vars = [tk.DoubleVar(value=a) for a in DEFAULT_MAX_ACCEL_DPS2]
        self.slow_var = tk.DoubleVar(value=DEFAULT_SLOW_DPS)
        self.fast_var = tk.DoubleVar(value=DEFAULT_FAST_DPS)
        self.prox_floor_var = tk.DoubleVar(value=DEFAULT_PROX_FLOOR_PCT)
        self.path_cutoff_var = tk.DoubleVar(value=DEFAULT_PATH_CUTOFF_DEG)
        self.path_shape_var = tk.StringVar(value="linear")  # or "exponential"
        self.exp_k_var = tk.DoubleVar(value=3.0)  # exp clamp steepness

        # Session log
        self.session_log_f = None
        self.session_log_path: str | None = None
        self.session_t0 = time.perf_counter()
        self.tick_n = 0

        self._build_ui()
        self._bind_keys()
        self._open_session_log()

        # Apply initial pose to GUI
        self._push_pose_to_gui()

        # Start ticking
        self.root.after(0, self._tick)

    # ------------------------------------------------------------------ UI

    def _build_ui(self) -> None:
        self.root.title("UR10e keyboard explorer  (touch lists)")
        self.root.geometry("1400x720")

        # --- top status row ---
        top = ttk.Frame(self.root, padding=(10, 8))
        top.pack(side=tk.TOP, fill=tk.X)
        ttk.Label(top, text="Current pose:", font=("Segoe UI", 11)).pack(side=tk.LEFT)
        self.status_label = tk.Label(
            top,
            text="(checking...)",
            font=("Segoe UI", 11, "bold"),
            fg="white",
            bg=COLOR_MARKER_UNKNOWN,
            width=12,
            anchor="center",
        )
        self.status_label.pack(side=tk.LEFT, padx=(8, 16))
        self.fps_label = tk.Label(
            top,
            text="FPS  --/-- ",
            font=("Consolas", 10),
            fg="black",
        )
        self.fps_label.pack(side=tk.LEFT, padx=(0, 16))
        self.clamp_label = ttk.Label(
            top, text="path=1.00  prox=1.00", font=("Consolas", 9)
        )
        self.clamp_label.pack(side=tk.LEFT)

        # --- touch-list banner ---
        banner = ttk.Frame(self.root, padding=(10, 0))
        banner.pack(side=tk.TOP, fill=tk.X)
        ttk.Label(
            banner,
            text=(
                "Touch lists ON  ({nb} bodies + {nt} tools, {tl} link-skips + {tb} body-skips)   "
                "Keys: 1..6 fast+   qwerty slow+   asdfgh slow-   zxcvbn fast-".format(
                    nb=self.patch_stats["n_bodies_patched"],
                    nt=self.patch_stats["n_tools_patched"],
                    tl=self.patch_stats["total_touch_links"],
                    tb=self.patch_stats["total_touch_bodies"],
                )
            ),
            font=("Segoe UI", 9),
            foreground="#2a6f3a",
        ).pack(side=tk.LEFT)

        # --- per-axis rows ---
        body = ttk.Frame(self.root, padding=(10, 6))
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=False)

        # column header
        hdr = ttk.Frame(body)
        hdr.pack(side=tk.TOP, fill=tk.X)
        ttk.Label(hdr, text="", width=22).pack(side=tk.LEFT)
        ttk.Label(hdr, text="value", width=10, font=("Consolas", 8)).pack(side=tk.LEFT)
        ttk.Label(
            hdr,
            text="proximity  (+/-{} deg around current pose)".format(PROBE_HALF_DEG),
            width=int(PROX_BAR_W / 7),
            font=("Consolas", 8),
        ).pack(side=tk.LEFT)
        ttk.Label(
            hdr, text="velocity (deg/s)", width=int(VEL_BAR_W / 7), font=("Consolas", 8)
        ).pack(side=tk.LEFT)

        self.value_labels: list[ttk.Label] = []
        self.prox_canvases: list[tk.Canvas] = []
        self.vel_canvases: list[tk.Canvas] = []
        for i, name in enumerate(JOINT_NAMES):
            row = ttk.Frame(body)
            row.pack(side=tk.TOP, fill=tk.X, pady=2)
            ttk.Label(
                row, text="J{}: {}".format(i, name), width=22, font=("Consolas", 9)
            ).pack(side=tk.LEFT)
            lbl = ttk.Label(
                row, text="+0.0", width=10, font=("Consolas", 9), anchor="e"
            )
            lbl.pack(side=tk.LEFT, padx=(0, 8))
            self.value_labels.append(lbl)
            c1 = tk.Canvas(
                row,
                width=PROX_BAR_W,
                height=PROX_BAR_H,
                bg=COLOR_BG,
                highlightthickness=1,
                highlightbackground="#888888",
            )
            c1.pack(side=tk.LEFT, padx=(0, 6))
            self.prox_canvases.append(c1)
            c3 = tk.Canvas(
                row,
                width=VEL_BAR_W,
                height=VEL_BAR_H,
                bg="#eeeeee",
                highlightthickness=1,
                highlightbackground="#888888",
            )
            c3.pack(side=tk.LEFT)
            self.vel_canvases.append(c3)

        # --- single forward-trajectory bar (the path is one 6D motion) ---
        fwd_frame = ttk.Frame(self.root, padding=(10, 4))
        fwd_frame.pack(side=tk.TOP, fill=tk.X)
        ttk.Label(
            fwd_frame,
            text="Forward path  ({} steps x {:.1f} deg in joint-space, total {:.0f} deg):".format(
                N_FORWARD_STEPS, FORWARD_STEP_DEG, FORWARD_HORIZON_DEG
            ),
            font=("Segoe UI", 9),
        ).pack(side=tk.LEFT)
        self.fwd_canvas = tk.Canvas(
            fwd_frame,
            width=FWD_BAR_W,
            height=FWD_BAR_H,
            bg=COLOR_BG,
            highlightthickness=1,
            highlightbackground="#888888",
        )
        self.fwd_canvas.pack(side=tk.LEFT, padx=(8, 0))

        # --- clamp diagnostics panel: 3 horizontal bars ---
        diag = ttk.LabelFrame(self.root, text="Clamp diagnostics", padding=(10, 6))
        diag.pack(side=tk.TOP, fill=tk.X, padx=10, pady=(4, 0))
        self.diag_bars: dict[str, tuple[tk.Canvas, ttk.Label, str]] = {}
        bar_specs = [
            ("path", "Path clamp", COLOR_AFTER_PATH),
            ("prox", "Proximity clamp", COLOR_VEL_FILL),
            ("speed", "Speed (% of max)", COLOR_FREE),
        ]
        for key, label, color in bar_specs:
            row = ttk.Frame(diag)
            row.pack(side=tk.TOP, fill=tk.X, pady=1)
            ttk.Label(row, text=label, width=18, font=("Consolas", 9)).pack(
                side=tk.LEFT
            )
            c = tk.Canvas(
                row,
                width=480,
                height=16,
                bg="#eeeeee",
                highlightthickness=1,
                highlightbackground="#888888",
            )
            c.pack(side=tk.LEFT, padx=(4, 4))
            vlbl = ttk.Label(row, text="--", width=10, font=("Consolas", 9), anchor="w")
            vlbl.pack(side=tk.LEFT)
            self.diag_bars[key] = (c, vlbl, color)
        self.diag_detail = ttk.Label(
            diag, text="", font=("Consolas", 8), foreground="#444444"
        )
        self.diag_detail.pack(side=tk.TOP, anchor="w", pady=(2, 0))

        # --- controls ---
        ctl = ttk.LabelFrame(self.root, text="Controls", padding=(10, 6))
        ctl.pack(side=tk.TOP, fill=tk.X, padx=10, pady=(8, 4))

        self._add_slider(ctl, "Target FPS", self.target_fps_var, 5, 120, 0)
        self._add_slider(ctl, "Slow key (deg/s)", self.slow_var, 1, 60, 1)
        self._add_slider(ctl, "Fast key (deg/s)", self.fast_var, 1, 90, 2)
        self._add_slider(ctl, "Prox floor (%)", self.prox_floor_var, 0, 100, 3)
        self._add_slider(ctl, "Path cutoff (deg)", self.path_cutoff_var, 0.0, 10.0, 4)

        # path shape selector + exp k
        sel = ttk.Frame(ctl)
        sel.grid(row=5, column=0, columnspan=4, sticky="w", pady=(4, 0))
        ttk.Label(sel, text="Path clamp shape:").pack(side=tk.LEFT)
        ttk.Radiobutton(
            sel, text="linear", variable=self.path_shape_var, value="linear"
        ).pack(side=tk.LEFT, padx=(6, 2))
        ttk.Radiobutton(
            sel, text="exponential", variable=self.path_shape_var, value="exponential"
        ).pack(side=tk.LEFT, padx=(2, 12))
        ttk.Label(sel, text="exp k:").pack(side=tk.LEFT)
        ttk.Scale(
            sel,
            from_=0.5,
            to=10.0,
            orient=tk.HORIZONTAL,
            variable=self.exp_k_var,
            length=140,
        ).pack(side=tk.LEFT, padx=(4, 12))

        ttk.Button(sel, text="Reset pose", command=self._reset_pose).pack(
            side=tk.LEFT, padx=(8, 0)
        )
        ttk.Button(sel, text="Stop (zero vel)", command=self._stop_now).pack(
            side=tk.LEFT, padx=(8, 0)
        )
        ttk.Label(
            sel,
            text="Click anywhere in window to give it keyboard focus.",
            font=("Segoe UI", 8),
            foreground="#666666",
        ).pack(side=tk.LEFT, padx=(16, 0))

        # --- per-axis limits (vel + accel) ---
        peraxis = ttk.LabelFrame(self.root, text="Per-axis limits", padding=(10, 6))
        peraxis.pack(side=tk.TOP, fill=tk.X, padx=10, pady=(4, 4))
        ttk.Label(peraxis, text="Max vel (deg/s)", width=18, font=("Consolas", 9)).grid(
            row=0, column=0, sticky="w"
        )
        for i in range(6):
            tk.Spinbox(
                peraxis,
                from_=1.0,
                to=180.0,
                increment=1.0,
                textvariable=self.max_vel_vars[i],
                width=6,
                font=("Consolas", 9),
            ).grid(row=0, column=1 + i, padx=4)
            ttk.Label(peraxis, text="J{}".format(i), font=("Consolas", 8)).grid(
                row=1, column=1 + i
            )
        ttk.Label(
            peraxis, text="Max accel (deg/s^2)", width=18, font=("Consolas", 9)
        ).grid(row=2, column=0, sticky="w", pady=(4, 0))
        for i in range(6):
            tk.Spinbox(
                peraxis,
                from_=1.0,
                to=500.0,
                increment=5.0,
                textvariable=self.max_accel_vars[i],
                width=6,
                font=("Consolas", 9),
            ).grid(row=2, column=1 + i, padx=4, pady=(4, 0))

    def _add_slider(self, parent, label, var, lo, hi, row):
        ttk.Label(parent, text=label, width=18).grid(row=row, column=0, sticky="w")
        ttk.Scale(
            parent, from_=lo, to=hi, orient=tk.HORIZONTAL, variable=var, length=260
        ).grid(row=row, column=1, sticky="w", padx=(4, 4))
        val_lbl = ttk.Label(parent, width=8, font=("Consolas", 9), anchor="e")
        val_lbl.grid(row=row, column=2, sticky="w")

        def upd(*_):
            val_lbl.config(text="{:.1f}".format(var.get()))

        var.trace_add("write", upd)
        upd()

    # ------------------------------------------------------------- keyboard

    def _open_session_log(self) -> None:
        """Open a JSON-Lines log file for this session and write the header line.

        The header captures all constants that the replay tool needs to
        reconstruct each tick. Subsequent lines are per-tick rows written
        from `_write_log_tick`.
        """
        try:
            os.makedirs(LOG_DIR, exist_ok=True)
            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            path = os.path.join(LOG_DIR, "session_{}.jsonl".format(ts))
            f = open(path, "w", encoding="utf-8", buffering=1)  # line-buffered
            header = {
                "header": True,
                "version": 2,
                "started_at": datetime.datetime.now().isoformat(timespec="seconds"),
                "joint_names": list(JOINT_NAMES),
                "initial_pos_deg": list(INITIAL_POS_DEG),
                "joint_limits_deg": [
                    [math.degrees(lo), math.degrees(hi)] for lo, hi in self.joint_limits
                ],
                "n_forward_steps": N_FORWARD_STEPS,
                "forward_step_deg": FORWARD_STEP_DEG,
                "probe_half_deg": PROBE_HALF_DEG,
                "probe_offsets_deg": list(PROBE_OFFSETS_DEG),
                "workers": {
                    "forward": self.n_forward_workers,
                    "prox": self.n_prox_workers,
                    "forward_chunks": [list(c) for c in self.forward_chunks],
                    "prox_axis_chunks": [list(c) for c in self.prox_axis_chunks],
                    "prox_mode": "async",
                },
                "keys": {
                    "fast_pos": list(FAST_POS_KEYS),
                    "slow_pos": list(SLOW_POS_KEYS),
                    "slow_neg": list(SLOW_NEG_KEYS),
                    "fast_neg": list(FAST_NEG_KEYS),
                },
                "defaults": {
                    "max_vel_dps": list(DEFAULT_MAX_VEL_DPS),
                    "max_accel_dps2": list(DEFAULT_MAX_ACCEL_DPS2),
                    "slow_dps": DEFAULT_SLOW_DPS,
                    "fast_dps": DEFAULT_FAST_DPS,
                    "prox_floor_pct": DEFAULT_PROX_FLOOR_PCT,
                    "path_cutoff_deg": DEFAULT_PATH_CUTOFF_DEG,
                    "target_fps": DEFAULT_FPS,
                },
            }
            f.write(json.dumps(header) + "\n")
            self.session_log_f = f
            self.session_log_path = path
            print("Logging session ticks to:", path)
        except Exception as exc:
            print("Could not open session log:", exc, file=sys.stderr)
            self.session_log_f = None

    def _write_log_tick(self, final_scalar: float) -> None:
        if self.session_log_f is None:
            return
        try:
            row = {
                "n": self.tick_n,
                "t": round(time.perf_counter() - self.session_t0, 6),
                "dt": round(self.last_tick_dt, 6),
                "keys": sorted(self.pressed),
                "v_des": [round(math.degrees(v), 4) for v in self.v_des_rad],
                "v_cmd": [round(math.degrees(v), 4) for v in self.v_cmd_rad],
                "v_out": [round(math.degrees(v), 4) for v in self.v_out_rad],
                "pos": [round(math.degrees(v), 4) for v in self.pos_rad],
                "vel": [round(math.degrees(v), 4) for v in self.vel_rad],
                "in_coll": (
                    None if self.current_in_coll is None else bool(self.current_in_coll)
                ),
                "ps": round(self.last_path_scalar, 6),
                "qs": round(self.last_prox_scalar, 6),
                "fs": round(final_scalar, 6),
                "p_near": (
                    None
                    if self.path_nearest_deg is None
                    else round(self.path_nearest_deg, 3)
                ),
                "q_near": (
                    None
                    if self.prox_nearest_deg is None
                    else round(self.prox_nearest_deg, 3)
                ),
                "fwd": _pack_bits(self.fwd_result),
                "prox": [_pack_bits(r) for r in self.prox_results],
                "prox_age_ms": round(self.prox_age_s * 1000.0, 2),
                "prox_pipe_ms": round(self.prox_pipeline_ms, 2),
                "cfg": {
                    "mv": [round(v.get(), 3) for v in self.max_vel_vars],
                    "ma": [round(v.get(), 3) for v in self.max_accel_vars],
                    "slow": round(self.slow_var.get(), 3),
                    "fast": round(self.fast_var.get(), 3),
                    "pf": round(self.prox_floor_var.get(), 3),
                    "pc": round(self.path_cutoff_var.get(), 3),
                    "sh": self.path_shape_var.get(),
                    "ek": round(self.exp_k_var.get(), 3),
                    "fps": round(self.target_fps_var.get(), 1),
                },
            }
            self.session_log_f.write(json.dumps(row) + "\n")
        except Exception as exc:
            print("Log write error:", exc, file=sys.stderr)

    def close_log(self) -> None:
        if self.session_log_f is not None:
            try:
                self.session_log_f.close()
            except Exception:
                pass
            self.session_log_f = None
            if self.session_log_path:
                print("Session log closed:", self.session_log_path)

    # ------------------------------------------------------------- keyboard

    def _bind_keys(self) -> None:
        all_keys = FAST_POS_KEYS + SLOW_POS_KEYS + SLOW_NEG_KEYS + FAST_NEG_KEYS
        for k in all_keys:
            self.root.bind(
                "<KeyPress-{}>".format(k), lambda e, key=k: self._on_press(key)
            )
            self.root.bind(
                "<KeyRelease-{}>".format(k), lambda e, key=k: self._on_release(key)
            )
        self.root.focus_force()

    def _on_press(self, key: str) -> None:
        if self._focus_is_text_entry():
            return
        self.pressed.add(key)

    def _on_release(self, key: str) -> None:
        if self._focus_is_text_entry():
            # Still discard so we don't get a stuck key from a release that
            # happened after focus moved away.
            self.pressed.discard(key)
            return
        self.pressed.discard(key)

    def _focus_is_text_entry(self) -> bool:
        """True if the focused widget should consume jog keys (Spinbox/Entry/Text)."""
        try:
            w = self.root.focus_get()
        except Exception:
            return False
        if w is None:
            return False
        cls = w.winfo_class()
        return cls in ("TEntry", "Entry", "Spinbox", "TSpinbox", "Text")

    def _desired_velocity_dps(self) -> list[float]:
        """Algebraic sum of held keys per axis, in deg/s."""
        slow = self.slow_var.get()
        fast = self.fast_var.get()
        out = [0.0] * 6
        for i in range(6):
            if FAST_POS_KEYS[i] in self.pressed:
                out[i] += fast
            if SLOW_POS_KEYS[i] in self.pressed:
                out[i] += slow
            if SLOW_NEG_KEYS[i] in self.pressed:
                out[i] -= slow
            if FAST_NEG_KEYS[i] in self.pressed:
                out[i] -= fast
        return out

    # ------------------------------------------------------------ buttons

    def _reset_pose(self) -> None:
        self.pos_rad = [math.radians(d) for d in INITIAL_POS_DEG]
        self.vel_rad = [0.0] * 6
        self._push_pose_to_gui()

    def _stop_now(self) -> None:
        self.vel_rad = [0.0] * 6

    # ------------------------------------------------------------- motion

    def _push_pose_to_gui(self) -> None:
        clamped = [
            max(lo, min(hi, v)) for v, (lo, hi) in zip(self.pos_rad, self.joint_limits)
        ]
        self.cfg.joint_values = clamped
        self.robot_cell_state.robot_configuration = self.cfg
        try:
            self.gui_planner.check_collision(
                self.robot_cell_state, options={"verbose": False}
            )
            self.current_in_coll = False
        except CollisionCheckError:
            self.current_in_coll = True

    def _accel_clamp(
        self, v_cur: float, v_des: float, max_accel: float, dt: float
    ) -> float:
        dv_max = max_accel * dt
        return v_cur + max(-dv_max, min(dv_max, v_des - v_cur))

    def _compute_path_scalar(self) -> float:
        """Find earliest collision step and convert distance-to-collision to scale.

        Spacing is FIXED in joint-space (self.fwd_step_deg_used per step), so
        the scalar is proportional to actual distance regardless of current
        speed. dist_deg = step_index * step_deg.

        Hard cutoff: if the nearest collision is closer than `path_cutoff_var`
        degrees, the scalar is forced to 0 to stop the motion before crashing.
        """
        self.path_nearest_deg = None
        for k, hit in enumerate(self.fwd_result):  # k = 0..N-1, step (k+1)
            if hit:
                dist_deg = (k + 1) * self.fwd_step_deg_used
                self.path_nearest_deg = dist_deg
                # Hard cutoff to zero if too close
                cutoff = max(0.0, self.path_cutoff_var.get())
                if dist_deg <= cutoff:
                    return 0.0
                max_dist = N_FORWARD_STEPS * self.fwd_step_deg_used
                # Re-map [cutoff, max_dist] -> [0, 1] so the curve is
                # continuous: at distance == cutoff the scale is 0, at
                # distance == max_dist the scale is 1.
                norm = (dist_deg - cutoff) / max(1e-6, (max_dist - cutoff))
                norm = max(0.0, min(1.0, norm))
                shape = self.path_shape_var.get()
                if shape == "linear":
                    scale = norm
                else:
                    k_steep = max(0.1, self.exp_k_var.get())
                    scale = 1.0 - math.exp(-k_steep * norm)
                    scale = max(0.0, min(1.0, scale))
                return scale
        return 1.0

    def _compute_prox_scalar(self, v_cmd: list[float]) -> float:
        """Global scalar from the nearest obstacle across ALL axes, BOTH directions.

        Independent of motion direction and user input -- this is a pure
        'how close are we to anything?' scaler that's always active. If no
        obstacle exists in the +/-PROBE_HALF_DEG window of any axis, scalar = 1.
        Otherwise the scalar drops linearly from 1 (at PROBE_HALF_DEG away) to
        `floor` (at 1 deg away).
        """
        floor = max(0.0, min(1.0, self.prox_floor_var.get() / 100.0))
        nearest_deg: float | None = None
        for axis in range(6):
            results = self.prox_results[axis]
            # Scan negative direction (offsets -1, -2, ..., -PROBE_HALF)
            for j in range(PROBE_HALF_DEG):
                if results[PROBE_HALF_DEG - 1 - j]:
                    d = j + 1
                    if nearest_deg is None or d < nearest_deg:
                        nearest_deg = d
                    break
            # Scan positive direction (offsets +1, +2, ..., +PROBE_HALF)
            for j in range(PROBE_HALF_DEG):
                if results[PROBE_HALF_DEG + j]:
                    d = j + 1
                    if nearest_deg is None or d < nearest_deg:
                        nearest_deg = d
                    break
        self.prox_nearest_deg = nearest_deg
        if nearest_deg is None:
            return 1.0
        if PROBE_HALF_DEG <= 1:
            return floor
        frac = (nearest_deg - 1) / (PROBE_HALF_DEG - 1)
        frac = max(0.0, min(1.0, frac))
        return floor + (1.0 - floor) * frac

    # --------------------------------------------------------------- tick

    def _run_forward_check(self, base_rad: tuple, step_vec: tuple) -> list[bool]:
        """Synchronous, deterministic forward-trajectory check.

        Dispatches exactly `n_forward_workers` chunks (one per worker), waits
        for ALL of them, then reassembles into a single ordered bool list of
        length N_FORWARD_STEPS. Each chunk has a fixed pre-defined set of
        step indices; the worker pool is not free to load-balance.
        """
        futures = [
            self.fwd_executor.submit(
                _proc_forward_chunk, (base_rad, step_vec, chunk)
            )
            for chunk in self.forward_chunks
        ]
        results = [f.result() for f in futures]  # blocks until each completes
        out = [False] * N_FORWARD_STEPS
        for chunk, chunk_result in zip(self.forward_chunks, results):
            for k, hit in zip(chunk, chunk_result):
                out[k - 1] = hit
        return out

    def _dispatch_proximity_async(self, base_rad: tuple) -> None:
        """Submit one proximity batch (n_prox_workers tasks) and return.

        Does NOT block. If a previous batch is still in flight we keep
        waiting for it and skip this dispatch -- never queue, never let the
        pool back up.
        """
        if self.prox_future is not None:
            return
        offs = tuple(PROBE_OFFSETS_RAD)
        futures = [
            self.prox_executor.submit(
                _proc_proximity_chunk, (base_rad, axes, offs)
            )
            for axes in self.prox_axis_chunks
        ]
        self.prox_future = futures
        self.prox_in_flight_t = time.perf_counter()

    def _harvest_proximity_nonblocking(self) -> bool:
        """If the latest proximity batch is done, harvest it. Return True iff
        we updated `self.prox_results`.
        """
        if self.prox_future is None:
            return False
        if not all(f.done() for f in self.prox_future):
            return False
        try:
            new_results: list[list[bool]] = [
                [False] * len(PROBE_OFFSETS_DEG) for _ in range(6)
            ]
            for f in self.prox_future:
                per_axis = f.result()
                for axis, bits in per_axis.items():
                    new_results[axis] = bits
            self.prox_results = new_results
            now = time.perf_counter()
            self.prox_pipeline_ms = (now - self.prox_in_flight_t) * 1000.0
            self.prox_last_harvest_t = now
        except Exception as exc:
            print("Proximity worker error:", exc, file=sys.stderr)
            # Keep prior prox_results (favours "slow down" over "speed up")
        finally:
            self.prox_future = None
        return True

    def _tick(self) -> None:
        t_now = time.perf_counter()
        dt = t_now - self.last_tick_t
        self.last_tick_t = t_now
        if dt <= 0:
            dt = 1e-3
        self.last_tick_dt = dt

        # FPS measurement (EMA on instantaneous 1/dt)
        inst_fps = 1.0 / dt
        if self.fps_ema == 0.0:
            self.fps_ema = inst_fps
        else:
            self.fps_ema = (
                1 - self.fps_alpha
            ) * self.fps_ema + self.fps_alpha * inst_fps

        # 0. Harvest any completed proximity batch BEFORE computing clamps,
        #    so we use the freshest available data.
        self._harvest_proximity_nonblocking()
        # Age of the proximity data we're about to use (s since harvest)
        self.prox_age_s = t_now - self.prox_last_harvest_t

        # 1. Desired velocity from keys
        v_des_dps = self._desired_velocity_dps()
        self.v_des_rad = [math.radians(v) for v in v_des_dps]

        # 2. Accel-clamp current velocity toward desired (per-axis max accel)
        new_v = []
        for i in range(6):
            max_accel_i = math.radians(self.max_accel_vars[i].get())
            new_v.append(
                self._accel_clamp(self.vel_rad[i], self.v_des_rad[i], max_accel_i, dt)
            )
        # 3. Max-velocity clamp (per-axis; symmetric)
        for i in range(6):
            max_vel_i = math.radians(self.max_vel_vars[i].get())
            new_v[i] = max(-max_vel_i, min(max_vel_i, new_v[i]))
        self.v_cmd_rad = list(new_v)

        # 4. SYNCHRONOUS forward path check for the EXACT v_cmd direction.
        #    This is the safety gate. We wait for ALL forward chunks.
        base = tuple(self.pos_rad)
        v_norm = math.sqrt(sum(v * v for v in self.v_cmd_rad))
        if v_norm > math.radians(0.5):  # > 0.5 deg/s combined
            step_rad = math.radians(FORWARD_STEP_DEG)
            step_vec = tuple((v / v_norm) * step_rad for v in self.v_cmd_rad)
            self.fwd_step_deg_used = FORWARD_STEP_DEG
            self.fwd_result = self._run_forward_check(base, step_vec)
        else:
            self.fwd_result = [False] * N_FORWARD_STEPS

        # 5. ASYNCHRONOUS proximity dispatch (soft slow-down, not safety).
        #    Fire-and-forget: harvested at the start of a future tick. We
        #    skip if a previous batch is still pending (no queueing).
        self._dispatch_proximity_async(base)

        # 6. Compute clamps from the FRESH forward + possibly-stale prox.
        path_scalar = self._compute_path_scalar()
        prox_scalar = self._compute_prox_scalar(self.v_cmd_rad)
        self.last_path_scalar = path_scalar
        self.last_prox_scalar = prox_scalar
        self.v_after_path_rad = [v * path_scalar for v in self.v_cmd_rad]
        final_scalar = min(path_scalar, prox_scalar)
        self.v_out_rad = [v * final_scalar for v in self.v_cmd_rad]

        # 7. Integrate -> new actual velocity for next tick is v_out
        self.vel_rad = list(self.v_out_rad)
        self.pos_rad = [self.pos_rad[i] + self.vel_rad[i] * dt for i in range(6)]
        # Clamp to URDF joint limits (and zero the velocity if hitting a wall)
        for i, (lo, hi) in enumerate(self.joint_limits):
            if self.pos_rad[i] < lo:
                self.pos_rad[i] = lo
                if self.vel_rad[i] < 0:
                    self.vel_rad[i] = 0.0
            elif self.pos_rad[i] > hi:
                self.pos_rad[i] = hi
                if self.vel_rad[i] > 0:
                    self.vel_rad[i] = 0.0

        # 8. Push to GUI
        self._push_pose_to_gui()

        # ---- update UI text ----
        self.clamp_label.config(
            text="path={:.2f}  prox={:.2f}  final={:.2f}".format(
                path_scalar, prox_scalar, final_scalar
            )
        )
        target = self.target_fps_var.get()
        fps_text = "FPS {:5.1f}/{:.0f}".format(self.fps_ema, target)
        fps_color = "black" if self.fps_ema >= target * 0.9 else "#a02020"
        self.fps_label.config(text=fps_text, fg=fps_color)
        if self.current_in_coll:
            self.status_label.config(text="COLLISION", bg=COLOR_MARKER_COLL)
        else:
            self.status_label.config(text="FREE", bg=COLOR_MARKER_FREE)
        for i in range(6):
            self.value_labels[i].config(
                text="{:+7.1f}".format(math.degrees(self.pos_rad[i]))
            )
            self._draw_prox(i)
            self._draw_vel(i)
        self._draw_fwd()
        self._write_diag()
        self._write_log_tick(final_scalar)
        self.tick_n += 1

        # 9. Schedule next tick to hit target FPS
        target_dt_ms = int(max(1, 1000.0 / max(1.0, target)))
        # subtract the work we already did to keep wall pacing on target
        spent_ms = (time.perf_counter() - t_now) * 1000.0
        sleep_ms = int(max(1, target_dt_ms - spent_ms))
        self.root.after(sleep_ms, self._tick)

    # ----------------------------------------------------------- drawing

    def _draw_prox(self, idx: int) -> None:
        canvas = self.prox_canvases[idx]
        canvas.delete("all")
        w = PROX_BAR_W
        h = PROX_BAR_H

        def deg_to_x(d: float) -> float:
            frac = (d - SLIDER_MIN_DEG) / (SLIDER_MAX_DEG - SLIDER_MIN_DEG)
            return frac * w

        cur_deg = math.degrees(self.pos_rad[idx])

        # tick marks every 30 deg
        for d in range(-180, 181, 30):
            x = deg_to_x(d)
            canvas.create_line(x, h - 4, x, h, fill="#888888")
        x0 = deg_to_x(0)
        canvas.create_line(x0, 0, x0, h, fill="#aaaaaa", dash=(2, 3))

        # probe cells
        results = self.prox_results[idx]
        for off_deg, result in zip(PROBE_OFFSETS_DEG, results):
            d = cur_deg + off_deg
            xa = deg_to_x(d - 0.5)
            xb = deg_to_x(d + 0.5)
            color = COLOR_COLL if result else COLOR_FREE
            canvas.create_rectangle(xa, 6, xb, h - 6, fill=color, outline="")

        # current-pose cell
        xa = deg_to_x(cur_deg - 0.5)
        xb = deg_to_x(cur_deg + 0.5)
        c_color = (
            COLOR_UNKNOWN
            if self.current_in_coll is None
            else (COLOR_COLL if self.current_in_coll else COLOR_FREE)
        )
        canvas.create_rectangle(xa, 6, xb, h - 6, fill=c_color, outline="black")

        # arrow showing v_out projected 1s (clamped final)
        v_out_dps = math.degrees(self.v_out_rad[idx])
        if abs(v_out_dps) > 0.05:
            xs = deg_to_x(cur_deg)
            xe = deg_to_x(cur_deg + v_out_dps)
            yy = h * 0.5
            canvas.create_line(xs, yy, xe, yy, fill="#1f78ff", width=2, arrow=tk.LAST)

        # desired-pos-after-1s vertical mark (uses pre-clamp v_des)
        v_des_dps = math.degrees(self.v_des_rad[idx])
        if abs(v_des_dps) > 0.05:
            xd = deg_to_x(cur_deg + v_des_dps)
            canvas.create_line(xd, 2, xd, h - 2, fill="#ff8000", width=2)

        # triangle marker on top
        cx = deg_to_x(cur_deg)
        mc = (
            COLOR_MARKER_UNKNOWN
            if self.current_in_coll is None
            else (COLOR_MARKER_COLL if self.current_in_coll else COLOR_MARKER_FREE)
        )
        canvas.create_polygon(cx - 5, -1, cx + 5, -1, cx, 7, fill=mc, outline="black")

    def _draw_fwd(self) -> None:
        """Forward-trajectory bar (single, global; not per-axis).

        Leftmost cell = step 1 (closest to current); rightmost = step N.
        Each cell represents FORWARD_STEP_DEG of joint-space distance along
        the unit direction of the current commanded velocity vector.
        """
        canvas = self.fwd_canvas
        canvas.delete("all")
        w = FWD_BAR_W
        h = FWD_BAR_H
        v_norm = math.sqrt(sum(v * v for v in self.v_cmd_rad))
        idle = v_norm <= math.radians(0.5)
        n = N_FORWARD_STEPS
        cw = w / n
        for k in range(n):
            xa = k * cw
            xb = (k + 1) * cw
            if idle:
                color = COLOR_UNKNOWN
            else:
                color = COLOR_COLL if self.fwd_result[k] else COLOR_FREE
            canvas.create_rectangle(xa, 6, xb, h - 14, fill=color, outline="")
            # step distance tick label every 5 steps
            if (k + 1) % 5 == 0:
                xc = (k + 1) * cw
                canvas.create_line(xc, h - 14, xc, h - 8, fill="#444444")
                canvas.create_text(
                    xc,
                    h - 7,
                    anchor="n",
                    text="{:.0f}".format((k + 1) * FORWARD_STEP_DEG),
                    font=("Consolas", 7),
                    fill="#444444",
                )
        canvas.create_text(
            2, h - 7, anchor="nw", text="deg", font=("Consolas", 7), fill="#444444"
        )
        # Mark first collision with a vertical line
        if not idle:
            for k, hit in enumerate(self.fwd_result):
                if hit:
                    x = (k + 0.5) * cw
                    canvas.create_line(x, 0, x, h - 6, fill="black", width=1)
                    break

    def _write_diag(self) -> None:
        """Update the three horizontal diagnostic bars + one detail line."""
        v_out_norm = math.sqrt(sum(v * v for v in self.v_out_rad))
        max_vel_rad = [math.radians(v.get()) for v in self.max_vel_vars]
        max_vel_norm = math.sqrt(sum(v * v for v in max_vel_rad))
        speed_frac = (v_out_norm / max_vel_norm) if max_vel_norm > 1e-9 else 0.0
        values = {
            "path": self.last_path_scalar,
            "prox": self.last_prox_scalar,
            "speed": min(1.0, max(0.0, speed_frac)),
        }
        for key, (canvas, vlbl, color) in self.diag_bars.items():
            canvas.delete("all")
            w = int(canvas["width"])
            h = int(canvas["height"])
            frac = max(0.0, min(1.0, values[key]))
            canvas.create_rectangle(0, 0, int(w * frac), h, fill=color, outline="")
            for f in (0.25, 0.5, 0.75):
                x = int(w * f)
                canvas.create_line(x, 0, x, h, fill="#bbbbbb")
            canvas.create_rectangle(0, 0, w - 1, h - 1, outline="#666666")
            vlbl.config(text="{:6.1%}".format(values[key]))
        v_cmd_norm_dps = math.degrees(math.sqrt(sum(v * v for v in self.v_cmd_rad)))
        v_out_norm_dps = math.degrees(v_out_norm)
        prox_d = (
            "--"
            if self.prox_nearest_deg is None
            else "{:.0f} deg".format(self.prox_nearest_deg)
        )
        path_d = (
            "--"
            if self.path_nearest_deg is None
            else "{:.1f} deg".format(self.path_nearest_deg)
        )
        self.diag_detail.config(
            text=(
                "nearest prox = {pd:>8s}   nearest path = {ad:>8s}   "
                "|v_cmd| = {vc:5.1f} dps   |v_out| = {vo:5.1f} dps   shape = {sh}   "
                "prox age = {age:4.0f} ms   prox pipe = {pipe:4.0f} ms".format(
                    pd=prox_d,
                    ad=path_d,
                    vc=v_cmd_norm_dps,
                    vo=v_out_norm_dps,
                    sh=self.path_shape_var.get(),
                    age=self.prox_age_s * 1000.0,
                    pipe=self.prox_pipeline_ms,
                )
            )
        )

    def _draw_vel(self, idx: int) -> None:
        canvas = self.vel_canvases[idx]
        canvas.delete("all")
        w = VEL_BAR_W
        h = VEL_BAR_H

        max_vel = max(0.1, self.max_vel_vars[idx].get())  # deg/s, per-axis

        def dps_to_x(v: float) -> float:
            return w * (v + max_vel) / (2 * max_vel)

        # background reference: max-vel band
        canvas.create_rectangle(0, h - 12, w, h - 2, fill="#cccccc", outline="")
        # zero line
        x0 = dps_to_x(0)
        canvas.create_line(x0, 0, x0, h, fill="#888888")
        # range ticks at +/- max_vel
        canvas.create_line(0, h - 12, 0, h - 2, fill="#444444")
        canvas.create_line(w - 1, h - 12, w - 1, h - 2, fill="#444444")

        v_des_dps = math.degrees(self.v_des_rad[idx])
        v_cmd_dps = math.degrees(self.v_cmd_rad[idx])
        v_path_dps = math.degrees(self.v_after_path_rad[idx])
        v_out_dps = math.degrees(self.v_out_rad[idx])

        # Filled bar from 0 -> v_out (final output)
        xa = min(x0, dps_to_x(v_out_dps))
        xb = max(x0, dps_to_x(v_out_dps))
        canvas.create_rectangle(xa, 6, xb, h - 14, fill=COLOR_VEL_FILL, outline="")

        # markers
        # desired (clipped visually so off-scale still shows at edge)
        def mark(v, color, label_off=0):
            xv = dps_to_x(max(-max_vel * 1.1, min(max_vel * 1.1, v)))
            canvas.create_line(xv, 2, xv, h - 14, fill=color, width=2)

        mark(v_des_dps, COLOR_DESIRED)
        mark(v_cmd_dps, "#777777")
        mark(v_path_dps, COLOR_AFTER_PATH)
        mark(v_out_dps, COLOR_AFTER_PROX)

        # numeric labels (compact)
        canvas.create_text(
            2,
            1,
            anchor="nw",
            text="d={:+.0f}".format(v_des_dps),
            fill=COLOR_DESIRED,
            font=("Consolas", 7),
        )
        canvas.create_text(
            w - 2,
            1,
            anchor="ne",
            text="o={:+.0f}".format(v_out_dps),
            fill=COLOR_AFTER_PROX,
            font=("Consolas", 7),
        )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def _wait_for_workers(executor: ProcessPoolExecutor, n: int) -> None:
    _ = list(executor.map(_proc_ping, range(n * 3)))


def _parse_args(argv: list[str] | None = None):
    import argparse

    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument(
        "--forward-workers",
        type=int,
        default=DEFAULT_FORWARD_WORKERS,
        help="processes dedicated to the synchronous forward path check "
        "(default {})".format(DEFAULT_FORWARD_WORKERS),
    )
    p.add_argument(
        "--prox-workers",
        type=int,
        default=DEFAULT_PROX_WORKERS,
        help="processes dedicated to the asynchronous proximity probe scan "
        "(default {})".format(DEFAULT_PROX_WORKERS),
    )
    return p.parse_args(argv)


def main() -> None:
    args = _parse_args()
    n_fwd = max(1, min(N_FORWARD_STEPS, args.forward_workers))
    n_prox = max(1, min(6, args.prox_workers))

    if not os.path.exists(DISCOVERY_PATH):
        raise SystemExit(
            "Discovery JSON not found: {}\n  Run bullet_collision_pair_discovery.py first.".format(
                DISCOVERY_PATH
            )
        )

    print("Loading scene + touch lists for GUI client ...")
    robot_cell, robot_cell_state, lower, upper = load_scene(apply_touch=True)
    # Override URDF joint limits with a uniform +/-180 deg range. The URDF
    # gives some joints +/-360 which lets the wrist (J3) drift past +/-180,
    # which is confusing in the bar visualisations.
    joint_limits = [(-math.pi, math.pi)] * 6

    # Capture patch stats from a throwaway state (counting only)
    _, rcs_stats, _, _ = load_scene(apply_touch=False)
    patch_stats = _apply_touch_lists(rcs_stats, _load_discovery())
    print(
        "  touch lists: {nb} bodies, {nt} tools, {tl} link-skips, {tb} body-skips".format(
            nb=patch_stats["n_bodies_patched"],
            nt=patch_stats["n_tools_patched"],
            tl=patch_stats["total_touch_links"],
            tb=patch_stats["total_touch_bodies"],
        )
    )

    print("Starting GUI PyBullet ...")
    gui_client = PyBulletClient(connection_type="gui", verbose=False)
    gui_client.__enter__()
    gui_planner = PyBulletPlanner(gui_client)
    gui_planner.set_robot_cell(robot_cell)
    gui_planner.set_robot_cell_state(robot_cell_state)
    try:
        gui_planner.check_collision(robot_cell_state, options={"verbose": False})
    except CollisionCheckError:
        pass

    print(
        "Spawning workers: {} forward (sync) + {} proximity (async) ...".format(
            n_fwd, n_prox
        )
    )
    fwd_executor = ProcessPoolExecutor(max_workers=n_fwd, initializer=_proc_init)
    prox_executor = ProcessPoolExecutor(max_workers=n_prox, initializer=_proc_init)
    print("Warming up workers ...")
    _wait_for_workers(fwd_executor, n_fwd)
    _wait_for_workers(prox_executor, n_prox)
    print("Workers ready.")
    fwd_chunks = _partition(range(1, N_FORWARD_STEPS + 1), n_fwd)
    prox_chunks = _partition(range(6), n_prox)
    print("  forward chunks   :", fwd_chunks)
    print("  prox axis chunks :", prox_chunks)

    print("Launching UI. Focus the window then press 1/q/a/z etc. to jog.")
    root = tk.Tk()
    app = KeyboardExplorer(
        root,
        fwd_executor,
        prox_executor,
        gui_planner,
        robot_cell_state,
        joint_limits,
        patch_stats,
        n_forward_workers=n_fwd,
        n_prox_workers=n_prox,
    )

    def on_close():
        print("Shutting down ...")
        try:
            app.close_log()
        except Exception:
            pass
        for ex in (fwd_executor, prox_executor):
            try:
                ex.shutdown(wait=False, cancel_futures=True)
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
            app.close_log()
        except Exception:
            pass
        for ex in (fwd_executor, prox_executor):
            try:
                ex.shutdown(wait=False, cancel_futures=True)
            except Exception:
                pass
        try:
            gui_client.__exit__(None, None, None)
        except Exception:
            pass


if __name__ == "__main__":
    main()
