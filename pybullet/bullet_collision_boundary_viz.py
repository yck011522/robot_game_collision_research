"""Joint-0 collision boundary visualization: LINEAR vs EXP_BINARY.

For each cycle both methods run on the SAME pre-sampled configuration.
A fixed-width text bar is printed per method per cycle showing what was
explored and where the collision boundary on joint-0 was found (if any).

Bar legend:
  ~  free (no collision)
  #  collision
  |  boundary (collision state transition)
  @  current joint-0 position
  ?  unexplored region

Usage:
    conda activate game
    python pybullet/bullet_collision_boundary_viz.py
    python pybullet/bullet_collision_boundary_viz.py --n-cycles 25 --step 0.08
"""

from __future__ import annotations

import argparse
import math
import os
import statistics
import sys
import time
from dataclasses import dataclass

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


# ---------------------------------------------------------------------------
# Data types
# ---------------------------------------------------------------------------


@dataclass
class SearchResult:
    found: bool
    boundary_q0: float | None  # q0 where state flips (if found)
    explored_q0: float  # furthest q0 confirmed same-state-as-base
    checks: int  # number of collision checks performed
    tries: int  # number of outward steps attempted
    reason: str  # "state_changed" | "max_tries" | "joint_limit"


# ---------------------------------------------------------------------------
# Scene loading
# ---------------------------------------------------------------------------


def load_scene():
    data = json_load(JSON_PATH)
    robot_cell = data["robot_cell"]
    robot_cell_state = data["robot_cell_state"]
    robot_cell.robot_model.attr.pop("transmission", None)  # URDF export workaround
    joints = {j.name: j for j in robot_cell.robot_model.get_configurable_joints()}
    lower = [
        joints[n].limit.lower if joints[n].limit else -math.pi for n in JOINT_NAMES
    ]
    upper = [joints[n].limit.upper if joints[n].limit else math.pi for n in JOINT_NAMES]
    return robot_cell, robot_cell_state, lower, upper


# ---------------------------------------------------------------------------
# Collision helper
# ---------------------------------------------------------------------------


def check_q(planner, robot_cell_state, cfg, q_values) -> bool:
    """Return True if the given joint config is in collision."""
    cfg.joint_values = list(q_values)
    robot_cell_state.robot_configuration = cfg
    try:
        planner.check_collision(robot_cell_state, options={"verbose": False})
        return False
    except CollisionCheckError:
        return True


# ---------------------------------------------------------------------------
# Search methods
# ---------------------------------------------------------------------------


def search_linear(
    planner,
    robot_cell_state,
    cfg,
    q_base,
    start_collision,
    lower0,
    upper0,
    direction,
    step_size,
    max_tries,
) -> SearchResult:
    """Fixed-step scan outward from q_base[0] in one direction."""
    q0_start = q_base[0]
    last_same = q0_start  # furthest q0 with same collision state as base
    prev_q0 = q0_start
    checks = 0

    for i in range(1, max_tries + 1):
        q0 = q0_start + direction * step_size * i
        q0 = max(lower0, min(upper0, q0))
        if q0 == prev_q0:
            return SearchResult(False, None, last_same, checks, i, "joint_limit")
        q_test = list(q_base)
        q_test[0] = q0
        in_coll = check_q(planner, robot_cell_state, cfg, q_test)
        checks += 1
        if in_coll != start_collision:
            return SearchResult(True, q0, last_same, checks, i, "state_changed")
        last_same = q0
        prev_q0 = q0

    return SearchResult(False, None, last_same, checks, max_tries, "max_tries")


def search_exp_binary(
    planner,
    robot_cell_state,
    cfg,
    q_base,
    start_collision,
    lower0,
    upper0,
    direction,
    step_size,
    max_tries,
    refine_iters,
) -> SearchResult:
    """Exponential bracketing followed by binary refinement."""
    q0_start = q_base[0]
    d_prev = 0.0
    q_prev = q0_start
    step = step_size
    checks = 0
    last_same = q0_start

    found_bracket = False
    d_low = 0.0
    d_high = 0.0

    for i in range(1, max_tries + 1):
        d = d_prev + step
        q0 = q0_start + direction * d
        q0 = max(lower0, min(upper0, q0))
        if q0 == q_prev:
            return SearchResult(False, None, last_same, checks, i, "joint_limit")
        q_test = list(q_base)
        q_test[0] = q0
        in_coll = check_q(planner, robot_cell_state, cfg, q_test)
        checks += 1
        if in_coll != start_collision:
            found_bracket = True
            d_low = d_prev
            d_high = abs(q0 - q0_start)
            break
        last_same = q0
        d_prev = abs(q0 - q0_start)
        q_prev = q0
        step *= 2.0

    if not found_bracket:
        return SearchResult(False, None, last_same, checks, max_tries, "max_tries")

    # Binary refinement within [d_low, d_high]
    exp_tries = i  # capture before refine loop overwrites i
    for _ in range(refine_iters):
        d_mid = 0.5 * (d_low + d_high)
        q0_mid = max(lower0, min(upper0, q0_start + direction * d_mid))
        q_test = list(q_base)
        q_test[0] = q0_mid
        in_coll = check_q(planner, robot_cell_state, cfg, q_test)
        checks += 1
        if in_coll == start_collision:
            d_low = d_mid
            last_same = q0_mid
        else:
            d_high = d_mid

    boundary = max(lower0, min(upper0, q0_start + direction * d_high))
    return SearchResult(True, boundary, last_same, checks, exp_tries, "state_changed")


# ---------------------------------------------------------------------------
# Config generator
# ---------------------------------------------------------------------------


def auto_config(t, lower, upper, speed) -> list[float]:
    return [
        0.5 * (lower[i] + upper[i])
        + 0.45 * (upper[i] - lower[i]) * math.sin(speed * t + i * 0.9)
        for i in range(6)
    ]


# ---------------------------------------------------------------------------
# Bar renderer
# ---------------------------------------------------------------------------


def render_bar(
    lower0,
    upper0,
    q_base0,
    start_collision,
    res_neg,
    res_pos,
    width=42,
) -> str:
    """Render a fixed-width text bar for joint-0 collision state.

    ~  confirmed free zone
    #  confirmed collision zone
    |  boundary (state transition point)
    @  current joint-0 value
    ?  unexplored / unknown
    """

    def to_idx(q):
        frac = (q - lower0) / (upper0 - lower0)
        return max(0, min(width - 1, int(frac * (width - 1))))

    base_ch = "~" if not start_collision else "#"
    opp_ch = "#" if not start_collision else "~"
    bar = ["?"] * width
    cur = to_idx(q_base0)

    # --- negative direction ---
    if res_neg.found and res_neg.boundary_q0 is not None:
        bi = to_idx(res_neg.boundary_q0)
        for i in range(0, bi):
            bar[i] = opp_ch  # opposite state beyond boundary
        bar[bi] = "|"
        for i in range(bi + 1, cur + 1):
            bar[i] = base_ch  # same state between boundary and current
    else:
        ei = to_idx(res_neg.explored_q0)
        for i in range(ei, cur + 1):
            bar[i] = base_ch  # explored same-state region; left of ei = ?

    # --- positive direction ---
    if res_pos.found and res_pos.boundary_q0 is not None:
        bi = to_idx(res_pos.boundary_q0)
        for i in range(cur, bi):
            bar[i] = base_ch  # same state between current and boundary
        bar[bi] = "|"
        for i in range(bi + 1, width):
            bar[i] = opp_ch  # opposite state beyond boundary
    else:
        ei = to_idx(res_pos.explored_q0)
        for i in range(cur, ei + 1):
            bar[i] = base_ch  # explored same-state region; right of ei = ?

    bar[cur] = "@"
    return "".join(bar)


# ---------------------------------------------------------------------------
# Formatting helpers
# ---------------------------------------------------------------------------


def fmt_bnd(res) -> str:
    """6-char boundary label: '+x.xx' or ' none'."""
    if res.found and res.boundary_q0 is not None:
        return "{:+.2f}".format(res.boundary_q0)
    return " none"


def found_flags(res_neg, res_pos) -> str:
    """2-char found indicator: first char = positive dir, second = negative."""
    return ("+" if res_pos.found else ".") + ("+" if res_neg.found else ".")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(
        description="Joint-0 boundary visualization: LINEAR vs EXP_BINARY"
    )
    parser.add_argument(
        "--n-cycles", type=int, default=25, help="Number of cycles to run"
    )
    parser.add_argument(
        "--dt", type=float, default=0.50, help="Time step between configs (s)"
    )
    parser.add_argument(
        "--speed", type=float, default=0.6, help="Auto motion speed (rad/s)"
    )
    parser.add_argument(
        "--bar-width", type=int, default=60, help="Width of the bar in chars"
    )
    parser.add_argument(
        "--step", type=float, default=0.05, help="Initial search step size (rad)"
    )
    parser.add_argument(
        "--max-tries", type=int, default=32, help="Max outward steps per direction"
    )
    parser.add_argument(
        "--refine-iters", type=int, default=8, help="Binary refine iterations"
    )
    args = parser.parse_args()

    robot_cell, robot_cell_state, lower, upper = load_scene()
    lower0, upper0 = lower[0], upper[0]

    # Pre-generate identical configs for both methods
    configs = [
        auto_config(i * args.dt, lower, upper, args.speed) for i in range(args.n_cycles)
    ]

    W = args.bar_width
    bar_range_label = "q0:[{:+.2f}..{:+.2f}]".format(lower0, upper0)
    bar_range_label = bar_range_label[:W].ljust(W)

    print(
        "\n=== Joint-0 Boundary Visualization: LINEAR vs EXP_BINARY (fail_fast) ===\n"
        "  cycles={n}  dt={dt:.2f}s  speed={spd:.2f} rad/s  step={step:.3f} rad  "
        "max_tries={mt}  refine={ri}\n"
        "  q0 range: [{lo:+.2f}, {hi:+.2f}]  bar_width={w}\n"
        "  Legend: ~ free  # collision  | boundary  @ current  ? unexplored\n"
        "  found flags: first char=+ direction, second=-  (+= boundary found, .= not found)\n".format(
            n=args.n_cycles,
            dt=args.dt,
            spd=args.speed,
            step=args.step,
            mt=args.max_tries,
            ri=args.refine_iters,
            lo=lower0,
            hi=upper0,
            w=W,
        )
    )

    # Column header
    prefix_w = 23  # width of the per-line prefix before the bar
    print(
        "{:<{pw}}  {bar_lbl}  neg    pos".format(
            "METHOD [  q,   ms] state  f",
            bar_lbl=bar_range_label,
            pw=prefix_w,
        )
    )
    print("-" * (prefix_w + 2 + W + 12 + 6))

    lin_ms_list: list[float] = []
    exp_ms_list: list[float] = []
    lin_found = 0
    exp_found = 0

    with PyBulletClient(connection_type="direct", verbose=False) as client:
        planner = PyBulletPlanner(client)
        planner.set_robot_cell(robot_cell)
        planner.set_robot_cell_state(robot_cell_state)
        cfg = robot_cell_state.robot_configuration.copy()

        for ci, q_base in enumerate(configs):
            # Base collision state (shared, not counted in method timing)
            base_in_coll = check_q(planner, robot_cell_state, cfg, q_base)
            state_str = "COLL" if base_in_coll else "FREE"

            # --- LINEAR ---
            t0 = time.perf_counter()
            res_lin_neg = search_linear(
                planner,
                robot_cell_state,
                cfg,
                q_base,
                base_in_coll,
                lower0,
                upper0,
                -1,
                args.step,
                args.max_tries,
            )
            res_lin_pos = search_linear(
                planner,
                robot_cell_state,
                cfg,
                q_base,
                base_in_coll,
                lower0,
                upper0,
                +1,
                args.step,
                args.max_tries,
            )
            lin_ms = (time.perf_counter() - t0) * 1000.0
            lin_ms_list.append(lin_ms)
            lin_checks = res_lin_neg.checks + res_lin_pos.checks
            lin_found += int(res_lin_neg.found) + int(res_lin_pos.found)

            bar_lin = render_bar(
                lower0,
                upper0,
                q_base[0],
                base_in_coll,
                res_lin_neg,
                res_lin_pos,
                W,
            )

            # --- EXP_BINARY ---
            t0 = time.perf_counter()
            res_exp_neg = search_exp_binary(
                planner,
                robot_cell_state,
                cfg,
                q_base,
                base_in_coll,
                lower0,
                upper0,
                -1,
                args.step,
                args.max_tries,
                args.refine_iters,
            )
            res_exp_pos = search_exp_binary(
                planner,
                robot_cell_state,
                cfg,
                q_base,
                base_in_coll,
                lower0,
                upper0,
                +1,
                args.step,
                args.max_tries,
                args.refine_iters,
            )
            exp_ms = (time.perf_counter() - t0) * 1000.0
            exp_ms_list.append(exp_ms)
            exp_checks = res_exp_neg.checks + res_exp_pos.checks
            exp_found += int(res_exp_neg.found) + int(res_exp_pos.found)

            bar_exp = render_bar(
                lower0,
                upper0,
                q_base[0],
                base_in_coll,
                res_exp_neg,
                res_exp_pos,
                W,
            )

            # --- Print two-line comparison for this cycle ---
            #
            # Line 1: #NNN  LIN [qqq, mmm.m ms]  [state] [ff]  [bar]  neg  pos
            # Line 2: (gap) EXP [qqq, mmm.m ms]  [state] [ff]  [bar]  neg  pos
            #
            cyc_pfx = "#{:03d}".format(ci + 1)
            pad = " " * len(cyc_pfx)

            def fmt_line(label, n_checks, cycle_ms, state, flags, bar, r_neg, r_pos):
                return "{label}[{q:3d}q,{ms:6.1f}ms] {state} {flags}  {bar}  {neg}  {pos}".format(
                    label=label,
                    q=n_checks,
                    ms=cycle_ms,
                    state=state,
                    flags=flags,
                    bar=bar,
                    neg=fmt_bnd(r_neg),
                    pos=fmt_bnd(r_pos),
                )

            print(
                cyc_pfx
                + " LIN "
                + fmt_line(
                    "",
                    lin_checks,
                    lin_ms,
                    state_str,
                    found_flags(res_lin_neg, res_lin_pos),
                    bar_lin,
                    res_lin_neg,
                    res_lin_pos,
                )
            )
            print(
                pad
                + " EXP "
                + fmt_line(
                    "",
                    exp_checks,
                    exp_ms,
                    state_str,
                    found_flags(res_exp_neg, res_exp_pos),
                    bar_exp,
                    res_exp_neg,
                    res_exp_pos,
                )
            )

            sys.stdout.flush()

    # Summary
    n = args.n_cycles
    motion_s = (n - 1) * args.dt
    lin_mean = statistics.fmean(lin_ms_list)
    exp_mean = statistics.fmean(exp_ms_list)
    lin_hz = 1000.0 / lin_mean if lin_mean > 0 else 0.0
    exp_hz = 1000.0 / exp_mean if exp_mean > 0 else 0.0
    lin_wall = sum(lin_ms_list) / 1000.0
    exp_wall = sum(exp_ms_list) / 1000.0

    print(
        "\n{sep}\n"
        "SUMMARY  ({n} cycles, dt={dt:.2f}s -> {mot:.2f}s motion coverage)\n"
        "{sep}\n"
        "{:<10}  {:>9}  {:>9}  {:>9}  {:>10}  {:>11}\n".format(
            "method",
            "mean_ms",
            "cycle_hz",
            "wall_s",
            "bnd_found",
            "bnd_success%",
            sep="=" * 68,
            n=n,
            dt=args.dt,
            mot=motion_s,
        )
        + "{:<10}  {:>9.2f}  {:>9.2f}  {:>9.2f}  {:>10d}  {:>10.1f}%\n".format(
            "linear", lin_mean, lin_hz, lin_wall, lin_found, lin_found / (2 * n) * 100
        )
        + "{:<10}  {:>9.2f}  {:>9.2f}  {:>9.2f}  {:>10d}  {:>10.1f}%\n".format(
            "exp_bin", exp_mean, exp_hz, exp_wall, exp_found, exp_found / (2 * n) * 100
        )
        + "{sep}\n".format(sep="=" * 68)
        + "speedup  exp_bin vs linear  {:.2f}x cycle_hz  {:.1f}s saved\n".format(
            exp_hz / lin_hz if lin_hz > 0 else 0,
            lin_wall - exp_wall,
        )
    )


if __name__ == "__main__":
    main()
