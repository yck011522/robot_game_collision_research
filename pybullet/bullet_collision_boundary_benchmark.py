"""Headless benchmark for joint-0 collision-boundary search per cycle.

Per cycle:
1) Generate an auto-moving 6D configuration q(t) from a deterministic timeline.
2) Keep joints 1..5 fixed, and search on joint 0 in both directions.
3) Find where collision state flips compared to the base configuration.
4) Stop each side after boundary found or after max tries / joint limit.

Search methods:
- linear: fixed-step scan
- exp_binary: exponential bracketing + binary refinement

Usage (as requested):
    conda activate game
    python pybullet/bullet_collision_boundary_benchmark.py --duration 10 --method all
"""

from __future__ import annotations

import argparse
import math
import os
import statistics
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


@dataclass
class SearchResult:
    found: bool
    boundary_q0: float | None
    checks: int
    tries: int
    reason: str


@dataclass
class CycleOutcome:
    q0_base: float
    start_collision: bool
    pos: SearchResult
    neg: SearchResult


def load_scene():
    data = json_load(JSON_PATH)
    robot_cell = data["robot_cell"]
    robot_cell_state = data["robot_cell_state"]

    # Workaround for URDF export issue in compas_fab/compas_robots path.
    robot_cell.robot_model.attr.pop("transmission", None)

    # Keep parity with existing scripts in this repo.
    if "RB8" in robot_cell_state.rigid_body_states:
        robot_cell_state.rigid_body_states["RB8"].touch_links = ["base_link_inertia"]

    joints = {j.name: j for j in robot_cell.robot_model.get_configurable_joints()}
    lower = [joints[n].limit.lower if joints[n].limit else -math.pi for n in JOINT_NAMES]
    upper = [joints[n].limit.upper if joints[n].limit else math.pi for n in JOINT_NAMES]
    return robot_cell, robot_cell_state, lower, upper


def check_collision_state(
    planner: PyBulletPlanner,
    robot_cell_state,
    cfg,
    q_values: list[float],
    full_report: bool,
) -> tuple[bool, float]:
    cfg.joint_values = q_values
    robot_cell_state.robot_configuration = cfg

    t0 = time.perf_counter()
    in_collision = False
    try:
        planner.check_collision(
            robot_cell_state,
            options={"full_report": full_report, "verbose": False},
        )
    except CollisionCheckError:
        in_collision = True
    cc_ms = (time.perf_counter() - t0) * 1000.0
    return in_collision, cc_ms


def search_linear(
    planner: PyBulletPlanner,
    robot_cell_state,
    cfg,
    q_base: list[float],
    start_collision: bool,
    lower0: float,
    upper0: float,
    direction: int,
    step_size: float,
    max_tries: int,
    full_report: bool,
) -> tuple[SearchResult, list[float]]:
    checks = 0
    tries = 0
    cc_times = []
    q0_start = q_base[0]
    last_q0 = q0_start

    for i in range(1, max_tries + 1):
        tries = i
        q0 = q0_start + direction * step_size * i
        q0 = min(max(q0, lower0), upper0)
        if q0 == last_q0:
            return (
                SearchResult(False, None, checks, tries, "reached_joint_limit"),
                cc_times,
            )

        q_test = list(q_base)
        q_test[0] = q0
        in_collision, cc_ms = check_collision_state(
            planner, robot_cell_state, cfg, q_test, full_report
        )
        cc_times.append(cc_ms)
        checks += 1

        if in_collision != start_collision:
            return SearchResult(True, q0, checks, tries, "state_changed"), cc_times

        last_q0 = q0

    return SearchResult(False, None, checks, tries, "max_tries"), cc_times


def search_exp_binary(
    planner: PyBulletPlanner,
    robot_cell_state,
    cfg,
    q_base: list[float],
    start_collision: bool,
    lower0: float,
    upper0: float,
    direction: int,
    step_size: float,
    max_tries: int,
    refine_iters: int,
    full_report: bool,
) -> tuple[SearchResult, list[float]]:
    checks = 0
    tries = 0
    cc_times = []

    q0_start = q_base[0]
    d_prev = 0.0
    q_prev = q0_start
    step = step_size

    found_bracket = False
    d_low = 0.0
    d_high = 0.0

    for i in range(1, max_tries + 1):
        tries = i
        d = d_prev + step
        q0 = q0_start + direction * d
        q0 = min(max(q0, lower0), upper0)

        if q0 == q_prev:
            return (
                SearchResult(False, None, checks, tries, "reached_joint_limit"),
                cc_times,
            )

        q_test = list(q_base)
        q_test[0] = q0
        in_collision, cc_ms = check_collision_state(
            planner, robot_cell_state, cfg, q_test, full_report
        )
        cc_times.append(cc_ms)
        checks += 1

        if in_collision != start_collision:
            found_bracket = True
            d_low = d_prev
            d_high = abs(q0 - q0_start)
            break

        d_prev = abs(q0 - q0_start)
        q_prev = q0
        step *= 2.0

    if not found_bracket:
        return SearchResult(False, None, checks, tries, "max_tries"), cc_times

    # Binary refine in distance from q0_start.
    for _ in range(refine_iters):
        d_mid = 0.5 * (d_low + d_high)
        q0_mid = q0_start + direction * d_mid
        q0_mid = min(max(q0_mid, lower0), upper0)

        q_test = list(q_base)
        q_test[0] = q0_mid
        in_collision, cc_ms = check_collision_state(
            planner, robot_cell_state, cfg, q_test, full_report
        )
        cc_times.append(cc_ms)
        checks += 1

        if in_collision == start_collision:
            d_low = d_mid
        else:
            d_high = d_mid

    boundary_q0 = q0_start + direction * d_high
    boundary_q0 = min(max(boundary_q0, lower0), upper0)
    return SearchResult(True, boundary_q0, checks, tries, "state_changed"), cc_times


def auto_config(
    t: float,
    lower: list[float],
    upper: list[float],
    speed: float,
) -> list[float]:
    vals = []
    for i in range(6):
        mid = 0.5 * (lower[i] + upper[i])
        amp = 0.45 * (upper[i] - lower[i])
        vals.append(mid + amp * math.sin(speed * t + i * 0.9))
    return vals


def build_cycle_configs(
    duration_s: float,
    cycle_dt: float,
    lower: list[float],
    upper: list[float],
    speed: float,
) -> list[list[float]]:
    # Deterministic and shared for all algorithms.
    count = max(1, int(duration_s / cycle_dt))
    return [auto_config(i * cycle_dt, lower, upper, speed) for i in range(count)]


def run_case(
    method: str,
    cycle_configs: list[list[float]],
    full_report: bool,
    step_size: float,
    max_tries: int,
    refine_iters: int,
) -> dict:
    robot_cell, robot_cell_state, lower, upper = load_scene()

    with PyBulletClient(connection_type="direct", verbose=False) as client:
        planner = PyBulletPlanner(client)
        planner.set_robot_cell(robot_cell)
        planner.set_robot_cell_state(robot_cell_state)

        cfg = robot_cell_state.robot_configuration.copy()
        lower0 = lower[0]
        upper0 = upper[0]

        cycle_times = []
        all_cc_ms = []
        checks_total = 0
        found_pos = 0
        found_neg = 0
        fail_pos = 0
        fail_neg = 0
        outcomes: list[CycleOutcome] = []

        t_begin = time.perf_counter()

        for q_base in cycle_configs:
            t_cycle_start = time.perf_counter()

            start_collision, cc_ms = check_collision_state(
                planner, robot_cell_state, cfg, q_base, full_report
            )
            all_cc_ms.append(cc_ms)
            checks_total += 1

            if method == "linear":
                fn = search_linear
                extra = {}
            else:
                fn = search_exp_binary
                extra = {"refine_iters": refine_iters}

            res_pos, cc_list_pos = fn(
                planner=planner,
                robot_cell_state=robot_cell_state,
                cfg=cfg,
                q_base=q_base,
                start_collision=start_collision,
                lower0=lower0,
                upper0=upper0,
                direction=+1,
                step_size=step_size,
                max_tries=max_tries,
                full_report=full_report,
                **extra,
            )
            res_neg, cc_list_neg = fn(
                planner=planner,
                robot_cell_state=robot_cell_state,
                cfg=cfg,
                q_base=q_base,
                start_collision=start_collision,
                lower0=lower0,
                upper0=upper0,
                direction=-1,
                step_size=step_size,
                max_tries=max_tries,
                full_report=full_report,
                **extra,
            )

            all_cc_ms.extend(cc_list_pos)
            all_cc_ms.extend(cc_list_neg)
            checks_total += res_pos.checks + res_neg.checks

            if res_pos.found:
                found_pos += 1
            else:
                fail_pos += 1

            if res_neg.found:
                found_neg += 1
            else:
                fail_neg += 1

            outcomes.append(
                CycleOutcome(
                    q0_base=q_base[0],
                    start_collision=start_collision,
                    pos=res_pos,
                    neg=res_neg,
                )
            )
            cycle_times.append(time.perf_counter() - t_cycle_start)

        total_s = time.perf_counter() - t_begin

    cycles = len(cycle_configs)
    mean_cycle_s = statistics.fmean(cycle_times) if cycle_times else float("nan")
    mean_cc_ms = statistics.fmean(all_cc_ms) if all_cc_ms else float("nan")
    p95_cc_ms = (
        sorted(all_cc_ms)[int(0.95 * (len(all_cc_ms) - 1))]
        if len(all_cc_ms) >= 2
        else mean_cc_ms
    )

    return {
        "method": method,
        "full_report": full_report,
        "duration_compute_s": total_s,
        "cycles": cycles,
        "cycle_hz": (cycles / total_s) if total_s > 0 else 0.0,
        "mean_cycle_ms": mean_cycle_s * 1000.0,
        "checks_total": checks_total,
        "check_hz": (checks_total / total_s) if total_s > 0 else 0.0,
        "mean_cc_ms": mean_cc_ms,
        "p95_cc_ms": p95_cc_ms,
        "found_pos": found_pos,
        "found_neg": found_neg,
        "fail_pos": fail_pos,
        "fail_neg": fail_neg,
        "success_ratio_pos": (found_pos / cycles) if cycles > 0 else 0.0,
        "success_ratio_neg": (found_neg / cycles) if cycles > 0 else 0.0,
        "outcomes": outcomes,
        "lower0": lower[0],
        "upper0": upper[0],
    }


def print_result(r: dict):
    mode = "full_report" if r["full_report"] else "fail_fast"
    print("\n=== method={} | mode={} ===".format(r["method"], mode))
    print("compute_time_s     : {:.3f}".format(r["duration_compute_s"]))
    print("cycles             : {}".format(r["cycles"]))
    print("cycle_hz           : {:.2f}".format(r["cycle_hz"]))
    print("mean_cycle_ms      : {:.3f}".format(r["mean_cycle_ms"]))
    print("checks_total       : {}".format(r["checks_total"]))
    print("check_hz           : {:.2f}".format(r["check_hz"]))
    print("mean_cc_ms         : {:.4f}".format(r["mean_cc_ms"]))
    print("p95_cc_ms          : {:.4f}".format(r["p95_cc_ms"]))
    print(
        "boundary_found(+/-): {}/{}  (success {:.1%}/{:.1%})".format(
            r["found_pos"],
            r["found_neg"],
            r["success_ratio_pos"],
            r["success_ratio_neg"],
        )
    )
    print("boundary_fail(+/-) : {}/{}".format(r["fail_pos"], r["fail_neg"]))


def print_comparison(a: dict, b: dict):
    print("\n=== comparison ({} vs {}) ===".format(b["method"], a["method"]))
    print("delta_cycle_hz     : {:+.2f}".format(b["cycle_hz"] - a["cycle_hz"]))
    ratio = b["cycle_hz"] / a["cycle_hz"] if a["cycle_hz"] > 0 else float("nan")
    print("ratio_cycle_hz     : {:.3f}x".format(ratio))
    print("delta_check_hz     : {:+.2f}".format(b["check_hz"] - a["check_hz"]))
    print("delta_mean_cc_ms   : {:+.4f}".format(b["mean_cc_ms"] - a["mean_cc_ms"]))


def clamp01(x: float) -> float:
    return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)


def to_bar_index(value: float, lower: float, upper: float, width: int) -> int:
    if upper <= lower:
        return 0
    t = clamp01((value - lower) / (upper - lower))
    return int(round(t * (width - 1)))


def place_char(buf: list[str], idx: int, ch: str):
    if idx < 0:
        idx = 0
    if idx >= len(buf):
        idx = len(buf) - 1
    if buf[idx] == ".":
        buf[idx] = ch
    elif buf[idx] != ch:
        buf[idx] = "*"


def render_cycle_bar(
    outcome: CycleOutcome,
    lower0: float,
    upper0: float,
    width: int,
) -> str:
    # Legend inside bar:
    # 0: base q0, N: negative boundary, P: positive boundary, !: boundary not found.
    buf = ["."] * width

    base_i = to_bar_index(outcome.q0_base, lower0, upper0, width)
    place_char(buf, base_i, "0")

    if outcome.neg.found and outcome.neg.boundary_q0 is not None:
        neg_i = to_bar_index(outcome.neg.boundary_q0, lower0, upper0, width)
        place_char(buf, neg_i, "N")
    else:
        place_char(buf, 0, "!")

    if outcome.pos.found and outcome.pos.boundary_q0 is not None:
        pos_i = to_bar_index(outcome.pos.boundary_q0, lower0, upper0, width)
        place_char(buf, pos_i, "P")
    else:
        place_char(buf, width - 1, "!")

    return "|{}|".format("".join(buf))


def print_visual_comparison(
    linear_result: dict,
    exp_result: dict,
    bar_width: int,
):
    lo = linear_result["lower0"]
    hi = linear_result["upper0"]
    linear_out = linear_result["outcomes"]
    exp_out = exp_result["outcomes"]

    print("\n=== cycle visualization (shared configs) ===")
    print("legend: 0=base q0, N=neg boundary, P=pos boundary, !=not found, *=overlap")
    print("joint0_range: [{:.3f}, {:.3f}] rad".format(lo, hi))

    for i, (o_lin, o_exp) in enumerate(zip(linear_out, exp_out), start=1):
        bar_lin = render_cycle_bar(o_lin, lo, hi, bar_width)
        bar_exp = render_cycle_bar(o_exp, lo, hi, bar_width)
        col = "C" if o_lin.start_collision else "F"
        print(
            "{idx:03d} {col} q0={q0:+.3f}  L {lin}  E {exp}".format(
                idx=i,
                col=col,
                q0=o_lin.q0_base,
                lin=bar_lin,
                exp=bar_exp,
            )
        )


def parse_args():
    parser = argparse.ArgumentParser(description="Joint-0 boundary-search benchmark")
    parser.add_argument("--duration", type=float, default=20.0, help="Nominal motion horizon in seconds")
    parser.add_argument("--speed", type=float, default=0.6, help="Auto motion speed (rad/s)")
    parser.add_argument(
        "--method",
        choices=["linear", "exp_binary", "all"],
        default="all",
        help="Boundary search method",
    )
    parser.add_argument("--step", type=float, default=0.03, help="Initial step size (rad)")
    parser.add_argument("--max-tries", type=int, default=64, help="Max outward tries per side")
    parser.add_argument(
        "--refine-iters",
        type=int,
        default=10,
        help="Binary refine iterations for exp_binary",
    )
    parser.add_argument(
        "--cycle-dt",
        type=float,
        default=0.1,
        help="Cycle sample interval for shared q list (seconds)",
    )
    parser.add_argument(
        "--bar-width",
        type=int,
        default=41,
        help="Fixed width of text bar visualization",
    )
    parser.add_argument(
        "--full-report",
        action="store_true",
        help="Use full_report collision mode (check all pairs)",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    methods = ["linear", "exp_binary"] if args.method == "all" else [args.method]

    # Build one deterministic cycle set so all methods see identical q(t).
    _, _, lower, upper = load_scene()
    cycle_configs = build_cycle_configs(
        duration_s=args.duration,
        cycle_dt=args.cycle_dt,
        lower=lower,
        upper=upper,
        speed=args.speed,
    )

    mode_label = "full_report" if args.full_report else "fail_fast"
    print(
        "Running joint-0 boundary benchmark with shared configs: "
        "duration={:.1f}s cycle_dt={:.3f}s cycles={} mode={}...".format(
            args.duration, args.cycle_dt, len(cycle_configs), mode_label
        )
    )

    results = []
    for method in methods:
        r = run_case(
            method=method,
            cycle_configs=cycle_configs,
            full_report=args.full_report,
            step_size=args.step,
            max_tries=args.max_tries,
            refine_iters=args.refine_iters,
        )
        print_result(r)
        results.append(r)

    if len(results) == 2:
        print_comparison(results[0], results[1])
        print_visual_comparison(results[0], results[1], args.bar_width)


if __name__ == "__main__":
    main()
