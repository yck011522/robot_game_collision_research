"""Headless benchmark for joint-0 collision-boundary search per cycle.

Per cycle:
1) Generate an auto-moving 6D configuration q(t).
2) Keep joints 1..5 fixed, and search on joint 0 in both directions.
3) Find where collision state flips compared to the base configuration.
4) Stop each side after boundary found or after max tries / joint limit.

Search methods:
- linear: fixed-step scan
- exp_binary: exponential bracketing + binary refinement

Usage:
    conda activate game
    python pybullet/bullet_collision_boundary_benchmark.py --duration 20 --method all
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


def load_scene():
    data = json_load(JSON_PATH)
    robot_cell = data["robot_cell"]
    robot_cell_state = data["robot_cell_state"]

    # Workaround for URDF export issue in compas_fab/compas_robots path.
    robot_cell.robot_model.attr.pop("transmission", None)

    joints = {j.name: j for j in robot_cell.robot_model.get_configurable_joints()}
    lower = [
        joints[n].limit.lower if joints[n].limit else -math.pi for n in JOINT_NAMES
    ]
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
    t: float, lower: list[float], upper: list[float], speed: float
) -> list[float]:
    vals = []
    for i in range(6):
        mid = 0.5 * (lower[i] + upper[i])
        amp = 0.45 * (upper[i] - lower[i])
        vals.append(mid + amp * math.sin(speed * t + i * 0.9))
    return vals


def run_case(
    duration_s: float,
    speed: float,
    full_report: bool,
    method: str,
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
        cycles = 0
        found_pos = 0
        found_neg = 0
        fail_pos = 0
        fail_neg = 0

        t_begin = time.perf_counter()
        while True:
            t_cycle_start = time.perf_counter()
            elapsed = t_cycle_start - t_begin
            if elapsed >= duration_s:
                break

            q_base = auto_config(elapsed, lower, upper, speed)
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

            cycles += 1
            cycle_times.append(time.perf_counter() - t_cycle_start)

        total_s = time.perf_counter() - t_begin

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
        "duration_s": total_s,
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
    }


def print_result(r: dict):
    mode = "full_report" if r["full_report"] else "fail_fast"
    print("\n=== method={} | mode={} ===".format(r["method"], mode))
    print("duration_s         : {:.3f}".format(r["duration_s"]))
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
    print(
        "ratio_cycle_hz     : {:.3f}x".format(
            b["cycle_hz"] / a["cycle_hz"] if a["cycle_hz"] > 0 else float("nan")
        )
    )
    print("delta_check_hz     : {:+.2f}".format(b["check_hz"] - a["check_hz"]))
    print("delta_mean_cc_ms   : {:+.4f}".format(b["mean_cc_ms"] - a["mean_cc_ms"]))


def parse_args():
    parser = argparse.ArgumentParser(description="Joint-0 boundary-search benchmark")
    parser.add_argument("--duration", type=float, default=20.0, help="Seconds per case")
    parser.add_argument(
        "--speed", type=float, default=0.6, help="Auto motion speed (rad/s)"
    )
    parser.add_argument(
        "--method",
        choices=["linear", "exp_binary", "all"],
        default="all",
        help="Boundary search method",
    )
    parser.add_argument(
        "--step", type=float, default=0.03, help="Initial step size (rad)"
    )
    parser.add_argument(
        "--max-tries", type=int, default=64, help="Max outward tries per side"
    )
    parser.add_argument(
        "--refine-iters",
        type=int,
        default=10,
        help="Binary refine iterations for exp_binary",
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

    print(
        "Running joint-0 boundary benchmark for {:.1f}s per method (mode={})...".format(
            args.duration, "full_report" if args.full_report else "fail_fast"
        )
    )

    results = []
    for method in methods:
        r = run_case(
            duration_s=args.duration,
            speed=args.speed,
            full_report=args.full_report,
            method=method,
            step_size=args.step,
            max_tries=args.max_tries,
            refine_iters=args.refine_iters,
        )
        print_result(r)
        results.append(r)

    if len(results) == 2:
        print_comparison(results[0], results[1])


if __name__ == "__main__":
    main()
