"""Headless collision-check benchmark for compas_fab + PyBullet.

This script loads RobotCell + RobotCellState from JSON and runs auto-moving
joint trajectories in headless (direct) PyBullet mode for a fixed duration.
It benchmarks two collision-check modes:
1) fail-fast (default): stop at first detected collision pair
2) full-report: check all pairs (`full_report=True`)

Usage:
    conda activate game
    python pybullet/bullet_collision_headless_benchmark.py --duration 20
"""

from __future__ import annotations

import argparse
import math
import os
import statistics
import time

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


def load_scene():
    data = json_load(JSON_PATH)
    robot_cell = data["robot_cell"]
    robot_cell_state = data["robot_cell_state"]

    # Workaround for URDF export issue in compas_fab/compas_robots path.
    robot_cell.robot_model.attr.pop("transmission", None)

    # Keep parity with the GUI script so baseline is consistent.

    joints = {j.name: j for j in robot_cell.robot_model.get_configurable_joints()}
    lower = [
        joints[n].limit.lower if joints[n].limit else -math.pi for n in JOINT_NAMES
    ]
    upper = [joints[n].limit.upper if joints[n].limit else math.pi for n in JOINT_NAMES]
    return robot_cell, robot_cell_state, lower, upper


def benchmark_case(duration_s: float, speed: float, full_report: bool) -> dict:
    robot_cell, robot_cell_state, lower, upper = load_scene()

    with PyBulletClient(connection_type="direct", verbose=False) as client:
        planner = PyBulletPlanner(client)
        planner.set_robot_cell(robot_cell)
        planner.set_robot_cell_state(robot_cell_state)

        cfg = robot_cell_state.robot_configuration.copy()
        collision_checks = 0
        collisions_detected = 0
        cc_times_ms = []
        loop_times_s = []

        t_begin = time.perf_counter()
        t_prev = t_begin

        while True:
            now = time.perf_counter()
            elapsed = now - t_begin
            if elapsed >= duration_s:
                break

            dt = now - t_prev
            t_prev = now
            loop_times_s.append(dt)

            vals = []
            for i in range(6):
                mid = 0.5 * (lower[i] + upper[i])
                amp = 0.45 * (upper[i] - lower[i])
                vals.append(mid + amp * math.sin(speed * elapsed + i * 0.9))
            cfg.joint_values = vals
            robot_cell_state.robot_configuration = cfg

            t0 = time.perf_counter()
            try:
                planner.check_collision(
                    robot_cell_state,
                    options={"full_report": full_report, "verbose": False},
                )
            except CollisionCheckError:
                collisions_detected += 1
            cc_ms = (time.perf_counter() - t0) * 1000.0
            cc_times_ms.append(cc_ms)
            collision_checks += 1

        total_s = time.perf_counter() - t_begin

    mean_cc = statistics.fmean(cc_times_ms) if cc_times_ms else float("nan")
    p95_cc = (
        sorted(cc_times_ms)[int(0.95 * (len(cc_times_ms) - 1))]
        if len(cc_times_ms) >= 2
        else mean_cc
    )
    mean_loop = statistics.fmean(loop_times_s) if loop_times_s else float("nan")

    return {
        "mode": "full_report" if full_report else "fail_fast",
        "duration_s": total_s,
        "checks": collision_checks,
        "collisions_detected": collisions_detected,
        "check_hz": (collision_checks / total_s) if total_s > 0 else 0.0,
        "mean_cc_ms": mean_cc,
        "p95_cc_ms": p95_cc,
        "mean_loop_hz": (1.0 / mean_loop) if mean_loop > 0 else 0.0,
    }


def print_result(r: dict):
    print("\n=== {} ===".format(r["mode"]))
    print("duration_s         : {:.3f}".format(r["duration_s"]))
    print("checks             : {}".format(r["checks"]))
    print("collisions_detected: {}".format(r["collisions_detected"]))
    print("check_hz           : {:.2f}".format(r["check_hz"]))
    print("mean_cc_ms         : {:.4f}".format(r["mean_cc_ms"]))
    print("p95_cc_ms          : {:.4f}".format(r["p95_cc_ms"]))
    print("mean_loop_hz       : {:.2f}".format(r["mean_loop_hz"]))


def print_comparison(a: dict, b: dict):
    # a: fail_fast, b: full_report
    delta_hz = b["check_hz"] - a["check_hz"]
    ratio_hz = (b["check_hz"] / a["check_hz"]) if a["check_hz"] > 0 else float("nan")
    delta_ms = b["mean_cc_ms"] - a["mean_cc_ms"]

    print("\n=== comparison (full_report vs fail_fast) ===")
    print("delta_check_hz     : {:+.2f}".format(delta_hz))
    print("ratio_check_hz     : {:.3f}x".format(ratio_hz))
    print("delta_mean_cc_ms   : {:+.4f}".format(delta_ms))


def parse_args():
    parser = argparse.ArgumentParser(description="Headless collision benchmark")
    parser.add_argument(
        "--duration", type=float, default=20.0, help="Seconds per scenario"
    )
    parser.add_argument(
        "--speed", type=float, default=0.6, help="Auto motion speed in rad/s"
    )
    return parser.parse_args()


def main():
    args = parse_args()
    print(
        "Running headless benchmark for {:.1f}s per scenario...".format(args.duration)
    )

    fail_fast = benchmark_case(args.duration, args.speed, full_report=False)
    print_result(fail_fast)

    full_report = benchmark_case(args.duration, args.speed, full_report=True)
    print_result(full_report)

    print_comparison(fail_fast, full_report)


if __name__ == "__main__":
    main()
