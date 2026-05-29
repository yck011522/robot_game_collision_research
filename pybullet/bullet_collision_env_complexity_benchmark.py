"""Sensitivity of collision-check throughput to environment rigid-body count.

Starting from the full scene (14 static rigid bodies RB0..RB13), this script
removes one rigid body at a time and re-measures collision-check throughput
in checks-per-second for two robot model variants:

  - assets_ur10e (full-polygon collision meshes)
  - assets_ur12e (reduced-polygon collision meshes)

It produces a table of (kept_rb_count, ur10e_hz, ur12e_hz) and writes a
Markdown report. The goal is to isolate how much of the collision-check
time is spent on robot self-collision vs robot-vs-environment pairs.

Engineering decisions
---------------------
- All rigid bodies in the scene are static, so removing them only changes
  the set of collision pairs considered, not the configurations sampled.
- Removal order is descending by name (RB13, RB12, ..., RB0). The tool
  "Bucket" is always kept (user request was about rigid bodies, not tools).
- Each kept_count is run on a fresh PyBullet "direct" client to avoid
  any residual broadphase state. Setup time is excluded from check_hz.
- The same seeded random configuration set is reused across all 15
  kept_count levels and both variants, so throughput differences reflect
  only the change in collision-pair workload.
- Configs are sampled uniformly within URDF joint limits.
- Fail-fast mode only. The production use case is "is this pose safe?"
  not "list every overlap", and fail-fast is what the interactive loop
  will actually call.
- Workarounds applied (transmission pop, RB8 touch_links) on every
  variant where the relevant pieces are still present.

Usage
-----
    conda activate game
    python pybullet/bullet_collision_env_complexity_benchmark.py --checks 1000
"""

from __future__ import annotations

# compas_fab must be imported before pybullet (LazyLoader workaround).
from compas.data import json_load
from compas_fab.backends import PyBulletClient, PyBulletPlanner
from compas_fab.backends.exceptions import CollisionCheckError
from compas_fab.robots import RobotCell, RobotCellState, RobotSemantics
from compas_robots import RobotModel
from compas_robots.resources import LocalPackageMeshLoader

import argparse
import math
import os
import random
import statistics
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(HERE)
JSON_PATH = os.path.join(HERE, "robot_cell_and_state.json")
ASSETS_DIR = os.path.join(REPO_ROOT, "assets")

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


# ---------------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------------


def _apply_workarounds(robot_cell: RobotCell, robot_cell_state: RobotCellState) -> None:
    robot_cell.robot_model.attr.pop("transmission", None)
    if "RB8" in robot_cell_state.rigid_body_states:
        robot_cell_state.rigid_body_states["RB8"].touch_links = ["base_link_inertia"]


def _load_asset_model(robot_dir: str):
    urdf = os.path.join(robot_dir, "urdf", "robot_description.urdf")
    srdf = os.path.join(robot_dir, "robot_description_semantic.srdf")
    model = RobotModel.from_urdf_file(urdf)
    semantics = RobotSemantics.from_srdf_file(srdf, model)
    loader = LocalPackageMeshLoader(robot_dir, "")
    model.load_geometry(loader)
    return model, semantics


def _load_template():
    """Load the JSON cell once to extract tools, rigid bodies, and state."""
    data = json_load(JSON_PATH)
    return data["robot_cell"], data["robot_cell_state"]


def _build_variant_with_rbs(robot_dir: str, kept_rb_names: list[str]):
    """Build (cell, state) with the given robot model and a subset of rigid bodies.

    Tools and tool_states are always kept as-is.
    """
    json_cell, json_state = _load_template()
    model, semantics = _load_asset_model(robot_dir)

    rb_models = {n: json_cell.rigid_body_models[n] for n in kept_rb_names}
    cell = RobotCell(
        robot_model=model,
        robot_semantics=semantics,
        tool_models=dict(json_cell.tool_models),
        rigid_body_models=rb_models,
    )

    # Build a fresh state with only the kept rigid_body_states.
    rb_states = {n: json_state.rigid_body_states[n] for n in kept_rb_names}
    state = RobotCellState(
        robot_configuration=json_state.robot_configuration.copy(),
        robot_base_frame=json_state.robot_base_frame,
        tool_states=dict(json_state.tool_states),
        rigid_body_states=rb_states,
    )

    _apply_workarounds(cell, state)
    return cell, state


# ---------------------------------------------------------------------------
# Sampling
# ---------------------------------------------------------------------------


def _joint_limits(robot_cell: RobotCell):
    joints = {j.name: j for j in robot_cell.robot_model.get_configurable_joints()}
    lower = [
        joints[n].limit.lower if joints[n].limit else -math.pi for n in JOINT_NAMES
    ]
    upper = [joints[n].limit.upper if joints[n].limit else math.pi for n in JOINT_NAMES]
    return lower, upper


def _sample_configs(n: int, lower, upper, seed: int) -> list[list[float]]:
    rng = random.Random(seed)
    return [[rng.uniform(lower[i], upper[i]) for i in range(6)] for _ in range(n)]


# ---------------------------------------------------------------------------
# Benchmark
# ---------------------------------------------------------------------------


def _run_case(
    robot_cell: RobotCell, robot_cell_state: RobotCellState, configs: list[list[float]]
) -> dict:
    cfg = robot_cell_state.robot_configuration.copy()
    cc_times_ms: list[float] = []
    collisions = 0

    with PyBulletClient(connection_type="direct", verbose=False) as client:
        planner = PyBulletPlanner(client)
        t_setup_start = time.perf_counter()
        planner.set_robot_cell(robot_cell)
        planner.set_robot_cell_state(robot_cell_state)
        setup_s = time.perf_counter() - t_setup_start

        # Untimed warmup.
        cfg.joint_values = list(configs[0])
        robot_cell_state.robot_configuration = cfg
        try:
            planner.check_collision(
                robot_cell_state, options={"full_report": False, "verbose": False}
            )
        except CollisionCheckError:
            pass

        t_begin = time.perf_counter()
        for joint_values in configs:
            cfg.joint_values = list(joint_values)
            robot_cell_state.robot_configuration = cfg
            t0 = time.perf_counter()
            try:
                planner.check_collision(
                    robot_cell_state, options={"full_report": False, "verbose": False}
                )
            except CollisionCheckError:
                collisions += 1
            cc_times_ms.append((time.perf_counter() - t0) * 1000.0)
        total_s = time.perf_counter() - t_begin

    n = len(cc_times_ms)
    sorted_ms = sorted(cc_times_ms)
    return {
        "setup_s": setup_s,
        "checks": n,
        "collisions": collisions,
        "total_s": total_s,
        "check_hz": n / total_s if total_s > 0 else 0.0,
        "mean_cc_ms": statistics.fmean(cc_times_ms),
        "p50_cc_ms": sorted_ms[n // 2],
        "p95_cc_ms": sorted_ms[max(0, int(0.95 * (n - 1)))],
    }


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------


def _print_summary(rows: list[dict]) -> None:
    print()
    print(
        f"{'kept':>4} | {'removed':<8} | "
        f"{'ur10e_hz':>9} | {'ur10e_ms':>8} | {'ur10e_coll':>10} | "
        f"{'ur12e_hz':>9} | {'ur12e_ms':>8} | {'ur12e_coll':>10}"
    )
    print("-" * 92)
    for r in rows:
        print(
            f"{r['kept_count']:>4} | {r['last_removed'] or '-':<8} | "
            f"{r['ur10e']['check_hz']:>9.2f} | {r['ur10e']['mean_cc_ms']:>8.4f} | "
            f"{r['ur10e']['collisions']:>10} | "
            f"{r['ur12e']['check_hz']:>9.2f} | {r['ur12e']['mean_cc_ms']:>8.4f} | "
            f"{r['ur12e']['collisions']:>10}"
        )


def _write_markdown(rows: list[dict], args, out_path: str) -> None:
    full_rbs = rows[0]["kept_rbs"] if rows else []
    base_ur10e = rows[0]["ur10e"]["check_hz"] if rows else 1.0
    base_ur12e = rows[0]["ur12e"]["check_hz"] if rows else 1.0

    lines: list[str] = []
    lines.append("# Environment Complexity Sensitivity\n")
    lines.append(
        "Effect of progressively removing static rigid bodies on collision-check "
        "throughput, measured in checks per second. Two robot model variants "
        "are run side-by-side on an identical, seeded set of random "
        "configurations.\n"
    )

    lines.append("## Run configuration\n")
    lines.append(f"- checks per case: **{args.checks}**")
    lines.append(f"- random seed: **{args.seed}**")
    lines.append("- backend: PyBullet `direct` (headless), single process")
    lines.append("- mode: fail_fast")
    lines.append(
        f"- starting rigid bodies ({len(full_rbs)}): " f"`{', '.join(full_rbs)}`"
    )
    lines.append(
        "- removal order: descending by name "
        "(`RB13` removed first, then `RB12`, ...)"
    )
    lines.append("- tool `Bucket` is kept in all cases")
    lines.append(
        "- each case uses a fresh PyBullet client; setup time is "
        "reported but excluded from `check_hz`\n"
    )

    lines.append("## Results\n")
    lines.append(
        "| kept | last_removed | ur10e_hz | ur10e_x | ur10e_ms | "
        "ur10e_coll | ur12e_hz | ur12e_x | ur12e_ms | ur12e_coll |"
    )
    lines.append("|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|")
    for r in rows:
        u10 = r["ur10e"]
        u12 = r["ur12e"]
        lines.append(
            f"| {r['kept_count']} | {r['last_removed'] or '-'} | "
            f"{u10['check_hz']:.2f} | {u10['check_hz']/base_ur10e:.3f}x | "
            f"{u10['mean_cc_ms']:.4f} | {u10['collisions']} | "
            f"{u12['check_hz']:.2f} | {u12['check_hz']/base_ur12e:.3f}x | "
            f"{u12['mean_cc_ms']:.4f} | {u12['collisions']} |"
        )
    lines.append("")

    lines.append("## Notes\n")
    lines.append(
        "- `kept` counts only rigid bodies. The robot itself (always self-"
        "checked) and the `Bucket` tool are present in every row."
    )
    lines.append(
        "- `ur10e_x` and `ur12e_x` are speedup multiples vs the full-scene "
        "row (top row) for that variant."
    )
    lines.append(
        "- A flat curve means environment rigid bodies contribute very "
        "little to the per-check cost — most time is spent on robot self-"
        "collision and the tool. A steep curve means environment pairs "
        "dominate, and reducing/simplifying environment geometry would pay "
        "off in the integration loop."
    )
    lines.append(
        "- Collision counts will drop as bodies are removed; this is "
        "expected and does not affect timing fairness because per-check "
        "wall-clock is measured around the call regardless of outcome."
    )

    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument(
        "--checks",
        type=int,
        default=1000,
        help="Collision checks per case (per variant)",
    )
    p.add_argument(
        "--seed",
        type=int,
        default=20260529,
        help="Seed for the shared random configuration set",
    )
    p.add_argument(
        "--out",
        type=str,
        default=os.path.join(HERE, "bullet_collision_env_complexity_benchmark.md"),
        help="Markdown report output path",
    )
    p.add_argument(
        "--no-write", action="store_true", help="Skip writing the markdown report"
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()

    # Discover the full set of rigid bodies once.
    json_cell, _ = _load_template()
    all_rbs = sorted(
        json_cell.rigid_body_models.keys(),
        key=lambda s: int(s[2:]) if s[2:].isdigit() else 0,
    )
    print(f"Discovered {len(all_rbs)} rigid bodies: {all_rbs}")

    # Sample configs once using ur10e limits (identical to ur12e).
    ur10e_dir = os.path.join(ASSETS_DIR, "ur10e_robot")
    ur12e_dir = os.path.join(ASSETS_DIR, "ur12e_robot")
    full_cell, _ = _build_variant_with_rbs(ur10e_dir, all_rbs)
    lower, upper = _joint_limits(full_cell)
    configs = _sample_configs(args.checks, lower, upper, args.seed)
    print(f"Generated {len(configs)} shared random configs (seed={args.seed}).")

    # Removal schedule: full set, then drop one at a time from the end.
    schedule: list[tuple[list[str], str | None]] = []
    kept = list(all_rbs)
    schedule.append((list(kept), None))
    while kept:
        removed = kept.pop()  # remove last (highest-numbered)
        schedule.append((list(kept), removed))

    rows: list[dict] = []
    for i, (kept_rbs, last_removed) in enumerate(schedule):
        kc = len(kept_rbs)
        label = f"[{i+1}/{len(schedule)}] kept={kc}"
        if last_removed:
            label += f" (removed {last_removed})"
        print(f"\n--- {label} ---")

        print(f"  ur10e ...")
        cell, state = _build_variant_with_rbs(ur10e_dir, kept_rbs)
        r10 = _run_case(cell, state, configs)
        print(
            f"    setup_s={r10['setup_s']:.3f} check_hz={r10['check_hz']:.2f} "
            f"mean_ms={r10['mean_cc_ms']:.4f} coll={r10['collisions']}/{r10['checks']}"
        )

        print(f"  ur12e ...")
        cell, state = _build_variant_with_rbs(ur12e_dir, kept_rbs)
        r12 = _run_case(cell, state, configs)
        print(
            f"    setup_s={r12['setup_s']:.3f} check_hz={r12['check_hz']:.2f} "
            f"mean_ms={r12['mean_cc_ms']:.4f} coll={r12['collisions']}/{r12['checks']}"
        )

        rows.append(
            {
                "kept_count": kc,
                "kept_rbs": kept_rbs,
                "last_removed": last_removed,
                "ur10e": r10,
                "ur12e": r12,
            }
        )

    print("\n=== Summary ===")
    _print_summary(rows)

    if not args.no_write:
        _write_markdown(rows, args, args.out)
        print(f"\nWrote markdown report to {args.out}")


if __name__ == "__main__":
    main()
