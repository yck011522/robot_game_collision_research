"""Compare collision-check throughput across robot model variants.

Loads the same RobotCellState (tools, rigid bodies, base configuration) and
swaps in three different RobotModel sources, then runs an identical, seeded
set of random joint configurations through each in headless PyBullet:

  1. json_ur10e   - baseline: model embedded in robot_cell_and_state.json
  2. assets_ur10e - same UR10e but loaded from assets/ur10e_robot
                    (sanity check that the swap path matches the baseline)
  3. assets_ur12e - the candidate with reduced-polygon collision meshes

Throughput is measured in both fail-fast and full-report modes. Setup time
(client startup, mesh upload) is excluded from the throughput numbers.

Engineering decisions
---------------------
- Single process, single PyBullet "direct" client per variant; this is a
  controlled A/B/C comparison, not a parallelism study.
- Random configs are drawn once with a fixed seed and reused across all
  three variants and both modes, so any throughput difference reflects the
  collision backend's work, not workload variance.
- Sampling range is each joint's URDF [lower, upper] limit. UR limits are
  very wide so a large fraction of samples collide; that is fine because
  the workload is identical across variants.
- For the asset-based variants the JSON cell's tool_models and
  rigid_body_models are carried over unchanged, so the only thing that
  changes between variants is robot_model + robot_semantics.
- The persistent RB8/base_link_inertia touch-link workaround and the
  transmission-tag URDF-export workaround are applied to every variant.

Usage
-----
    conda activate game
    python pybullet/bullet_collision_robot_model_comparison.py --checks 2000
"""

from __future__ import annotations

# compas_fab must be imported before pybullet (LazyLoader workaround).
from compas.data import json_load
from compas_fab.backends import PyBulletClient, PyBulletPlanner
from compas_fab.backends.exceptions import CollisionCheckError
from compas_fab.robots import RobotCell, RobotSemantics
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
# Scene loading
# ---------------------------------------------------------------------------


def _apply_workarounds(robot_cell: RobotCell, robot_cell_state) -> None:
    """Apply the three workarounds used by every script in this repo."""
    # URDF export crash from tag-less URDFGenericElement.
    robot_cell.robot_model.attr.pop("transmission", None)
    # Persistent overlap allow-list for the bucket rigid body.
    if robot_cell_state is not None and "RB8" in robot_cell_state.rigid_body_states:
        robot_cell_state.rigid_body_states["RB8"].touch_links = ["base_link_inertia"]


def _load_json_cell():
    """Load the baseline RobotCell + RobotCellState straight from JSON."""
    data = json_load(JSON_PATH)
    cell = data["robot_cell"]
    state = data["robot_cell_state"]
    _apply_workarounds(cell, state)
    return cell, state


def _load_asset_model(robot_dir: str) -> tuple[RobotModel, RobotSemantics]:
    """Load a RobotModel + RobotSemantics from a local asset folder.

    Mirrors the path that compas_fab.robots.RobotCellLibrary takes for its
    bundled robots, but resolves paths against this repository's assets/
    instead of the compas_fab data directory.
    """
    urdf = os.path.join(robot_dir, "urdf", "robot_description.urdf")
    srdf = os.path.join(robot_dir, "robot_description_semantic.srdf")
    model = RobotModel.from_urdf_file(urdf)
    semantics = RobotSemantics.from_srdf_file(srdf, model)
    loader = LocalPackageMeshLoader(robot_dir, "")
    model.load_geometry(loader)
    return model, semantics


def _build_asset_variant(robot_dir: str):
    """Build a (cell, state) pair using an asset-folder robot model.

    Tools and rigid bodies are carried over from the JSON cell so only the
    robot model itself differs between variants.
    """
    json_cell, json_state = _load_json_cell()
    model, semantics = _load_asset_model(robot_dir)
    new_cell = RobotCell(
        robot_model=model,
        robot_semantics=semantics,
        tool_models=json_cell.tool_models,
        rigid_body_models=json_cell.rigid_body_models,
    )
    _apply_workarounds(new_cell, json_state)
    return new_cell, json_state


# ---------------------------------------------------------------------------
# Configuration sampling
# ---------------------------------------------------------------------------


def _joint_limits(robot_cell: RobotCell) -> tuple[list[float], list[float]]:
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


def _run_variant(
    label: str,
    robot_cell: RobotCell,
    robot_cell_state,
    configs: list[list[float]],
    full_report: bool,
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

        # One untimed warmup check.
        cfg.joint_values = list(configs[0])
        robot_cell_state.robot_configuration = cfg
        try:
            planner.check_collision(
                robot_cell_state,
                options={"full_report": full_report, "verbose": False},
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
                    robot_cell_state,
                    options={"full_report": full_report, "verbose": False},
                )
            except CollisionCheckError:
                collisions += 1
            cc_times_ms.append((time.perf_counter() - t0) * 1000.0)
        total_s = time.perf_counter() - t_begin

    n = len(cc_times_ms)
    mean_ms = statistics.fmean(cc_times_ms)
    sorted_ms = sorted(cc_times_ms)
    p50_ms = sorted_ms[n // 2]
    p95_ms = sorted_ms[max(0, int(0.95 * (n - 1)))]
    return {
        "variant": label,
        "mode": "full_report" if full_report else "fail_fast",
        "setup_s": setup_s,
        "checks": n,
        "collisions": collisions,
        "total_s": total_s,
        "check_hz": n / total_s if total_s > 0 else 0.0,
        "mean_cc_ms": mean_ms,
        "p50_cc_ms": p50_ms,
        "p95_cc_ms": p95_ms,
    }


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------


def _fmt_row(r: dict) -> str:
    return (
        f"{r['variant']:<14} | {r['mode']:<11} | "
        f"{r['checks']:>5} | {r['collisions']:>5} | "
        f"{r['setup_s']:>7.3f} | {r['total_s']:>7.3f} | "
        f"{r['check_hz']:>8.2f} | {r['mean_cc_ms']:>7.4f} | "
        f"{r['p50_cc_ms']:>7.4f} | {r['p95_cc_ms']:>7.4f}"
    )


def _print_table(results: list[dict]) -> None:
    header = (
        f"{'variant':<14} | {'mode':<11} | "
        f"{'N':>5} | {'coll':>5} | "
        f"{'setup_s':>7} | {'total_s':>7} | "
        f"{'check_hz':>8} | {'mean_ms':>7} | "
        f"{'p50_ms':>7} | {'p95_ms':>7}"
    )
    sep = "-" * len(header)
    print(header)
    print(sep)
    for r in results:
        print(_fmt_row(r))


def _write_markdown(results: list[dict], args, out_path: str) -> None:
    by_variant = {}
    for r in results:
        by_variant.setdefault(r["variant"], {})[r["mode"]] = r

    base_ff = by_variant.get("json_ur10e", {}).get("fail_fast")
    base_fr = by_variant.get("json_ur10e", {}).get("full_report")

    lines: list[str] = []
    lines.append("# Robot Model Collision-Check Comparison\n")
    lines.append(
        "Comparison of collision-check throughput between the JSON-embedded "
        "UR10e baseline, the same UR10e reloaded from `assets/ur10e_robot/`, "
        "and the candidate `assets/ur12e_robot/` with reduced-polygon "
        "collision meshes.\n"
    )
    lines.append("## Run configuration\n")
    lines.append(f"- checks per variant: **{args.checks}**")
    lines.append(f"- random seed: **{args.seed}**")
    lines.append("- backend: PyBullet `direct` (headless), single process")
    lines.append(
        "- configs: uniformly sampled in URDF joint limits; identical "
        "set across all variants and modes"
    )
    lines.append(
        "- one untimed warmup check before timing begins; setup time "
        "is reported separately and excluded from `check_hz`\n"
    )

    lines.append("## Results\n")
    lines.append(
        "| variant | mode | N | collisions | setup_s | total_s | check_hz | "
        "mean_ms | p50_ms | p95_ms |"
    )
    lines.append("|---|---|---:|---:|---:|---:|---:|---:|---:|---:|")
    for r in results:
        lines.append(
            f"| {r['variant']} | {r['mode']} | {r['checks']} | {r['collisions']} | "
            f"{r['setup_s']:.3f} | {r['total_s']:.3f} | {r['check_hz']:.2f} | "
            f"{r['mean_cc_ms']:.4f} | {r['p50_cc_ms']:.4f} | {r['p95_cc_ms']:.4f} |"
        )
    lines.append("")

    if base_ff and base_fr:
        lines.append("## Speedup vs json_ur10e baseline\n")
        lines.append(
            "| variant | fail_fast hz | x baseline | full_report hz | x baseline |"
        )
        lines.append("|---|---:|---:|---:|---:|")
        for variant_name in ("json_ur10e", "assets_ur10e", "assets_ur12e"):
            v = by_variant.get(variant_name, {})
            ff = v.get("fail_fast")
            fr = v.get("full_report")
            if not ff or not fr:
                continue
            ff_ratio = ff["check_hz"] / base_ff["check_hz"]
            fr_ratio = fr["check_hz"] / base_fr["check_hz"]
            lines.append(
                f"| {variant_name} | {ff['check_hz']:.2f} | {ff_ratio:.3f}x | "
                f"{fr['check_hz']:.2f} | {fr_ratio:.3f}x |"
            )
        lines.append("")

    lines.append("## Notes\n")
    lines.append(
        "- `assets_ur10e` should match `json_ur10e` within run-to-run noise. "
        "A significant gap there would mean the model-swap path is not "
        "faithful and any ur12e numbers are suspect."
    )
    lines.append(
        "- `assets_ur12e` uses reduced-polygon collision meshes. The "
        "expectation is a measurable speedup; if there is none, the "
        "PyBullet broadphase or the convex-hull cache is already "
        "saturating before triangle count matters."
    )
    lines.append(
        "- Collision counts are reported per mode; identical configs are "
        "fed to all variants, so identical collision counts also serve as a "
        "consistency check between models."
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
        default=2000,
        help="Number of collision checks per variant per mode",
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
        default=os.path.join(HERE, "bullet_collision_robot_model_comparison.md"),
        help="Markdown report output path",
    )
    p.add_argument(
        "--no-write", action="store_true", help="Skip writing the markdown report"
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()

    print("Loading baseline cell (json_ur10e) ...")
    json_cell, json_state = _load_json_cell()
    lower, upper = _joint_limits(json_cell)

    configs = _sample_configs(args.checks, lower, upper, args.seed)
    print(f"Generated {len(configs)} shared random configs " f"(seed={args.seed}).")

    variants: list[tuple[str, callable]] = [
        ("json_ur10e", lambda: (json_cell, json_state)),
        (
            "assets_ur10e",
            lambda: _build_asset_variant(os.path.join(ASSETS_DIR, "ur10e_robot")),
        ),
        (
            "assets_ur12e",
            lambda: _build_asset_variant(os.path.join(ASSETS_DIR, "ur12e_robot")),
        ),
    ]

    results: list[dict] = []
    for label, build in variants:
        for full_report in (False, True):
            mode = "full_report" if full_report else "fail_fast"
            print(f"\n--- {label} / {mode} ---")
            cell, state = build()
            r = _run_variant(label, cell, state, configs, full_report)
            results.append(r)
            print(
                f"  setup_s={r['setup_s']:.3f}  total_s={r['total_s']:.3f}  "
                f"check_hz={r['check_hz']:.2f}  mean_ms={r['mean_cc_ms']:.4f}  "
                f"collisions={r['collisions']}/{r['checks']}"
            )

    print("\n=== Summary ===")
    _print_table(results)

    if not args.no_write:
        _write_markdown(results, args, args.out)
        print(f"\nWrote markdown report to {args.out}")


if __name__ == "__main__":
    main()
