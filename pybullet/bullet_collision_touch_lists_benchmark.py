"""Compare collision-check throughput with vs without discovered touch-lists.

Hypothesis
----------
The discovery script (``bullet_collision_pair_discovery.py``) lists, for every
rigid body and tool, the robot links / other bodies it was never observed
colliding with.  Loading those names into the corresponding
``RigidBodyState.touch_links`` / ``RigidBodyState.touch_bodies`` should let
compas_fab's PyBullet planner skip those checks and run faster.

This benchmark verifies that hypothesis on an equal workload.

Method
------
- Generate 5000 unique configs deterministically (sinusoidal trajectory,
  identical to the multi-instance benchmark so numbers are comparable).
- Run twice, once per scenario:
    A) baseline   : robot cell state as loaded from JSON, no extra
                    touch lists.
    B) with_touch : the same state, but for every rigid body / tool we
                    fill in ``touch_links`` / ``touch_bodies`` from
                    ``bullet_collision_pair_discovery.json``.
- Both scenarios use:
    - ProcessPoolExecutor with N workers (default 12)
    - Chunks of 10 configs each (~500 chunks per scenario)
    - ``fail_fast`` collision mode (matches production)
- Setup (client + cell load + warmup) is excluded from the timed region
  via the executor ``initializer`` and a warmup ping round.

Sanity check
------------
Touch lists in scenario B contain only pairs that the discovery run
*never observed*. So the number of configs flagged as colliding must be
exactly the same in A and B. If it isn't, the touch lists are too
aggressive -- the script flags a mismatch.

Usage
-----
    conda activate game
    python pybullet/bullet_collision_touch_lists_benchmark.py
    python pybullet/bullet_collision_touch_lists_benchmark.py --total-configs 10000 --workers 10
"""

from __future__ import annotations

import argparse
import json
import math
import multiprocessing
import os
import platform
import statistics
import time
from concurrent.futures import ProcessPoolExecutor
from datetime import datetime

from compas.data import json_load
from compas_fab.backends import PyBulletClient, PyBulletPlanner
from compas_fab.backends.exceptions import CollisionCheckError

HERE = os.path.dirname(os.path.abspath(__file__))
JSON_PATH = os.path.join(HERE, "robot_cell_and_state.json")
DISCOVERY_PATH = os.path.join(HERE, "bullet_collision_pair_discovery.json")
SCRIPT_BASENAME = os.path.splitext(os.path.basename(__file__))[0]
REPORT_PATH = os.path.join(HERE, SCRIPT_BASENAME + ".md")

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


# ---------------------------------------------------------------------------
# Scene loading + touch-list patching
# ---------------------------------------------------------------------------


def _load_scene():
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


def _load_discovery():
    with open(DISCOVERY_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


def _apply_touch_lists(robot_cell_state, discovery: dict) -> dict:
    """Mutate robot_cell_state in place; return stats about what was applied."""
    per_body = discovery.get("per_rigid_body", {})
    per_tool = discovery.get("per_tool", {})

    n_bodies_patched = 0
    n_tools_patched = 0
    total_touch_links = 0
    total_touch_bodies = 0

    for key, info in per_body.items():
        state = robot_cell_state.rigid_body_states.get(key)
        if state is None:
            continue
        tl = list(info.get("touch_links_candidates", []))
        tb = list(info.get("touch_bodies_candidates", []))
        state.touch_links = tl
        state.touch_bodies = tb
        n_bodies_patched += 1
        total_touch_links += len(tl)
        total_touch_bodies += len(tb)

    if hasattr(robot_cell_state, "tool_states"):
        for key, info in per_tool.items():
            state = robot_cell_state.tool_states.get(key)
            if state is None:
                continue
            tl = list(info.get("touch_links_candidates", []))
            tb = list(info.get("touch_bodies_candidates", []))
            # ToolState may or may not have touch_bodies; set both defensively.
            if hasattr(state, "touch_links"):
                state.touch_links = tl
            if hasattr(state, "touch_bodies"):
                state.touch_bodies = tb
            n_tools_patched += 1
            total_touch_links += len(tl)
            total_touch_bodies += len(tb)

    return {
        "n_bodies_patched": n_bodies_patched,
        "n_tools_patched": n_tools_patched,
        "total_touch_links": total_touch_links,
        "total_touch_bodies": total_touch_bodies,
    }


# ---------------------------------------------------------------------------
# Deterministic config generator (same shape as multi-instance benchmark)
# ---------------------------------------------------------------------------


def _config_at_step(step, lower, upper, speed, dt):
    t = step * dt
    return [
        0.5 * (lower[i] + upper[i])
        + 0.45 * (upper[i] - lower[i]) * math.sin(speed * t + i * 0.9)
        for i in range(6)
    ]


def generate_configs(n: int, speed: float, dt: float) -> list:
    _, _, lower, upper = _load_scene()
    return [_config_at_step(i, lower, upper, speed, dt) for i in range(n)]


# ---------------------------------------------------------------------------
# Worker state (per process)
# ---------------------------------------------------------------------------

_W: dict = {}
_APPLY_TOUCH: bool = False  # set per-pool via initializer arg


def _proc_init(apply_touch: bool):
    robot_cell, robot_cell_state, _, _ = _load_scene()

    if apply_touch:
        discovery = _load_discovery()
        _apply_touch_lists(robot_cell_state, discovery)

    client = PyBulletClient(connection_type="direct", verbose=False)
    client.__enter__()
    planner = PyBulletPlanner(client)
    planner.set_robot_cell(robot_cell)
    planner.set_robot_cell_state(robot_cell_state)

    # Warmup
    try:
        planner.check_collision(robot_cell_state, options={"verbose": False})
    except CollisionCheckError:
        pass

    _W["client"] = client
    _W["planner"] = planner
    _W["robot_cell_state"] = robot_cell_state
    _W["cfg"] = robot_cell_state.robot_configuration.copy()


def _proc_ping(_):
    return os.getpid()


def _proc_check_batch(configs):
    planner = _W["planner"]
    rcs = _W["robot_cell_state"]
    cfg = _W["cfg"]

    collisions = 0
    t0 = time.perf_counter()
    for q in configs:
        cfg.joint_values = q
        rcs.robot_configuration = cfg
        try:
            planner.check_collision(rcs, options={"verbose": False})
        except CollisionCheckError:
            collisions += 1
    worker_s = time.perf_counter() - t0
    return len(configs), collisions, worker_s, os.getpid()


# ---------------------------------------------------------------------------
# Chunking
# ---------------------------------------------------------------------------


def _split(configs: list, chunk_size: int) -> list:
    return [configs[i : i + chunk_size] for i in range(0, len(configs), chunk_size)]


# ---------------------------------------------------------------------------
# One scenario
# ---------------------------------------------------------------------------


def run_scenario(
    label: str, configs: list, n_workers: int, chunk_size: int, apply_touch: bool
) -> dict:
    chunks = _split(configs, chunk_size)
    n_chunks = len(chunks)

    executor = ProcessPoolExecutor(
        max_workers=n_workers,
        initializer=_proc_init,
        initargs=(apply_touch,),
    )
    try:
        # Force every worker through setup before the timed region.
        # Many pings so the OS scheduler reaches every process.
        _ = list(executor.map(_proc_ping, range(n_workers * 3)))

        # ----- TIMED REGION -----
        t0 = time.perf_counter()
        results = list(executor.map(_proc_check_batch, chunks))
        wall_s = time.perf_counter() - t0
        # ----- END TIMED REGION -----
    finally:
        executor.shutdown(wait=True)

    total_checks = sum(r[0] for r in results)
    total_collisions = sum(r[1] for r in results)

    per_worker_s: dict = {}
    for _cnt, _col, ws, pid in results:
        per_worker_s[pid] = per_worker_s.get(pid, 0.0) + ws
    worker_secs = list(per_worker_s.values())

    return {
        "label": label,
        "apply_touch": apply_touch,
        "n_workers": n_workers,
        "chunk_size": chunk_size,
        "n_chunks": n_chunks,
        "total_checks": total_checks,
        "collisions": total_collisions,
        "wall_s": wall_s,
        "total_hz": total_checks / wall_s if wall_s > 0 else 0.0,
        "per_inst_hz": (total_checks / wall_s / n_workers) if wall_s > 0 else 0.0,
        "worker_wall_min_s": min(worker_secs) if worker_secs else 0.0,
        "worker_wall_max_s": max(worker_secs) if worker_secs else 0.0,
        "worker_wall_mean_s": statistics.fmean(worker_secs) if worker_secs else 0.0,
        "imbalance_pct": (
            (max(worker_secs) - min(worker_secs)) / max(worker_secs) * 100.0
            if worker_secs and max(worker_secs) > 0
            else 0.0
        ),
    }


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------


def _print_result(r: dict):
    print(
        "\n  [{lbl}]  total_hz={hz:.1f}   per_inst={pi:.1f}   wall={w:.2f}s   "
        "collisions={c}/{n}   imbal={ib:.1f}%".format(
            lbl=r["label"],
            hz=r["total_hz"],
            pi=r["per_inst_hz"],
            w=r["wall_s"],
            c=r["collisions"],
            n=r["total_checks"],
            ib=r["imbalance_pct"],
        )
    )


def _write_sweep_report(
    path, pairs, total_configs, chunk_size, patch_stats, discovery_meta
):
    """pairs: list of (n, baseline_result, with_touch_result, sanity_msg)."""
    lines = []
    lines.append(
        "# Touch-list benchmark: baseline vs discovered touch lists (N sweep)\n"
    )
    lines.append("Generated: {}\n".format(datetime.now().isoformat(timespec="seconds")))

    lines.append("\n## Host\n")
    lines.append("- Platform: `{}`".format(platform.platform()))
    lines.append("- Python: `{}`".format(platform.python_version()))
    lines.append("- CPU logical cores: `{}`".format(multiprocessing.cpu_count()))
    lines.append("- Processor: `{}`\n".format(platform.processor() or "n/a"))

    lines.append("\n## Workload\n")
    lines.append("- Total unique configs: **{}** per scenario".format(total_configs))
    lines.append("- N sweep: `{}`".format([p[0] for p in pairs]))
    lines.append(
        "- Chunk size: **{}**  ({} chunks)".format(
            chunk_size, math.ceil(total_configs / chunk_size)
        )
    )
    lines.append("- Backend: `ProcessPoolExecutor`, `fail_fast` collision mode")
    lines.append("- Setup excluded from timing (initializer + warmup pings)\n")

    lines.append("\n## Discovery source\n")
    lines.append("- File: `{}`".format(os.path.basename(DISCOVERY_PATH)))
    if discovery_meta:
        lines.append(
            "- Discovery run: workers={w}, duration/worker={d:.1f}s, "
            "total_checks={tc}, distinct pairs={dp}".format(
                w=discovery_meta.get("workers"),
                d=discovery_meta.get("duration_per_worker_s", 0.0),
                tc=discovery_meta.get("totals", {}).get("n_checks"),
                dp=discovery_meta.get("totals", {}).get("distinct_pairs"),
            )
        )
    lines.append("- Bodies patched: **{}**".format(patch_stats["n_bodies_patched"]))
    lines.append("- Tools patched: **{}**".format(patch_stats["n_tools_patched"]))
    lines.append(
        "- Total `touch_links` entries added: **{}**".format(
            patch_stats["total_touch_links"]
        )
    )
    lines.append(
        "- Total `touch_bodies` entries added: **{}**".format(
            patch_stats["total_touch_bodies"]
        )
    )

    lines.append("\n## Sweep results\n")
    lines.append(
        "| N | base_hz | touch_hz | speedup | base_wall_s | touch_wall_s | wall_save | sanity |"
    )
    lines.append(
        "|--:|--------:|---------:|--------:|------------:|-------------:|----------:|:-------|"
    )
    for n, b, t, sanity in pairs:
        spd = t["total_hz"] / b["total_hz"] if b["total_hz"] > 0 else 0
        saved = (1.0 - t["wall_s"] / b["wall_s"]) * 100.0 if b["wall_s"] > 0 else 0
        ok = "OK" if sanity.startswith("OK") else sanity
        lines.append(
            "| {n} | {bh:.1f} | {th:.1f} | {spd:.2f}x | {bw:.2f} | {tw:.2f} | {sv:+.1f}% | {ok} |".format(
                n=n,
                bh=b["total_hz"],
                th=t["total_hz"],
                spd=spd,
                bw=b["wall_s"],
                tw=t["wall_s"],
                sv=saved,
                ok=ok,
            )
        )

    lines.append("\n## Per-N detail\n")
    for n, b, t, sanity in pairs:
        lines.append("\n### N = {}\n".format(n))
        lines.append("- Sanity: {}".format(sanity))
        lines.append(
            "| scenario | total_hz | per_inst_hz | wall_s | collisions | imbal% |"
        )
        lines.append(
            "|:---------|---------:|------------:|-------:|-----------:|-------:|"
        )
        for r in (b, t):
            lines.append(
                "| {lbl} | {hz:.1f} | {pi:.1f} | {w:.2f} | {c} | {ib:.1f}% |".format(
                    lbl=r["label"],
                    hz=r["total_hz"],
                    pi=r["per_inst_hz"],
                    w=r["wall_s"],
                    c=r["collisions"],
                    ib=r["imbalance_pct"],
                )
            )

    # Pick best/worst speedup
    speedups = [
        (n, (t["total_hz"] / b["total_hz"]) if b["total_hz"] > 0 else 0)
        for n, b, t, _ in pairs
    ]
    best_n, best_spd = max(speedups, key=lambda x: x[1])
    worst_n, worst_spd = min(speedups, key=lambda x: x[1])

    lines.append("\n## Summary\n")
    lines.append("- **Best speedup:** {:.2f}x at N={}".format(best_spd, best_n))
    lines.append("- **Worst speedup:** {:.2f}x at N={}".format(worst_spd, worst_n))
    lines.append(
        "- A speedup < 1.0 means the touch-list bookkeeping cost exceeds "
        "the pair-check cost it saves for that worker count."
    )

    lines.append("\n## Notes\n")
    lines.append(
        "- The `with_touch` scenario fills in `RigidBodyState.touch_links` / "
        "`touch_bodies` for every rigid body and tool using the `*_candidates` "
        "lists in the discovery JSON. These are pairs the discovery run never "
        "observed colliding, so they are skipped by the planner."
    )
    lines.append(
        "- Because every skipped pair was previously NEVER observed in collision, "
        "the count of colliding configurations should match between the two "
        "scenarios. A mismatch (`sanity != OK`) indicates the touch lists are "
        "too aggressive — rerun discovery for longer before trusting them."
    )
    lines.append(
        "- The N sweep isolates whether the touch-list cost/benefit ratio depends "
        "on worker count. It should not, in theory — both scenarios pay the same "
        "per-worker overhead — but if the per-call cost is dominated by Python "
        "scheduling, sweep behaviour can diverge."
    )

    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines).rstrip() + "\n")


# ---------------------------------------------------------------------------
# CLI / main
# ---------------------------------------------------------------------------


def _parse_args():
    p = argparse.ArgumentParser(description="Touch-list collision benchmark")
    p.add_argument("--total-configs", type=int, default=5000)
    p.add_argument(
        "--workers",
        type=str,
        default="2,4,6,8,10,12",
        help="Comma-separated list of worker counts to sweep (default 2,4,6,8,10,12). "
        "Pass a single integer to test only one N.",
    )
    p.add_argument("--chunk-size", type=int, default=10)
    p.add_argument("--speed", type=float, default=0.6)
    p.add_argument("--dt", type=float, default=0.01)
    return p.parse_args()


def main():
    args = _parse_args()

    if not os.path.exists(DISCOVERY_PATH):
        raise SystemExit(
            "Discovery JSON not found: {}\n  Run bullet_collision_pair_discovery.py first.".format(
                DISCOVERY_PATH
            )
        )

    n_values = [int(x.strip()) for x in args.workers.split(",") if x.strip()]

    discovery = _load_discovery()
    discovery_meta = discovery.get("meta", {})

    # Patch stats in main (workers do the same work, but we report once).
    _, rcs_for_stats, _, _ = _load_scene()
    patch_stats = _apply_touch_lists(rcs_for_stats, discovery)

    print(
        "\n=== touch-list benchmark (N sweep) ===\n"
        "  workers sweep : {ns}\n"
        "  configs       : {tc}  (chunk_size={cs} -> {nc} chunks)\n"
        "  discovery file: {f}\n"
        "  patched       : {nb} bodies, {nt} tools, "
        "{tl} touch_links, {tb} touch_bodies\n".format(
            ns=n_values,
            tc=args.total_configs,
            cs=args.chunk_size,
            nc=math.ceil(args.total_configs / args.chunk_size),
            f=os.path.basename(DISCOVERY_PATH),
            nb=patch_stats["n_bodies_patched"],
            nt=patch_stats["n_tools_patched"],
            tl=patch_stats["total_touch_links"],
            tb=patch_stats["total_touch_bodies"],
        )
    )

    print("Generating {} unique configs ...".format(args.total_configs))
    configs = generate_configs(args.total_configs, args.speed, args.dt)
    print("  done.")

    pairs = []  # list of (n, baseline_result, with_touch_result, sanity_msg)
    for n in n_values:
        print("\n========== N = {} ==========".format(n))

        print("  --> Scenario A: baseline")
        baseline = run_scenario(
            "baseline", configs, n, args.chunk_size, apply_touch=False
        )
        _print_result(baseline)

        print("  --> Scenario B: with discovered touch lists")
        with_touch = run_scenario(
            "with_touch", configs, n, args.chunk_size, apply_touch=True
        )
        _print_result(with_touch)

        if baseline["collisions"] == with_touch["collisions"]:
            sanity = "OK ({} colliding configs)".format(baseline["collisions"])
        else:
            sanity = "MISMATCH baseline={} touch={}".format(
                baseline["collisions"], with_touch["collisions"]
            )
        pairs.append((n, baseline, with_touch, sanity))

    # ----- print comparison table -----
    print("\n\n=== sweep comparison ===")
    print(
        "  {:>3}  {:>10}  {:>10}  {:>7}  {:>9}  {:>6}".format(
            "N", "base_hz", "touch_hz", "speedup", "wall_save", "sanity"
        )
    )
    print("  " + "-" * 56)
    for n, b, t, sanity in pairs:
        spd = t["total_hz"] / b["total_hz"] if b["total_hz"] > 0 else 0
        saved = (1.0 - t["wall_s"] / b["wall_s"]) * 100.0 if b["wall_s"] > 0 else 0
        ok = "OK" if sanity.startswith("OK") else "FAIL"
        print(
            "  {:>3d}  {:>10.1f}  {:>10.1f}  {:>6.2f}x  {:>+8.1f}%  {:>6s}".format(
                n, b["total_hz"], t["total_hz"], spd, saved, ok
            )
        )

    _write_sweep_report(
        REPORT_PATH,
        pairs,
        args.total_configs,
        args.chunk_size,
        patch_stats,
        discovery_meta,
    )
    print("\nWrote: {}".format(REPORT_PATH))


if __name__ == "__main__":
    main()
