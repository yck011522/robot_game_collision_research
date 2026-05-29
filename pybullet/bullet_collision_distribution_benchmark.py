"""Static vs dynamic config distribution for multi-instance PyBullet (v3).

Builds on the result that ProcessPoolExecutor with persistent workers gives
the best throughput.  This benchmark isolates ONE variable: how configurations
are distributed to workers.

================================================================
ENGINEERING DECISIONS  (read these before interpreting results)
================================================================

1. BASELINE FROM v2: process backend wins decisively.  This script does
   NOT re-test threads.  Process pool only.

2. WORKLOAD: 5000 unique configurations, pre-generated deterministically
   from a step-indexed sinusoidal trajectory.  Identical inputs across
   all strategies and all N values.

3. STRATEGIES COMPARED (per N value):
   - static:   `N` chunks of `ceil(5000/N)` configs.  One submit per worker.
               Equivalent to v2 behaviour.  Best case for IPC cost,
               worst case for load balance.
   - dyn-100:  `ceil(5000/100)` = 50 chunks of up to 100 configs.
               Workers pull from a shared queue.  Moderate IPC cost,
               good load balance.
   - dyn-10:   `500` chunks of up to 10 configs.  Aggressive dynamic.
               Higher IPC cost, near-perfect load balance.
   - dyn-1:    `5000` chunks of 1 config.  Pathological: IPC dominates.
               Included to show the overhead floor.

   Dynamic strategies use `executor.map(batch_fn, chunks)` with many more
   chunks than workers; ProcessPoolExecutor's internal work queue gives
   true work-stealing behaviour.

4. TIMING: only `executor.map(...) -> list(...)` is timed (dispatch +
   IPC + worker compute + result gather).  Setup is excluded via the
   executor `initializer` and a warmup ping round.  This matches the
   v2 methodology so numbers are directly comparable.

5. FAIRNESS: the executor is recreated for every (N, strategy) pair so
   that warmup state does not leak between cases.  This adds setup time
   to the *wall clock* of the script but NOT to the measured numbers.

6. EXPECTED RESULTS:
   - At low N: all strategies similar (no load imbalance to fix).
   - At moderate N: dynamic strategies should match or slightly beat
     static when the workload has per-config cost variance.  Pure IPC
     overhead may make dyn-1 the worst.
   - At high N: load imbalance grows with static (some workers finish
     early); dynamic should pull ahead UNLESS IPC overhead dominates.
   - dyn-1 should consistently be the worst (each config is a separate
     IPC round trip).

7. OBSERVED RESULTS (5000 configs, 12-core Win11 host):
   - Peak throughput identical: dyn-10 @ N=10 = 886.9 hz vs static @
     N=10 = 880.4 hz.  Dynamic gives no peak improvement here.
   - dyn-1 is consistently the worst (~5-10% slower than static).
   - dyn-10 / dyn-100 occasionally beat static by 7-10% at specific N
     (6, 7, 13, 16) but lose at others (5, 8, 12) by similar margins.
   - Crossover rule of thumb: dynamic helps only when static chunks fall
     below ~300 configs/worker (i.e. small total workloads or very high N).
   - At 5000 / 16 = ~312 configs per static chunk, the workloads are
     already big enough that per-config cost variance averages out.
   - High-N throughput drop (>11 workers on 12-logical-core host) is a
     worker-saturation effect, not a distribution issue.  All strategies
     degrade together past the physical-core count.

8. RESULT FILE: a markdown file with the same basename is written next
   to this script.  Re-running OVERWRITES it.

================================================================

Usage
-----
    conda activate game
    python pybullet/bullet_collision_distribution_benchmark.py
    python pybullet/bullet_collision_distribution_benchmark.py --instances 1,4,8,16
    python pybullet/bullet_collision_distribution_benchmark.py --chunk-sizes 1,10,50,250
"""

from __future__ import annotations

import argparse
import math
import multiprocessing
import os
import platform
import statistics
import time
from concurrent.futures import ProcessPoolExecutor
from datetime import datetime

# CRITICAL: import compas_fab before any code that could trigger a bare
# `import pybullet` — the LazyLoader inside compas_fab only activates when
# pybullet is NOT yet in sys.modules.
from compas.data import json_load
from compas_fab.backends import PyBulletClient, PyBulletPlanner
from compas_fab.backends.exceptions import CollisionCheckError

HERE = os.path.dirname(os.path.abspath(__file__))
JSON_PATH = os.path.join(HERE, "robot_cell_and_state.json")
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
# Scene loading
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


# ---------------------------------------------------------------------------
# Deterministic config generator
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
# Worker state
# ---------------------------------------------------------------------------

_PROC_STATE: dict = {}


def _proc_initializer() -> None:
    class _NS:
        pass

    ns = _NS()
    robot_cell, robot_cell_state, _, _ = _load_scene()
    client = PyBulletClient(connection_type="direct", verbose=False)
    client.__enter__()
    planner = PyBulletPlanner(client)
    planner.set_robot_cell(robot_cell)
    planner.set_robot_cell_state(robot_cell_state)

    ns.client = client
    ns.planner = planner
    ns.robot_cell_state = robot_cell_state
    ns.cfg = robot_cell_state.robot_configuration.copy()

    # Warmup
    try:
        planner.check_collision(robot_cell_state, options={"verbose": False})
    except CollisionCheckError:
        pass

    _PROC_STATE["ns"] = ns


def _proc_ping(_):
    return os.getpid()


def _proc_check_batch(configs):
    """Run collision checks on a list of configs.  Returns (count, collisions, worker_s, pid)."""
    ns = _PROC_STATE["ns"]
    planner = ns.planner
    rcs = ns.robot_cell_state
    cfg = ns.cfg

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


def _split_into_chunks(configs: list, chunk_size: int) -> list:
    """Split into chunks of size `chunk_size` (last chunk may be smaller)."""
    return [configs[i : i + chunk_size] for i in range(0, len(configs), chunk_size)]


# ---------------------------------------------------------------------------
# Per-case runner
# ---------------------------------------------------------------------------


def run_case(n_workers: int, configs: list, chunk_size: int) -> dict:
    """Run one (N, chunk_size) case.  Setup excluded; only dispatch+wait is timed."""
    chunks = _split_into_chunks(configs, chunk_size)
    n_chunks = len(chunks)
    # Use min(n_workers, n_chunks) workers since extra workers would never get work.
    n_real_workers = min(n_workers, n_chunks)

    executor = ProcessPoolExecutor(
        max_workers=n_real_workers, initializer=_proc_initializer
    )
    try:
        # Force every worker to complete setup BEFORE we start the timer.
        _ = list(executor.map(_proc_ping, range(n_real_workers * 3)))

        # ----- TIMED REGION -----
        t_start = time.perf_counter()
        results = list(executor.map(_proc_check_batch, chunks))
        wall_s = time.perf_counter() - t_start
        # ----- END TIMED REGION -----
    finally:
        executor.shutdown(wait=True)

    total_checks = sum(r[0] for r in results)
    total_collisions = sum(r[1] for r in results)

    # Aggregate per-worker compute time by pid (workers handle multiple chunks
    # in dynamic strategies).
    per_worker_s: dict = {}
    for _cnt, _col, ws, pid in results:
        per_worker_s[pid] = per_worker_s.get(pid, 0.0) + ws
    worker_secs = list(per_worker_s.values())

    return {
        "n": n_workers,
        "n_real_workers": n_real_workers,
        "chunk_size": chunk_size,
        "n_chunks": n_chunks,
        "total_checks": total_checks,
        "collisions": total_collisions,
        "wall_s": wall_s,
        "total_hz": total_checks / wall_s if wall_s > 0 else 0.0,
        "per_inst_hz": (total_checks / wall_s / n_real_workers) if wall_s > 0 else 0.0,
        "worker_wall_max_s": max(worker_secs) if worker_secs else 0.0,
        "worker_wall_min_s": min(worker_secs) if worker_secs else 0.0,
        "imbalance_pct": (
            (max(worker_secs) - min(worker_secs)) / max(worker_secs) * 100.0
            if worker_secs and max(worker_secs) > 0
            else 0.0
        ),
    }


# ---------------------------------------------------------------------------
# Strategy descriptors
# ---------------------------------------------------------------------------


def build_strategies(total_configs: int, n_workers: int, chunk_sizes: list) -> list:
    """Return [(label, chunk_size), ...] for this N.

    - 'static'   uses ceil(total/N) -> exactly N chunks
    - 'dyn-K'    uses fixed K -> ceil(total/K) chunks (dynamic if > N)
    """
    static_cs = math.ceil(total_configs / max(1, n_workers))
    out = [("static", static_cs)]
    for k in chunk_sizes:
        out.append(("dyn-{}".format(k), k))
    return out


# ---------------------------------------------------------------------------
# Output formatting
# ---------------------------------------------------------------------------


def _print_grid(rows_by_strategy: dict, n_values: list, total_configs: int) -> None:
    print("\n--- total_hz  (rows = N workers, columns = strategy) ---")
    strategies = list(rows_by_strategy.keys())
    header = "  {:>3}".format("N") + "".join("  {:>10}".format(s) for s in strategies)
    print(header)
    print("  " + "-" * (len(header) - 2))
    for n in n_values:
        cells = []
        for s in strategies:
            r = next((x for x in rows_by_strategy[s] if x["n"] == n), None)
            cells.append(
                "{:>10.1f}".format(r["total_hz"]) if r else "{:>10}".format("-")
            )
        print("  {:>3}".format(n) + "".join("  " + c for c in cells))


# ---------------------------------------------------------------------------
# Markdown report
# ---------------------------------------------------------------------------


def _md_strategy_table(rows: list) -> str:
    if not rows:
        return "_(no data)_\n"
    single_hz = rows[0]["total_hz"]
    lines = [
        "| N | chunks | total_hz | per_inst_hz | speedup | wall_s | imbal% |",
        "|--:|-------:|---------:|------------:|--------:|-------:|-------:|",
    ]
    for r in rows:
        spd = r["total_hz"] / single_hz if single_hz else 1.0
        lines.append(
            "| {n} | {nc} | {hz:.1f} | {pinst:.1f} | {spd:.2f}x | {wall:.2f} | {imb:.1f}% |".format(
                n=r["n"],
                nc=r["n_chunks"],
                hz=r["total_hz"],
                pinst=r["per_inst_hz"],
                spd=spd,
                wall=r["wall_s"],
                imb=r["imbalance_pct"],
            )
        )
    return "\n".join(lines) + "\n"


def _md_throughput_grid(rows_by_strategy: dict, n_values: list) -> str:
    strategies = list(rows_by_strategy.keys())
    header = "| N | " + " | ".join(strategies) + " |"
    sep = "|--:|" + "|".join(["--------:"] * len(strategies)) + "|"
    lines = [header, sep]
    for n in n_values:
        cells = []
        for s in strategies:
            r = next((x for x in rows_by_strategy[s] if x["n"] == n), None)
            cells.append("{:.1f}".format(r["total_hz"]) if r else "-")
        lines.append("| {} | ".format(n) + " | ".join(cells) + " |")
    return "\n".join(lines) + "\n"


def _md_winner_grid(rows_by_strategy: dict, n_values: list) -> str:
    strategies = list(rows_by_strategy.keys())
    lines = [
        "| N | best strategy | total_hz | margin vs static |",
        "|--:|:--------------|---------:|-----------------:|",
    ]
    for n in n_values:
        best_label = None
        best_hz = -1.0
        static_hz = None
        for s in strategies:
            r = next((x for x in rows_by_strategy[s] if x["n"] == n), None)
            if r is None:
                continue
            if r["total_hz"] > best_hz:
                best_hz = r["total_hz"]
                best_label = s
            if s == "static":
                static_hz = r["total_hz"]
        margin = ""
        if static_hz is not None and best_hz > 0:
            pct = (best_hz - static_hz) / static_hz * 100.0
            margin = "{:+.1f}%".format(pct)
        lines.append("| {} | {} | {:.1f} | {} |".format(n, best_label, best_hz, margin))
    return "\n".join(lines) + "\n"


def write_report(
    path: str,
    rows_by_strategy: dict,
    n_values: list,
    total_configs: int,
    speed: float,
    dt: float,
) -> None:
    lines = []
    lines.append("# Static vs dynamic config distribution\n")
    lines.append("Generated: {}\n".format(datetime.now().isoformat(timespec="seconds")))

    lines.append("\n## Host\n")
    lines.append("- Platform: `{}`".format(platform.platform()))
    lines.append("- Python: `{}`".format(platform.python_version()))
    lines.append("- CPU logical cores: `{}`".format(multiprocessing.cpu_count()))
    lines.append("- Processor: `{}`\n".format(platform.processor() or "n/a"))

    lines.append("\n## Workload\n")
    lines.append("- Total unique configs: **{}**".format(total_configs))
    lines.append("- N values tested: `{}`".format(n_values))
    lines.append("- Backend: `ProcessPoolExecutor` only (threads ruled out by v2)")
    lines.append("- Strategies: `{}`".format(list(rows_by_strategy.keys())))
    lines.append(
        "- Trajectory: step-indexed sinusoidal `mid + amp*sin(speed*t + i*0.9)` "
        "with `speed={}` rad/s, `dt={}` s".format(speed, dt)
    )
    lines.append("- Collision mode: `fail_fast`")
    lines.append("- Setup excluded from timing (initializer + warmup ping round)\n")

    lines.append("\n## Throughput grid (checks/s)\n")
    lines.append(_md_throughput_grid(rows_by_strategy, n_values))

    lines.append("\n## Best strategy per N\n")
    lines.append(_md_winner_grid(rows_by_strategy, n_values))

    lines.append("\n## Detailed results per strategy\n")
    for label, rows in rows_by_strategy.items():
        lines.append("### {}\n".format(label))
        lines.append(_md_strategy_table(rows))

    # Global summary
    overall_best = None
    overall_best_label = None
    overall_best_n = None
    for label, rows in rows_by_strategy.items():
        for r in rows:
            if overall_best is None or r["total_hz"] > overall_best:
                overall_best = r["total_hz"]
                overall_best_label = label
                overall_best_n = r["n"]

    static_rows = rows_by_strategy.get("static", [])
    static_best = max((r["total_hz"] for r in static_rows), default=0.0)

    lines.append("\n## Summary\n")
    if overall_best:
        lines.append(
            "- **Overall best:** `{label}` @ N={n} -> {hz:.1f} checks/s".format(
                label=overall_best_label,
                n=overall_best_n,
                hz=overall_best,
            )
        )
    if static_best > 0:
        lines.append("- **Static best:** {:.1f} checks/s".format(static_best))
        if overall_best:
            delta = (overall_best - static_best) / static_best * 100.0
            lines.append(
                "- **Dynamic vs static:** {:+.1f}%  ({})".format(
                    delta,
                    "dynamic wins" if delta > 0 else "static is fine",
                )
            )

    lines.append("\n## Notes\n")
    lines.append(
        "- `chunks` = number of work units submitted.  For `static` it equals N. "
        "For `dyn-K` it equals `ceil(total_configs / K)`."
    )
    lines.append(
        "- `imbal%` = `(max_worker - min_worker) / max_worker * 100`, summed across "
        "all chunks each worker processed.  Dynamic strategies should drive this "
        "toward 0% as chunk size shrinks."
    )
    lines.append(
        "- Very small chunks (e.g. `dyn-1`) make every config a separate IPC round "
        "trip.  Pickle/unpickle of 6 floats per task is cheap but Python-side "
        "scheduler bookkeeping dominates at this scale."
    )
    lines.append(
        "- The expected sweet spot is the largest chunk that still gives "
        "**chunks >> workers** (rule of thumb: 4-10 chunks per worker is enough "
        "for good balancing without much IPC penalty)."
    )

    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines).rstrip() + "\n")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _parse_args():
    p = argparse.ArgumentParser(
        description="Static vs dynamic config distribution benchmark",
    )
    p.add_argument(
        "--total-configs",
        type=int,
        default=5000,
        help="Total unique configurations (default 5000)",
    )
    p.add_argument(
        "--max-instances", type=int, default=16, help="Test N = 1 .. max (default 16)"
    )
    p.add_argument(
        "--instances",
        type=str,
        default=None,
        help="Explicit comma-separated N values (overrides --max-instances)",
    )
    p.add_argument(
        "--chunk-sizes",
        type=str,
        default="100,10,1",
        help="Comma-separated dynamic chunk sizes (default 100,10,1). "
        "'static' is always included.",
    )
    p.add_argument(
        "--speed", type=float, default=0.6, help="Trajectory speed rad/s (default 0.6)"
    )
    p.add_argument(
        "--dt", type=float, default=0.01, help="Trajectory dt s (default 0.01)"
    )
    p.add_argument(
        "--no-report", action="store_true", help="Skip writing the markdown report"
    )
    return p.parse_args()


def main():
    args = _parse_args()

    if args.instances:
        n_values = [int(x.strip()) for x in args.instances.split(",")]
    else:
        n_values = list(range(1, args.max_instances + 1))

    chunk_sizes = [int(x.strip()) for x in args.chunk_sizes.split(",") if x.strip()]

    print(
        "\n=== Static vs Dynamic Distribution Benchmark ===\n"
        "  CPU logical cores : {cpu}\n"
        "  total configs     : {tc}\n"
        "  N values          : {ns}\n"
        "  dynamic chunks    : {cs}  (+ 'static' = N chunks of ceil(total/N))\n"
        "  backend           : ProcessPoolExecutor\n"
        "  trajectory        : sinusoidal step-indexed  speed={spd}  dt={dt}\n"
        "  collision mode    : fail_fast\n"
        "  setup time        : EXCLUDED via initializer + warmup ping round\n".format(
            cpu=multiprocessing.cpu_count(),
            tc=args.total_configs,
            ns=n_values,
            cs=chunk_sizes,
            spd=args.speed,
            dt=args.dt,
        )
    )

    print("Generating {} unique configs in main process ...".format(args.total_configs))
    configs = generate_configs(args.total_configs, args.speed, args.dt)
    print("  done.\n")

    rows_by_strategy: dict = {"static": []}
    for k in chunk_sizes:
        rows_by_strategy["dyn-{}".format(k)] = []

    for n in n_values:
        print("==> N={}".format(n))
        strategies = build_strategies(args.total_configs, n, chunk_sizes)
        for label, cs in strategies:
            n_chunks = math.ceil(args.total_configs / cs)
            print(
                "  {:<10s} (chunk_size={:>4d}, n_chunks={:>4d}) ... ".format(
                    label, cs, n_chunks
                ),
                end="",
                flush=True,
            )
            r = run_case(n, configs, cs)
            rows_by_strategy[label].append(r)
            print(
                "{hz:>8.1f} hz  (wall {wall:>5.2f}s  imb {imb:>4.1f}%)".format(
                    hz=r["total_hz"],
                    wall=r["wall_s"],
                    imb=r["imbalance_pct"],
                )
            )
        print()

    _print_grid(rows_by_strategy, n_values, args.total_configs)

    # Per-N winner summary
    print("\n--- best strategy per N ---")
    strategies = list(rows_by_strategy.keys())
    for n in n_values:
        best_label = None
        best_hz = -1.0
        static_hz = None
        for s in strategies:
            r = next((x for x in rows_by_strategy[s] if x["n"] == n), None)
            if r is None:
                continue
            if r["total_hz"] > best_hz:
                best_hz = r["total_hz"]
                best_label = s
            if s == "static":
                static_hz = r["total_hz"]
        margin = ""
        if static_hz is not None and best_hz > 0:
            pct = (best_hz - static_hz) / static_hz * 100.0
            margin = "  ({:+.1f}% vs static)".format(pct)
        print(
            "  N={:>2}  best={:<10s}  {:>7.1f} hz{}".format(
                n, best_label, best_hz, margin
            )
        )

    if not args.no_report:
        write_report(
            REPORT_PATH,
            rows_by_strategy,
            n_values=n_values,
            total_configs=args.total_configs,
            speed=args.speed,
            dt=args.dt,
        )
        print("\nMarkdown report written: {}".format(REPORT_PATH))

    print()


if __name__ == "__main__":
    main()
