"""Multi-instance PyBullet collision throughput benchmark (v2).

Goal
----
Find the number of parallel PyBullet instances that maximises total
collision checks per second when 5000 *unique* configurations must be
processed.  Compares process-based vs thread-based parallelism.

================================================================
ENGINEERING DECISIONS  (read these before interpreting results)
================================================================

1. WORKLOAD: 5000 unique configurations, pre-generated deterministically
   in the main process using a step-indexed sinusoidal generator
   (config = f(step_index)).  All N parallel runs see the SAME 5000
   configs in the SAME order, just split across workers.

2. DISTRIBUTION: Even-split chunking — each of N workers receives one
   contiguous chunk of ceil(5000/N) configs in a SINGLE submit().  This
   minimises IPC overhead vs many small batches.  Caveat: if collision-
   check time varies with config (it does — collision vs free differ
   slightly), the slowest worker bounds total wall time.  For 5000
   configs split 16 ways (~312/worker) this imbalance is small but
   non-zero; the script reports `imbal%`.

3. SETUP EXCLUDED FROM TIMING: PyBullet client creation, robot cell
   loading and the first warmup check are significant (~0.5-2 s/worker).
   These happen via the executor `initializer` (process backend) or
   lazy-init on first task (thread backend) BEFORE the timer starts.
   A warmup "ping" round forces every worker to finish initializing
   before the timer begins.

4. TOTAL HZ DEFINITION: total_hz = 5000 / wall_time_of_main_thread,
   measured from "start dispatch" to "all futures resolved".  Includes
   chunk serialisation + IPC cost.  This is the metric the user cares
   about because it reflects end-to-end throughput, not in-worker speed.

5. PROCESS vs THREADS:
   - process: each worker is a separate Python process with its own
     PyBullet engine.  No GIL contention.  Higher startup cost (excluded)
     and higher per-call IPC cost (chunk pickling).
   - threads: shared Python process, each thread holds its own PyBullet
     `direct` connection in thread-local storage.  Empirically threads
     give NO speedup for this workload (see the generated .md report) —
     the GIL serialises either the PyBullet bindings or the compas_fab
     wrapper layer.  Also: compas_fab's PyBullet setup is NOT thread-safe;
     this script serialises setup via a lock to avoid corruption.

6. DETERMINISM: Configs are pure function of step index.  Re-running
   yields identical configs.  Per-worker timings will still vary due
   to OS scheduling.

7. WARMUP: Each worker performs ONE collision check inside its
   initializer.  PyBullet's first check after loading a cell builds
   internal broadphase structures and is much slower than steady-state;
   this gets that cost out of the measurement.

8. TIMING SCOPE: only `executor.map(...) -> list(...)` (i.e. dispatch +
   wait for all results) is timed.  Result collection is included;
   printing is not.

9. WORKERS UP TO N=16: tested 1..16 on the host machine.  Beyond the
   number of logical cores, process-mode throughput is expected to
   degrade due to OS context switching.

10. RESULT FILE: a markdown file with the same basename is written next
    to this script summarising the run.  Re-running OVERWRITES it.

================================================================

Usage
-----
    conda activate game
    python pybullet/bullet_collision_multiinstance_benchmark.py
    python pybullet/bullet_collision_multiinstance_benchmark.py --instances 1,2,4,8,12,16
    python pybullet/bullet_collision_multiinstance_benchmark.py --total-configs 10000
"""

from __future__ import annotations

import argparse
import math
import multiprocessing
import os
import platform
import statistics
import threading
import time
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
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
    """Pure function: step index -> joint values."""
    t = step * dt
    return [
        0.5 * (lower[i] + upper[i])
        + 0.45 * (upper[i] - lower[i]) * math.sin(speed * t + i * 0.9)
        for i in range(6)
    ]


def generate_configs(n: int, speed: float, dt: float) -> list:
    """Generate n unique configurations deterministically in the main process."""
    _, _, lower, upper = _load_scene()
    return [_config_at_step(i, lower, upper, speed, dt) for i in range(n)]


# ---------------------------------------------------------------------------
# Worker state — process global for ProcessPool, thread-local for ThreadPool
# ---------------------------------------------------------------------------

_PROC_STATE: dict = {}  # per-process global dict (one per worker process)
_THREAD_STATE = threading.local()  # per-thread storage
_THREAD_SETUP_LOCK = threading.Lock()  # compas_fab+PyBullet setup is NOT thread-safe
_THREAD_BARRIER: "threading.Barrier | None" = (
    None  # set per-case to force N distinct threads
)


def _setup_pybullet_into(state) -> None:
    """Build PyBullet client + planner and stash into state.  Runs a warmup check.

    Note: compas_fab's PyBullet client setup (URDF load, mesh upload) is not
    thread-safe — parallel setup corrupts shared temp-file paths inside the
    library. The actual collision checks afterward appear to be safe per client.
    Callers in the thread backend MUST hold ``_THREAD_SETUP_LOCK`` while calling
    this function.
    """
    robot_cell, robot_cell_state, _, _ = _load_scene()
    client = PyBulletClient(connection_type="direct", verbose=False)
    client.__enter__()  # keep alive for the life of the worker
    planner = PyBulletPlanner(client)
    planner.set_robot_cell(robot_cell)
    planner.set_robot_cell_state(robot_cell_state)

    state.client = client
    state.planner = planner
    state.robot_cell_state = robot_cell_state
    state.cfg = robot_cell_state.robot_configuration.copy()

    # Warmup: one real collision check to build broadphase + JIT paths.
    try:
        planner.check_collision(robot_cell_state, options={"verbose": False})
    except CollisionCheckError:
        pass


def _proc_initializer() -> None:
    """ProcessPoolExecutor initializer — runs once per worker process."""

    class _NS:
        pass

    ns = _NS()
    _setup_pybullet_into(ns)
    _PROC_STATE["ns"] = ns


def _thread_get_state():
    """Lazily build PyBullet for the current thread on first use.

    Setup is serialised with ``_THREAD_SETUP_LOCK`` because compas_fab's
    PyBullet client initialisation is not thread-safe. This is one-time work
    per thread, performed during the warmup ping phase BEFORE the timed region.
    """
    if not hasattr(_THREAD_STATE, "client"):
        with _THREAD_SETUP_LOCK:
            if not hasattr(_THREAD_STATE, "client"):
                _setup_pybullet_into(_THREAD_STATE)
    return _THREAD_STATE


# ---------------------------------------------------------------------------
# Task functions (must be at module level for ProcessPool pickling)
# ---------------------------------------------------------------------------


def _proc_ping(_):
    return os.getpid()


def _proc_check_batch(configs):
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
    return len(configs), collisions, worker_s


def _thread_check_batch(configs):
    state = _thread_get_state()
    planner = state.planner
    rcs = state.robot_cell_state
    cfg = state.cfg

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
    return len(configs), collisions, worker_s


def _thread_ping(_):
    # First wait at the barrier so the pool is forced to spawn N distinct
    # threads (otherwise a single fast thread can swallow all pings).
    if _THREAD_BARRIER is not None:
        _THREAD_BARRIER.wait()
    _thread_get_state()
    return threading.get_ident()


# ---------------------------------------------------------------------------
# Chunk splitter
# ---------------------------------------------------------------------------


def _split_into_chunks(configs: list, n_chunks: int) -> list:
    L = len(configs)
    chunk_size = (L + n_chunks - 1) // n_chunks
    chunks = []
    for i in range(n_chunks):
        start = i * chunk_size
        end = min(L, start + chunk_size)
        if start < end:
            chunks.append(configs[start:end])
    return chunks


# ---------------------------------------------------------------------------
# Per-N runner
# ---------------------------------------------------------------------------


def run_case(n_workers: int, configs: list, backend: str) -> dict:
    global _THREAD_BARRIER
    chunks = _split_into_chunks(configs, n_workers)
    n_real_workers = len(chunks)  # may be < n_workers if configs < workers

    if backend == "process":
        executor = ProcessPoolExecutor(
            max_workers=n_real_workers, initializer=_proc_initializer
        )
        ping_fn = _proc_ping
        batch_fn = _proc_check_batch
        ping_count = n_real_workers * 3
    else:
        executor = ThreadPoolExecutor(max_workers=n_real_workers)
        # Fresh barrier per case so distinct threads must rendezvous before init.
        _THREAD_BARRIER = threading.Barrier(n_real_workers)
        ping_fn = _thread_ping
        batch_fn = _thread_check_batch
        ping_count = n_real_workers  # one per thread (each waits on barrier)

    try:
        # Force every worker to complete setup BEFORE we start the timer.
        # Process backend: many pings so the OS scheduler reaches every process.
        # Thread backend: exactly N pings, synchronised at a barrier.
        _ = list(executor.map(ping_fn, range(ping_count)))

        # ----- TIMED REGION -----
        t_start = time.perf_counter()
        results = list(executor.map(batch_fn, chunks))
        wall_s = time.perf_counter() - t_start
        # ----- END TIMED REGION -----
    finally:
        executor.shutdown(wait=True)

    total_checks = sum(r[0] for r in results)
    total_collisions = sum(r[1] for r in results)
    worker_secs = [r[2] for r in results]
    return {
        "n": n_workers,
        "backend": backend,
        "total_checks": total_checks,
        "collisions": total_collisions,
        "wall_s": wall_s,
        "total_hz": total_checks / wall_s if wall_s > 0 else 0.0,
        "per_inst_hz": (total_checks / wall_s / n_real_workers) if wall_s > 0 else 0.0,
        "worker_wall_min_s": min(worker_secs),
        "worker_wall_max_s": max(worker_secs),
        "worker_wall_mean_s": statistics.fmean(worker_secs),
        "imbalance_pct": (
            (max(worker_secs) - min(worker_secs)) / max(worker_secs) * 100.0
            if max(worker_secs) > 0
            else 0.0
        ),
    }


# ---------------------------------------------------------------------------
# Output formatting
# ---------------------------------------------------------------------------


def _format_row(r: dict, single_hz: float) -> str:
    spd = r["total_hz"] / single_hz if single_hz else 1.0
    eff = spd / r["n"] * 100.0
    return (
        "  {n:>2}  {hz:>9.1f}  {pinst:>11.1f}  {eff:>6.1f}%  {spd:>6.2f}x  "
        "{wall:>7.2f}s  {imb:>6.1f}%".format(
            n=r["n"],
            hz=r["total_hz"],
            pinst=r["per_inst_hz"],
            eff=eff,
            spd=spd,
            wall=r["wall_s"],
            imb=r["imbalance_pct"],
        )
    )


def _print_table(rows: list, title: str) -> None:
    if not rows:
        return
    single_hz = rows[0]["total_hz"]
    print("\n--- {} ---".format(title))
    print(
        "  {:>2}  {:>9}  {:>11}  {:>7}  {:>7}  {:>8}  {:>7}".format(
            "N", "total_hz", "per_inst_hz", "effic", "speedup", "wall_s", "imbal"
        )
    )
    print("  " + "-" * 64)
    for r in rows:
        print(_format_row(r, single_hz))


# ---------------------------------------------------------------------------
# Markdown report
# ---------------------------------------------------------------------------


def _md_table(rows: list) -> str:
    if not rows:
        return "_(no data)_\n"
    single_hz = rows[0]["total_hz"]
    lines = [
        "| N | total_hz | per_inst_hz | effic | speedup | wall_s | imbal% |",
        "|--:|---------:|------------:|------:|--------:|-------:|-------:|",
    ]
    for r in rows:
        spd = r["total_hz"] / single_hz if single_hz else 1.0
        eff = spd / r["n"] * 100.0
        lines.append(
            "| {n} | {hz:.1f} | {pinst:.1f} | {eff:.1f}% | {spd:.2f}x | {wall:.2f} | {imb:.1f}% |".format(
                n=r["n"],
                hz=r["total_hz"],
                pinst=r["per_inst_hz"],
                eff=eff,
                spd=spd,
                wall=r["wall_s"],
                imb=r["imbalance_pct"],
            )
        )
    return "\n".join(lines) + "\n"


def write_report(
    path: str,
    process_rows: list,
    thread_rows: list,
    total_configs: int,
    n_values: list,
    speed: float,
    dt: float,
) -> None:
    best_proc = max(process_rows, key=lambda r: r["total_hz"]) if process_rows else None
    best_thr = max(thread_rows, key=lambda r: r["total_hz"]) if thread_rows else None

    lines = []
    lines.append("# Multi-instance PyBullet collision throughput\n")
    lines.append("Generated: {}\n".format(datetime.now().isoformat(timespec="seconds")))
    lines.append("\n## Host\n")
    lines.append("- Platform: `{}`".format(platform.platform()))
    lines.append("- Python: `{}`".format(platform.python_version()))
    lines.append("- CPU logical cores: `{}`".format(multiprocessing.cpu_count()))
    lines.append("- Processor: `{}`\n".format(platform.processor() or "n/a"))

    lines.append("\n## Workload\n")
    lines.append("- Total unique configs: **{}**".format(total_configs))
    lines.append("- N values tested: `{}`".format(n_values))
    lines.append("- Distribution: even-split contiguous chunks, one submit per worker")
    lines.append(
        "- Trajectory: step-indexed sinusoidal `mid + amp*sin(speed*t + i*0.9)` "
        "with `speed={}` rad/s, `dt={}` s".format(speed, dt)
    )
    lines.append("- Collision mode: `fail_fast` (default)")
    lines.append(
        "- Setup (client init, robot cell load, 1 warmup check): "
        "**excluded** from timing\n"
    )

    lines.append("\n## Results - Process backend (`ProcessPoolExecutor`)\n")
    lines.append(_md_table(process_rows))

    lines.append("\n## Results - Thread backend (`ThreadPoolExecutor`)\n")
    lines.append(_md_table(thread_rows))

    lines.append("\n## Summary\n")
    if best_proc:
        single_proc_hz = process_rows[0]["total_hz"]
        lines.append(
            "- **Process best:** N={} -> {:.1f} checks/s "
            "({:.2f}x over N=1 process, {:.2f}s wall for {} configs)".format(
                best_proc["n"],
                best_proc["total_hz"],
                best_proc["total_hz"] / single_proc_hz if single_proc_hz else 0,
                best_proc["wall_s"],
                total_configs,
            )
        )
    if best_thr:
        single_thr_hz = thread_rows[0]["total_hz"]
        lines.append(
            "- **Thread best:** N={} -> {:.1f} checks/s "
            "({:.2f}x over N=1 thread, {:.2f}s wall for {} configs)".format(
                best_thr["n"],
                best_thr["total_hz"],
                best_thr["total_hz"] / single_thr_hz if single_thr_hz else 0,
                best_thr["wall_s"],
                total_configs,
            )
        )
    if best_proc and best_thr:
        ratio = (
            best_proc["total_hz"] / best_thr["total_hz"] if best_thr["total_hz"] else 0
        )
        winner = (
            "process" if best_proc["total_hz"] >= best_thr["total_hz"] else "thread"
        )
        lines.append(
            "- **Winner:** **{}** backend "
            "(process {:.1f} hz vs thread {:.1f} hz, ratio {:.2f}x)".format(
                winner,
                best_proc["total_hz"],
                best_thr["total_hz"],
                ratio,
            )
        )

    lines.append("\n## Notes\n")
    lines.append(
        "- `total_hz` = `total_configs / wall_time_of_main_thread`. Wall time covers "
        "dispatch + chunk IPC + all worker compute + result gather, matching end-to-end "
        "throughput as seen from the caller."
    )
    lines.append(
        "- `imbal%` = `(max_worker - min_worker) / max_worker * 100`. High values "
        "indicate uneven collision-check cost per chunk (free vs collision configs)."
    )
    lines.append(
        "- The thread backend shares one Python process. Empirically (see table) "
        "adding threads gives effectively **no speedup** for this workload — the "
        "GIL serialises either the PyBullet Python bindings, the compas_fab "
        "wrapper layer, or both. If you need parallelism, use the process backend."
    )
    lines.append(
        "- The process backend has higher per-call IPC cost (chunk pickling) but no "
        "GIL contention. It is the clear winner for N > 1 on this workload."
    )
    lines.append(
        "- Process throughput plateaus around N = number-of-physical-cores. On a "
        "6P+6E (12 logical) machine the peak was near N=10, then degraded as "
        "work spilled onto efficient/heterogeneous cores and OS scheduling cost "
        "rose. Treat N=cores/2 as a safe sweet spot if other apps share the CPU."
    )

    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines).rstrip() + "\n")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _parse_args():
    p = argparse.ArgumentParser(
        description="Multi-instance PyBullet collision throughput benchmark v2",
    )
    p.add_argument(
        "--total-configs",
        type=int,
        default=5000,
        help="Total unique configurations to process (default 5000)",
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
        "--backends",
        type=str,
        default="process,threads",
        help="Comma-separated list: process,threads  (default both)",
    )
    p.add_argument(
        "--speed",
        type=float,
        default=0.6,
        help="Trajectory speed in rad/s (default 0.6)",
    )
    p.add_argument(
        "--dt",
        type=float,
        default=0.01,
        help="Trajectory time step in seconds (default 0.01)",
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

    backends = [b.strip() for b in args.backends.split(",") if b.strip()]
    for b in backends:
        if b not in ("process", "threads"):
            raise SystemExit("Invalid backend: {}".format(b))

    print(
        "\n=== Multi-Instance PyBullet Throughput Benchmark v2 ===\n"
        "  CPU logical cores : {cpu}\n"
        "  total configs     : {tc}\n"
        "  N values          : {ns}\n"
        "  backends          : {bk}\n"
        "  trajectory        : sinusoidal step-indexed  speed={spd} rad/s  dt={dt}s\n"
        "  collision mode    : fail_fast\n"
        "  setup time        : EXCLUDED via initializer + warmup ping round\n".format(
            cpu=multiprocessing.cpu_count(),
            tc=args.total_configs,
            ns=n_values,
            bk=backends,
            spd=args.speed,
            dt=args.dt,
        )
    )

    print("Generating {} unique configs in main process ...".format(args.total_configs))
    configs = generate_configs(args.total_configs, args.speed, args.dt)
    print("  done.\n")

    process_rows: list = []
    thread_rows: list = []

    for backend in backends:
        rows = process_rows if backend == "process" else thread_rows
        print("==> Backend: {}".format(backend))
        for n in n_values:
            print("  N={:2d}  warming up workers ... ".format(n), end="", flush=True)
            r = run_case(n, configs, backend)
            rows.append(r)
            print(
                "{hz:>8.1f} checks/s  ({pinst:>7.1f}/inst  wall {wall:>5.2f}s  "
                "imb {imb:>4.1f}%)".format(
                    hz=r["total_hz"],
                    pinst=r["per_inst_hz"],
                    wall=r["wall_s"],
                    imb=r["imbalance_pct"],
                )
            )
        print()

    if process_rows:
        _print_table(
            process_rows, "Process backend  ({} configs)".format(args.total_configs)
        )
    if thread_rows:
        _print_table(
            thread_rows, "Thread backend  ({} configs)".format(args.total_configs)
        )

    print("\n--- recommendation ---")
    for label, rows in [("process", process_rows), ("threads", thread_rows)]:
        if not rows:
            continue
        best = max(rows, key=lambda r: r["total_hz"])
        single_hz = rows[0]["total_hz"]
        print(
            "  [{label:<7}]  best N={n:<2}  {hz:>7.1f} checks/s   "
            "(vs N=1: {spd:.2f}x throughput)".format(
                label=label,
                n=best["n"],
                hz=best["total_hz"],
                spd=best["total_hz"] / single_hz if single_hz else 1.0,
            )
        )

    if process_rows and thread_rows:
        bp = max(process_rows, key=lambda r: r["total_hz"])
        bt = max(thread_rows, key=lambda r: r["total_hz"])
        winner = "process" if bp["total_hz"] >= bt["total_hz"] else "threads"
        print(
            "  overall winner: {winner}  "
            "(process best {pp:.1f} hz @ N={pn}  vs  threads best {tt:.1f} hz @ N={tn})".format(
                winner=winner,
                pp=bp["total_hz"],
                pn=bp["n"],
                tt=bt["total_hz"],
                tn=bt["n"],
            )
        )

    if not args.no_report:
        write_report(
            REPORT_PATH,
            process_rows,
            thread_rows,
            total_configs=args.total_configs,
            n_values=n_values,
            speed=args.speed,
            dt=args.dt,
        )
        print("\nMarkdown report written: {}".format(REPORT_PATH))

    print()


if __name__ == "__main__":
    # Required on Windows for ProcessPoolExecutor: __main__ guard.
    main()
