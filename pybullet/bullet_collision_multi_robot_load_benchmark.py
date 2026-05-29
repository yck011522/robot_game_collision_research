"""Multi-robot load benchmark for the collision-aware controller.

Simulates running N independent robot controllers from one host computer,
each with its own dedicated forward (sync) + proximity (async) worker
pools. The dispatch pattern is byte-for-byte the same one the live
keyboard explorer uses (``_proc_forward_chunk`` and
``_proc_proximity_chunk``), so the wall-clock cost on this hardware is
representative of what the explorer would see under the same allocation.

Used to predict performance on the production target
(Intel Core Ultra 5 225: 10 P+E cores, 10 threads, no SMT). Run on the
dev laptop today; rerun on the target hardware in a few days with the
same flags to see the delta.

The motion is fully synthetic and deterministic (sine-sweep per axis
with fixed frequencies, fixed seed), so a comparison between runs is
apples-to-apples without any operator input.

Usage
-----
    conda activate game

    # 1 robot, default 6 + 6 = 12 workers (current dev-laptop default)
    python pybullet/bullet_collision_multi_robot_load_benchmark.py \
        --robots 1 --forward-workers 6 --prox-workers 6 --duration 20

    # Production-equivalent on 10 threads (2 robots x (3 fwd + 2 prox))
    python pybullet/bullet_collision_multi_robot_load_benchmark.py \
        --robots 2 --forward-workers 3 --prox-workers 2 --duration 20

    # Stress test (3 robots; will oversubscribe a 12-thread laptop)
    python pybullet/bullet_collision_multi_robot_load_benchmark.py \
        --robots 3 --forward-workers 3 --prox-workers 2 --duration 20

Outputs per-robot FPS, dt percentiles, proximity staleness, and a
combined summary so you can see how badly two robots interfere.
"""

from __future__ import annotations

import argparse
import math
import os
import statistics
import sys
import threading
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, field

# Reuse the exact worker functions + chunking helpers used by the explorer.
HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from bullet_collision_keyboard_explorer import (  # noqa: E402
    DISCOVERY_PATH,
    FORWARD_STEP_DEG,
    INITIAL_POS_DEG,
    N_FORWARD_STEPS,
    PROBE_OFFSETS_RAD,
    _partition,
    _proc_forward_chunk,
    _proc_init,
    _proc_ping,
    _proc_proximity_chunk,
)


# ---------------------------------------------------------------------------
# Per-robot deterministic motion generator
# ---------------------------------------------------------------------------


def _sine_motion_v_cmd_dps(t: float, robot_idx: int) -> list[float]:
    """Deterministic synthetic v_cmd for axis i in degrees/sec.

    Each robot has a distinct phase offset so the two pools aren't perfectly
    in lockstep (mimics real independent operators). Each joint has its own
    period so the pose explores 6D space without ever sitting idle.
    """
    base_periods = [3.7, 4.3, 5.1, 2.9, 3.3, 2.3]  # seconds, all coprime-ish
    phase = robot_idx * 0.7  # different per robot
    out = []
    # Amplitudes mirror the explorer's per-axis max_vel defaults.
    amps = [20.0, 20.0, 20.0, 30.0, 30.0, 30.0]
    for i in range(6):
        out.append(amps[i] * math.sin(2.0 * math.pi * (t / base_periods[i] + phase)))
    return out


# ---------------------------------------------------------------------------
# Per-tick measurements
# ---------------------------------------------------------------------------


@dataclass
class RobotStats:
    robot_idx: int
    n_fwd: int
    n_prox: int
    dts_ms: list[float] = field(default_factory=list)
    fwd_ms: list[float] = field(default_factory=list)
    prox_pipe_ms: list[float] = field(default_factory=list)
    prox_age_ms: list[float] = field(default_factory=list)
    ticks: int = 0
    duration_s: float = 0.0


def _percentile(xs: list[float], p: float) -> float:
    if not xs:
        return 0.0
    s = sorted(xs)
    k = max(0, min(len(s) - 1, int(round((p / 100.0) * (len(s) - 1)))))
    return s[k]


# ---------------------------------------------------------------------------
# Controller loop (one thread per robot)
# ---------------------------------------------------------------------------


def _run_controller(
    robot_idx: int,
    n_fwd: int,
    n_prox: int,
    duration_s: float,
    target_fps: float,
    stats: RobotStats,
    barrier: threading.Barrier,
) -> None:
    """Drives one robot for `duration_s` seconds. Mirrors explorer _tick."""
    # Build dedicated pools. Each robot has its OWN forward + prox pools, so
    # the OS scheduler is the only thing arbitrating between robots.
    fwd_exec = ProcessPoolExecutor(max_workers=n_fwd, initializer=_proc_init)
    prox_exec = ProcessPoolExecutor(max_workers=n_prox, initializer=_proc_init)
    # Warm up (initializer is lazy until first task)
    list(fwd_exec.map(_proc_ping, range(n_fwd * 3)))
    list(prox_exec.map(_proc_ping, range(n_prox * 3)))

    fwd_chunks = _partition(range(1, N_FORWARD_STEPS + 1), n_fwd)
    prox_chunks = _partition(range(6), n_prox)

    pos = [math.radians(d) for d in INITIAL_POS_DEG]
    prox_results = [[False] * len(PROBE_OFFSETS_RAD) for _ in range(6)]
    prox_future = None
    prox_in_flight_t = 0.0
    prox_last_harvest_t = time.perf_counter()

    target_dt_s = 1.0 / max(1.0, target_fps)

    # All robots cross the starting line together so per-tick contention is
    # measured from steady state, not from staggered warm-ups.
    barrier.wait()
    t_start = time.perf_counter()
    last_tick_t = t_start

    while True:
        t_now = time.perf_counter()
        if t_now - t_start >= duration_s:
            break

        dt = t_now - last_tick_t
        last_tick_t = t_now

        # Harvest any completed proximity batch (non-blocking)
        if prox_future is not None and all(f.done() for f in prox_future):
            try:
                merged = [[False] * len(PROBE_OFFSETS_RAD) for _ in range(6)]
                for f in prox_future:
                    per_axis = f.result()
                    for axis, bits in per_axis.items():
                        merged[axis] = bits
                prox_results = merged
                stats.prox_pipe_ms.append((t_now - prox_in_flight_t) * 1000.0)
                prox_last_harvest_t = t_now
            except Exception as exc:
                print("R{} prox error: {}".format(robot_idx, exc), file=sys.stderr)
            prox_future = None

        prox_age_ms = (t_now - prox_last_harvest_t) * 1000.0

        # Synthetic v_cmd (deg/s) -> rad/s
        v_cmd_dps = _sine_motion_v_cmd_dps(t_now - t_start, robot_idx)
        v_cmd_rad = [math.radians(v) for v in v_cmd_dps]
        v_norm = math.sqrt(sum(v * v for v in v_cmd_rad))

        # SYNCHRONOUS forward check (the safety gate)
        t_fwd0 = time.perf_counter()
        if v_norm > math.radians(0.5):
            step_rad = math.radians(FORWARD_STEP_DEG)
            step_vec = tuple((v / v_norm) * step_rad for v in v_cmd_rad)
            base = tuple(pos)
            futures = [
                fwd_exec.submit(_proc_forward_chunk, (base, step_vec, chunk))
                for chunk in fwd_chunks
            ]
            for f in futures:
                f.result()  # block
        t_fwd_ms = (time.perf_counter() - t_fwd0) * 1000.0

        # ASYNCHRONOUS proximity dispatch (skip if previous still in flight)
        if prox_future is None:
            offs = tuple(PROBE_OFFSETS_RAD)
            base = tuple(pos)
            prox_future = [
                prox_exec.submit(_proc_proximity_chunk, (base, axes, offs))
                for axes in prox_chunks
            ]
            prox_in_flight_t = time.perf_counter()

        # Integrate (no clamps; this is a load benchmark, not behaviour test)
        pos = [pos[i] + v_cmd_rad[i] * dt for i in range(6)]
        # Bound pose to +/- pi just so we don't drift forever
        pos = [max(-math.pi, min(math.pi, p)) for p in pos]

        # Record this tick
        stats.dts_ms.append(dt * 1000.0)
        stats.fwd_ms.append(t_fwd_ms)
        stats.prox_age_ms.append(prox_age_ms)
        stats.ticks += 1

        # Pace to target FPS
        spent = time.perf_counter() - t_now
        sleep = target_dt_s - spent
        if sleep > 0:
            time.sleep(sleep)

    stats.duration_s = time.perf_counter() - t_start

    # Wait briefly for any in-flight proximity batch then shut down pools.
    if prox_future is not None:
        try:
            for f in prox_future:
                f.result(timeout=5.0)
        except Exception:
            pass
    fwd_exec.shutdown(wait=False, cancel_futures=True)
    prox_exec.shutdown(wait=False, cancel_futures=True)


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------


def _report_one(s: RobotStats) -> None:
    if s.ticks < 2:
        print("R{}: NO DATA".format(s.robot_idx))
        return
    fps = s.ticks / s.duration_s if s.duration_s > 0 else 0.0
    # Skip the first 5 ticks (cold) for percentile calculations
    skip = min(5, s.ticks // 10)
    dts = s.dts_ms[skip:]
    fwd = s.fwd_ms[skip:]
    age = s.prox_age_ms[skip:]
    pipe = s.prox_pipe_ms[1:] if len(s.prox_pipe_ms) > 1 else s.prox_pipe_ms
    print(
        "R{idx}  workers={f}+{p}  ticks={n:5d}  fps={fps:6.2f}".format(
            idx=s.robot_idx, f=s.n_fwd, p=s.n_prox, n=s.ticks, fps=fps
        )
    )
    print(
        "   dt   ms : median={:6.1f}  p90={:6.1f}  p99={:6.1f}  max={:6.1f}".format(
            _percentile(dts, 50),
            _percentile(dts, 90),
            _percentile(dts, 99),
            max(dts) if dts else 0.0,
        )
    )
    print(
        "   fwd  ms : median={:6.1f}  p90={:6.1f}  p99={:6.1f}  max={:6.1f}".format(
            _percentile(fwd, 50),
            _percentile(fwd, 90),
            _percentile(fwd, 99),
            max(fwd) if fwd else 0.0,
        )
    )
    if pipe:
        print(
            "   prox pipeline (dispatch->harvest) ms : "
            "median={:6.1f}  p90={:6.1f}  max={:6.1f}".format(
                _percentile(pipe, 50),
                _percentile(pipe, 90),
                max(pipe),
            )
        )
    print(
        "   prox AGE used by clamp        ms : median={:6.1f}  p90={:6.1f}  max={:6.1f}".format(
            _percentile(age, 50),
            _percentile(age, 90),
            max(age) if age else 0.0,
        )
    )


def _report_combined(all_stats: list[RobotStats]) -> None:
    n = len(all_stats)
    fpses = [s.ticks / s.duration_s for s in all_stats if s.duration_s > 0]
    if not fpses:
        return
    total_fps = sum(fpses)
    print()
    print("Combined: {n} robot(s)   total tick rate = {tt:.2f} ticks/s   "
          "mean per-robot fps = {m:.2f}   min={mn:.2f}".format(
              n=n, tt=total_fps, m=statistics.fmean(fpses), mn=min(fpses)
          ))
    # Worst-case dt across any robot (interference indicator)
    worst_p99 = 0.0
    for s in all_stats:
        if len(s.dts_ms) > 10:
            worst_p99 = max(worst_p99, _percentile(s.dts_ms[5:], 99))
    print("Worst-robot dt p99 = {:.1f} ms (lower = less interference)".format(worst_p99))


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--robots", type=int, default=2, help="number of parallel robot controllers")
    p.add_argument(
        "--forward-workers",
        type=int,
        default=3,
        help="forward (sync) worker processes PER ROBOT (default 3 for prod target)",
    )
    p.add_argument(
        "--prox-workers",
        type=int,
        default=2,
        help="proximity (async) worker processes PER ROBOT (default 2 for prod target)",
    )
    p.add_argument(
        "--duration", type=float, default=20.0, help="seconds to run (default 20)"
    )
    p.add_argument(
        "--target-fps",
        type=float,
        default=120.0,
        help="ceiling on tick rate; set high to measure raw throughput (default 120)",
    )
    args = p.parse_args(argv)

    if not os.path.exists(DISCOVERY_PATH):
        raise SystemExit(
            "Discovery JSON not found: {}\n  Run bullet_collision_pair_discovery.py first.".format(
                DISCOVERY_PATH
            )
        )

    total_workers = args.robots * (args.forward_workers + args.prox_workers)
    print("=" * 72)
    print("Multi-robot load benchmark")
    print(
        "  robots={r}   per-robot workers = {f} fwd + {p} prox = {pr}   "
        "total worker procs = {t}".format(
            r=args.robots,
            f=args.forward_workers,
            p=args.prox_workers,
            pr=args.forward_workers + args.prox_workers,
            t=total_workers,
        )
    )
    print(
        "  duration={d}s   target fps ceiling={tf}   logical CPUs={cpu}".format(
            d=args.duration, tf=args.target_fps, cpu=os.cpu_count()
        )
    )
    print("=" * 72)

    all_stats = [
        RobotStats(robot_idx=i, n_fwd=args.forward_workers, n_prox=args.prox_workers)
        for i in range(args.robots)
    ]
    barrier = threading.Barrier(args.robots)
    threads = []
    for i, s in enumerate(all_stats):
        t = threading.Thread(
            target=_run_controller,
            name="robot-{}".format(i),
            args=(
                i,
                args.forward_workers,
                args.prox_workers,
                args.duration,
                args.target_fps,
                s,
                barrier,
            ),
            daemon=False,
        )
        threads.append(t)
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    print()
    for s in all_stats:
        _report_one(s)
    _report_combined(all_stats)
    return 0


if __name__ == "__main__":
    sys.exit(main())
