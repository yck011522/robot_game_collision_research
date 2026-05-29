"""Profile compas_fab+PyBullet collision checks: baseline vs touch lists.

Single-process, single-thread.  Wraps each scenario in cProfile so we can
see where the time goes function-by-function and decide whether the
touch-list machinery is paying for itself (or being undercut by Python
overhead).

What it does
------------
1. Generates ``--configs`` deterministic configurations (default 2000).
2. Runs scenario A (baseline -- robot cell state as loaded from JSON)
   inside ``cProfile``, dumps ``..._baseline.prof`` next to the script.
3. Runs scenario B (with discovered touch lists applied) inside
   ``cProfile``, dumps ``..._with_touch.prof``.
4. Prints two "top N by cumulative time" tables and a side-by-side
   comparison of the same functions across both scenarios -- this is the
   table to read when answering "did touch lists make Python overhead go
   up faster than it made PyBullet calls go down?".

How to interpret the output
---------------------------
The columns shown for each function are:

  ncalls  : number of times the function was called
  tottime : seconds spent *inside* the function, excluding sub-calls
  cumtime : seconds spent in the function AND everything it called

Read it like this:

- Look at the TOP of the cumtime list. Functions with the largest cumtime
  are where total time is being spent (including nested calls). The first
  pybullet/compas_fab function near the top is the "hot path".

- Compare baseline vs with_touch for the SAME function name:
  * ``pybullet.getClosestPoints`` (or similar): if ``ncalls`` drops
    significantly in with_touch, the touch lists really are short-circuiting
    pair queries. If ``ncalls`` is unchanged, the touch list is not being
    consulted on the dimension you expected.
  * compas_fab functions that handle the touch-list test (look for names
    containing "touch", "allowed", "skip", "is_collision_allowed" or the
    method that iterates pairs): if ``tottime`` *grows* in with_touch,
    that is the Python overhead added by membership tests. Subtract this
    growth from the cumtime savings on ``getClosestPoints`` to see the
    net win.

- If with_touch's TOTAL tottime is lower but the Python helper that does
  the gating got more expensive, the savings are real but smaller than
  the raw broadphase-skip count suggests.

You can also open the dumped ``.prof`` files in a visual tool, e.g.::

    pip install snakeviz
    snakeviz pybullet/bullet_collision_touch_lists_profile_baseline.prof
    snakeviz pybullet/bullet_collision_touch_lists_profile_with_touch.prof

Usage
-----
    conda activate game
    python pybullet/bullet_collision_touch_lists_profile.py
    python pybullet/bullet_collision_touch_lists_profile.py --configs 5000 --top 50
"""

from __future__ import annotations

import argparse
import cProfile
import io
import json
import math
import os
import pstats
import time

from compas.data import json_load
from compas_fab.backends import PyBulletClient, PyBulletPlanner
from compas_fab.backends.exceptions import CollisionCheckError

HERE = os.path.dirname(os.path.abspath(__file__))
JSON_PATH = os.path.join(HERE, "robot_cell_and_state.json")
DISCOVERY_PATH = os.path.join(HERE, "bullet_collision_pair_discovery.json")
SCRIPT_BASENAME = os.path.splitext(os.path.basename(__file__))[0]

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


# ---------------------------------------------------------------------------
# Scene + touch list helpers (same shape as the benchmark script)
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


def _apply_touch_lists(robot_cell_state, discovery: dict):
    per_body = discovery.get("per_rigid_body", {})
    per_tool = discovery.get("per_tool", {})

    for key, info in per_body.items():
        state = robot_cell_state.rigid_body_states.get(key)
        if state is None:
            continue
        state.touch_links = list(info.get("touch_links_candidates", []))
        state.touch_bodies = list(info.get("touch_bodies_candidates", []))

    if hasattr(robot_cell_state, "tool_states"):
        for key, info in per_tool.items():
            state = robot_cell_state.tool_states.get(key)
            if state is None:
                continue
            if hasattr(state, "touch_links"):
                state.touch_links = list(info.get("touch_links_candidates", []))
            if hasattr(state, "touch_bodies"):
                state.touch_bodies = list(info.get("touch_bodies_candidates", []))


# ---------------------------------------------------------------------------
# Config generator (matches the benchmark)
# ---------------------------------------------------------------------------


def _generate_configs(n: int, speed: float, dt: float):
    _, _, lower, upper = _load_scene()
    out = []
    for step in range(n):
        t = step * dt
        out.append(
            [
                0.5 * (lower[i] + upper[i])
                + 0.45 * (upper[i] - lower[i]) * math.sin(speed * t + i * 0.9)
                for i in range(6)
            ]
        )
    return out


# ---------------------------------------------------------------------------
# One scenario, single process, single thread
# ---------------------------------------------------------------------------


def _run_scenario(label: str, configs, apply_touch: bool, prof_path: str):
    robot_cell, robot_cell_state, _, _ = _load_scene()
    if apply_touch:
        _apply_touch_lists(robot_cell_state, _load_discovery())

    client = PyBulletClient(connection_type="direct", verbose=False)
    client.__enter__()
    try:
        planner = PyBulletPlanner(client)
        planner.set_robot_cell(robot_cell)
        planner.set_robot_cell_state(robot_cell_state)
        cfg = robot_cell_state.robot_configuration.copy()

        # Warmup -- NOT profiled
        for q in configs[: min(20, len(configs))]:
            cfg.joint_values = q
            robot_cell_state.robot_configuration = cfg
            try:
                planner.check_collision(robot_cell_state, options={"verbose": False})
            except CollisionCheckError:
                pass

        collisions = 0
        profiler = cProfile.Profile()
        t0 = time.perf_counter()
        profiler.enable()
        for q in configs:
            cfg.joint_values = q
            robot_cell_state.robot_configuration = cfg
            try:
                planner.check_collision(robot_cell_state, options={"verbose": False})
            except CollisionCheckError:
                collisions += 1
        profiler.disable()
        wall_s = time.perf_counter() - t0
        profiler.dump_stats(prof_path)
    finally:
        client.__exit__(None, None, None)

    return {
        "label": label,
        "n_checks": len(configs),
        "collisions": collisions,
        "wall_s": wall_s,
        "hz": len(configs) / wall_s if wall_s > 0 else 0.0,
        "prof_path": prof_path,
    }


# ---------------------------------------------------------------------------
# Pretty-print pstats output
# ---------------------------------------------------------------------------


def _print_top(prof_path: str, label: str, top_n: int, sort_key: str):
    print("\n========== {} :: top {} by {} ==========".format(label, top_n, sort_key))
    buf = io.StringIO()
    stats = pstats.Stats(prof_path, stream=buf)
    stats.strip_dirs().sort_stats(sort_key).print_stats(top_n)
    print(buf.getvalue())


def _collect_stats(prof_path: str):
    """Return {function_key: (ncalls, tottime, cumtime)} dict.

    function_key is "filename:lineno(funcname)" -- the same key pstats uses
    after strip_dirs(), so it lines up between the two scenarios.
    """
    stats = pstats.Stats(prof_path).strip_dirs()
    out = {}
    for func, (cc, nc, tt, ct, _callers) in stats.stats.items():
        filename, lineno, funcname = func
        key = "{}:{}({})".format(filename, lineno, funcname)
        out[key] = (nc, tt, ct)
    return out


def _print_side_by_side(baseline_path, with_touch_path, top_n: int):
    """Show the same functions in both runs, sorted by baseline cumtime."""
    a = _collect_stats(baseline_path)
    b = _collect_stats(with_touch_path)
    keys = sorted(a.keys(), key=lambda k: a[k][2], reverse=True)[:top_n]

    print(
        "\n========== side-by-side  (top {} by baseline cumtime) ==========".format(
            top_n
        )
    )
    print("Columns: ncalls | tottime(s) | cumtime(s)\n")
    fmt_h = "{:>9} {:>9} {:>9} | {:>9} {:>9} {:>9} | {}"
    print(
        fmt_h.format(
            "B_ncalls",
            "B_tot",
            "B_cum",
            "T_ncalls",
            "T_tot",
            "T_cum",
            "function",
        )
    )
    print("-" * 100)
    for k in keys:
        an, at, ac = a[k]
        bn, bt, bc = b.get(k, (0, 0.0, 0.0))
        print(
            "{:>9d} {:>9.3f} {:>9.3f} | {:>9d} {:>9.3f} {:>9.3f} | {}".format(
                an, at, ac, bn, bt, bc, k
            )
        )


def _print_delta(baseline_path, with_touch_path, top_n: int):
    """Show functions where with_touch differs most from baseline (by cumtime delta).

    A negative delta = touch lists made this function cheaper (good --
    usually means fewer pair checks).
    A positive delta = touch lists made this function more expensive
    (the price we pay -- usually the gating helper itself).
    """
    a = _collect_stats(baseline_path)
    b = _collect_stats(with_touch_path)
    all_keys = set(a) | set(b)
    rows = []
    for k in all_keys:
        an, at, ac = a.get(k, (0, 0.0, 0.0))
        bn, bt, bc = b.get(k, (0, 0.0, 0.0))
        rows.append((bc - ac, k, an, at, ac, bn, bt, bc))

    # Sort by abs(delta cumtime) descending
    rows.sort(key=lambda r: abs(r[0]), reverse=True)

    print(
        "\n========== biggest cumtime deltas  (with_touch - baseline, top {}) ==========".format(
            top_n
        )
    )
    print("Negative delta = function got CHEAPER (touch lists saved time here)")
    print("Positive delta = function got MORE EXPENSIVE (touch lists cost time here)\n")
    fmt_h = "{:>10} | {:>9} {:>9} | {:>9} {:>9} | {}"
    print(
        fmt_h.format(
            "delta_cum",
            "B_ncalls",
            "B_cum",
            "T_ncalls",
            "T_cum",
            "function",
        )
    )
    print("-" * 100)
    for delta, k, an, at, ac, bn, bt, bc in rows[:top_n]:
        print(
            "{:>+10.3f} | {:>9d} {:>9.3f} | {:>9d} {:>9.3f} | {}".format(
                delta, an, ac, bn, bc, k
            )
        )


# ---------------------------------------------------------------------------
# CLI / main
# ---------------------------------------------------------------------------


def _parse_args():
    p = argparse.ArgumentParser(description="cProfile baseline vs with_touch")
    p.add_argument(
        "--configs",
        type=int,
        default=2000,
        help="Number of configs to profile per scenario (default 2000)",
    )
    p.add_argument(
        "--top",
        type=int,
        default=25,
        help="Top-N rows to show in each table (default 25)",
    )
    p.add_argument("--speed", type=float, default=0.6)
    p.add_argument("--dt", type=float, default=0.01)
    return p.parse_args()


def main():
    args = _parse_args()

    if not os.path.exists(DISCOVERY_PATH):
        raise SystemExit("Discovery JSON not found: {}".format(DISCOVERY_PATH))

    baseline_prof = os.path.join(HERE, SCRIPT_BASENAME + "_baseline.prof")
    touch_prof = os.path.join(HERE, SCRIPT_BASENAME + "_with_touch.prof")

    print("Generating {} configs (deterministic) ...".format(args.configs))
    configs = _generate_configs(args.configs, args.speed, args.dt)

    print("\nRunning scenario A: baseline ...")
    a = _run_scenario("baseline", configs, apply_touch=False, prof_path=baseline_prof)
    print(
        "  wall={:.2f}s   hz={:.1f}   collisions={}/{}".format(
            a["wall_s"], a["hz"], a["collisions"], a["n_checks"]
        )
    )

    print("\nRunning scenario B: with_touch ...")
    b = _run_scenario("with_touch", configs, apply_touch=True, prof_path=touch_prof)
    print(
        "  wall={:.2f}s   hz={:.1f}   collisions={}/{}".format(
            b["wall_s"], b["hz"], b["collisions"], b["n_checks"]
        )
    )

    sanity = "OK" if a["collisions"] == b["collisions"] else "MISMATCH"
    print("\nSanity (collision counts match): {}".format(sanity))
    print(
        "Speedup (with_touch / baseline): {:.2f}x".format(
            b["hz"] / a["hz"] if a["hz"] > 0 else 0
        )
    )

    # Per-scenario top tables
    _print_top(baseline_prof, "baseline", args.top, "cumulative")
    _print_top(touch_prof, "with_touch", args.top, "cumulative")

    # The two tables that actually answer the question:
    _print_side_by_side(baseline_prof, touch_prof, args.top)
    _print_delta(baseline_prof, touch_prof, args.top)

    print("\nProfile files written:")
    print("  {}".format(baseline_prof))
    print("  {}".format(touch_prof))
    print("\nFor an interactive view:  pip install snakeviz  &&  snakeviz <file>")


if __name__ == "__main__":
    main()
