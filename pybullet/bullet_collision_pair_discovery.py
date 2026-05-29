"""Discover which robot/rigid-body/tool collision pairs are reachable.

Goal
----
Sample many random configurations within the robot joint limits, run a
FULL collision report on each, and accumulate the set of all distinct
(entity_a, entity_b) collision pairs that ever occur.

From that, for every rigid body and the attached tool, we derive:

  * `touch_links_candidates`  : robot links that were NEVER seen colliding
                                with this body across the whole run.
  * `touch_bodies_candidates` : other rigid bodies / tools that were NEVER
                                seen colliding with this body.

These candidates can be pasted into the `touch_links` / `touch_bodies`
fields of a RigidBodyState in `robot_cell_and_state.json` to short-circuit
those pair checks in the collision checker.

WARNING: this is a *sampling* result. Pairs that did not appear within
the sampling window are not proven to be unreachable -- they are only
"unobserved". Run for longer (e.g. 1 hour) to gain confidence before
committing the touch lists.

Parallelism
-----------
Uses a ProcessPoolExecutor (default 10 workers) modeled after
``bullet_collision_multiinstance_benchmark.py``. Each worker owns its
own PyBullet client and runs random configurations for the requested
duration; the main process unions all worker pair sets at the end.

Usage
-----
    conda activate game

    # 5 second smoke test across 10 workers (default)
    python pybullet/bullet_collision_pair_discovery.py

    # 1 hour run with 10 workers
    python pybullet/bullet_collision_pair_discovery.py --duration 3600

    # Custom worker count
    python pybullet/bullet_collision_pair_discovery.py --workers 8 --duration 600
"""

from __future__ import annotations

import argparse
import json
import math
import os
import platform
import random
import time
from concurrent.futures import ProcessPoolExecutor
from datetime import datetime

from compas.data import json_load
from compas_fab.backends import PyBulletClient, PyBulletPlanner
from compas_fab.backends.exceptions import CollisionCheckError

HERE = os.path.dirname(os.path.abspath(__file__))
JSON_PATH = os.path.join(HERE, "robot_cell_and_state.json")
SCRIPT_BASENAME = os.path.splitext(os.path.basename(__file__))[0]
JSON_OUT_PATH = os.path.join(HERE, SCRIPT_BASENAME + ".json")
MD_OUT_PATH = os.path.join(HERE, SCRIPT_BASENAME + ".md")

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


# ---------------------------------------------------------------------------
# Entity tagging
# ---------------------------------------------------------------------------
# Every entity that can appear in a collision pair is tagged with a kind
# prefix so link/body/tool names cannot collide in the set.

KIND_LINK = "link"
KIND_BODY = "body"
KIND_TOOL = "tool"


def _tag(kind: str, name: str) -> str:
    return "{}:{}".format(kind, name)


def _untag(tagged: str):
    kind, _, name = tagged.partition(":")
    return kind, name


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


def _scene_entities(robot_cell, robot_cell_state):
    """Return (link_names, body_names, tool_names) for the scene."""
    link_names = [link.name for link in robot_cell.robot_model.links]
    body_names = list(robot_cell.rigid_body_models.keys())
    tool_names = (
        list(robot_cell_state.tool_states.keys())
        if hasattr(robot_cell_state, "tool_states")
        else []
    )
    return link_names, body_names, tool_names


# ---------------------------------------------------------------------------
# Worker (process global) state
# ---------------------------------------------------------------------------

_W: dict = {}


def _proc_init():
    robot_cell, robot_cell_state, lower, upper = _load_scene()

    client = PyBulletClient(connection_type="direct", verbose=False)
    client.__enter__()
    planner = PyBulletPlanner(client)
    planner.set_robot_cell(robot_cell)
    planner.set_robot_cell_state(robot_cell_state)

    # Reverse map: object identity -> key. The planner returns the same
    # RigidBody objects that live in robot_cell.rigid_body_models, so id()
    # lookup is reliable inside this process.
    rb_id_to_key = {id(v): k for k, v in robot_cell.rigid_body_models.items()}
    tool_id_to_key = (
        {id(v): k for k, v in robot_cell.tool_models.items()}
        if hasattr(robot_cell, "tool_models")
        else {}
    )

    # Warmup check (excluded from any timing the caller cares about).
    try:
        planner.check_collision(
            robot_cell_state, options={"full_report": True, "verbose": False}
        )
    except CollisionCheckError:
        pass

    _W["client"] = client
    _W["planner"] = planner
    _W["robot_cell_state"] = robot_cell_state
    _W["cfg"] = robot_cell_state.robot_configuration.copy()
    _W["lower"] = lower
    _W["upper"] = upper
    _W["rb_id_to_key"] = rb_id_to_key
    _W["tool_id_to_key"] = tool_id_to_key


def _resolve_entity(obj) -> str:
    """Map a collision-pair entity (Link / RigidBody / ToolModel) to a tagged name."""
    cls = type(obj).__name__
    if cls == "Link":
        return _tag(KIND_LINK, obj.name)
    if cls == "RigidBody":
        key = _W["rb_id_to_key"].get(id(obj))
        if key is None:
            # Fallback for safety -- should not happen if scene is shared.
            key = getattr(obj, "name", "unknown")
        return _tag(KIND_BODY, key)
    if cls == "ToolModel":
        key = _W["tool_id_to_key"].get(id(obj))
        if key is None:
            key = getattr(obj, "name", "unknown")
        return _tag(KIND_TOOL, key)
    return _tag("other", "{}#{}".format(cls, getattr(obj, "name", "?")))


def _normalize_pair(a_tagged: str, b_tagged: str):
    """Canonicalise pair direction so (a,b) and (b,a) hash equal."""
    return (a_tagged, b_tagged) if a_tagged <= b_tagged else (b_tagged, a_tagged)


def _proc_task(args):
    """Random-sample collisions for `duration_s`. Return aggregated stats."""
    seed, duration_s = args
    rng = random.Random(seed)

    planner = _W["planner"]
    rcs = _W["robot_cell_state"]
    cfg = _W["cfg"]
    lower = _W["lower"]
    upper = _W["upper"]

    seen_pairs: set = set()
    n_checks = 0
    n_collisions = 0
    n_pairs_total = 0

    t_end = time.perf_counter() + duration_s
    while time.perf_counter() < t_end:
        cfg.joint_values = [rng.uniform(lower[i], upper[i]) for i in range(6)]
        rcs.robot_configuration = cfg
        n_checks += 1
        try:
            planner.check_collision(
                rcs, options={"full_report": True, "verbose": False}
            )
        except CollisionCheckError as exc:
            n_collisions += 1
            for pair in exc.collision_pairs:
                a, b = pair
                tag_a = _resolve_entity(a)
                tag_b = _resolve_entity(b)
                seen_pairs.add(_normalize_pair(tag_a, tag_b))
                n_pairs_total += 1

    return {
        "pid": os.getpid(),
        "n_checks": n_checks,
        "n_collisions": n_collisions,
        "n_pairs_total": n_pairs_total,
        "seen_pairs": list(seen_pairs),  # serialise as list for IPC
    }


# ---------------------------------------------------------------------------
# Consolidation
# ---------------------------------------------------------------------------


def _consolidate(seen_pairs: set, link_names, body_names, tool_names):
    """For every body+tool, compute candidate touch_links / touch_bodies."""

    # Build adjacency: entity_tag -> set of entity_tags it ever collided with
    adjacency: dict = {}
    for a, b in seen_pairs:
        adjacency.setdefault(a, set()).add(b)
        adjacency.setdefault(b, set()).add(a)

    all_link_tags = {_tag(KIND_LINK, n) for n in link_names}
    all_body_tags = {_tag(KIND_BODY, n) for n in body_names}
    all_tool_tags = {_tag(KIND_TOOL, n) for n in tool_names}

    def _strip_kind(tagged_set, kind):
        return sorted(_untag(t)[1] for t in tagged_set if _untag(t)[0] == kind)

    per_body = {}
    for name in body_names:
        self_tag = _tag(KIND_BODY, name)
        seen = adjacency.get(self_tag, set())

        seen_links = _strip_kind(seen, KIND_LINK)
        seen_bodies = _strip_kind(seen, KIND_BODY)
        seen_tools = _strip_kind(seen, KIND_TOOL)

        never_links = sorted(_untag(t)[1] for t in (all_link_tags - seen - {self_tag}))
        never_bodies = sorted(_untag(t)[1] for t in (all_body_tags - seen - {self_tag}))
        never_tools = sorted(_untag(t)[1] for t in (all_tool_tags - seen - {self_tag}))

        per_body[name] = {
            "collided_with_links": seen_links,
            "collided_with_bodies": seen_bodies,
            "collided_with_tools": seen_tools,
            "touch_links_candidates": never_links,
            "touch_bodies_candidates": sorted(never_bodies + never_tools),
        }

    per_tool = {}
    for name in tool_names:
        self_tag = _tag(KIND_TOOL, name)
        seen = adjacency.get(self_tag, set())

        seen_links = _strip_kind(seen, KIND_LINK)
        seen_bodies = _strip_kind(seen, KIND_BODY)
        seen_tools = _strip_kind(seen, KIND_TOOL)

        never_links = sorted(_untag(t)[1] for t in (all_link_tags - seen - {self_tag}))
        never_bodies = sorted(_untag(t)[1] for t in (all_body_tags - seen - {self_tag}))
        never_tools = sorted(_untag(t)[1] for t in (all_tool_tags - seen - {self_tag}))

        per_tool[name] = {
            "collided_with_links": seen_links,
            "collided_with_bodies": seen_bodies,
            "collided_with_tools": seen_tools,
            "touch_links_candidates": never_links,
            "touch_bodies_candidates": sorted(never_bodies + never_tools),
        }

    # Per-link summary too -- useful when reasoning the other direction.
    per_link = {}
    for name in link_names:
        self_tag = _tag(KIND_LINK, name)
        seen = adjacency.get(self_tag, set())
        per_link[name] = {
            "collided_with_links": _strip_kind(seen, KIND_LINK),
            "collided_with_bodies": _strip_kind(seen, KIND_BODY),
            "collided_with_tools": _strip_kind(seen, KIND_TOOL),
        }

    return per_body, per_tool, per_link


# ---------------------------------------------------------------------------
# Output writers
# ---------------------------------------------------------------------------


def _write_json(path, payload):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, sort_keys=True)


def _write_markdown(
    path,
    per_body,
    per_tool,
    per_link,
    link_names,
    body_names,
    tool_names,
    seen_pairs,
    totals,
    args,
    workers_results,
):
    lines = []
    lines.append("# Collision pair discovery\n")
    lines.append("Generated: {}\n".format(datetime.now().isoformat(timespec="seconds")))

    lines.append("\n## Run\n")
    lines.append("- Platform: `{}`".format(platform.platform()))
    lines.append("- Python: `{}`".format(platform.python_version()))
    lines.append("- Workers (processes): **{}**".format(args.workers))
    lines.append("- Per-worker duration: **{:.1f} s**".format(args.duration))
    lines.append("- Wall time (incl. setup): **{:.2f} s**".format(totals["wall_s"]))
    lines.append("- Total checks: **{}**".format(totals["n_checks"]))
    lines.append(
        "- Total collisions: **{}** ({:.1f}%)".format(
            totals["n_collisions"],
            100.0 * totals["n_collisions"] / max(1, totals["n_checks"]),
        )
    )
    lines.append(
        "- Total reported pair-instances: **{}**".format(totals["n_pairs_total"])
    )
    lines.append("- Distinct pairs observed: **{}**".format(len(seen_pairs)))
    lines.append(
        "- Sampling check rate: **{:.0f} configs/s aggregate**".format(
            totals["n_checks"] / totals["wall_s"] if totals["wall_s"] > 0 else 0.0
        )
    )

    lines.append("\n### Per-worker stats\n")
    lines.append("| pid | checks | collisions | pair-instances | distinct pairs |")
    lines.append("|----:|-------:|-----------:|---------------:|---------------:|")
    for r in workers_results:
        lines.append(
            "| {} | {} | {} | {} | {} |".format(
                r["pid"],
                r["n_checks"],
                r["n_collisions"],
                r["n_pairs_total"],
                len(r["seen_pairs"]),
            )
        )

    lines.append("\n## Caveat\n")
    lines.append(
        "Pairs that did NOT appear in this run are **unobserved**, not proven "
        "unreachable. The `touch_*_candidates` below are *suggestions* — they "
        "are only safe to commit into `robot_cell_and_state.json` after a long "
        "enough sampling run (e.g. 1 hour) AND a sanity check against known "
        "kinematic constraints."
    )

    # --- Rigid body section
    lines.append("\n## Per rigid body\n")
    for name in body_names:
        info = per_body[name]
        lines.append("### `{}`\n".format(name))
        lines.append(
            "- Collided with links: `{}`".format(info["collided_with_links"] or "[]")
        )
        if info["collided_with_bodies"] or info["collided_with_tools"]:
            other = info["collided_with_bodies"] + [
                "tool:" + t for t in info["collided_with_tools"]
            ]
            lines.append("- Collided with other bodies/tools: `{}`".format(other))
        lines.append(
            "- **`touch_links_candidates` (never seen):** `{}`".format(
                info["touch_links_candidates"]
            )
        )
        lines.append(
            "- **`touch_bodies_candidates` (never seen):** `{}`".format(
                info["touch_bodies_candidates"]
            )
        )
        lines.append("")

    # --- Tool section
    if tool_names:
        lines.append("\n## Per tool\n")
        for name in tool_names:
            info = per_tool[name]
            lines.append("### `{}`\n".format(name))
            lines.append(
                "- Collided with links: `{}`".format(
                    info["collided_with_links"] or "[]"
                )
            )
            if info["collided_with_bodies"] or info["collided_with_tools"]:
                other = info["collided_with_bodies"] + [
                    "tool:" + t for t in info["collided_with_tools"]
                ]
                lines.append("- Collided with other bodies/tools: `{}`".format(other))
            lines.append(
                "- **`touch_links_candidates` (never seen):** `{}`".format(
                    info["touch_links_candidates"]
                )
            )
            lines.append(
                "- **`touch_bodies_candidates` (never seen):** `{}`".format(
                    info["touch_bodies_candidates"]
                )
            )
            lines.append("")

    # --- Compact "skip matrix" table (body x link)
    lines.append("\n## Skip matrix (body x link)\n")
    lines.append(
        "`.` = never observed colliding (safe to add to `touch_links`); `X` = observed.\n"
    )
    header = "| body \\\\ link | " + " | ".join(link_names) + " |"
    sep = "|" + "---|" * (1 + len(link_names))
    lines.append(header)
    lines.append(sep)
    for body in body_names:
        seen_set = set(per_body[body]["collided_with_links"])
        row = (
            "| `{}` | ".format(body)
            + " | ".join("X" if ln in seen_set else "." for ln in link_names)
            + " |"
        )
        lines.append(row)

    if tool_names:
        lines.append("\n### Skip matrix (tool x link)\n")
        lines.append(header)
        lines.append(sep)
        for tool in tool_names:
            seen_set = set(per_tool[tool]["collided_with_links"])
            row = (
                "| `{}` | ".format(tool)
                + " | ".join("X" if ln in seen_set else "." for ln in link_names)
                + " |"
            )
            lines.append(row)

    # --- All observed pairs (debugging aid)
    lines.append("\n## All distinct observed pairs\n")
    lines.append(
        "Sorted alphabetically. Each entity tagged with kind (`link:`, `body:`, `tool:`).\n"
    )
    for a, b in sorted(seen_pairs):
        lines.append("- `{}`  <->  `{}`".format(a, b))

    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines).rstrip() + "\n")


# ---------------------------------------------------------------------------
# CLI / main
# ---------------------------------------------------------------------------


def _parse_args():
    p = argparse.ArgumentParser(description="Random-config collision pair discovery")
    p.add_argument(
        "--duration",
        type=float,
        default=5.0,
        help="Per-worker sampling duration in seconds (default 5; use 3600 for 1h)",
    )
    p.add_argument(
        "--workers",
        type=int,
        default=10,
        help="Number of parallel processes (default 10)",
    )
    p.add_argument(
        "--seed",
        type=int,
        default=12345,
        help="Base RNG seed (worker i uses seed + i)",
    )
    return p.parse_args()


def main():
    args = _parse_args()

    # Inspect the scene once in the main process for entity name lists.
    robot_cell, robot_cell_state, _, _ = _load_scene()
    link_names, body_names, tool_names = _scene_entities(robot_cell, robot_cell_state)

    print(
        "\n=== Collision pair discovery ===\n"
        "  workers      : {w}\n"
        "  duration/wkr : {d:.1f} s\n"
        "  base seed    : {s}\n"
        "  links        : {ln}\n"
        "  bodies       : {bn}\n"
        "  tools        : {tn}\n".format(
            w=args.workers,
            d=args.duration,
            s=args.seed,
            ln=len(link_names),
            bn=len(body_names),
            tn=len(tool_names),
        )
    )

    tasks = [(args.seed + i, args.duration) for i in range(args.workers)]

    t0 = time.perf_counter()
    with ProcessPoolExecutor(max_workers=args.workers, initializer=_proc_init) as ex:
        worker_results = list(ex.map(_proc_task, tasks))
    wall_s = time.perf_counter() - t0

    seen_pairs: set = set()
    n_checks = 0
    n_collisions = 0
    n_pairs_total = 0
    for r in worker_results:
        n_checks += r["n_checks"]
        n_collisions += r["n_collisions"]
        n_pairs_total += r["n_pairs_total"]
        for pair in r["seen_pairs"]:
            seen_pairs.add(tuple(pair))

    totals = {
        "wall_s": wall_s,
        "n_checks": n_checks,
        "n_collisions": n_collisions,
        "n_pairs_total": n_pairs_total,
    }

    print("--- aggregate ---")
    print("  wall            : {:.2f} s".format(wall_s))
    print(
        "  total checks    : {} ({:.0f} configs/s)".format(
            n_checks, n_checks / wall_s if wall_s > 0 else 0
        )
    )
    print(
        "  collisions      : {} ({:.1f}%)".format(
            n_collisions, 100 * n_collisions / max(1, n_checks)
        )
    )
    print("  distinct pairs  : {}".format(len(seen_pairs)))

    per_body, per_tool, per_link = _consolidate(
        seen_pairs, link_names, body_names, tool_names
    )

    payload = {
        "meta": {
            "generated": datetime.now().isoformat(timespec="seconds"),
            "workers": args.workers,
            "duration_per_worker_s": args.duration,
            "base_seed": args.seed,
            "wall_s": wall_s,
            "totals": {
                "n_checks": n_checks,
                "n_collisions": n_collisions,
                "n_pairs_total": n_pairs_total,
                "distinct_pairs": len(seen_pairs),
            },
        },
        "scene": {
            "link_names": link_names,
            "body_names": body_names,
            "tool_names": tool_names,
        },
        "observed_pairs": sorted(["{} <-> {}".format(a, b) for a, b in seen_pairs]),
        "per_rigid_body": per_body,
        "per_tool": per_tool,
        "per_link": per_link,
    }

    _write_json(JSON_OUT_PATH, payload)
    _write_markdown(
        MD_OUT_PATH,
        per_body,
        per_tool,
        per_link,
        link_names,
        body_names,
        tool_names,
        seen_pairs,
        totals,
        args,
        worker_results,
    )

    print("\nWrote: {}".format(JSON_OUT_PATH))
    print("Wrote: {}".format(MD_OUT_PATH))


if __name__ == "__main__":
    main()
