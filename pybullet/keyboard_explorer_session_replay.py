"""Offline replay + analysis for keyboard-explorer session logs.

A session log file is JSON-Lines (``.jsonl``), written by
``bullet_collision_keyboard_explorer.py`` into ``pybullet/explorer_logs/``:

  - line 1: a header dict with all constants needed to reconstruct the
    motion model (joint limits, forward-check geometry, probe offsets,
    keymap, defaults, etc.).
  - lines 2..N: one row per tick with inputs, intermediate scalars,
    final output, pose, velocity, the cached forward + proximity worker
    snapshots (bit-packed), and the active tunables for that tick.

This file does NOT need PyBullet. It re-executes the exact same motion-
model arithmetic that the live explorer runs in ``_tick`` and compares
the recomputed values against what was recorded. Any disagreement (above
a tiny float tolerance) indicates a discrepancy between the explorer
code and this offline model, which is exactly the situation you want to
catch when reproducing instabilities.

Usage
-----
    # Summary + replay-mismatch check
    python pybullet/keyboard_explorer_session_replay.py LOG.jsonl

    # Inspect a single tick (recorded + recomputed side by side)
    python pybullet/keyboard_explorer_session_replay.py LOG.jsonl --tick 357

    # Find ticks where motion happened while obstacle was within cutoff
    # (i.e. 'creep into obstacle' suspects)
    python pybullet/keyboard_explorer_session_replay.py LOG.jsonl --creep

    # Find the N largest replay mismatches
    python pybullet/keyboard_explorer_session_replay.py LOG.jsonl --top-mismatch 10
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from dataclasses import dataclass

# ---------------------------------------------------------------------------
# I/O
# ---------------------------------------------------------------------------


def unpack_bits(value: int, length: int) -> list[bool]:
    return [bool((value >> i) & 1) for i in range(length)]


@dataclass
class Session:
    header: dict
    rows: list[dict]
    path: str

    @classmethod
    def load(cls, path: str) -> "Session":
        rows = []
        header = None
        with open(path, "r", encoding="utf-8") as f:
            for i, line in enumerate(f):
                line = line.strip()
                if not line:
                    continue
                obj = json.loads(line)
                if i == 0:
                    if not obj.get("header"):
                        raise ValueError(
                            "First line of {} is not a header dict".format(path)
                        )
                    header = obj
                else:
                    rows.append(obj)
        if header is None:
            raise ValueError("No header found in {}".format(path))
        return cls(header=header, rows=rows, path=path)


# ---------------------------------------------------------------------------
# Motion model (mirror of KeyboardExplorer._tick)
# ---------------------------------------------------------------------------


def _accel_clamp(v_cur: float, v_des: float, max_accel: float, dt: float) -> float:
    dv_max = max_accel * dt
    return v_cur + max(-dv_max, min(dv_max, v_des - v_cur))


def compute_path_scalar(
    fwd_bits: list[bool],
    step_deg: float,
    n_steps: int,
    cutoff_deg: float,
    shape: str,
    exp_k: float,
) -> tuple[float, float | None]:
    """Return (scalar, nearest_deg or None). Identical to the live code."""
    for k, hit in enumerate(fwd_bits):
        if hit:
            dist_deg = (k + 1) * step_deg
            if dist_deg <= cutoff_deg:
                return 0.0, dist_deg
            max_dist = n_steps * step_deg
            norm = (dist_deg - cutoff_deg) / max(1e-6, max_dist - cutoff_deg)
            norm = max(0.0, min(1.0, norm))
            if shape == "linear":
                scale = norm
            else:
                k_s = max(0.1, exp_k)
                scale = max(0.0, min(1.0, 1.0 - math.exp(-k_s * norm)))
            return scale, dist_deg
    return 1.0, None


def compute_prox_scalar(
    prox_bits_per_axis: list[list[bool]],
    probe_half_deg: int,
    floor_pct: float,
) -> tuple[float, float | None]:
    floor = max(0.0, min(1.0, floor_pct / 100.0))
    nearest = None
    for axis_bits in prox_bits_per_axis:
        for j in range(probe_half_deg):
            if axis_bits[probe_half_deg - 1 - j]:
                d = j + 1
                if nearest is None or d < nearest:
                    nearest = d
                break
        for j in range(probe_half_deg):
            if axis_bits[probe_half_deg + j]:
                d = j + 1
                if nearest is None or d < nearest:
                    nearest = d
                break
    if nearest is None:
        return 1.0, None
    if probe_half_deg <= 1:
        return floor, nearest
    frac = (nearest - 1) / (probe_half_deg - 1)
    frac = max(0.0, min(1.0, frac))
    return floor + (1.0 - floor) * frac, nearest


def replay_tick(state: dict, row: dict, header: dict) -> dict:
    """Recompute one tick from `state` (carried) and `row` (recorded inputs).

    Returns a dict matching the recorded fields (v_cmd, v_out, pos, vel,
    ps, qs, fs, p_near, q_near).
    """
    dt = row["dt"]
    cfg = row["cfg"]
    n_steps = header["n_forward_steps"]
    step_deg = header["forward_step_deg"]
    probe_half = header["probe_half_deg"]
    probe_len = len(header["probe_offsets_deg"])
    joint_limits_deg = header["joint_limits_deg"]

    # 1. v_des already given (deg/s). Convert to rad/s? We can stay in dps
    #    throughout for simplicity since everything else is also dps.
    v_des = row["v_des"]

    # 2. accel-clamp from carried vel toward v_des
    new_v = [
        _accel_clamp(state["vel"][i], v_des[i], cfg["ma"][i], dt) for i in range(6)
    ]
    # 3. velocity clamp
    for i in range(6):
        new_v[i] = max(-cfg["mv"][i], min(cfg["mv"][i], new_v[i]))
    v_cmd = new_v

    # 4. path scalar
    fwd_bits = unpack_bits(row["fwd"], n_steps)
    ps, p_near = compute_path_scalar(
        fwd_bits, step_deg, n_steps, cfg["pc"], cfg["sh"], cfg["ek"]
    )
    # 5. prox scalar
    prox_bits = [unpack_bits(p, probe_len) for p in row["prox"]]
    qs, q_near = compute_prox_scalar(prox_bits, probe_half, cfg["pf"])

    fs = min(ps, qs)
    v_out = [v * fs for v in v_cmd]

    # 6. integrate then joint-limit clamp
    new_vel = list(v_out)
    new_pos = [state["pos"][i] + v_out[i] * dt for i in range(6)]
    for i, (lo, hi) in enumerate(joint_limits_deg):
        if new_pos[i] < lo:
            new_pos[i] = lo
            if new_vel[i] < 0:
                new_vel[i] = 0.0
        elif new_pos[i] > hi:
            new_pos[i] = hi
            if new_vel[i] > 0:
                new_vel[i] = 0.0

    return {
        "v_cmd": v_cmd,
        "v_out": v_out,
        "pos": new_pos,
        "vel": new_vel,
        "ps": ps,
        "qs": qs,
        "fs": fs,
        "p_near": p_near,
        "q_near": q_near,
    }


def replay_session(session: Session) -> list[dict]:
    """Replay every row; return list of dicts {row, recomputed, mismatch}.

    The carried state (pos, vel) is taken from the PRIOR row's recorded
    values, so replay errors do not accumulate -- each tick is checked
    against the live recording in isolation.
    """
    header = session.header
    out = []
    prev_pos = list(header["initial_pos_deg"])
    prev_vel = [0.0] * 6
    for row in session.rows:
        state = {"pos": prev_pos, "vel": prev_vel}
        rec = replay_tick(state, row, header)
        mismatch = _diff(row, rec)
        out.append({"row": row, "rec": rec, "mismatch": mismatch})
        prev_pos = list(row["pos"])  # carry from RECORDED state
        prev_vel = list(row["vel"])
    return out


def _diff(row: dict, rec: dict) -> dict:
    """Per-field max-abs difference between recorded and recomputed."""
    out = {}
    for key in ("v_cmd", "v_out", "pos", "vel"):
        a = row.get(key)
        b = rec.get(key)
        if a is None or b is None:
            continue
        out[key] = max(abs(ai - bi) for ai, bi in zip(a, b))
    for key in ("ps", "qs", "fs"):
        out[key] = abs(row.get(key, 0.0) - rec.get(key, 0.0))
    out["max"] = max(
        max(out.get("v_cmd", 0.0), out.get("v_out", 0.0)),
        max(out.get("pos", 0.0), out.get("vel", 0.0)),
        max(out.get("ps", 0.0), out.get("qs", 0.0), out.get("fs", 0.0)),
    )
    return out


# ---------------------------------------------------------------------------
# Reports
# ---------------------------------------------------------------------------


def _vnorm(vec) -> float:
    return math.sqrt(sum(v * v for v in vec))


def print_summary(session: Session, replay: list[dict]) -> None:
    h = session.header
    n = len(replay)
    if n == 0:
        print("Empty session.")
        return
    dur = session.rows[-1]["t"] - session.rows[0]["t"]
    fps_eff = (n - 1) / dur if dur > 0 else 0.0
    max_v_out = max(_vnorm(r["row"]["v_out"]) for r in replay)
    max_v_des = max(_vnorm(r["row"]["v_des"]) for r in replay)
    n_path_zero = sum(1 for r in replay if r["row"]["ps"] == 0.0)
    n_in_coll = sum(1 for r in replay if r["row"].get("in_coll"))
    worst = max(replay, key=lambda r: r["mismatch"]["max"])
    print("=" * 70)
    print("Session   : {}".format(session.path))
    print("Started   : {}".format(h.get("started_at")))
    print("Ticks     : {}".format(n))
    print("Duration  : {:.2f} s   (eff fps = {:.1f})".format(dur, fps_eff))
    print("Max |v_des| = {:.2f} dps".format(max_v_des))
    print("Max |v_out| = {:.2f} dps".format(max_v_out))
    print(
        "path_scalar == 0 : {} ticks ({:.1f}%)".format(
            n_path_zero, 100.0 * n_path_zero / n
        )
    )
    print(
        "in_collision     : {} ticks ({:.1f}%)".format(n_in_coll, 100.0 * n_in_coll / n)
    )
    print("-" * 70)
    print("Replay mismatch (worst tick):")
    print(
        "  tick #{n}   max-diff = {d:.3e}".format(
            n=worst["row"]["n"], d=worst["mismatch"]["max"]
        )
    )
    print("  per-field:", {k: "{:.3e}".format(v) for k, v in worst["mismatch"].items()})
    if worst["mismatch"]["max"] > 1e-6:
        print("  WARNING: replay does not match recording above 1e-6.")
        print("           the offline model may be out of sync with the explorer.")
    else:
        print("  OK: offline replay reproduces recording within 1e-6.")
    print("=" * 70)


def print_tick(session: Session, replay: list[dict], n: int) -> None:
    matches = [r for r in replay if r["row"]["n"] == n]
    if not matches:
        print("No tick #{} in this session.".format(n))
        return
    r = matches[0]
    row = r["row"]
    rec = r["rec"]
    print(
        "Tick #{}   t={:.3f}s   dt={:.4f}s   keys={}".format(
            row["n"], row["t"], row["dt"], row["keys"]
        )
    )
    print(
        "  cfg     : mv={mv}  ma={ma}  slow={slow} fast={fast}  pf={pf} pc={pc} sh={sh} ek={ek}".format(
            **row["cfg"]
        )
    )
    print("  v_des   : {}".format(_fmt_vec(row["v_des"])))
    print("  v_cmd   : recorded {}".format(_fmt_vec(row["v_cmd"])))
    print("            replayed {}".format(_fmt_vec(rec["v_cmd"])))
    print("  v_out   : recorded {}".format(_fmt_vec(row["v_out"])))
    print("            replayed {}".format(_fmt_vec(rec["v_out"])))
    print("  pos     : recorded {}".format(_fmt_vec(row["pos"])))
    print("            replayed {}".format(_fmt_vec(rec["pos"])))
    print("  vel     : recorded {}".format(_fmt_vec(row["vel"])))
    print("            replayed {}".format(_fmt_vec(rec["vel"])))
    print("  scalars : ps={:.4f} (rec) vs {:.4f} (rep)".format(row["ps"], rec["ps"]))
    print("            qs={:.4f} (rec) vs {:.4f} (rep)".format(row["qs"], rec["qs"]))
    print("            fs={:.4f} (rec) vs {:.4f} (rep)".format(row["fs"], rec["fs"]))
    print("  near    : path={}  prox={}".format(row["p_near"], row["q_near"]))
    print("  in_coll : {}".format(row["in_coll"]))
    print(
        "  fwd bits: {}".format(
            unpack_bits(row["fwd"], session.header["n_forward_steps"])
        )
    )
    print(
        "  mismatch: {}".format(
            {k: "{:.3e}".format(v) for k, v in r["mismatch"].items()}
        )
    )


def _fmt_vec(v) -> str:
    return "[" + ", ".join("{:+7.2f}".format(x) for x in v) + "]"


def find_creep(
    session: Session, replay: list[dict], v_thresh_dps: float = 0.05
) -> list[dict]:
    """Ticks where |v_out|>thresh but nearest path obstacle is at or below cutoff.

    These are the suspect cases for 'pump key -> creep into obstacle'.
    """
    suspects = []
    for r in replay:
        row = r["row"]
        v_out_norm = _vnorm(row["v_out"])
        if v_out_norm <= v_thresh_dps:
            continue
        p_near = row.get("p_near")
        cutoff = row["cfg"]["pc"]
        if p_near is not None and p_near <= cutoff:
            suspects.append(r)
    return suspects


def find_top_mismatches(replay: list[dict], n: int = 10) -> list[dict]:
    return sorted(replay, key=lambda r: -r["mismatch"]["max"])[:n]


# ---------------------------------------------------------------------------
# Full-session analyzer
# ---------------------------------------------------------------------------


def _percentile(sorted_xs: list[float], p: float) -> float:
    if not sorted_xs:
        return 0.0
    k = max(0, min(len(sorted_xs) - 1, int(round((p / 100.0) * (len(sorted_xs) - 1)))))
    return sorted_xs[k]


def _hist(xs: list[float], edges: list[float]) -> list[int]:
    out = [0] * (len(edges) + 1)  # last bin = > edges[-1]
    for x in xs:
        placed = False
        for i, e in enumerate(edges):
            if x <= e:
                out[i] += 1
                placed = True
                break
        if not placed:
            out[-1] += 1
    return out


def analyze(session: Session, replay: list[dict]) -> None:
    h = session.header
    rows = [r["row"] for r in replay]
    n = len(rows)
    if n < 2:
        print("Not enough ticks to analyze.")
        return
    n_steps = h["n_forward_steps"]
    step_deg = h["forward_step_deg"]
    probe_half = h["probe_half_deg"]
    probe_len = len(h["probe_offsets_deg"])

    # --- performance ---
    dts = [r["dt"] for r in rows[1:]]  # skip first (cold)
    dts_sorted = sorted(dts)
    fps_overall = (n - 1) / (rows[-1]["t"] - rows[0]["t"])

    # split by "was forward dispatched" (v_cmd norm > 0.5 dps)
    moving_dts = []
    idle_dts = []
    for r in rows[1:]:
        vn = _vnorm(r["v_cmd"])
        (moving_dts if vn > 0.5 else idle_dts).append(r["dt"])

    print("\n" + "=" * 70)
    print("PERFORMANCE")
    print("=" * 70)
    print(
        "  total ticks      : {:d}  ({:.1f} s of recording)".format(
            n, rows[-1]["t"] - rows[0]["t"]
        )
    )
    print(
        "  effective FPS    : {:.2f}    (target {:g})".format(
            fps_overall, h["defaults"].get("target_fps", 30)
        )
    )
    print(
        "  dt overall  ms   : median={:.1f}  p90={:.1f}  p99={:.1f}  max={:.1f}".format(
            1000 * _percentile(dts_sorted, 50),
            1000 * _percentile(dts_sorted, 90),
            1000 * _percentile(dts_sorted, 99),
            1000 * dts_sorted[-1],
        )
    )
    if moving_dts:
        ms = sorted(moving_dts)
        print(
            "  dt MOVING   ms   : n={:d}  median={:.1f}  p90={:.1f}  p99={:.1f}".format(
                len(ms),
                1000 * _percentile(ms, 50),
                1000 * _percentile(ms, 90),
                1000 * _percentile(ms, 99),
            )
        )
    if idle_dts:
        ids = sorted(idle_dts)
        print(
            "  dt IDLE     ms   : n={:d}  median={:.1f}  p90={:.1f}  p99={:.1f}".format(
                len(ids),
                1000 * _percentile(ids, 50),
                1000 * _percentile(ids, 90),
                1000 * _percentile(ids, 99),
            )
        )
        # Forward-dispatch cost estimate = moving median - idle median
        if moving_dts:
            fwd_cost_ms = 1000 * (
                _percentile(sorted(moving_dts), 50) - _percentile(ids, 50)
            )
            print(
                "  forward dispatch cost (median moving - median idle):"
                "  {:+.1f} ms / tick".format(fwd_cost_ms)
            )

    # --- safety: how close did we get? ---
    p_nears = [r["p_near"] for r in rows if r["p_near"] is not None]
    q_nears = [r["q_near"] for r in rows if r["q_near"] is not None]
    in_coll = [r for r in rows if r.get("in_coll")]

    print("\n" + "=" * 70)
    print("SAFETY")
    print("=" * 70)
    print(
        "  in_collision      : {:d} tick(s)  ({:.2f}% of session)".format(
            len(in_coll), 100.0 * len(in_coll) / n
        )
    )
    if in_coll:
        print("    first 10 collision ticks:")
        for r in in_coll[:10]:
            print(
                "      #{:5d}  t={:6.2f}s  keys={}  pos={}".format(
                    r["n"], r["t"], r["keys"], _fmt_vec(r["pos"])
                )
            )

    print(
        "  ticks w/ fwd hit  : {:d}  ({:.1f}% of session)".format(
            len(p_nears), 100.0 * len(p_nears) / n
        )
    )
    if p_nears:
        ps = sorted(p_nears)
        print(
            "  p_near deg        : min={:.0f}  p10={:.0f}  median={:.0f}  max={:.0f}".format(
                ps[0],
                _percentile(ps, 10),
                _percentile(ps, 50),
                ps[-1],
            )
        )
        # bucket histogram of p_near in degrees (1..12)
        edges = list(range(1, n_steps + 1))
        hist = _hist(ps, edges)
        labels = ["<= {:d} deg".format(e) for e in edges] + [
            "> {:d} deg".format(edges[-1])
        ]
        print("  p_near histogram :")
        for lbl, c in zip(labels, hist):
            bar = "#" * int(60 * c / max(1, max(hist)))
            print("    {:<14s} {:5d}  {}".format(lbl, c, bar))

    # Closest cartesian-ish proxy: minimum p_near across all motion ticks
    moving_with_hit = [
        r for r in rows if r["p_near"] is not None and _vnorm(r["v_out"]) > 0.05
    ]
    if moving_with_hit:
        ps = sorted(r["p_near"] for r in moving_with_hit)
        print(
            "  WHILE MOVING (|v_out|>0.05dps) and fwd hit present:"
        )
        print(
            "    p_near deg     : min={:.0f}  p10={:.0f}  median={:.0f}".format(
                ps[0], _percentile(ps, 10), _percentile(ps, 50)
            )
        )

    print(
        "  ticks w/ prox hit : {:d}  ({:.1f}% of session)".format(
            len(q_nears), 100.0 * len(q_nears) / n
        )
    )
    if q_nears:
        qs = sorted(q_nears)
        print(
            "  q_near deg        : min={:.0f}  p10={:.0f}  median={:.0f}  max={:.0f}".format(
                qs[0],
                _percentile(qs, 10),
                _percentile(qs, 50),
                qs[-1],
            )
        )

    # Detect "frozen against obstacle" runs: ps==0 streaks
    streaks = []
    cur = 0
    for r in rows:
        if r["ps"] == 0.0:
            cur += 1
        else:
            if cur > 0:
                streaks.append(cur)
            cur = 0
    if cur:
        streaks.append(cur)
    if streaks:
        s = sorted(streaks, reverse=True)
        print("  path-stop streaks : count={:d}  longest={:d} ticks  median={:d}".format(
            len(s), s[0], s[len(s) // 2]
        ))

    # --- closest approach across whole session ---
    # joint-space distance is bounded by min(p_near, q_near). Smaller = more dangerous.
    closest = None
    closest_tick = None
    for r in rows:
        d_candidates = []
        if r["p_near"] is not None:
            d_candidates.append(r["p_near"])
        if r["q_near"] is not None:
            d_candidates.append(r["q_near"])
        if not d_candidates:
            continue
        d = min(d_candidates)
        if closest is None or d < closest:
            closest = d
            closest_tick = r["n"]
    if closest is not None:
        print(
            "  closest approach  : {:.0f} deg (joint-space, at tick #{:d})".format(
                closest, closest_tick
            )
        )

    # --- input usage ---
    active_ticks = sum(1 for r in rows if r["keys"])
    print("\n" + "=" * 70)
    print("INPUT / MOTION")
    print("=" * 70)
    print(
        "  active ticks (any key) : {:d}  ({:.1f}% of session)".format(
            active_ticks, 100.0 * active_ticks / n
        )
    )
    # per-axis motion in degrees integrated
    axis_motion = [0.0] * 6
    for r in rows:
        for i in range(6):
            axis_motion[i] += abs(r["v_out"][i]) * r["dt"]
    print("  total |motion| per axis (deg):")
    for i, name in enumerate(h["joint_names"]):
        print("    J{:d} {:<22s} {:7.1f} deg".format(i, name, axis_motion[i]))
    # key usage count
    key_count: dict[str, int] = {}
    for r in rows:
        for k in r["keys"]:
            key_count[k] = key_count.get(k, 0) + 1
    if key_count:
        top = sorted(key_count.items(), key=lambda kv: -kv[1])[:10]
        print("  top 10 keys by frequency:")
        for k, c in top:
            print("    '{}'  {:d} ticks".format(k, c))

    # --- scalar distribution ---
    print("\n" + "=" * 70)
    print("CLAMP SCALAR DISTRIBUTION (final_scalar = min(path, prox))")
    print("=" * 70)
    edges = [0.0, 0.25, 0.5, 0.75, 0.99, 1.0]
    labels = [
        "== 0.00         (full stop, path block)",
        "(0.00, 0.25]    (heavy clamp)",
        "(0.25, 0.50]    (moderate clamp)",
        "(0.50, 0.75]    (prox floor region)",
        "(0.75, 0.99]    (mild slowdown)",
        "== 1.00         (free motion)",
    ]
    buckets = [0] * len(labels)
    for r in rows:
        fs = r["fs"]
        if fs == 0.0:
            buckets[0] += 1
        elif fs <= 0.25:
            buckets[1] += 1
        elif fs <= 0.50:
            buckets[2] += 1
        elif fs <= 0.75:
            buckets[3] += 1
        elif fs < 1.0:
            buckets[4] += 1
        else:
            buckets[5] += 1
    for lbl, c in zip(labels, buckets):
        pct = 100.0 * c / n
        bar = "#" * int(60 * c / max(1, max(buckets)))
        print("  {:<42s} {:5d}  {:5.1f}%  {}".format(lbl, c, pct, bar))


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _resolve_path(arg: str) -> str:
    if os.path.exists(arg):
        return arg
    here = os.path.dirname(os.path.abspath(__file__))
    cand = os.path.join(here, "explorer_logs", arg)
    if os.path.exists(cand):
        return cand
    raise SystemExit("Log file not found: {}".format(arg))


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("log", help="path to a session_*.jsonl file")
    p.add_argument("--tick", type=int, help="print a single tick by index")
    p.add_argument(
        "--creep",
        action="store_true",
        help="list ticks where v_out > 0 but obstacle <= cutoff (suspect creep)",
    )
    p.add_argument(
        "--top-mismatch",
        type=int,
        metavar="N",
        default=0,
        help="print the N ticks with the largest replay vs recording diff",
    )
    p.add_argument(
        "--v-thresh",
        type=float,
        default=0.05,
        help="dps threshold for the --creep filter (default 0.05)",
    )
    p.add_argument(
        "--analyze",
        action="store_true",
        help="full-session performance + safety analysis (timing, p_near hist, etc.)",
    )
    args = p.parse_args(argv)

    path = _resolve_path(args.log)
    session = Session.load(path)
    replay = replay_session(session)

    if args.tick is not None:
        print_tick(session, replay, args.tick)
        return 0

    print_summary(session, replay)

    if args.analyze:
        analyze(session, replay)

    if args.creep:
        suspects = find_creep(session, replay, args.v_thresh)
        print(
            "\n[creep] {} tick(s) where |v_out| > {:.2f} dps but path obstacle <= cutoff:".format(
                len(suspects), args.v_thresh
            )
        )
        for r in suspects[:50]:
            row = r["row"]
            print(
                "  #{n:5d} t={t:6.2f}s  |v_out|={vo:5.2f} dps  p_near={pn} cutoff={c}  keys={k}".format(
                    n=row["n"],
                    t=row["t"],
                    vo=_vnorm(row["v_out"]),
                    pn=row["p_near"],
                    c=row["cfg"]["pc"],
                    k=row["keys"],
                )
            )
        if len(suspects) > 50:
            print("  ... ({} more)".format(len(suspects) - 50))

    if args.top_mismatch:
        worst = find_top_mismatches(replay, args.top_mismatch)
        print("\n[top mismatch] {} worst ticks:".format(args.top_mismatch))
        for r in worst:
            row = r["row"]
            print(
                "  #{n:5d} t={t:6.2f}s  max-diff={d:.3e}".format(
                    n=row["n"], t=row["t"], d=r["mismatch"]["max"]
                )
            )

    return 0


if __name__ == "__main__":
    sys.exit(main())
