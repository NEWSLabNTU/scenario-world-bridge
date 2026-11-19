#!/usr/bin/env python3
"""078 P2g: Autoware's launch from rest and czp's seen-start delay, from the P2f crossing grid.

    python3 scripts/coop/p2g_launch.py GRID_DIR [--json OUT.json]

GRID_DIR is P2f's `grid/` (NAS "2026-11-18 078 P2 crossing grid/grid"): per run, coop_a.jsonl
(both cars' ground truth at 20 Hz: `u` the signed distance of the car's front to the conflict
zone, `v` its speed; B is the Autoware ego) and, on the czp arm, decisions.jsonl (czp's
decisions as they change, on sim time 1000 + elapsed). Clocks: `tr` = `el` - shift, with the
shift from coop_a's own ticks; a decision's `tr` = (t - 1000) - shift.

Measured (all in seconds):

- **czp go from rest -> motion -> zone.** Each czp `go` in the crossing's zone while the ego is
  at rest (v < REST_V for >= 0.5 s): to the ego's first motion (v >= MOTION_V), to v >=
  moved_speed (0.5 m/s), and to entering the zone (u >= 0). "clean": czp did not decide
  anything but go/inside between the go and the motion.
- **Autoware's kinematic launch** (both arms): from the first motion after a rest before the
  zone to entering it, with the rest's distance to the zone.
- **Stock's own release -> motion** (stock arm): the intersection module's `safe` going true at
  rest (bagfacts' rtc_changes) to the first motion.
- **Seen-start delay** (czp arm): each restart of the other car (A) after a rest >= 1 s: from
  its first motion (and from v >= moved_speed) to czp's first decision that reacts to it, i.e.
  the first decision change after the start whose per-agent verdict for A is yield, or whose
  reason is `turn`.
"""
import argparse
import glob
import json
import os
import statistics as st
import sys

REST_V = 0.05      # m/s: at rest below it (ground truth)
MOTION_V = 0.05    # m/s: first motion
MOVED_V = 0.5      # m/s: czp's moved_speed
REST_MIN = 0.5     # s at rest before a go counts as a go from rest
A_REST_MIN = 1.0   # s at rest before an A restart counts


def load(run):
    ticks = [json.loads(line) for line in open(os.path.join(run, "coop_a.jsonl"))]
    ticks = [c for c in ticks if c.get("tr") is not None and "ub" in c]
    if not ticks:
        return None
    shift = round(ticks[0]["el"] - ticks[0]["tr"], 3)
    dec = []
    f = os.path.join(run, "decisions.jsonl")
    if os.path.exists(f):
        for line in open(f):
            d = json.loads(line)
            d["tr"] = round(d["t"] - 1000.0 - shift, 3)
            dec.append(d)
    return ticks, dec


def zone_of(dec):
    """The crossing's zone: the one with most decisions that name an agent."""
    count = {}
    for d in dec:
        if d.get("agents"):
            count[d["zone"]] = count.get(d["zone"], 0) + 1
    return max(count, key=count.get) if count else None


def rests(ticks, key):
    out, start = [], None
    for c in ticks:
        if abs(c[key]) < REST_V:
            start = c["tr"] if start is None else start
        elif start is not None:
            out.append((start, c["tr"]))
            start = None
    if start is not None:
        out.append((start, None))
    return out


def first(ticks, t0, pred):
    for c in ticks:
        if c["tr"] >= t0 - 1e-9 and pred(c):
            return c["tr"]
    return None


def at(ticks, t):
    best = None
    for c in ticks:
        if c["tr"] <= t + 1e-9:
            best = c
        else:
            break
    return best


def go_launches(ticks, dec, zone):
    out = []
    zd = [d for d in dec if d["zone"] == zone]
    for i, d in enumerate(zd):
        if d["decision"] != "go" or d["reason"] == "inside":
            continue
        c = at(ticks, d["tr"])
        if c is None or abs(c["vb"]) >= REST_V or c["ub"] >= 0:
            continue
        rest_start = max((s for s, e in rests(ticks, "vb") if s <= d["tr"] and (e is None or e > d["tr"])),
                         default=None)
        if rest_start is None or d["tr"] - rest_start < REST_MIN:
            continue
        t_move = first(ticks, d["tr"], lambda c: abs(c["vb"]) >= MOTION_V)
        t_moved = first(ticks, d["tr"], lambda c: abs(c["vb"]) >= MOVED_V)
        t_in = first(ticks, d["tr"], lambda c: c["ub"] >= 0)
        between = [x["decision"] for x in zd[i + 1:] if t_move is None or x["tr"] < t_move]
        out.append({"t_go": d["tr"], "reason": d["reason"], "u_rest": round(c["ub"], 2),
                    "go_move": None if t_move is None else round(t_move - d["tr"], 2),
                    "go_moved": None if t_moved is None else round(t_moved - d["tr"], 2),
                    "go_in": None if t_in is None else round(t_in - d["tr"], 2),
                    "clean": all(x in ("go",) for x in between)})
    return out


def kinematic_launches(ticks):
    """Each rest of the ego before the zone, then its first motion to the zone entry."""
    out = []
    for s, e in rests(ticks, "vb"):
        if e is None or e - s < REST_MIN:
            continue
        c = at(ticks, s)
        if c is None or c["ub"] >= 0 or c["ub"] < -15:
            continue
        t_in = first(ticks, e, lambda c: c["ub"] >= 0)
        t_moved = first(ticks, e, lambda c: abs(c["vb"]) >= MOVED_V)
        if t_in is None:
            continue
        out.append({"t_move": e, "u_rest": round(c["ub"], 2), "move_in": round(t_in - e, 2),
                    "move_moved": None if t_moved is None else round(t_moved - e, 2)})
    return out


def stock_release(ticks, facts):
    """Stock arm: the intersection module's safe -> true while the ego rests, to its motion."""
    out = []
    for s, e in rests(ticks, "vb"):
        if e is None or e - s < REST_MIN:
            continue
        rel = [r["tr"] for r in facts.get("rtc_changes", []) if r.get("safe") and s <= r["tr"] <= e]
        if rel:
            out.append({"t_safe": rel[-1], "safe_move": round(e - rel[-1], 2)})
    return out


def reacts(d, agent=None):
    if d["reason"] == "turn":
        return True
    for aid, a in (d.get("agents") or {}).items():
        if (agent is None or aid == agent) and a.get("verdict") == "yield":
            return True
    return False


def seen_starts(ticks, dec, zone):
    out = []
    zd = [d for d in dec if d["zone"] == zone]
    for s, e in rests(ticks, "va"):
        if e is None or e - s < A_REST_MIN:
            continue
        t_moved = first(ticks, e, lambda c: abs(c["va"]) >= MOVED_V)
        prior = [d for d in zd if d["tr"] < e]
        if prior and reacts(prior[-1]):
            continue  # czp already yields to A: nothing to react to
        r = next((d for d in zd if d["tr"] >= e - 1e-9 and reacts(d)), None)
        c = at(ticks, e)
        out.append({"t_start": e, "rest": round(e - s, 2), "ego_u": None if c is None else round(c["ub"], 2),
                    "ego_v": None if c is None else round(c["vb"], 2),
                    "react": None if r is None else r["reason"],
                    "start_react": None if r is None else round(r["tr"] - e, 2),
                    "moved_react": None if (r is None or t_moved is None) else round(r["tr"] - t_moved, 2)})
    return out


def dist(xs):
    xs = sorted(x for x in xs if x is not None)
    if not xs:
        return {"n": 0}
    q = lambda p: xs[min(len(xs) - 1, int(round(p * (len(xs) - 1))))]
    return {"n": len(xs), "min": xs[0], "p10": q(0.1), "median": round(st.median(xs), 2), "p90": q(0.9),
            "max": xs[-1], "mean": round(st.mean(xs), 2)}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("grid")
    ap.add_argument("--json")
    a = ap.parse_args(argv)
    runs = {}
    for run in sorted(glob.glob(os.path.join(a.grid, "*_s[0-9]*"))):
        if not os.path.isdir(run):
            continue
        tag = os.path.basename(run)
        got = load(run)
        if got is None:
            continue
        ticks, dec = got
        fpath = os.path.join(a.grid, "bagfacts", tag + ".json")
        facts = json.load(open(fpath)) if os.path.exists(fpath) else {}
        zone = zone_of(dec)
        runs[tag] = {"arm": tag.split("_")[0], "zone": zone,
                     "go": go_launches(ticks, dec, zone) if zone else [],
                     "kin": kinematic_launches(ticks),
                     "stock_release": stock_release(ticks, facts) if tag.startswith("off") else [],
                     "seen": seen_starts(ticks, dec, zone) if zone else []}
    go = [g for r in runs.values() for g in r["go"]]
    clean = [g for g in go if g["clean"]]
    kin = [k for r in runs.values() for k in r["kin"]]
    seen = [s for r in runs.values() for s in r["seen"]]
    summary = {
        "runs": {"on": sum(r["arm"] == "on" for r in runs.values()),
                 "off": sum(r["arm"] == "off" for r in runs.values())},
        "czp_go_from_rest": {"n": len(go), "clean": len(clean),
                             "go_to_motion": dist([g["go_move"] for g in clean]),
                             "go_to_moved_speed": dist([g["go_moved"] for g in clean]),
                             "go_to_zone": dist([g["go_in"] for g in clean]),
                             "by_reason": {k: sum(g["reason"] == k for g in go) for k in sorted({g["reason"] for g in go})}},
        "kinematic_launch": {"motion_to_zone": dist([k["move_in"] for k in kin]),
                             "motion_to_moved_speed": dist([k["move_moved"] for k in kin]),
                             "rest_distance_to_zone": dist([-k["u_rest"] for k in kin])},
        "stock_release_to_motion": dist([s["safe_move"] for r in runs.values() for s in r["stock_release"]]),
        "seen_start": {"n": len(seen), "reacted": sum(s["react"] is not None for s in seen),
                       "start_to_reaction": dist([s["start_react"] for s in seen]),
                       "moved_speed_to_reaction": dist([s["moved_react"] for s in seen]),
                       "by_reaction": {k: sum(str(s["react"]) == k for s in seen) for k in sorted({str(s["react"]) for s in seen})}},
    }
    out = {"summary": summary, "runs": runs}
    if a.json:
        with open(a.json, "w") as f:
            json.dump(out, f, indent=1)
    json.dump(summary, sys.stdout, indent=1)
    print()


if __name__ == "__main__":
    main()
