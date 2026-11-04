#!/usr/bin/env python3
"""Roadmap 072, the tour: one row per leg from a run_tour batch directory.

Reads legs.log (wall start and end per leg), monitor.jsonl (tour_monitor.py), NN_state.jsonl
(carla_watch_video.py --state) and NN/scenario_test_runner/result.junit.xml. Per leg: junit,
driven m, sim time, MRM episodes, stops away from the goal (with the velocity factors standing
then, and where), NVTL min and mean while moving.

    python3 scripts/ntu_carla/tour_table.py RUN_DIR [--json OUT.json]
"""
import argparse
import json
import math
import pathlib
import re

ORIGIN = (52500.0, 67700.0)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("run")
    ap.add_argument("--json")
    ap.add_argument("--legs", default=str(pathlib.Path(__file__).resolve().parents[2]
                                          / "scenarios/ntu_twin/tour_legs.json"))
    a = ap.parse_args()
    d = pathlib.Path(a.run)
    legs = {f"{r['leg']:02d}": r for r in json.load(open(a.legs))}
    win = {}
    for line in open(d / "legs.log"):
        m = re.match(r"== (\S+) start (\d+)", line)
        if m:
            win[m.group(1)] = [int(m.group(2)), None, ""]
        m = re.match(r"== (\S+) end (\d+) rc=(\d+) junit=\[([^\]]*)\]", line)
        if m:
            win[m.group(1)][1] = int(m.group(2))
            win[m.group(1)][2] = m.group(4)
    mon = [json.loads(x) for x in open(d / "monitor.jsonl")] if (d / "monitor.jsonl").exists() else []
    rows = []
    for nn, (t0, t1, junit) in win.items():
        if t1 is None:
            continue
        ev = [x for x in mon if t0 <= x["wall"] <= t1]
        st = d / f"{nn}_state.jsonl"
        states = [json.loads(x) for x in open(st)] if st.exists() else []
        driven = sum(math.hypot(b["x"] - p["x"], b["y"] - p["y"]) for p, b in zip(states, states[1:]))
        goal = legs[nn]["goal"] if nn in legs else None
        moved = [x for x in ev if x["kind"] == "s" and x.get("speed") and x["speed"] > 0.3]
        nv = [x["nvtl"] for x in moved if x.get("nvtl") is not None]
        mrm = [x for x in ev if x["kind"] == "mrm" and x["state"] not in ("NORMAL", "UNKNOWN")]
        # stops: samples at rest, engaged (after the first motion), away from the goal
        stops, cur = [], None
        first = moved[0]["wall"] if moved else None
        for x in ev:
            if x["kind"] != "s" or first is None or x["wall"] < first or x.get("pose") is None:
                continue
            far = goal is None or math.hypot(x["pose"][0] - goal[0], x["pose"][1] - goal[1]) > 6.0
            if x["speed"] is not None and x["speed"] < 0.05 and far and x.get("route") != "ARRIVED":
                if cur is None:
                    cur = {"from": x["wall"], "pose": x["pose"], "vf": set(), "mrm": set()}
                cur["to"] = x["wall"]
                cur["vf"] |= set(x.get("vf") or [])
                if x.get("mrm"):
                    cur["mrm"].add(x["mrm"][0])
            elif cur is not None:
                stops.append(cur)
                cur = None
        if cur is not None:
            stops.append(cur)
        stops = [s for s in stops if s["to"] - s["from"] >= 2.0]
        vf_detail = sorted({f"{x['behavior']}:{x.get('detail', '')}" for x in ev
                            if x["kind"] == "vf" and x["status"] == "STOPPED"})
        rows.append({
            "leg": nn, "junit": junit, "driven": round(driven), "sim_s": round(states[-1]["t"]) if states else None,
            "planned": legs[nn]["length"] if nn in legs else None,
            "mrm": len(mrm), "mrm_at": [x["pose"] for x in mrm],
            "stops": [{"s": round(s["to"] - s["from"], 1),
                       "at": [round(s["pose"][0] - ORIGIN[0], 1), round(s["pose"][1] - ORIGIN[1], 1)],
                       "vf": sorted(s["vf"]), "mrm": sorted(s["mrm"])} for s in stops],
            "vf_stopped": vf_detail,
            "nvtl_min": round(min(nv), 2) if nv else None,
            "nvtl_mean": round(sum(nv) / len(nv), 2) if nv else None,
            "wall_s": t1 - t0,
        })
    for r in rows:
        ok = "pass" if r["junit"] == 'failures="0" errors="0"' else ("FAIL " + r["junit"] if r["junit"] else "no junit")
        st = "; ".join(f"{s['s']}s at ({s['at'][0]},{s['at'][1]}) {'/'.join(s['vf']) or '-'}"
                       f"{' MRM' if set(s['mrm']) - {'NORMAL'} else ''}" for s in r["stops"])
        print(f"{r['leg']:>4} {ok:<26} {r['driven']:>5} m /{r['planned'] or 0:>6.0f} {r['sim_s'] or 0:>5} s "
              f"mrm {r['mrm']} nvtl {r['nvtl_min']}/{r['nvtl_mean']} stops: {st or '-'}")
    if a.json:
        json.dump(rows, open(a.json, "w"), indent=1)


if __name__ == "__main__":
    main()
