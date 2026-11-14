#!/usr/bin/env python3
"""Roadmap 074 S1 on the real stack: the table from s1_aw.py's runs.

    python3 scripts/coop/s1_table.py <runs dir> [<runs dir> ...] [--out <dir>]

One row per run: the achieved context (h0, B's speed at t = 0), L's cut-out and its body leaving
B's lane (measured, and on its planned path), when B's stack first saw O (CenterPoint, the
tracker) and the planner's first obstacle factor naming O, B's braking onset and peak, clearance
to O, B's risk and utility; next to cutout_game at the same context, look-through off and on, for
B's gaps 0.8 / 1.5 / 2.5 s. Then per (v, T_L): means over runs. Writes s1_table.md and .json.
"""
import argparse
import json
import os
import statistics as st

PERC = ("perc:detection/centerpoint/objects", "perc:detection/clustering/objects", "perc:tracking/objects")


def f(x, n=2):
    return "" if x is None else (f"{x:.{n}f}" if isinstance(x, float) else str(x))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    rows = []
    for d in a.runs:
        p = os.path.join(d, "s1.jsonl")
        if os.path.exists(p):
            rows += [json.loads(line) for line in open(p) if line.strip()]
    ok = [r for r in rows if "error" not in r]
    hdr = ("| v | T_L | h0 set | h0 | vB0 | cut-out | reveal (path) | CenterPoint | clustering | tracker | "
           "factor | B brakes | peakB | clear O m | contact | rhoB | uB | kin off 0.8 / 1.5 / 2.5 uB | kin on 2.5 uB |")
    lines = [hdr, "|" + "---|" * (hdr.count("|") - 1)]
    for r in sorted(ok, key=lambda r: (r["v"], -r["t_l"], r["h0_set"], r["seed"])):
        fac = r.get("factors") or {}
        f0 = min(fac.items(), key=lambda kv: kv[1][0]) if fac else None
        k = r.get("kin", {})
        lines.append(
            f"| {r['v']:g} | {r['t_l']:g} | {r['h0_set']:g} | {f(r['h0'])} | {f(r['vB0'])} | {f(r['t_cutout'])} | "
            f"{f(r['t_reveal'])} ({f(r.get('t_reveal_path'))}) | {f(r.get(PERC[0]))} | {f(r.get(PERC[1]))} | "
            f"{f(r.get(PERC[2]))} | {'' if not f0 else f'{f0[0]} {f0[1][0]}'} | {f(r['t_brakeB'])} | {f(r['peakB'])} | "
            f"{f(r['clearBO'])} | {r['contact']} | {f(r['rhoB'], 3)} | {f(r['uB'], 3)} | "
            + " / ".join(f(k.get(f"off_{g}", {}).get("uB"), 3) for g in ("0.8", "1.5", "2.5"))
            + f" | {f(k.get('on_2.5', {}).get('uB'), 3)} |")
    groups = {}
    for r in ok:
        groups.setdefault((r["v"], r["t_l"]), []).append(r)
    mean = lambda rs, key: (round(st.mean([r[key] for r in rs if isinstance(r.get(key), (int, float))]), 3)
                            if any(isinstance(r.get(key), (int, float)) for r in rs) else None)
    summ = []
    sl = ["| v | T_L | runs | h0 | perceive O after cut-out s | B brakes after cut-out s | clear O m | contact | uB | "
          "kin off 2.5 uB | kin on 2.5 uB |", "|---|---|---|---|---|---|---|---|---|---|---|"]
    for (v, tl), rs in sorted(groups.items(), key=lambda kv: (kv[0][0], -kv[0][1])):
        for r in rs:
            p = r.get(PERC[0]) if r.get(PERC[0]) is not None else r.get(PERC[2])
            r["_perc_after"] = None if p is None or r.get("t_cutout") is None else p - r["t_cutout"]
            r["_brake_after"] = None if r.get("t_brakeB") is None or r.get("t_cutout") is None else r["t_brakeB"] - r["t_cutout"]
            r["_koff"] = r.get("kin", {}).get("off_2.5", {}).get("uB")
            r["_kon"] = r.get("kin", {}).get("on_2.5", {}).get("uB")
        s = {"v": v, "t_l": tl, "n": len(rs), "h0": mean(rs, "h0"), "perc_after": mean(rs, "_perc_after"),
             "brake_after": mean(rs, "_brake_after"), "clearBO": mean(rs, "clearBO"),
             "contact": sum(r["contact"] for r in rs), "uB": mean(rs, "uB"), "kin_off_2.5": mean(rs, "_koff"),
             "kin_on_2.5": mean(rs, "_kon")}
        summ.append(s)
        sl.append(f"| {v:g} | {tl:g} | {s['n']} | {f(s['h0'])} | {f(s['perc_after'])} | {f(s['brake_after'])} | "
                  f"{f(s['clearBO'])} | {s['contact']} | {f(s['uB'], 3)} | {f(s['kin_off_2.5'], 3)} | {f(s['kin_on_2.5'], 3)} |")
    bad = [r for r in rows if "error" in r]
    md = "\n".join(lines) + "\n\n" + "\n".join(sl) + "\n" + (
        "\nFailed runs: " + "; ".join(f"v {r['v']:g} T_L {r['t_l']:g} h0 {r['h0_set']:g} s{r['seed']}: {r['error']}"
                                      for r in bad) + "\n" if bad else "")
    out = a.out or a.runs[0]
    open(os.path.join(out, "s1_table.md"), "w").write(md)
    json.dump({"runs": rows, "summary": summ}, open(os.path.join(out, "s1_table.json"), "w"), indent=1)
    print(md)


if __name__ == "__main__":
    main()
