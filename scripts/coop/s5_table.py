#!/usr/bin/env python3
"""Roadmap 074 S5 on the real stack: the table from s5_aw.py's runs.

    python3 scripts/coop/s5_table.py <runs dir> [<runs dir> ...] [--out <dir>]

One row per run: the gap A aimed at and what it achieved, the lag at B's decision, B's decision
(go or wait) and whether it stopped first, when B's stack first tracked A and the intersection
module's first stop, PET, clearance, contact, peak braking, uA and uB; next to left_turn_game
(U1) at the same lag and v: its threshold G*, its decision and PET. Then per v: B's critical lag
(between the largest lag it yielded to and the smallest it went at; a yield is the
intersection module's stop) against U1's G*. Writes
s5_table.md and .json.
"""
import argparse
import json
import os
import statistics as st


def f(x, n=2):
    return "" if x is None else (f"{x:.{n}f}" if isinstance(x, float) else str(x))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    rows = []
    for d in a.runs:
        p = os.path.join(d, "s5.jsonl")
        if os.path.exists(p):
            rows += [json.loads(line) for line in open(p) if line.strip()]
    ok = [r for r in rows if "error" not in r]
    for r in ok:
        # B's decision is Autoware's: it yields when the intersection module stops it (B may then
        # still be hit, or enter A's lane first); it goes when the module never stops it. Who
        # crossed first is s5_aw's "go" (B's centre across A's lane before A passed), kept as b_first.
        r["b_first"] = r.get("b_first", r["go"])
        r["go"] = int(not any(k.startswith("intersection") for k in (r.get("factors") or {})))
    hdr = ("| v | gap set | gap achieved | lag at decision | B (Autoware) | B through first | tracker has A | intersection stop | "
           "PET | clear m | contact | peakA | peakB | uA | uB | U1 G* | U1 at this lag | U1 PET |")
    lines = [hdr, "|" + "---|" * (hdr.count("|") - 1)]
    for r in sorted(ok, key=lambda r: (r["v"], r["gap"], r["seed"])):
        fac = r.get("factors") or {}
        stop = [v for k, v in fac.items() if k.startswith("intersection")]
        stop_s = f(min(s["first"] for s in stop)) + "-" + f(max(s["last"] for s in stop)) if stop else "none"
        k = r.get("kin", {})
        lines.append(
            f"| {r['v']:g} | {r['gap']:g} | {f(r['gap_achieved'])} | {f(r['lag'])} | {'go' if r['go'] else 'yield'} | "
            f"{'yes' if r['b_first'] else 'no'} | {f(r.get('perc:tracking/objects'))} | {stop_s} | {f(r['pet'])} | "
            f"{f(r['clear'])} | {r['contact']} | {f(r['peakA'])} | {f(r['peakB'])} | {f(r['uA'], 3)} | {f(r['uB'], 3)} | "
            f"{k.get('G_star', '')} | {'' if 'go' not in k else ('go' if k['go'] else 'wait')} | {f(k.get('pet'))} |")
    summ = []
    sl = ["| v | runs | go | yield | largest lag yielded to | smallest lag gone at | critical lag (Autoware) | U1 G* | "
          "decisions as U1 | min PET | contacts | mean uB go / wait | mean uA go / wait |",
          "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for v in sorted({r["v"] for r in ok}):
        rs = [r for r in ok if r["v"] == v]
        acc = [r["lag"] for r in rs if r["go"] and r["lag"] is not None and r["lag"] > 0]
        rej = [r["lag"] for r in rs if not r["go"] and r["lag"] is not None and r["lag"] > 0]
        lo, hi = (max(rej) if rej else None), (min(acc) if acc else None)
        crit = None if lo is None or hi is None else round((lo + hi) / 2, 2)
        g_star = next((r["kin"]["G_star"] for r in rs if "kin" in r), None)
        same = sum(1 for r in rs if "go" in r.get("kin", {}) and r["kin"]["go"] == r["go"])
        n_k = sum(1 for r in rs if "go" in r.get("kin", {}))
        pets = [r["pet"] for r in rs if r["pet"] is not None]
        m = lambda key, g: (round(st.mean([r[key] for r in rs if r["go"] == g]), 3) if any(r["go"] == g for r in rs) else None)
        s = {"v": v, "n": len(rs), "go": len(acc), "wait": len(rej), "max_rejected": lo, "min_accepted": hi,
             "critical": crit, "overlap": lo is not None and hi is not None and lo > hi, "G_star": g_star,
             "as_u1": f"{same}/{n_k}", "min_pet": min(pets) if pets else None, "contacts": sum(r["contact"] for r in rs),
             "uB_go": m("uB", 1), "uB_wait": m("uB", 0), "uA_go": m("uA", 1), "uA_wait": m("uA", 0)}
        summ.append(s)
        sl.append(f"| {v:g} | {s['n']} | {s['go']} | {s['wait']} | {f(lo)} | {f(hi)} | "
                  f"{f(crit) if not s['overlap'] else 'overlap'} | {g_star} | {s['as_u1']} | {f(s['min_pet'])} | {s['contacts']} | "
                  f"{f(s['uB_go'], 3)} / {f(s['uB_wait'], 3)} | {f(s['uA_go'], 3)} / {f(s['uA_wait'], 3)} |")
    bad = [r for r in rows if "error" in r]
    md = "\n".join(lines) + "\n\n" + "\n".join(sl) + "\n" + (
        "\nFailed runs: " + "; ".join(f"v {r['v']:g} gap {r['gap']:g} s{r['seed']}: {r['error']}" for r in bad) + "\n" if bad else "")
    out = a.out or a.runs[0]
    open(os.path.join(out, "s5_table.md"), "w").write(md)
    json.dump({"runs": rows, "summary": summ}, open(os.path.join(out, "s5_table.json"), "w"), indent=1)
    print(md)


if __name__ == "__main__":
    main()
