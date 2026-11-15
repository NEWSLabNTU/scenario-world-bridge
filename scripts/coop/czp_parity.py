#!/usr/bin/env python3
"""Roadmap 078 EV1, kinematic side: does czp, through the arena adapter (czp_adapter.py), reproduce the
studies' outcomes and classes within D2's tolerance?

Per study, czp in the place of the study's convention, on the study's own kinematic model (with the
car's command lags, the 074 kinematic defaults):

  u3     the four-way stop (U3 / C8): the protocol rule P in {fcfse (C8's (e), the reference), czp0
         (czp without C8's fixes), czp (with them)} against every U3 rule in both places and against
         itself. Per pair the outcome (first, contact, deadlock) and payoffs (D2: within max(2 sd, 0.02);
         the kinematic model is deterministic, so 0.02); per context the class of the game over U3's
         rules + P. czp and czp0 are compared with fcfse.
  s2     the crossing (S2 with C7's binding priority, the convention czp's table carries at a tie): czp
         self-play against the equilibria of C7's bind convention over C2's rules (tiebreak.py, computed
         here on the current model): the outcome agrees when czp's first car is one of the equilibria's
         first cars, with the same contact; the class (A / B / W / !) when it is the equilibria's (C:
         either car).
  s4     the narrow passage (S4 with C7's bind, the obstructed side yields): as s2.
  s3     the lane drop (S3): czp deciding A's merge against each of B's rules; per context the class of
         S3's game (merge_game) and czp's outcome (A ahead / behind, forced stop, contact) against it.

Near-tie band (|dt| <= NEAR): parity on the contact share, not the class (078 EV1, 074's S2 follow-up).

Usage:
  scripts/coop/czp_parity.py --out <dir> [--studies u3,s2,s4,s3] [--jobs N]
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from dataclasses import replace
from multiprocessing import Pool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import crossing_game as X  # noqa: E402
import czp_adapter as CA  # noqa: E402

NEAR = 0.1
TOL = 0.02  # D2's payoff floor


# --- U3 / C8 -------------------------------------------------------------------------------------------
U3_OTHERS = ("aggr", "yield_c1.5", "stop1y_c1.5", "fcfs_c1.5", "fcfsr_c1.5", "rbl_c1.5")
PROTO = ("fcfse_c1.5", "czp0_c1.5", "czp_c1.5")


def u3_context(job):
    dt, v = job
    import four_way_game as F
    CA.enable()
    rule = {r[0]: r for r in F.RULES}
    lags = X.car_lags()
    out = {"dt": dt, "v": v, "pairs": {}, "class": {}}
    for p in PROTO:
        names = list(U3_OTHERS) + [p]
        outs = {}
        for a in names:
            for b in names:
                o = F.simulate(rule[a], rule[b], dt, v, v, lags)
                outs[(a, b)] = o
        ua = {k: X.utility(replace(o.o, peak=o.o.peak_w), 0) for k, o in outs.items()}
        ub = {k: X.utility(replace(o.o, peak=o.o.peak_w), 1) for k, o in outs.items()}
        pure, mixed = X.solve(ua, ub, names, names)
        out["class"][p] = F.klass(pure, mixed, outs)
        for (a, b), o in outs.items():
            if p in (a, b):
                key = (a.replace(p, "P"), b.replace(p, "P"))
                out["pairs"].setdefault("|".join(key), {})[p] = (o.o.first, int(o.o.contact), int(o.deadlock),
                                                                 round(ua[(a, b)], 4), round(ub[(a, b)], 4))
    return out


def u3(args):
    dts = sorted({round(x * 0.25, 2) for x in range(-8, 9)} | {-0.1, 0.1, -3.0, 3.0, -4.0, 4.0, -6.0, 6.0})
    jobs = [(dt, v) for v in (6.0, 10.0) for dt in dts]
    with Pool(args.jobs) as pool:
        res = pool.map(u3_context, jobs)
    rows, summ = [], {}
    for cand in ("czp0_c1.5", "czp_c1.5"):
        n = agree = pay_in = pay_n = cls = 0
        near_c = {"ref": [0, 0], "cand": [0, 0]}
        cls_miss = []
        for r in res:
            near = abs(r["dt"]) <= NEAR + 1e-9
            for key, d in r["pairs"].items():
                ref, c = d["fcfse_c1.5"], d[cand]
                same = ref[:3] == c[:3]
                pin = sum(abs(ref[i] - c[i]) <= TOL for i in (3, 4))
                if near:
                    near_c["ref"][0] += ref[1]
                    near_c["ref"][1] += 1
                    near_c["cand"][0] += c[1]
                    near_c["cand"][1] += 1
                else:
                    n += 1
                    agree += same
                pay_in += pin
                pay_n += 2
                rows.append({"study": "u3", "cand": cand, "dt": r["dt"], "v": r["v"], "pair": key,
                             "ref": "/".join(map(str, ref)), "czp": "/".join(map(str, c)), "outcome_same": int(same)})
            if not near:
                ok = r["class"]["fcfse_c1.5"] == r["class"][cand]
                cls += ok
                if not ok:
                    cls_miss.append(f"v{r['v']:g} dt{r['dt']:+g}: {r['class']['fcfse_c1.5']}->{r['class'][cand]}")
        n_ctx = sum(1 for r in res if abs(r["dt"]) > NEAR + 1e-9)
        summ[cand] = {"outcomes_agree": f"{agree}/{n}", "outcomes_pct": round(100 * agree / max(n, 1), 1),
                      "payoffs_within_0.02": f"{pay_in}/{pay_n}", "classes_agree": f"{cls}/{n_ctx}",
                      "class_misses": cls_miss,
                      "near_tie_contact_share": {k: f"{x[0]}/{x[1]}" for k, x in near_c.items()}}
    # the protocol's own safety in the kinematic model: self-play contacts and deadlocks
    for p in PROTO:
        sp = [r["pairs"]["P|P"][p] for r in res]
        summ.setdefault("self_play", {})[p] = {"contacts": sum(x[1] for x in sp), "deadlocks": sum(x[2] for x in sp),
                                               "contexts": len(sp)}
    return rows, summ


# --- S2 / S4 with C7's bind ----------------------------------------------------------------------------
def passage_context(job):
    geom, dt, v, wn = job
    import tiebreak as TB
    import narrow_game as N
    lags = X.car_lags()
    ctx = (dt, v, wn)
    a_acts, b_acts, keys = TB.acts(geom)
    runs = {}
    for a in a_acts:
        for b in b_acts:
            for k in keys[b]:
                runs[("bind", None, a, k)] = TB.run_one(geom, a, k, ctx, ("bind", "bind"), None, lags)
    ua, ub, phys, prof, pure, mixed = TB.game(geom, runs, "bind")
    eqr = [r for p in pure for r in prof[p]]
    ref_class = TB.klass(prof, pure, mixed)
    ref_first = "".join(sorted({r.o.first for r in eqr}))
    ref_contact = int(any(r.o.contact for r in eqr))
    zk = CA.CM.CROSS_UNSIGNALLED if geom == "cross" else CA.CM.NARROW_PASSAGE
    out = {"geom": geom, "dt": dt, "v": v, "wn": wn, "ref_class": ref_class, "ref_first": ref_first,
           "ref_contact": ref_contact, "ref_restart_share": None}
    waited = [r for r in runs.values() if r.o.both_stopped]
    out["ref_restart_share"] = round(sum(r.restart for r in waited) / len(waited), 3) if waited else None
    out["ref_contact_share_pairs"] = round(sum(r.o.contact for r in runs.values()) / len(runs), 3)
    for kind in ("czp0", "czp"):
        with CA.patched_crossing(("A", "B"), zk, kind):
            if geom == "cross":
                o = X.simulate(X.A_RULES[0], X.B_RULES[0], dt, v, v, "hold", lags=lags)
                dead = not o.contact and o.first == ""
            else:
                res = N.simulate("go", "go", dt, v, v, wn, lags)
                o, dead = res.o, res.deadlock
        cls = "!" if o.contact else ("W" if o.both_stopped else (o.first or "?"))
        out[kind] = {"first": o.first, "contact": int(o.contact), "deadlock": int(dead), "both_stopped": int(o.both_stopped),
                     "class": cls, "W": round(X.utility(replace(o, peak=o.peak_w), 0) + X.utility(replace(o, peak=o.peak_w), 1), 4)}
    return out


def passage(args, geom):
    dts = [-3.0, -2.0, -1.5, -1.0, -0.7, -0.5, -0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.5, 0.7, 1.0, 1.5, 2.0, 3.0]
    if geom == "cross":
        jobs = [("cross", dt, v, 0.0) for v in (6.0, 10.0, 14.0) for dt in dts]
    else:
        jobs = [("narrow", dt, v, wn) for wn in (6.0, 30.0) for v in (6.0, 10.0) for dt in dts]
    with Pool(args.jobs) as pool:
        res = pool.map(passage_context, jobs)
    rows, summ = [], {}
    for kind in ("czp0", "czp"):
        n = agree = cls = 0
        near = {"czp_contact": 0, "ref_contact": 0, "n": 0}
        misses = []
        contacts = deadlocks = 0
        for r in res:
            c = r[kind]
            contacts += c["contact"]
            deadlocks += c["deadlock"]
            out_ok = (c["first"] in r["ref_first"] if r["ref_first"] else c["first"] == "") and c["contact"] == r["ref_contact"]
            rc = r["ref_class"]
            cls_ok = c["class"] == rc or (rc == "C" and c["class"] in ("A", "B")) or (rc == "W" and c["class"] in ("W",))
            rows.append({"study": geom, "cand": kind, "dt": r["dt"], "v": r["v"], "wn": r["wn"], "ref_class": rc,
                         "ref_first": r["ref_first"], "ref_contact": r["ref_contact"], "czp_class": c["class"],
                         "czp_first": c["first"], "czp_contact": c["contact"], "czp_W": c["W"],
                         "outcome_ok": int(out_ok), "class_ok": int(cls_ok)})
            if abs(r["dt"]) <= NEAR + 1e-9:
                near["n"] += 1
                near["czp_contact"] += c["contact"]
                near["ref_contact"] += r["ref_contact"]
                continue
            n += 1
            agree += out_ok
            cls += cls_ok
            if not (out_ok and cls_ok):
                misses.append(f"v{r['v']:g} wn{r['wn']:g} dt{r['dt']:+g}: ref {rc}/{r['ref_first']} czp {c['class']}/{c['first']}")
        summ[kind] = {"outcomes_agree": f"{agree}/{n}", "outcomes_pct": round(100 * agree / max(n, 1), 1),
                      "classes_agree": f"{cls}/{n}", "classes_pct": round(100 * cls / max(n, 1), 1),
                      "near_tie_contacts": f"czp {near['czp_contact']}/{near['n']}, bind's equilibria {near['ref_contact']}/{near['n']}",
                      "contacts": contacts, "deadlocks": deadlocks, "misses": misses}
    return rows, summ


# --- S3 ------------------------------------------------------------------------------------------------
def s3_context(job):
    d_end, d0, dv, vb = job
    import merge_game as MG
    lags = X.car_lags()
    row = MG.context((d_end, d0, dv, vb, lags))
    out = {"d_end": d_end, "d0": d0, "dv": dv, "vb": vb, "ref_class": row["class"]}
    ra = next(r for r in MG.A_RULES if r[0] == "z0.6d")  # the zipper's timing; czp decides the merge ahead
    for kind in ("czp",):
        res = {}
        for rb in MG.B_RULES:
            conts = ("yield", "hold") if rb[0] == "react" else ("hold",)
            for c in conts:
                with CA.patched_merge(kind):
                    o = MG.simulate(ra, rb, d_end, d0, dv, vb, c, lags)
                res[f"{rb[0]}:{c}"] = {"ahead": int(o.ahead), "contact": int(o.contact), "stopped": int(o.stopped),
                                       "merged": int(o.merged)}
        out[kind] = res
    return out


def s3(args):
    jobs = [(d, d0, dv, 12.0) for d in (40.0, 80.0) for d0 in [-10.0 + 5 * i for i in range(9)]
            for dv in (-3.0, -1.5, 0.0, 1.5, 3.0)]
    with Pool(args.jobs) as pool:
        res = pool.map(s3_context, jobs)
    rows = []
    n = ok = contacts = stops = unmerged = 0
    misses = []
    for r in res:
        rc = r["ref_class"]
        for b, c in r["czp"].items():
            n += 1
            order = "A" if c["ahead"] else "B"
            good = (not c["contact"]) and (rc == "C" or rc in ("M", "?") or (rc in ("A", "B") and order == rc)
                                           or (rc == "S" and (c["stopped"] or order == "B")) or rc == "!")
            ok += good
            contacts += c["contact"]
            stops += c["stopped"]
            unmerged += not c["merged"]
            rows.append({"study": "s3", "d_end": r["d_end"], "d0": r["d0"], "dv": r["dv"], "b": b, "ref_class": rc,
                         "czp_order": order, "czp_contact": c["contact"], "czp_stopped": c["stopped"], "czp_merged": c["merged"], "ok": int(good)})
            if not good:
                misses.append(f"d_end{r['d_end']:g} d0{r['d0']:+g} dv{r['dv']:+g} B {b}: S3 {rc}, czp {order}"
                              f"{' contact' if c['contact'] else ''}{' stop' if c['stopped'] else ''}")
    summ = {"czp": {"outcomes_agree": f"{ok}/{n}", "outcomes_pct": round(100 * ok / max(n, 1), 1),
                    "contacts": contacts, "forced_stops": stops, "not_merged_by_T_MAX": unmerged, "misses": misses}}
    return rows, summ


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out", required=True)
    p.add_argument("--studies", default="u3,s2,s4,s3")
    p.add_argument("--jobs", type=int, default=12)
    a = p.parse_args()
    os.makedirs(a.out, exist_ok=True)
    allsum = {}
    for s in a.studies.split(","):
        if s == "u3":
            rows, summ = u3(a)
        elif s == "s2":
            rows, summ = passage(a, "cross")
        elif s == "s4":
            rows, summ = passage(a, "narrow")
        elif s == "s3":
            rows, summ = s3(a)
        else:
            continue
        allsum[s] = summ
        with open(os.path.join(a.out, f"parity_{s}.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(dict.fromkeys(k for r in rows for k in r)))
            w.writeheader()
            w.writerows(rows)
        print(s, json.dumps(summ, indent=1)[:3000], flush=True)
    json.dump(allsum, open(os.path.join(a.out, "parity_summary.json"), "w"), indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
