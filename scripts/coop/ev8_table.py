#!/usr/bin/env python3
"""Roadmap 078 EV8: per-class braking bounds from ev8_brake.py's runs.

Per blueprint and surface, its bound is the lowest over the speeds of a_eff (v0^2 / 2 d_stop,
dead time included: the braking a car of that class can be relied on for, from the command). The
class bound is the Q quantile (linear) of those across the class's blueprints. Held out: each
blueprint's bound against the class bound computed without it. The spec Model 3 (spec.model3)
is reported on its own and is not one of the car blueprints (vehicle.tesla.model3 is).

  scripts/coop/ev8_table.py <runs.jsonl> [--q 0.1] [--json out.json] > table.md
"""

from __future__ import annotations

import argparse
import json
import statistics as st
import sys

CLASS = {"car": "car", "truck": "truck", "van": "truck", "bus": "bus", "motorcycle": "two_wheeler",
         "bicycle": "two_wheeler"}
ASSUMED = {"car": 4.0, "truck": 3.0, "bus": 3.0, "two_wheeler": 4.0}
WET = 0.6


def quantile(xs, q):
    xs = sorted(xs)
    if len(xs) == 1:
        return xs[0]
    p = q * (len(xs) - 1)
    i = int(p)
    return xs[i] + (xs[min(i + 1, len(xs) - 1)] - xs[i]) * (p - i)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs")
    ap.add_argument("--q", type=float, default=0.1)
    ap.add_argument("--json")
    a = ap.parse_args()
    rows = [json.loads(l) for l in open(a.runs)]
    per = {}  # (bp, surface, lags) -> list of runs
    for r in rows:
        if r.get("error") or not r.get("stopped"):
            continue
        per.setdefault((r["bp"], r["surface"], bool(r.get("lags"))), []).append(r)
    out = {"q": a.q, "classes": {}, "blueprints": {}}
    surfaces = sorted({k[1] for k in per})
    print(f"# EV8 per blueprint (min over speeds of a_eff; mean decel a_mean; peak; dead time)\n")
    print("| blueprint | class | surface | v0 | a_eff | a_mean | a_peak | dead s | d_stop 6/10/14 m |")
    print("|---|---|---|---|---|---|---|---|---|")
    for (bp, s, lag), rs in sorted(per.items()):
        cls = "spec" if bp.startswith("spec.") else CLASS.get(rs[0].get("base_type", "").lower(), "?")
        rs.sort(key=lambda r: r["v0_cmd"])
        b = {"class": cls, "surface": s, "lags": lag, "bound": min(r["a_eff"] for r in rs),
             "a_mean_min": min(r["a_mean"] for r in rs), "a_peak_max": max((r["a_peak"] or 0) for r in rs),
             "dead_s": max(r["dead_s"] for r in rs), "v0s": [r["v0_cmd"] for r in rs],
             "d_stop": {str(r["v0_cmd"]): r["d_stop"] for r in rs}}
        out["blueprints"][f"{bp}|{s}|{int(lag)}"] = b
        ds = " / ".join("%.1f" % r["d_stop"] for r in rs)
        print(f"| {bp}{' +lags' if lag else ''} | {cls} | {s} | {','.join(f'{v:g}' for v in b['v0s'])} | {b['bound']:.2f} | "
              f"{b['a_mean_min']:.2f} | {b['a_peak_max']:.2f} | {b['dead_s']:.3f} | "
              f"{ds} |")
    print(f"\n# EV8 per class: bound = q{a.q:g} across blueprints of min-over-speeds a_eff (m/s^2)\n")
    print("| class | surface | n | bound | assumed | min | median | max (b_max) | dead s max | held-out: below own LOO bound | LOO bound range |")
    print("|---|---|---|---|---|---|---|---|---|---|---|")
    for cls in ("car", "truck", "bus", "two_wheeler"):
        for s in surfaces:
            bs = {k.split("|")[0]: v for k, v in out["blueprints"].items() if v["class"] == cls and v["surface"] == s and not v["lags"]}
            if not bs:
                continue
            xs = [v["bound"] for v in bs.values()]
            bound = quantile(xs, a.q)
            loo, below = [], []
            for k, v in bs.items():
                rest = [w["bound"] for kk, w in bs.items() if kk != k]
                if not rest:
                    continue
                lb = quantile(rest, a.q)
                loo.append(lb)
                if v["bound"] < lb:
                    below.append(f"{k.split('.', 1)[1]} {v['bound']:.2f}<{lb:.2f}")
            assumed = ASSUMED[cls] * (WET if s in ("mu0.6", "f0.72") else 1.0)
            c = {"n": len(xs), "bound": round(bound, 3), "assumed": assumed, "min": min(xs), "median": st.median(xs),
                 "max": max(v["a_peak_max"] for v in bs.values()), "dead_max": max(v["dead_s"] for v in bs.values()),
                 "loo_below": below, "loo_range": [round(min(loo), 3), round(max(loo), 3)] if loo else None,
                 "members": sorted(bs)}
            out["classes"][f"{cls}|{s}"] = c
            print(f"| {cls} | {s} | {c['n']} | **{bound:.2f}** | {assumed:.1f} | {c['min']:.2f} | {c['median']:.2f} | {c['max']:.2f} | "
                  f"{c['dead_max']:.3f} | {len(below)}/{len(xs)} {'; '.join(below)} | {c['loo_range']} |")
    if a.json:
        json.dump(out, open(a.json, "w"), indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
