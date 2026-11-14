#!/usr/bin/env python3
"""Roadmap 074 C5, kinematic: the cut-in (C1's game) for pairs of measured car models, and the
commitment hypothesis.

C1-C3 played the cut-in with first-number capabilities (cutin_game.TYPES). C5 plays it with
measured ones:
  m3     the spec's Model 3 as the arena measured it (E1, TYPES["carla_model3"]): 0-12 m/s at
         4.7 m/s^2 mean, full braking 9.6 m/s^2 peak, 4.79 x 2.16 m
  hgv    CARLA 0.9.16's vehicle.carlamotors.european_hgv, stock physics, as E4 measured it
         (e4_heavy.py): 7000 kg, 7.94 x 2.89 m, 0-12 m/s at 1.38 m/s^2 mean, full braking from
         12 m/s 3.22 mean / 4.56 peak (22.4 m)
  fire   vehicle.carlamotors.firetruck, stock (E4): 10 000 kg, 8.47 x 2.89 m, 1.13 m/s^2,
         2.78 mean / 3.67 peak (26.0 m)
  golf   cutin_game's golf cart (TYPES["golfcart"], assumed: no golf cart is in the spec or in
         CARLA): 2.4 m, 1.5 m/s^2, 4.0 m/s^2, v_max 6.9 m/s; played at its speeds (v_B 6 m/s)
The heavy cars' lane change takes 4 s (assumed, as C6's truck_like); reaction 1 s for every car.

The commitment hypothesis (docs/design/survey-multi-vehicle-cooperation.md, "Capability in the
game"): in chicken, inability is commitment; a car that visibly cannot brake hard (a loaded
truck) or cannot accelerate (the golf cart) is yielded to, so a lower capability can earn a
higher equilibrium payoff. Tested per context against the same pair with the Model 3 in the
weak car's role (the other car, the context and the rules the same):
  yielded   the other car yields in every pure equilibrium: A drops behind B (X is B), or B
            pre-yields or yields after seeing A (X is A)
  flips     contexts where X is yielded to and the Model 3 in its place is not (commitment), and
            the reverse
  dU        X's equilibrium payoff minus the Model 3's in the same role (pure equilibria
            averaged, else the mixed one); "pays" where dU > +0.01

Usage:
  scripts/coop/c5_cars.py --out <dir> [--e4 <e4.json>] [--jobs 7]
  scripts/coop/c5_cars.py --selftest
"""

from __future__ import annotations

import argparse
import csv
import json
import multiprocessing as mp
import os
import statistics as st
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cutin_game as C1  # noqa: E402

# E4's measurements (e4_heavy.py on CARLA 0.9.16, 2026-11-15); --e4 re-reads them from e4.json
E4_DEFAULT = {"carlamotors.european_hgv": {"length": 7.94, "width": 2.89, "a": 1.38, "b": 4.56},
              "carlamotors.firetruck": {"length": 8.47, "width": 2.89, "a": 1.13, "b": 3.67}}
HEAVY_T_LC = 4.0  # s, assumed (C6's truck_like)


def types(e4=None):
    m = dict(E4_DEFAULT)
    if e4:
        for n, r in json.load(open(e4)).items():
            if n in m:
                p, t = r["physics"], r["t1"]
                m[n] = {"length": p["length_m"], "width": p["width_m"], "a": t["mean_accel_0_12"], "b": t["brake_peak_decel"]}
    mk = lambda r: C1.Capability(r["length"], a_max=r["a"], b_max=r["b"], v_max=25.0, reaction=1.0, t_prep=0.0,
                                 t_lc=HEAVY_T_LC, width=r["width"])
    hgv = mk(m["carlamotors.european_hgv"])
    return {"m3": C1.TYPES["carla_model3"], "hgv": hgv, "fire": mk(m["carlamotors.firetruck"]),
            "golf": C1.TYPES["golfcart"],
            # the HGV with the Model 3's 3 s lane change: what its limits and size do without the
            # slower (gentler) lane change, whose lower lateral acceleration alone pays A ~0.04
            "hgv3": C1.Capability(hgv.length, a_max=hgv.a_max, b_max=hgv.b_max, v_max=hgv.v_max, reaction=1.0,
                                  t_prep=0.0, t_lc=3.0, width=hgv.width)}


# (A, B, v_B): the pairs, and each one's Model 3 counterpart is (m3 in the weak car's role)
PAIRS = (("m3", "m3", 12.0), ("m3", "hgv", 12.0), ("hgv", "m3", 12.0), ("m3", "fire", 12.0), ("fire", "m3", 12.0),
         ("hgv3", "m3", 12.0), ("m3", "m3", 6.0), ("golf", "m3", 6.0))
D0S = C1.grid(-6.0, 30.0, 1.0)
DVS = C1.grid(-4.0, 4.0, 0.5)


def eq_payoff(sol):
    """(U_A, U_B) at equilibrium: the pure ones' mean, else the first mixed one's expectation."""
    if sol.pure:
        return st.mean(sol.ua[p] for p in sol.pure), st.mean(sol.ub[p] for p in sol.pure)
    if sol.mixed:
        pa, pb = sol.mixed[0]
        ua = sum(pa[a] * pb[b] * sol.ua[(a, b)] for a in pa for b in pb)
        ub = sum(pa[a] * pb[b] * sol.ub[(a, b)] for a in pa for b in pb)
        return ua, ub
    return None


def yields(sol, prof, to):
    """Whether, at the pure equilibrium prof, the other car yields to `to` ('A' or 'B')."""
    a, b = prof
    o = sol.outcomes[prof]
    if to == "B":  # A gives way to B: it drops behind
        return not o.went
    return o.went and (b == C1.PRE_YIELD or o.b_cont == C1.YIELD)


def one_map(job):
    a, b, vb, ty = job
    cap_a, cap_b = ty[a], ty[b]
    pref = C1.Preference()
    out = {}
    for dv in DVS:
        for d0 in D0S:
            sol = C1.play(d0, dv, vb, cap_a, cap_b, pref, pref)
            e = eq_payoff(sol)
            c_opt, c_worst, _ = C1.welfare(sol)
            row = {"d0": d0, "dv": dv, "class": C1.label(sol), "pure": " ".join(f"{x}/{y}" for x, y in sol.pure),
                   "mixed": " | ".join(" ".join(f"{k}:{v:.2f}" for k, v in {**pa, **pb}.items()) for pa, pb in sol.mixed),
                   "C_opt": f"{c_opt:.3f}", "C_worst_eq": "" if c_worst is None else f"{c_worst:.3f}"}
            for (x, y) in sol.outcomes:
                row[f"uA_{x}_{y}"] = f"{sol.ua[(x, y)]:.3f}"
                row[f"uB_{x}_{y}"] = f"{sol.ub[(x, y)]:.3f}"
            out[(d0, dv)] = {"cls": C1.label(sol), "u": e, "loss": None if c_worst is None else c_worst - c_opt, "row": row,
                             "yield_to_A": bool(sol.pure) and all(yields(sol, p, "A") for p in sol.pure),
                             "yield_to_B": bool(sol.pure) and all(yields(sol, p, "B") for p in sol.pure),
                             "contact_off_eq": sum(o.contact for o in sol.outcomes.values())}
    return (a, b, vb), out


def map_text(key, m):
    a, b, vb = key
    L = [f"# cut-in equilibrium map: A={a} B={b} vB={vb} m/s (kinematic, C1's game, measured capabilities)",
         f"# rows: dv (A minus B, m/s), top = fastest; columns: d0 (m) {D0S[0]}..{D0S[-1]} step 1"]
    L += ["#   " + l.strip() for l in C1.LEGEND[2:] if l.strip()]
    L += [f"{dv:+6.1f} " + "".join(m[(d0, dv)]["cls"] for d0 in D0S) for dv in reversed(DVS)]
    L.append("# welfare loss of the worst equilibrium: . <0.05  - <0.2  + <0.5  # >=0.5")
    for dv in reversed(DVS):
        L.append(f"{dv:+6.1f} " + "".join("?" if m[(d0, dv)]["loss"] is None else "." if m[(d0, dv)]["loss"] < 0.05 else
                                          "-" if m[(d0, dv)]["loss"] < 0.2 else "+" if m[(d0, dv)]["loss"] < 0.5 else "#"
                                          for d0 in D0S))
    return "\n".join(L) + "\n"


def compare(maps, weak, role, vb):
    """The weak car X in `role` against the Model 3 there, the other car the Model 3."""
    kx = (weak, "m3", vb) if role == "A" else ("m3", weak, vb)
    km = ("m3", "m3", vb)
    mx, mm = maps[kx], maps[km]
    i = 0 if role == "A" else 1
    yk = "yield_to_" + role
    n = len(mx)
    yx = sum(mx[c][yk] for c in mx)
    ym = sum(mm[c][yk] for c in mm)
    flip_to = [c for c in mx if mx[c][yk] and not mm[c][yk]]
    flip_from = [c for c in mx if mm[c][yk] and not mx[c][yk]]
    du = [mx[c]["u"][i] - mm[c]["u"][i] for c in mx if mx[c]["u"] and mm[c]["u"]]
    pays = [c for c in mx if mx[c]["u"] and mm[c]["u"] and mx[c]["u"][i] - mm[c]["u"][i] > 0.01]
    pays_y = [c for c in pays if c in flip_to]
    other = 1 - i
    du_o = [mx[c]["u"][other] - mm[c]["u"][other] for c in mx if mx[c]["u"] and mm[c]["u"]]
    same = sum(mx[c]["cls"] == mm[c]["cls"] for c in mx)
    return {"pair": f"{kx[0]} x {kx[1]} @ {vb:g}", "weak": weak, "role": role, "contexts": n,
            "yielded_weak": yx, "yielded_m3": ym, "flip_to_weak": len(flip_to), "flip_from_weak": len(flip_from),
            "pays": len(pays), "pays_and_yielded": len(pays_y),
            "dU_mean": st.mean(du) if du else None, "dU_max": max(du) if du else None,
            "dU_other_mean": st.mean(du_o) if du_o else None, "same_class": same,
            "flip_to_ctx": flip_to[:12], "pays_ctx": pays[:12]}


def run(out, e4, jobs):
    ty = types(e4)
    os.makedirs(out, exist_ok=True)
    with mp.Pool(jobs) as pool:
        res = dict(pool.map(one_map, [(a, b, vb, ty) for a, b, vb in PAIRS]))
    for key, m in res.items():
        tag = f"{key[0]}_x_{key[1]}_v{key[2]:g}"
        open(os.path.join(out, f"map_{tag}.txt"), "w").write(map_text(key, m))
        with open(os.path.join(out, f"map_{tag}.jsonl"), "w") as f:
            for (d0, dv), r in m.items():
                f.write(json.dumps({"d0": d0, "dv": dv, **{k: v for k, v in r.items() if k != "row"}}) + "\n")
        rows = [r["row"] for r in m.values()]  # cutin_game's cutin_map.csv columns (the atlas reads them)
        with open(os.path.join(out, f"map_{tag}.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
    cmp = [compare(res, "hgv", "B", 12.0), compare(res, "hgv", "A", 12.0), compare(res, "fire", "B", 12.0),
           compare(res, "fire", "A", 12.0), compare(res, "hgv3", "A", 12.0), compare(res, "golf", "A", 6.0)]
    L = ["C5 kinematic: the commitment hypothesis (the weak car X against the Model 3 in its role; the other car m3)",
         "types: " + "; ".join(f"{k} L {c.length:.2f} W {c.width:.2f} a {c.a_max:.2f} b {c.b_max:.2f} vmax {c.v_max:g} t_lc {c.t_lc:g}"
                               for k, c in ty.items()), ""]
    L.append(f"{'pair':18s} {'X':5s} {'role':4s} {'ctx':>4s} {'yielded X':>9s} {'yielded m3':>10s} {'flip->X':>7s} "
             f"{'flip<-X':>7s} {'pays':>5s} {'pays&yld':>8s} {'dU mean':>8s} {'dU max':>7s} {'dU other':>8s} {'same cls':>8s}")
    for c in cmp:
        f = lambda x: "-" if x is None else f"{x:+.3f}"
        L.append(f"{c['pair']:18s} {c['weak']:5s} {c['role']:4s} {c['contexts']:4d} {c['yielded_weak']:9d} {c['yielded_m3']:10d} "
                 f"{c['flip_to_weak']:7d} {c['flip_from_weak']:7d} {c['pays']:5d} {c['pays_and_yielded']:8d} "
                 f"{f(c['dU_mean']):>8s} {f(c['dU_max']):>7s} {f(c['dU_other_mean']):>8s} {c['same_class']:8d}")
    L.append("")
    for c in cmp:
        L.append(f"{c['pair']}: flips to X (first 12, d0/dv): {c['flip_to_ctx']}; pays: {c['pays_ctx']}")
    L.append("")
    for key, m in res.items():
        cls = {}
        for r in m.values():
            cls[r["cls"]] = cls.get(r["cls"], 0) + 1
        L.append(f"classes {key[0]} x {key[1]} @ {key[2]:g}: " + " ".join(f"{k}:{v}" for k, v in sorted(cls.items()))
                 + f" | contexts with off-equilibrium contact {sum(1 for r in m.values() if r['contact_off_eq'])}")
    txt = "\n".join(L) + "\n"
    open(os.path.join(out, "c5_commitment.txt"), "w").write(txt)
    json.dump(cmp, open(os.path.join(out, "c5_commitment.json"), "w"), indent=1, default=str)
    print(txt)
    for key, m in res.items():
        print(map_text(key, m))


def selftest():
    ty = types()
    assert ty["hgv"].b_max < ty["m3"].b_max and ty["hgv"].a_max < ty["m3"].a_max and ty["hgv"].length > 7.5
    assert ty["fire"].b_max < ty["hgv"].b_max and ty["golf"].v_max < 7.0
    pref = C1.Preference()
    # the same context with the heavy car as B: B's braking limit binds (its peak never above b_max)
    o = C1.simulate(C1.GO, C1.REACT, 2.0, 0.0, 12.0, ty["m3"], ty["hgv"], C1.YIELD)
    assert o.peak_brake[1] <= ty["hgv"].b_max + 1e-9
    # eq_payoff on a solved context, and yields() reads the outcome
    sol = C1.play(25.0, 2.0, 12.0, ty["m3"], ty["m3"], pref, pref)
    e = eq_payoff(sol)
    assert e is not None and all(-1.5 < x <= 0.0 for x in e), e
    assert not any(yields(sol, p, "B") for p in sol.pure)  # far ahead and faster: A goes
    sol = C1.play(-6.0, -4.0, 12.0, ty["m3"], ty["m3"], pref, pref)
    assert all(yields(sol, p, "B") for p in sol.pure)  # beside and slower: A drops behind
    # compare() on two hand-made maps
    mk = lambda y, u: {(0.0, 0.0): {"cls": "W", "u": (u, -0.1), "loss": 0.0, "yield_to_B": y, "yield_to_A": False,
                                    "contact_off_eq": 0}}
    c = compare({("m3", "hgv", 12.0): mk(True, -0.2), ("m3", "m3", 12.0): mk(False, -0.1)}, "hgv", "B", 12.0)
    assert c["flip_to_weak"] == 1 and c["yielded_weak"] == 1 and c["yielded_m3"] == 0 and abs(c["dU_other_mean"] + 0.1) < 1e-9
    print("c5_cars selftest ok")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out")
    ap.add_argument("--e4")
    ap.add_argument("--jobs", type=int, default=8)
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    run(a.out, a.e4, a.jobs)


if __name__ == "__main__":
    main()
