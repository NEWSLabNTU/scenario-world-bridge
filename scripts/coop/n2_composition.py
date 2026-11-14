#!/usr/bin/env python3
"""Roadmap 074 N2: do pair terms, composed per measure, predict three cars?

Two three-car scenes in the arena (arena_n.py):
  a  the cut-in with a follower: C follows B at headway hf; A cuts in ahead of B or drops back
  b  the cut-in with a lead: L leads A in A's own lane, gl m ahead, at B's speed

For every context, rule profile and seed, the three-car group and each two-car group of the
same cars (same start states, rules and random streams) are run. Each car's utility on the
three-car run, U_i, is compared with the composed one:

  risk      rho_i = max over i's pairs j of rho_ij (each from the two-car run of i and j)
  delay     d_i = max(max_j d_i|ij, d_lead(i) - slack_i): max-plus down the leader-follower DAG,
            the edges the scene's initial lanes give (a: B -> C; b: L -> A); d = lost * HORIZON,
            slack_i = the time headway i holds beyond the desired 1 s (HEADWAY)
  braking   peak_i = max_j peak_i|ij (and lateral the same)
  contact   any pair run with contact involving i; lost = 1 then, as on a run
  U_hat_i = measures.utility(min(1, d_i / H), rho_i, peak_i, b_max, lat_i, contact_i)

The residual r_i = U_i - U_hat_i per seed (common random numbers), its mean and a bootstrap 95 %
interval over seeds; D9 judges |mean r| against the D2 tolerance max(2 sd(U_i), 0.02).

Profiles: A_aw_perm and do4's equilibrium A (A1_s0l22_2), both against B_aw_react.

Usage:
  scripts/coop/n2_composition.py run --ports 2600,2610,2620 --out <dir> [--seeds 6] [--k 6]
  scripts/coop/n2_composition.py pair-check --ports 2600,2610,2620 --out <dir>
  scripts/coop/n2_composition.py analyze --out <dir>
"""

from __future__ import annotations

import argparse
import itertools
import json
import multiprocessing as mp
import os
import random
import statistics as st
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import cutin_game as C1  # noqa: E402
import double_oracle as DO  # noqa: E402
import measures  # noqa: E402

A_EQ4 = {"name": "A1_s0l22_2", "tau": 0.71, "a_rear": 2.52, "buffer": 4.3, "patience": 5.72, "use_yield": 0, "t_prep": 0.0}
PROFILES = [(DO.A_AW_PERM, DO.B_AW_REACT), (A_EQ4, DO.B_AW_REACT)]
PA = {pa["name"]: pa for pa, _ in PROFILES}
D0S, DVS, HFS, GLS = (2.0, 6.0, 10.0, 14.0), (-2.0, 0.0, 2.0), (1.0, 2.0), (15.0, 30.0)
VB = 12.0
SCENES = {"a": ("ABC", ("AB", "AC", "BC")), "b": ("ABL", ("AB", "AL", "BL"))}
H = C1.HORIZON


def contexts(scene):
    extra = HFS if scene == "a" else GLS
    return list(itertools.product(D0S, DVS, extra))


def jobs(seeds):
    out = []
    for scene, (full, pairs) in SCENES.items():
        key = "hf" if scene == "a" else "gl"
        for (d0, dv, x), (pa, pb), s in itertools.product(contexts(scene), PROFILES, range(seeds)):
            for roles in (full, *pairs):
                out.append({"scene": scene, "d0": d0, "dv": dv, key: x, "vb": VB, "pa": pa, "pb": pb,
                            "seed": s, "roles": roles, "trace": roles == full})
    return out


class Args:
    obs_delay, obs_sigma_x, obs_sigma_v, reaction_jitter = 0.19, 0.3, 0.2, 0.2


def worker(port, k, mode, inbox, outbox):
    import arena as AR
    import arena_n as AN
    ar = AR.Arena(port, k)
    try:
        cars = AN.setup_cars(ar, k) if mode == "groups" else AR.setup_cars(ar, k)
        while True:
            msg = inbox.get()
            if msg is None:
                break
            tag, work = msg
            if mode == "groups":
                rows = AN.run_groups(ar, cars, work, Args())
            else:
                args = Args()
                args.k = k
                rows = AR.run_jobs(ar, cars, work, random.Random(tag), args)
            outbox.put((tag, rows))
    finally:
        ar.close()


def run_pool(ports, k, mode, work, out_path, chunk):
    inbox = mp.Queue()
    outbox = mp.Queue()
    procs = [mp.Process(target=worker, args=(p, k, mode, inbox, outbox), daemon=True) for p in ports]
    for p in procs:
        p.start()
    chunks = [work[i:i + chunk] for i in range(0, len(work), chunk)]
    for i, c in enumerate(chunks):
        inbox.put((i, c))
    for _ in procs:
        inbox.put(None)
    t0 = time.monotonic()
    n = 0
    with open(out_path, "w") as f:
        for _ in chunks:
            tag, rows = outbox.get()
            for r in rows:
                f.write(json.dumps(r) + "\n")
            f.flush()
            n += len(rows)
            print(f"  {n}/{len(work)} runs, {time.monotonic() - t0:.0f} s", flush=True)
    for p in procs:
        p.join(timeout=120)


# --- composition -------------------------------------------------------------------------------
def boot_ci(xs, n=2000, seed=7):
    rng = random.Random(seed)
    ms = sorted(st.mean(rng.choice(xs) for _ in xs) for _ in range(n))
    return ms[int(0.025 * n)], ms[int(0.975 * n) - 1]


def compose(scene, car, pr, ctx_x):
    """Car `car`'s composed (U_hat, parts) from its pair rows pr {roles: row}."""
    mine = [(p, r) for p, r in pr.items() if car in p]
    d = max(r[f"lost{car}"] * H for _, r in mine)
    contact = any(r[f"contact{car}"] for _, r in mine)
    rho = max(r[f"rho_{p}"] for p, r in mine)
    peak = max(r[f"peak{car}"] for _, r in mine)
    lat = max(r[f"lat{car}"] for _, r in mine)
    # max-plus down the leader-follower DAG of the scene's initial lanes
    lead = {"a": {"C": "B"}, "b": {"A": "L"}}[scene].get(car)
    if lead is not None:
        dl, _ = compose_delay(scene, lead, pr)
        r0 = next(r for p, r in mine if lead in p)
        v = r0[f"v0{car}"]
        if scene == "a":
            slack = max(0.0, ctx_x - C1.HEADWAY)
        else:
            slack = max(0.0, (ctx_x - C1.S0) / max(v, 0.1) - C1.HEADWAY)
        d = max(d, dl - slack)
    lost = 1.0 if contact else min(1.0, d / H)
    u = measures.utility(lost, rho, peak, C1.TYPES["car"].b_max, lat, contact)
    return u, {"lost": lost, "rho": rho, "peak": peak, "lat": lat, "contact": int(contact)}


def compose_v2(scene, car, pr, ctx_x, pa):
    """The relation-aware composition (scene a only; found after v1, a hypothesis for N3, not a
    validated model): a pair term enters only if its relation exists once the primary pair's
    (A, B) outcome is known, and a car that drops behind a queue pays the queue's length.

      A ahead of B in the AB pair: A and C never meet; C feels A only through B (max-plus).
      A dropped behind B: the B-C slot holds A iff gap_BC >= S0 + v HEADWAY + L + need_C (A's
      own gap check against C); if not, A drops behind C as well, its delay is the AB pair's
      plus (gap_BC + L) / v, and A and C meet only as a follow (the AC pair's cut-in is not
      their relation); C is untouched."""
    if scene != "a":
        return compose(scene, car, pr, ctx_x)
    ab = pr["AB"]
    keep = {"AB", "BC"}
    extra = 0.0
    if ab["a_pos"] != "ahead":
        v = ab["v0A"]
        gap_bc = C1.S0 + VB * ctx_x
        need_c = pa["buffer"] + v * pa["tau"]
        if gap_bc >= C1.S0 + v * C1.HEADWAY + 4.79 + need_c:
            keep = {"AB", "AC", "BC"}  # A fits between B and C: the literal pairs (not reached here)
        elif car == "A":
            extra = (gap_bc + 4.79) / max(v, 0.1)
    u, parts = compose(scene, car, {p: r for p, r in pr.items() if p in keep}, ctx_x)
    if extra:
        lost = 1.0 if parts["contact"] else min(1.0, parts["lost"] + extra / H)
        u = measures.utility(lost, parts["rho"], parts["peak"], C1.TYPES["car"].b_max, parts["lat"], parts["contact"])
        parts = dict(parts, lost=lost)
    return u, parts


def compose_delay(scene, car, pr):
    mine = [(p, r) for p, r in pr.items() if car in p]
    return max(r[f"lost{car}"] * H for _, r in mine), None


def analyze(out):
    rows = [json.loads(l) for l in open(os.path.join(out, "groups.jsonl")) if l.strip()]
    by = {}
    for r in rows:
        x = r.get("hf") if r["scene"] == "a" else r.get("gl")
        by.setdefault((r["scene"], r["d0"], r["dv"], x, r["a"], r["b"], r["seed"]), {})[r["roles"]] = r
    cells = {}
    for (scene, d0, dv, x, a, b, seed), g in by.items():
        full, pairs = SCENES[scene]
        if full not in g or any(p not in g for p in pairs):
            continue
        r3 = g[full]
        pr = {p: g[p] for p in pairs}
        for car in full:
            uh, parts = compose(scene, car, pr, x)
            uh2, _ = compose_v2(scene, car, pr, x, PA[a])
            c = cells.setdefault((scene, d0, dv, x, a, b, car), [])
            c.append({"seed": seed, "u3": r3[f"u{car}"], "uh": uh, "uh2": uh2,
                      **{f"{k}3": r3[f"{k}{car}" if k != "rho" else f"rho{car}"] for k in ("lost", "rho", "peak", "lat", "contact")},
                      **{f"{k}h": v for k, v in parts.items()}, "a_pos": r3.get("a_pos"),
                      "a_pos2": pr["AB"].get("a_pos")})
    table = []
    for key, xs in sorted(cells.items()):
        scene, d0, dv, x, a, b, car = key
        u3 = [z["u3"] for z in xs]
        r = [z["u3"] - z["uh"] for z in xs]
        sd = st.pstdev(u3) if len(u3) > 1 else 0.0
        tol = max(2 * sd, 0.02)
        lo, hi = boot_ci(r)
        mr = st.mean(r)
        r2 = st.mean(z["u3"] - z["uh2"] for z in xs)
        parts = {}
        for k in ("lost", "rho", "peak", "lat", "contact"):
            parts[f"d_{k}"] = round(st.mean(z[f"{k}3"] - z[f"{k}h"] for z in xs), 4)
        pos3 = max(set(z["a_pos"] for z in xs), key=[z["a_pos"] for z in xs].count)
        pos2 = max(set(z["a_pos2"] for z in xs), key=[z["a_pos2"] for z in xs].count)
        table.append({"scene": scene, "d0": d0, "dv": dv, "x": x, "a": a, "b": b, "car": car, "n": len(xs),
                      "u3": round(st.mean(u3), 4), "sd3": round(sd, 4), "uh": round(st.mean(z["uh"] for z in xs), 4),
                      "r": round(mr, 4), "r_lo": round(lo, 4), "r_hi": round(hi, 4), "tol": round(tol, 4),
                      "within": int(abs(mr) <= tol), "ci_excl0": int(lo > 0 or hi < 0),
                      "a_pos3": pos3, "a_pos2": pos2, **parts,
                      "uh2": round(st.mean(z["uh2"] for z in xs), 4), "r2": round(r2, 4), "within2": int(abs(r2) <= tol)})
    with open(os.path.join(out, "residuals.jsonl"), "w") as f:
        for t in table:
            f.write(json.dumps(t) + "\n")
    report(out, table, rows)


def report(out, table, rows):
    L = []
    p = L.append
    p(f"N2 composition: {len(table)} (scene, context, profile, car) cells, "
      f"{len(rows)} group runs")
    p("")
    p("within D2 tolerance (|mean r| <= max(2 sd(U_3car), 0.02)), by scene / car / profile:")
    for scene in "ab":
        for car in SCENES[scene][0]:
            for a in sorted({t["a"] for t in table}):
                ts = [t for t in table if t["scene"] == scene and t["car"] == car and t["a"] == a]
                if not ts:
                    continue
                w = sum(t["within"] for t in ts)
                rs = [t["r"] for t in ts]
                p(f"  {scene} {car} {a:12s}: {w:2d}/{len(ts)} within; r mean {st.mean(rs):+.4f}, "
                  f"min {min(rs):+.4f}, max {max(rs):+.4f}; CI excludes 0 in {sum(t['ci_excl0'] for t in ts)}")
    w = sum(t["within"] for t in table)
    p(f"  all: {w}/{len(table)} ({100 * w / max(1, len(table)):.0f} %)")
    # contexts: a context counts as predicted when every car in it is within
    ctx = {}
    for t in table:
        ctx.setdefault((t["scene"], t["d0"], t["dv"], t["x"], t["a"]), []).append(t["within"])
    wc = sum(all(v) for v in ctx.values())
    p(f"  contexts with every car within: {wc}/{len(ctx)} ({100 * wc / max(1, len(ctx)):.0f} %)")
    p("")
    p("v2, the relation-aware composition (scene a; post hoc, see compose_v2), within tolerance:")
    for car in "ABC":
        ts = [t for t in table if t["scene"] == "a" and t["car"] == car]
        p(f"  a {car}: v1 {sum(t['within'] for t in ts)}/{len(ts)}, v2 {sum(t['within2'] for t in ts)}/{len(ts)}; "
          f"v2 |r| max {max(abs(t['r2']) for t in ts):.4f}")
    ta = [t for t in table if t["scene"] == "a"]
    tb = [t for t in table if t["scene"] == "b"]
    w2 = sum(t["within2"] for t in ta) + sum(t["within"] for t in tb)
    p(f"  all cells with v2 for scene a, v1 for b: {w2}/{len(table)} ({100 * w2 / max(1, len(table)):.0f} %)")
    p("")
    p("scene b, A closing on L (dv +2): the triple against the AB pair at the context L imposes (dv 0)")
    p("  d0 gl profile | U_A triple (A's slot) | AB pair at dv +2 (slot) | AB pair at dv 0 (slot)")
    for t in tb:
        if t["car"] != "A" or t["dv"] != 2.0:
            continue
        z = [u for u in tb if u["car"] == "A" and u["d0"] == t["d0"] and u["x"] == t["x"] and u["a"] == t["a"] and u["dv"] == 0.0]
        ab0 = pair_mean(rows, "b", t["d0"], 0.0, t["x"], t["a"], "A")
        ab2 = pair_mean(rows, "b", t["d0"], 2.0, t["x"], t["a"], "A")
        p(f"  {t['d0']:4.0f} {t['x']:3.0f} {t['a']:11s} | {t['u3']:+.3f} ({t['a_pos3']}) | {ab2[0]:+.3f} ({ab2[1]}) | "
          f"{ab0[0]:+.3f} ({ab0[1]})" + ("" if z else ""))
    p("")
    p("cells outside the tolerance (scene d0 dv x profile car: U3 sd | U_hat | r [CI] tol | "
      "d_lost d_rho d_peak d_lat d_contact | A's slot 3-car / AB pair):")
    for t in table:
        if not t["within"]:
            p(f"  {t['scene']} {t['d0']:4.0f} {t['dv']:+3.0f} {t['x']:4.0f} {t['a']:11s} {t['car']}: "
              f"{t['u3']:+.3f} {t['sd3']:.3f} | {t['uh']:+.3f} | {t['r']:+.3f} [{t['r_lo']:+.3f},{t['r_hi']:+.3f}] "
              f"{t['tol']:.3f} | {t['d_lost']:+.3f} {t['d_rho']:+.3f} {t['d_peak']:+.2f} {t['d_lat']:+.2f} "
              f"{t['d_contact']:+.2f} | {t['a_pos3']}/{t['a_pos2']}")
    txt = "\n".join(L)
    open(os.path.join(out, "n2_summary.txt"), "w").write(txt + "\n")
    print(txt)


def pair_mean(rows, scene, d0, dv, x, a, car):
    key = "hf" if scene == "a" else "gl"
    rs = [r for r in rows if r["scene"] == scene and r["roles"] == "AB" and r["d0"] == d0 and r["dv"] == dv
          and r.get(key) == x and r["a"] == a]
    pos = [r["a_pos"] for r in rs]
    return st.mean(r[f"u{car}"] for r in rs), max(set(pos), key=pos.count)


def pair_check(ports, out, k, seeds):
    """arena.Pair (the two-car game as C2a/do4 run it) on scene a's AB contexts, to compare with
    arena_n's AB group (its ghost warm-up and per-car random streams)."""
    work = []
    for (d0, dv), (pa, pb), s in itertools.product(itertools.product(D0S, DVS), PROFILES, range(seeds)):
        work.append({"d0": d0, "dv": dv, "vb": VB, "pa": pa, "pb": pb, "seed": s, "a": pa["name"], "b": pb["name"]})
    run_pool(ports, k, "pairs", work, os.path.join(out, "pair_check.jsonl"), chunk=4 * k)


def pair_check_report(out):
    pc = [json.loads(l) for l in open(os.path.join(out, "pair_check.jsonl"))]
    gr = [json.loads(l) for l in open(os.path.join(out, "groups.jsonl"))]
    gr = [r for r in gr if r["roles"] == "AB" and r["scene"] == "a"]
    L = ["Pair (arena.py) against arena_n's AB group, scene a contexts (both hf pooled for the group):",
         "d0 dv profile | Pair uA mean sd, uB mean sd, ahead | group uA, uB, ahead | in Pair's interval (A, B)"]
    n_in = n = 0
    for (d0, dv), (pa, _) in itertools.product(itertools.product(D0S, DVS), PROFILES):
        p_ = [r for r in pc if r["d0"] == d0 and r["dv"] == dv and r["a"] == pa["name"]]
        g_ = [r for r in gr if r["d0"] == d0 and r["dv"] == dv and r["a"] == pa["name"]]
        if not p_ or not g_:
            continue
        line = f"{d0:4.0f} {dv:+3.0f} {pa['name']:11s} |"
        ins = []
        for who in "AB":
            xs = [r[f"u{who}"] for r in p_]
            mu, sd = st.mean(xs), st.pstdev(xs)
            gm = st.mean(r[f"u{who}"] for r in g_)
            ins.append(abs(gm - mu) <= max(2 * sd, 0.02))
            line += f" {mu:+.3f} {sd:.3f} / {gm:+.3f} |"
        line += f" ahead {st.mean(r['ahead'] for r in p_):.2f} / {st.mean(r['a_pos'] == 'ahead' for r in g_):.2f} | {ins}"
        n += 2
        n_in += sum(ins)
        L.append(line)
    L.append(f"group means inside Pair's interval: {n_in}/{n}")
    txt = "\n".join(L)
    open(os.path.join(out, "pair_check.txt"), "w").write(txt + "\n")
    print(txt)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=("run", "pair-check", "analyze", "smoke"))
    ap.add_argument("--ports", default="2600")
    ap.add_argument("--out", required=True)
    ap.add_argument("--seeds", type=int, default=6)
    ap.add_argument("--k", type=int, default=6)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    ports = [int(p) for p in a.ports.split(",")]
    if a.mode == "run":
        work = jobs(a.seeds)
        print(f"{len(work)} group runs on {ports}", flush=True)
        run_pool(ports, a.k, "groups", work, os.path.join(a.out, "groups.jsonl"), chunk=4 * a.k)
        analyze(a.out)
    elif a.mode == "smoke":
        work = [j for j in jobs(1) if j["d0"] == 6.0 and j["dv"] == 0.0 and j.get("hf", 1.0) == 1.0
                and j.get("gl", 15.0) == 15.0 and j["pa"]["name"] == "A_aw_perm"]
        run_pool(ports[:1], a.k, "groups", work, os.path.join(a.out, "groups.jsonl"), chunk=len(work))
        for l in open(os.path.join(a.out, "groups.jsonl")):
            r = json.loads(l)
            r.pop("trace", None)
            print(json.dumps(r))
    elif a.mode == "pair-check":
        pair_check(ports, a.out, a.k, a.seeds)
        pair_check_report(a.out)
    else:
        analyze(a.out)
        if os.path.exists(os.path.join(a.out, "pair_check.jsonl")):
            pair_check_report(a.out)


if __name__ == "__main__":
    mp.set_start_method("spawn")
    main()
