#!/usr/bin/env python3
"""Roadmap 074 C2a: double oracle over the cut-in's parameterized rules, in the arena.

A rule is a light policy with a few readable parameters (arena.py, `pa` / `pb`):

  A  go when the gap ahead of B (A's rear to B's front, as seen) is at least
         buffer + v_B * tau + max(0, v_B^2 - v_A^2) / (2 a_rear)
     -- the shape of Autoware's lane-change safety check, whose rear-vehicle assumptions are a
     reaction time tau and a deceleration a_rear -- checking for `patience` s; with `use_yield`,
     also go once B is seen braking; after that drop behind B. `t_prep`: the time from the
     decision to the first lateral motion (Autoware's prepare phase).
  B  yield (decelerate at b_y until a following gap is open) when A is beside or ahead within
     `pre_band` m before any lane change is seen, or within `react_gap` m once it is seen; hold
     otherwise. The safety layer brakes under every rule.

The game's payoff for a pair of rules is each car's mean utility (cutin_game's) over a set of
contexts (the band where C1 and the arena disagree or are unsure, and the cut-ins that ended in
contact) and seeds. Double oracle: solve the meta-game over the current populations (all Nash
equilibria by support enumeration; the one with the highest welfare is used), find each side's
best response to the other's equilibrium mixture by search (random rules, then local
refinement), add them if they gain, repeat. Each round reports the equilibrium, its welfare and
the search's exploitability gain, a lower bound.

The Autoware-like rules start in the populations: A_aw_cons (tau 2 s, a_rear 1 m/s^2) and
A_aw_perm (1 s, 2 m/s^2), each with a 3 s prepare phase; B_aw holds (yields to no cut-in; only
its safety layer brakes); B_aw_react is Autoware as C3 measured it on the real stack (it yields
to a lane change seen within 20 m, braking at 2.5 m/s^2). At the end, each one's best counter-rule is searched and reported.

Usage:
  scripts/coop/double_oracle.py --ports 2600,2610 --out <dir> [--rounds 4] [--k 4]
"""

import argparse
import itertools
import json
import math
import multiprocessing as mp
import os
import random
import sys
import time
import zlib
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

CONTEXTS = [(2.0, 2.0), (6.0, 0.0), (6.0, 3.0), (10.0, 0.0), (10.0, -2.0), (14.0, -3.0)]
SEEDS = 2

A_AW_CONS = {"name": "A_aw_cons", "tau": 2.0, "a_rear": 1.0, "buffer": 2.0, "patience": 5.0, "use_yield": 0, "t_prep": 3.0}
A_AW_PERM = {"name": "A_aw_perm", "tau": 1.0, "a_rear": 2.0, "buffer": 2.0, "patience": 5.0, "use_yield": 0, "t_prep": 3.0}
A_BOLD = {"name": "A_bold", "tau": 0.3, "a_rear": 4.0, "buffer": 1.0, "patience": 2.0, "use_yield": 1, "t_prep": 0.0}
B_AW = {"name": "B_aw", "pre_band": -99.0, "react_gap": -99.0, "b_y": 2.0, "stop": 0}
# Autoware as C3 measured it (roadmap 074 C3, c3_fit.py): it brakes on the lane change's onset,
# 0.5 s after it (its own "react"; the arena's car default is 1.0 s), not once A is in its lane
# as B_aw does; fitted at v_B 6 m/s on Town03's west straight (C3a ground truth and C3b
# CenterPoint, the outcome shares against Autoware's binomial intervals; 074 job 5)
B_AW_REACT = {"name": "B_aw_react", "pre_band": -99.0, "react_gap": 20.0, "b_y": 2.5, "stop": 0, "react": 0.5}
B_COURTEOUS = {"name": "B_court", "pre_band": 8.0, "react_gap": 15.0, "b_y": 2.0, "stop": 0}
B_REACT = {"name": "B_react", "pre_band": -99.0, "react_gap": 10.0, "b_y": 2.5, "stop": 0}


def rand_a(rng, name):
    return {"name": name, "tau": round(rng.uniform(0.0, 3.0), 2), "a_rear": round(rng.uniform(0.5, 6.0), 2),
            "buffer": round(rng.uniform(0.0, 8.0), 2), "patience": round(rng.uniform(0.5, 6.0), 2),
            "use_yield": rng.randint(0, 1), "t_prep": 0.0}


def rand_b(rng, name):
    return {"name": name,
            "pre_band": -99.0 if rng.random() < 0.5 else round(rng.uniform(-4.0, 15.0), 2),
            "react_gap": -99.0 if rng.random() < 0.3 else round(rng.uniform(0.0, 25.0), 2),
            "b_y": round(rng.uniform(1.0, 4.0), 2), "stop": 0}


def jitter(p, rng, name, side):
    q = dict(p, name=name)
    keys = ("tau", "a_rear", "buffer", "patience") if side == "A" else ("pre_band", "react_gap", "b_y")
    lim = {"tau": (0, 3), "a_rear": (0.5, 6), "buffer": (0, 8), "patience": (0.5, 6),
           "pre_band": (-4, 15), "react_gap": (0, 25), "b_y": (1, 4)}
    for k in keys:
        if q[k] <= -98:
            continue
        lo, hi = lim[k]
        q[k] = round(min(hi, max(lo, q[k] + rng.gauss(0, 0.15 * (hi - lo)))), 2)
    return q


# --- the arena workers ---------------------------------------------------------------------------
def worker(port, k, inbox, outbox):
    import arena as AR

    class Args:
        pass
    args = Args()
    args.k, args.obs_delay, args.obs_sigma_x, args.obs_sigma_v, args.reaction_jitter = k, 0.19, 0.3, 0.2, 0.2
    ar = AR.Arena(port, k)
    try:
        cars = AR.setup_cars(ar, k)
        while True:
            msg = inbox.get()
            if msg is None:
                break
            tag, jobs = msg
            rng = random.Random(zlib.crc32(tag.encode()))  # stable across processes; each job seeds its own
            rows = AR.run_jobs(ar, cars, jobs, rng, args)
            keep = ("uA", "uB", "contact", "ahead", "got_in", "t_run",
                    # arena.Pair's planned merge end and the legacy end's shadow (074, the merge-end fix), when present
                    "t_plan", "uA_legacy", "uB_legacy", "got_in_legacy", "ahead_legacy", "peakA", "peakB", "t_lat")
            outbox.put((tag, [{k: r[k] for k in keep if k in r} for r in rows]))
    finally:
        ar.close()


class Pool:
    def __init__(self, ports, k):
        self.inbox = [mp.Queue() for _ in ports]
        self.outbox = mp.Queue()
        self.procs = [mp.Process(target=worker, args=(p, k, q, self.outbox), daemon=True) for p, q in zip(ports, self.inbox)]
        for p in self.procs:
            p.start()
        self.runs = 0

    def evaluate(self, pairs, seeds=SEEDS, salt=0, per_run=False):
        """{(a_name, b_name): {uA, uB, contact, ahead}} for each (pa, pb), over CONTEXTS x seeds.

        Each run is seeded from its context, seed index and `salt` (crc32), so every rule pair
        meets the same noise in the same run (common random numbers) and an evaluation repeats
        exactly; another salt draws fresh seeds. per_run: also {key: {(context, seed): row}}."""
        jobs = []
        for pa, pb in pairs:
            for (d0, dv), s in itertools.product(CONTEXTS, range(seeds)):
                jobs.append({"d0": d0, "dv": dv, "vb": 12.0, "pa": pa, "pb": pb, "seed": s,
                             "rng_seed": zlib.crc32(f"{d0}:{dv}:{s}:{salt}".encode()),
                             "key": (pa["name"], pb["name"]), "run": ((d0, dv), s)})
        n = len(self.inbox)
        shards = [jobs[i::n] for i in range(n)]
        for i, sh in enumerate(shards):
            self.inbox[i].put((f"{time.time()}-{i}", sh))
        got, runs = {}, {}
        for _ in range(n):
            tag, rows = self.outbox.get()
            i = int(tag.rsplit("-", 1)[1])
            for j, r in zip(shards[i], rows):
                got.setdefault(j["key"], []).append(r)
                runs.setdefault(j["key"], {})[j["run"]] = r
        self.runs += len(jobs)
        out = {}
        for key, rs in got.items():
            out[key] = {m: sum(float(r[m]) for r in rs) / len(rs) for m in ("uA", "uB", "contact", "ahead")}
        return (out, runs) if per_run else out

    def close(self):
        for q in self.inbox:
            q.put(None)
        for p in self.procs:
            p.join(timeout=60)


# --- the meta-game ---------------------------------------------------------------------------------
def nash_all(A, B):
    """All Nash equilibria of the bimatrix game (A, B), by support enumeration (small games)."""
    m, n = A.shape
    eqs = []
    for k in range(1, min(m, n) + 1):
        for I in itertools.combinations(range(m), k):
            for J in itertools.combinations(range(n), k):
                # q on J makes A's rows in I indifferent; p on I makes B's columns in J indifferent
                def solve(M, rows, cols):
                    k = len(rows)
                    mat = np.zeros((k + 1, k + 1))
                    rhs = np.zeros(k + 1)
                    for a, i in enumerate(rows):
                        mat[a, :k] = M[i, list(cols)]
                        mat[a, k] = -1.0
                    mat[k, :k] = 1.0
                    rhs[k] = 1.0
                    try:
                        sol = np.linalg.solve(mat, rhs)
                    except np.linalg.LinAlgError:
                        return None
                    return sol[:k], sol[k]
                r1 = solve(A, I, J)
                r2 = solve(B.T, J, I)
                if r1 is None or r2 is None:
                    continue
                q, va = r1
                p, vb = r2
                if (q < -1e-9).any() or (p < -1e-9).any():
                    continue
                P = np.zeros(m)
                Q = np.zeros(n)
                P[list(I)] = p
                Q[list(J)] = q
                if (A @ Q > va + 1e-7).any() or (P @ B > vb + 1e-7).any():
                    continue
                eqs.append((P.clip(0), Q.clip(0)))
    return eqs


def search_br(pool, side, opp, mix, rng, base_name, n_rand=40, n_local=12, extra=()):
    """Best rule for `side` against the mixture `mix` over the rules `opp`; returns (rule, value)."""
    support = [(o, w) for o, w in zip(opp, mix) if w > 1e-6]
    cands = [(rand_a if side == "A" else rand_b)(rng, f"{base_name}_r{i}") for i in range(n_rand)] + list(extra)

    def value(cands):
        pairs = [((c, o) if side == "A" else (o, c)) for c in cands for o, _ in support]
        res = pool.evaluate(pairs)
        vals = []
        for c in cands:
            v = 0.0
            for o, w in support:
                key = (c["name"], o["name"]) if side == "A" else (o["name"], c["name"])
                v += w * res[key]["uA" if side == "A" else "uB"]
            vals.append(v)
        return vals

    vals = value(cands)
    allc, allv = list(cands), list(vals)
    # two rounds of local refinement around the three best so far
    for stage in range(2):
        top = sorted(range(len(allc)), key=lambda i: -allv[i])[:3]
        loc = [jitter(allc[t], rng, f"{base_name}_s{stage}l{t}_{j}", side) for t in top for j in range(n_local // 3 + 1)]
        allc += loc
        allv += value(loc)
    i = max(range(len(allc)), key=lambda i: allv[i])
    return allc[i], allv[i]


VALIDATE_SEEDS = 4  # fresh seeds per context for a best response's check


def validate(pool, side, br, own, own_mix, opp, opp_mix, salt):
    """A searched best response against the incumbent (`side`'s equilibrium mixture), both against
    the opponent's mixture, on fresh seeds shared by the two (common random numbers): returns
    (gain, se). The search picks the best of many noisy estimates, so its own gain is biased up
    (074 do5: 0.018 that the table never showed)."""
    u = "uA" if side == "A" else "uB"
    sup_o = [(o, w) for o, w in zip(opp, opp_mix) if w > 1e-6]
    sup_i = [(r, w) for r, w in zip(own, own_mix) if w > 1e-6]
    rules = [br] + [r for r, _ in sup_i if r["name"] != br["name"]]
    pairs = [((r, o) if side == "A" else (o, r)) for r in rules for o, _ in sup_o]
    _, runs = pool.evaluate(pairs, seeds=VALIDATE_SEEDS, salt=salt, per_run=True)
    key = lambda r, o: (r["name"], o["name"]) if side == "A" else (o["name"], r["name"])
    gain, var = 0.0, 0.0
    for o, wo in sup_o:
        rb = runs[key(br, o)]
        d = []
        for run in rb:
            inc = sum(wi * float(runs[key(r, o)][run][u]) for r, wi in sup_i)
            d.append(float(rb[run][u]) - inc)
        m = sum(d) / len(d)
        v = sum((x - m) ** 2 for x in d) / max(1, len(d) - 1)
        gain += wo * m
        var += wo * wo * v / len(d)
    return gain, var ** 0.5


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ports", default="2600,2610")
    ap.add_argument("--k", type=int, default=4)
    ap.add_argument("--rounds", type=int, default=4)
    ap.add_argument("--out", required=True)
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    rng = random.Random(args.seed)
    pool = Pool([int(p) for p in args.ports.split(",")], args.k)
    log = open(os.path.join(args.out, "log.jsonl"), "w")
    t0 = time.time()
    try:
        pop_a = [A_AW_CONS, A_AW_PERM, A_BOLD]
        pop_b = [B_AW, B_AW_REACT, B_COURTEOUS, B_REACT]
        table = pool.evaluate([(a, b) for a in pop_a for b in pop_b])
        found_a, found_b = [], []  # every best response found so far, tried again in later searches
        for rnd in range(args.rounds + 1):
            A = np.array([[table[(a["name"], b["name"])]["uA"] for b in pop_b] for a in pop_a])
            B = np.array([[table[(a["name"], b["name"])]["uB"] for b in pop_b] for a in pop_a])
            eqs = nash_all(A, B) or [(np.ones(len(pop_a)) / len(pop_a), np.ones(len(pop_b)) / len(pop_b))]
            P, Q = max(eqs, key=lambda e: e[0] @ (A + B) @ e[1])
            va, vb = P @ A @ Q, P @ B @ Q
            contact = sum(P[i] * Q[j] * table[(a["name"], b["name"])]["contact"]
                          for i, a in enumerate(pop_a) for j, b in enumerate(pop_b))
            rec = {"round": rnd, "runs": pool.runs, "wall_s": round(time.time() - t0),
                   "n_eq": len(eqs), "A_mix": {a["name"]: round(float(w), 3) for a, w in zip(pop_a, P) if w > 1e-3},
                   "B_mix": {b["name"]: round(float(w), 3) for b, w in zip(pop_b, Q) if w > 1e-3},
                   "vA": round(float(va), 4), "vB": round(float(vb), 4), "welfare": round(float(va + vb), 4),
                   "contact_rate": round(float(contact), 4)}
            if rnd == args.rounds:
                print(json.dumps(rec), flush=True)
                log.write(json.dumps(rec) + "\n")
                break
            br_a, val_a = search_br(pool, "A", pop_b, Q, rng, f"A{rnd + 1}", extra=[x for x in found_a])
            br_b, val_b = search_br(pool, "B", pop_a, P, rng, f"B{rnd + 1}", extra=[x for x in found_b])
            found_a.append(br_a)
            found_b.append(br_b)
            # the searched gains are biased up (best of many noisy estimates): each best response
            # is checked against the incumbent on fresh seeds, common to both, and enters the
            # pool only if its gain there exceeds 2 standard errors
            ga, sa = validate(pool, "A", br_a, pop_a, P, pop_b, Q, salt=1000 + 2 * rnd)
            gb, sb = validate(pool, "B", br_b, pop_b, Q, pop_a, P, salt=1001 + 2 * rnd)
            rec.update({"BR_A": br_a, "gain_A": round(val_a - float(va), 4),
                        "gain_A_check": round(ga, 4), "se_A": round(sa, 4),
                        "BR_B": br_b, "gain_B": round(val_b - float(vb), 4),
                        "gain_B_check": round(gb, 4), "se_B": round(sb, 4)})
            keep_a = ga > 2 * sa and ga > 0.0
            keep_b = gb > 2 * sb and gb > 0.0
            # exploitability's lower bound from the checked gains (the searched ones, as before,
            # in exploitability_lb_search)
            rec["exploitability_lb"] = round(max(0.0, ga) * keep_a + max(0.0, gb) * keep_b, 4)
            rec["exploitability_lb_search"] = round(max(0.0, rec["gain_A"]) + max(0.0, rec["gain_B"]), 4)
            print(json.dumps(rec), flush=True)
            log.write(json.dumps(rec) + "\n")
            log.flush()
            new_a = [br_a] if keep_a else []
            new_b = [br_b] if keep_b else []
            if not new_a and not new_b and rnd >= 1:
                print("no best response gains: an approximate equilibrium", flush=True)
                log.write(json.dumps({"round": rnd, "converged": True}) + "\n")
                break
            new_a = [a for a in new_a if a["name"] not in {x["name"] for x in pop_a}]
            new_b = [b for b in new_b if b["name"] not in {x["name"] for x in pop_b}]
            pairs = [(a, b) for a in new_a for b in pop_b + new_b] + [(a, b) for a in pop_a for b in new_b]
            table.update(pool.evaluate(pairs))
            pop_a += new_a
            pop_b += new_b
        # the Autoware-like rules, exploited
        expl = {}
        for side, rule in (("B", A_AW_CONS), ("B", A_AW_PERM), ("A", B_AW), ("A", B_AW_REACT)):
            br, val = search_br(pool, side, [rule], [1.0], rng, f"X_{rule['name']}")
            opp_key = {"A": "uA", "B": "uB"}[side]
            res = pool.evaluate([(br, rule) if side == "A" else (rule, br)])
            r = next(iter(res.values()))
            expl[rule["name"]] = {"counter_rule": br, "counter_value": round(val, 4),
                                  "rule_value_vs_counter": round(r["uB" if side == "A" else "uA"], 4),
                                  "contact_rate": round(r["contact"], 3), "A_ahead_rate": round(r["ahead"], 3)}
        with open(os.path.join(args.out, "populations.json"), "w") as f:
            json.dump({"A": pop_a, "B": pop_b, "table": {f"{k[0]}|{k[1]}": v for k, v in table.items()},
                       "autoware_like_exploited": expl, "runs": pool.runs, "wall_s": round(time.time() - t0)}, f, indent=1)
        print(json.dumps({"autoware_like_exploited": expl, "runs": pool.runs}, indent=1), flush=True)
    finally:
        pool.close()


if __name__ == "__main__":
    mp.set_start_method("spawn")
    main()
