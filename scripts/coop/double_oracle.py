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
its safety layer brakes). At the end, each one's best counter-rule is searched and reported.

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
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

CONTEXTS = [(2.0, 2.0), (6.0, 0.0), (6.0, 3.0), (10.0, 0.0), (10.0, -2.0), (14.0, -3.0)]
SEEDS = 2

A_AW_CONS = {"name": "A_aw_cons", "tau": 2.0, "a_rear": 1.0, "buffer": 2.0, "patience": 5.0, "use_yield": 0, "t_prep": 3.0}
A_AW_PERM = {"name": "A_aw_perm", "tau": 1.0, "a_rear": 2.0, "buffer": 2.0, "patience": 5.0, "use_yield": 0, "t_prep": 3.0}
A_BOLD = {"name": "A_bold", "tau": 0.3, "a_rear": 4.0, "buffer": 1.0, "patience": 2.0, "use_yield": 1, "t_prep": 0.0}
B_AW = {"name": "B_aw", "pre_band": -99.0, "react_gap": -99.0, "b_y": 2.0}
B_COURTEOUS = {"name": "B_court", "pre_band": 8.0, "react_gap": 15.0, "b_y": 2.0}
B_REACT = {"name": "B_react", "pre_band": -99.0, "react_gap": 10.0, "b_y": 2.5}


def rand_a(rng, name):
    return {"name": name, "tau": round(rng.uniform(0.0, 3.0), 2), "a_rear": round(rng.uniform(0.5, 6.0), 2),
            "buffer": round(rng.uniform(0.0, 8.0), 2), "patience": round(rng.uniform(0.5, 6.0), 2),
            "use_yield": rng.randint(0, 1), "t_prep": 0.0}


def rand_b(rng, name):
    return {"name": name,
            "pre_band": -99.0 if rng.random() < 0.5 else round(rng.uniform(-4.0, 15.0), 2),
            "react_gap": -99.0 if rng.random() < 0.3 else round(rng.uniform(0.0, 25.0), 2),
            "b_y": round(rng.uniform(1.0, 4.0), 2)}


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
            rng = random.Random(hash(tag) & 0xffffffff)
            rows = AR.run_jobs(ar, cars, jobs, rng, args)
            outbox.put((tag, [{k: r[k] for k in ("uA", "uB", "contact", "ahead", "got_in", "t_run")} for r in rows]))
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

    def evaluate(self, pairs):
        """{(a_name, b_name): {uA, uB, contact, ahead}} for each (pa, pb), over CONTEXTS x SEEDS."""
        jobs = []
        for pa, pb in pairs:
            for (d0, dv), s in itertools.product(CONTEXTS, range(SEEDS)):
                jobs.append({"d0": d0, "dv": dv, "vb": 12.0, "pa": pa, "pb": pb, "seed": s,
                             "key": (pa["name"], pb["name"])})
        n = len(self.inbox)
        shards = [jobs[i::n] for i in range(n)]
        for i, sh in enumerate(shards):
            self.inbox[i].put((f"{time.time()}-{i}", sh))
        got = {}
        for _ in range(n):
            tag, rows = self.outbox.get()
            i = int(tag.rsplit("-", 1)[1])
            for j, r in zip(shards[i], rows):
                got.setdefault(j["key"], []).append(r)
        self.runs += len(jobs)
        out = {}
        for key, rs in got.items():
            out[key] = {m: sum(float(r[m]) for r in rs) / len(rs) for m in ("uA", "uB", "contact", "ahead")}
        return out

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
        pop_b = [B_AW, B_COURTEOUS, B_REACT]
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
            rec.update({"BR_A": br_a, "gain_A": round(val_a - float(va), 4),
                        "BR_B": br_b, "gain_B": round(val_b - float(vb), 4)})
            rec["exploitability_lb"] = round(max(0.0, rec["gain_A"]) + max(0.0, rec["gain_B"]), 4)
            print(json.dumps(rec), flush=True)
            log.write(json.dumps(rec) + "\n")
            log.flush()
            new_a = [br_a] if rec["gain_A"] > 0.005 else []
            new_b = [br_b] if rec["gain_B"] > 0.005 else []
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
        for side, rule in (("B", A_AW_CONS), ("B", A_AW_PERM), ("A", B_AW)):
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
