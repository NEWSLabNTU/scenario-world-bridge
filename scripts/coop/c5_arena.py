#!/usr/bin/env python3
"""Roadmap 074 C5 in the arena: the cut-in with a heavy car on its own CARLA physics.

arena.py's cut-in (C2a's runs: A's three strategies x B's three profiles, pre_yield, react_yield,
react_hold, seeded per context and seed), with each car its own blueprint:
  m3   the spec's Model 3 (carla_physics applied, acb's controls: the spec's pedal maps and lags),
       as every arena study before C5
  hgv  CARLA 0.9.16's vehicle.carlamotors.european_hgv with its stock physics (nothing applied:
       no truck is in sim_vehicle_spec, E4), driven through the pedal maps E4 measured for it
       (e4_heavy.py: <e4>/maps/carlamotors.european_hgv/{accel,brake}_map.csv) and no added lag
       (CARLA's own brake build-up, t63 about 0.25 s, is in its physics)
Each car's rules (IDM following, cruise, lane change time) and utility use its own capability
(arena.Pair.cap_a / cap_b, c5_cars.types: E4's 0-12 m/s mean and peak braking, the heavy's
4 s lane change). The game per context is arena_game's (B's REACT its better continuation),
the equilibrium payoffs the pure equilibria's mean, and the commitment measures c5_cars's.

Usage (a CARLA of its own; this script makes it synchronous and puts it back):
  scripts/coop/c5_arena.py run --port 2600 --out <dir> --e4 <e4 dir> [--pairs m3:m3,m3:hgv,hgv:m3] [--k 6] [--seeds 8]
  scripts/coop/c5_arena.py analyze --out <dir> --kin <c5_cars out dir>
  scripts/coop/c5_arena.py --selftest
"""

from __future__ import annotations

import argparse
import csv
import itertools
import json
import math
import os
import random
import statistics as st
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import c5_cars as K  # noqa: E402
import cutin_game as C1  # noqa: E402

BP = {"hgv": "vehicle.carlamotors.european_hgv", "fire": "vehicle.carlamotors.firetruck"}
D0S = [float(d) for d in range(-6, 31, 4)]
DVS = [-4.0, -2.0, 0.0, 2.0, 4.0]
B_PROFILES = (C1.PRE_YIELD, "react_yield", "react_hold")


def jobs(vb, seeds):
    out = []
    for (d0, dv), a, b, s in itertools.product(itertools.product(D0S, DVS), C1.A_STRATS, B_PROFILES, range(seeds)):
        out.append({"d0": d0, "dv": dv, "vb": vb, "a": a, "b": b, "seed": s,
                    "rng_seed": f"c5|{d0}|{dv}|{s}"})
    return out


def make_classes(e4dir):
    import arena as AR

    class StockCar(AR.Car):
        """A blueprint with its stock physics, driven through E4's pedal maps for it, no added lag;
        its geometry from its wheels whatever their number (the HGV has six)."""

        def __init__(self, arena, actor, name):
            self.arena = arena
            self.a = actor
            pc = actor.get_physics_control()
            xs = [(w.position.x / 100.0, w.position.y / 100.0) for w in pc.wheels]
            fx = max(x for x, _ in xs)
            front = [p for p in xs if p[0] >= fx - 0.1]
            rear = [p for p in xs if p[0] < fx - 0.1]
            self.wheelbase = fx - st.mean(x for x, _ in rear)
            track = max(y for _, y in front) - min(y for _, y in front)
            self.steering = AR.Steering(math.radians(pc.wheels[0].max_steer_angle), abs(track) / self.wheelbase)
            self.lon = AR.Longitudinal.__new__(AR.Longitudinal)
            d = os.path.join(e4dir, "maps", BP[name].split(".", 1)[1])
            self.lon.accel = AR.PedalMap(os.path.join(d, "accel_map.csv"))
            self.lon.brake = AR.PedalMap(os.path.join(d, "brake_map.csv"))
            self.lag = AR.PedalLag({})
            bb = actor.bounding_box
            self.length, self.width = 2 * bb.extent.x, 2 * bb.extent.y
            self.v_prev = 0.0
            self.acc = 0.0

        def park(self, tf):
            super().park(tf)
            self.lag = AR.PedalLag({})

    class C5Pair(AR.Pair):
        caps = (None, None)

        def __init__(self, *a, **k):
            super().__init__(*a, **k)
            self.cap_a, self.cap_b = C5Pair.caps
            # B's reaction as the type's (every type here reacts in 1 s, as cutin_game's)

    return StockCar, C5Pair


def run(args):
    import arena as AR
    ty = K.types(os.path.join(args.e4, "e4.json"))
    StockCar, C5Pair = make_classes(args.e4)
    AR.Pair = C5Pair  # arena.run_jobs builds its pairs by this name
    os.makedirs(args.out, exist_ok=True)
    for pair in args.pairs.split(","):
        na, nb = pair.split(":")
        path = os.path.join(args.out, f"runs_{na}_x_{nb}.csv")
        work = jobs(args.vb, args.seeds)
        done = 0
        if os.path.exists(path):
            done = sum(1 for _ in open(path)) - 1
        work = work[max(0, done):]
        if args.limit:
            work = work[:args.limit]
        if not work:
            continue
        print(f"[{pair}] {len(work)} runs (of {len(jobs(args.vb, args.seeds))})", flush=True)
        ar = AR.Arena(args.port, args.k)
        try:
            cars = []
            m3 = []
            lib = ar.world.get_blueprint_library()
            import carla
            for r in range(args.k):
                row = []
                for role, n, lane, x in (("A", na, -2, 60.0), ("B", nb, -1, 30.0)):
                    if n == "m3":
                        act = ar.spawn(r, lane, x)
                        m3.append(act)
                    else:
                        y, z = ar.lane_y(r, lane)
                        act = ar.world.spawn_actor(lib.find(BP[n]), carla.Transform(carla.Location(x=x, y=y, z=z + 1.0)))
                        ar.actors.append(act)
                    row.append((n, act))
                cars.append(row)
            ar.put_physics(m3)
            for _ in range(10):
                ar.tick()
            cars = [tuple(AR.Car(ar, act) if n == "m3" else StockCar(ar, act, n) for n, act in row) for row in cars]
            C5Pair.caps = (ty[na], ty[nb])
            rng = random.Random(1)
            for i in range(0, len(work), args.chunk):
                rows = AR.run_jobs(ar, cars, work[i:i + args.chunk], rng, args, None, log=False)
                new = not os.path.exists(path) or os.path.getsize(path) == 0
                with open(path, "a", newline="") as f:
                    w = csv.DictWriter(f, fieldnames=FIELDS, extrasaction="ignore")
                    if new:
                        w.writeheader()
                    w.writerows(rows)
                print(f"  [{pair}] {done + i + len(rows)} runs", flush=True)
        finally:
            ar.close()


FIELDS = ("d0", "dv", "vb", "a", "b", "seed", "road", "d0_real", "dv_real", "vb_real", "uA", "uB", "lostA", "lostB",
          "ttc", "thw", "contact", "got_in", "ahead", "peakA", "peakB", "t_run")


# --- analysis ------------------------------------------------------------------------------------
def game(cc):
    """arena_game.classify's game, with the equilibrium payoffs: (class, pure, (U_A, U_B) or None,
    yield_to_A, yield_to_B)."""
    import arena_game as G
    lab, pure = G.classify(cc)
    pay, ya, yb = [], [], []
    for a, b in pure:
        if b == C1.PRE_YIELD:
            rows, cont = cc[(a, "pre_yield")], "yield"
        else:
            ry, rh = cc[(a, "react_yield")], cc[(a, "react_hold")]
            rows, cont = (ry, "yield") if G.mean(ry, "uB") >= G.mean(rh, "uB") else (rh, "hold")
        went = G.mean(rows, "ahead") > 0.5 and a != C1.WAIT
        pay.append((G.mean(rows, "uA"), G.mean(rows, "uB")))
        yb.append(not went)
        ya.append(went and cont == "yield")
    u = (st.mean(p[0] for p in pay), st.mean(p[1] for p in pay)) if pay else None
    return lab, pure, u, bool(pure) and all(ya), bool(pure) and all(yb)


def analyze(out, kin):
    import arena_game as G
    maps = {}
    for f in sorted(os.listdir(out)):
        if not (f.startswith("runs_") and f.endswith(".csv")):
            continue
        na, nb = f[5:-4].split("_x_")
        cells = G.load([os.path.join(out, f)])
        m = {}
        for d0, dv in sorted({(k[0], k[1]) for k in cells}):
            cc = {(a, b): cells.get((d0, dv, a, b), []) for a in C1.A_STRATS for b in B_PROFILES}
            if any(not v for v in cc.values()):
                continue
            lab, pure, u, ya, yb = game(cc)
            m[(d0, dv)] = {"cls": lab, "u": u, "yield_to_A": ya, "yield_to_B": yb, "loss": None, "contact_off_eq": 0}
        maps[(na, nb, 12.0)] = m
    L = ["C5 arena: classes per context (rows dv top = +4; columns d0 " + " ".join(f"{d:g}" for d in D0S) + ")",
         "arena | kinematic (c5_cars at the same contexts)"]
    kinm = {}
    for key in maps:
        p = os.path.join(kin, f"map_{key[0]}_x_{key[1]}_v12.jsonl")
        kinm[key] = {(r["d0"], r["dv"]): r for r in map(json.loads, open(p))} if os.path.exists(p) else {}
    for key, m in maps.items():
        L.append(f"{key[0]} x {key[1]} @ 12  ({len(m)} contexts)")
        agree = 0
        for dv in reversed(DVS):
            ra = "".join(m[(d0, dv)]["cls"] if (d0, dv) in m else " " for d0 in D0S)
            rk = "".join(kinm[key][(d0, dv)]["cls"] if (d0, dv) in kinm[key] else " " for d0 in D0S)
            agree += sum(1 for d0 in D0S if (d0, dv) in m and (d0, dv) in kinm[key] and m[(d0, dv)]["cls"] == kinm[key][(d0, dv)]["cls"])
            L.append(f"  {dv:+4.0f}  {ra}  |  {rk}")
        L.append(f"  class equal to the kinematic one: {agree} of {len(m)}")
    L.append("")
    L.append("commitment (as c5_cars.compare; the weak car against the Model 3 in its role):")
    for weak, role in (("hgv", "B"), ("hgv", "A")):
        kx = ("m3", weak, 12.0) if role == "B" else (weak, "m3", 12.0)
        if kx in maps and ("m3", "m3", 12.0) in maps:
            common = set(maps[kx]) & set(maps[("m3", "m3", 12.0)])
            sub = {k: {c: v[c] for c in common} for k, v in maps.items()}
            c = K.compare(sub, weak, role, 12.0)
            L.append(f"  {c['pair']} ({role}): yielded to X {c['yielded_weak']}/{c['contexts']}, to m3 {c['yielded_m3']}; "
                     f"flips to X {c['flip_to_weak']}, from X {c['flip_from_weak']}; X pays (dU > 0.01) {c['pays']} "
                     f"(and yielded {c['pays_and_yielded']}); dU mean {c['dU_mean']:+.3f}, max {c['dU_max']:+.3f}; "
                     f"other's dU mean {c['dU_other_mean']:+.3f}; same class {c['same_class']}")
            L.append(f"     flips to X: {c['flip_to_ctx']}")
    # contact and context accuracy
    for f in sorted(os.listdir(out)):
        if f.startswith("runs_") and f.endswith(".csv"):
            rows = list(csv.DictReader(open(os.path.join(out, f))))
            L.append(f"{f}: {len(rows)} runs, contact {sum(int(r['contact']) for r in rows)}, "
                     f"|d0 err| max {max(abs(float(r['d0_real']) - float(r['d0'])) for r in rows):.2f} m, "
                     f"|dv err| max {max(abs(float(r['dv_real']) - float(r['dv'])) for r in rows):.2f} m/s, "
                     f"v_B {st.mean(float(r['vb_real']) for r in rows):.2f}")
    txt = "\n".join(L) + "\n"
    open(os.path.join(out, "c5_arena.txt"), "w").write(txt)
    print(txt)


def selftest():
    js = jobs(12.0, 2)
    assert len(js) == len(D0S) * len(DVS) * 3 * 3 * 2 and len({j["rng_seed"] for j in js}) == len(D0S) * len(DVS) * 2
    # game() on a made-up context: A going is dominated, A drops in every equilibrium
    mk = lambda ua, ub, ahead: [{"uA": ua, "uB": ub, "ahead": ahead, "got_in": 1, "contact": 0}]
    cc = {}
    for a in C1.A_STRATS:
        for b in B_PROFILES:
            cc[(a, b)] = mk(-0.1, -0.01, 0) if a == C1.WAIT else mk(-0.5, -0.2, 1)
    lab, pure, u, ya, yb = game(cc)
    assert lab == "W" and yb and not ya and u == (-0.1, -0.01), (lab, pure, u)
    print("c5_arena selftest ok")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", nargs="?", choices=("run", "analyze"))
    ap.add_argument("--port", type=int, default=2600)
    ap.add_argument("--out")
    ap.add_argument("--e4")
    ap.add_argument("--kin")
    ap.add_argument("--pairs", default="m3:m3,m3:hgv,hgv:m3")
    ap.add_argument("--k", type=int, default=6)
    ap.add_argument("--seeds", type=int, default=8)
    ap.add_argument("--chunk", type=int, default=120)
    ap.add_argument("--limit", type=int, default=0, help="run: at most this many runs per pair (a smoke test)")
    ap.add_argument("--vb", type=float, default=12.0)
    ap.add_argument("--obs-delay", type=float, default=0.19)
    ap.add_argument("--obs-sigma-x", type=float, default=0.3)
    ap.add_argument("--obs-sigma-v", type=float, default=0.2)
    ap.add_argument("--reaction-jitter", type=float, default=0.2)
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    if a.mode == "run":
        run(a)
    else:
        analyze(a.out, a.kin)


if __name__ == "__main__":
    main()
