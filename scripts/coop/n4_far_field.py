#!/usr/bin/env python3
"""Roadmap 074 N4: the far field. A merges into a long stream; the near field composed exactly,
the rest as density and mean speed, against every car explicit, at N = 10, 20, 30, 50.

The scene (a long merge, on arena.py's generated straight two-lane road): A in lane -2 beside
the head of a stream of N - 1 cars in lane -1, B the stream's first car, A d0 ahead of B's front
at dv = v_A - v_B. Behind B the stream's time headways are drawn per draw from a shifted
exponential, h = H_MIN + Exp(h_mean - H_MIN), so the far field is a density (1 / (v_B h_mean +
L) per metre) and a mean speed v_B; draws are common random numbers across N (N = 20's first
nine gaps are N = 10's). A long straight with a finite stream stands in for a ring: arena.py's
generator makes straight roads only (no loop geometry), and a ring of N cars would let A's
shock come back round to its own leader, which a merge does not have. The stream ends at its
last car, so a small N is a finite-size effect a ring would not have; that is what the error
against N measures.

The rules are arena_n's (074 N2, N3), the same code paths generalized to a queue of any length:
A's gap check against the stream's head, dropping back slot by slot; B arena.Pair's rule
against A; every follower cruising, capped by the IDM interaction term toward its predecessor
(and A once A is in its lane); all through arena.Observer (10 Hz, delayed, noisy).

The model (`analyze`):
  near  n3_model's composition for A, B and A's conflict set, the first M_NEAR followers (C, D,
        E: the five-car queue the window cap was tested on): the A-B pair term (a run of A and B
        alone, the same context and draw), the queue walk over the near slots' realized gaps at
        A's speed when it drops, A's delay capped at what the window holds (window_cap)
  far   past the near field the stream is a density and a mean speed: slots arrive with the
        headway distribution, each fits A's gap check with p = P(h >= h_need); A skips k more
        slots with probability (1 - p)^k p, each costing (E[g | g < need] + L) / v_A as in the
        walk and taking (E[g | g < need] + L) / (v_B - v_w) of real time; past T_RUN with no
        slot A never got in (lost = 1, arena_n's rule). U_A = E_k[U(k)].
  comp  the reference: the same composition with every realized gap of the explicit run (no
        mean field), so that composition error and mean-field error separate.

Usage:
  scripts/coop/n4_far_field.py run --ports 2600,2610,2620 --out <dir> [--ns 10,20,30,50] [--draws 16]
  scripts/coop/n4_far_field.py smoke --port 2600 --out <dir>    # tick time per N
  scripts/coop/n4_far_field.py analyze --out <dir>
  scripts/coop/n4_far_field.py --selftest
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import multiprocessing as mp
import os
import random
import statistics as st
import sys
import time
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import cutin_game as C1  # noqa: E402
import measures  # noqa: E402
import n2_composition as N2  # noqa: E402
import n3_model as N3  # noqa: E402

H = C1.HORIZON
L_CAR = N3.L_CAR
VB = 12.0
NS = (10, 20, 30, 50)
N_MAX = max(NS)
M_NEAR = 3  # followers in the near field: C, D, E
H_MIN = 1.0  # s, the shortest drawn headway (IDM's s* at 12 m/s is S0 + v 1 s: no follower brakes at the start)
H_MEANS = (3.0, 4.0)  # s: 1200 and 900 veh/h
D0S = (2.0, 10.0)
DVS = (-2.0, 0.0)
DRAWS = 16
T_RUN = C1.T_MAX
ROAD_LEN = 4600.0  # the longest drawn stream of 49 cars is 3.2 km (h_mean 4 s), then the warm-up and T_RUN
PARK_DX = 8.0


def names(n):
    """A, then the stream front to back: B, then S2 .. S{n-1}."""
    return ["A", "B"] + [f"S{i}" for i in range(2, n)]


def headways(h_mean, draw, n=N_MAX):
    """The stream's time headways behind B (follower i behind car i - 1), i = 1 .. n - 2."""
    rng = random.Random(f"n4|{h_mean}|{draw}")
    return [round(H_MIN + rng.expovariate(1.0 / (h_mean - H_MIN)), 3) for _ in range(n - 2)]


def jobs(ns=NS, draws=DRAWS):
    out = []
    for d0, dv, hm, (pa, pb), draw in itertools.product(D0S, DVS, H_MEANS, N2.PROFILES, range(draws)):
        for roles in ("AB", *(f"N{n}" for n in ns)):
            out.append({"set": "n4", "d0": d0, "dv": dv, "hm": hm, "vb": VB, "pa": pa, "pb": pb,
                        "seed": draw, "roles": roles})
    return out


# --- the arena -----------------------------------------------------------------------------------
def _rng(job, *tag):
    key = "|".join(str(x) for x in ("n4", job["d0"], job["dv"], job["hm"], job["pa"]["name"], job["pb"]["name"],
                                    job["seed"], *tag))
    return random.Random(key)


def setup_cars(ar, k):
    import arena as AR
    actors = []
    for r in range(k):
        d = {}
        for i, nm in enumerate(names(N_MAX)):
            lane = -1 if i % 2 == 0 else -2
            d[nm] = ar.spawn(r, lane, park_x(i))
        actors.append(d)
    ar.put_physics([a for d in actors for a in d.values()])
    return [{nm: AR.Car(ar, a) for nm, a in d.items()} for d in actors]


def park_x(i):
    return ROAD_LEN - 20.0 - PARK_DX * (i // 2)


X_MAX = park_x(N_MAX - 1) - 30.0


class Group:
    def __init__(self, ar, road, cars, job, args):
        import arena as AR
        self.AR = AR
        self.ar, self.road, self.job, self.args = ar, road, job, args
        n = 2 if job["roles"] == "AB" else int(job["roles"][1:])
        self.order = names(n)
        self.queue = self.order[1:]  # the stream, front to back
        self.cars = {r: cars[r] for r in self.order}
        self.absent = {r: c for r, c in cars.items() if r not in self.cars}
        self.idx = {nm: i for i, nm in enumerate(names(N_MAX))}
        self.y = {r: ar.lane_y(road, -2 if r == "A" else -1)[0] for r in self.cars}
        self.y_park = {r: ar.lane_y(road, -1 if self.idx[r] % 2 == 0 else -2)[0] for r in self.absent}
        self.ln = cars["B"].length
        self.hw = headways(job["hm"], job["seed"])
        jit = args.reaction_jitter
        cap = C1.TYPES["car"]
        self.react = {r: cap.reaction + _rng(job, "react", r).uniform(-jit, jit) for r in self.cars}
        self.obs = {}
        pairs = [("A", q) for q in self.queue] + [("B", "A")]
        for i, f in enumerate(self.queue[1:], 1):
            pairs += [(f, self.queue[i - 1]), (f, "A")]
        for v, u in pairs:
            self.obs[(v, u)] = AR.Observer(args.obs_delay, args.obs_sigma_x, args.obs_sigma_v, _rng(job, "obs", v, u))
        self.phase = "warmup"
        self.rec = None

    def offsets(self, t):
        j, W = self.job, self.AR.WARMUP
        rA = j["d0"] - j["dv"] * max(0.0, W - t)
        off = {"B": (0.0, 0.0), "A": (self.ln + rA, j["dv"] if t < W else 0.0)}
        x = 0.0
        for i, q in enumerate(self.queue[1:], 1):
            x -= self.ln + C1.S0 + j["vb"] * self.hw[i - 1]
            off[q] = (x, 0.0)
        return off

    def place(self):
        import carla
        off = self.offsets(0.0)
        x0 = 30.0 - min(o[0] for o in off.values()) + 5.0
        for r, c in self.cars.items():
            loc = c.a.get_transform().location
            c.park(carla.Transform(carla.Location(x=x0 + off[r][0], y=self.y[r], z=loc.z + 0.3), carla.Rotation(yaw=0)))
        for r, c in self.absent.items():
            loc = c.a.get_transform().location
            c.park(carla.Transform(carla.Location(x=park_x(self.idx[r]), y=self.y_park[r], z=loc.z + 0.3),
                                   carla.Rotation(yaw=0)))
        self.xg, self.vg = x0, 0.0
        self.tw = self.ar.t

    def hold_absent(self):
        import carla
        for c in self.absent.values():
            self.ar.pending.append(carla.command.ApplyVehicleControl(c.a.id, carla.VehicleControl(brake=1.0, hand_brake=True)))

    def step_warmup(self):
        t = self.ar.t - self.tw
        vb = self.job["vb"]
        ag = max(-3.0, min(3.0, 1.5 * (vb - self.vg)))
        off = self.offsets(t)
        for r, c in self.cars.items():
            x, _, _, v = c.state()
            o, orate = off[r]
            a = ag + 0.6 * (self.xg + o - x) + 1.8 * (self.vg + orate - v)
            c.drive(max(-4.0, min(3.0, a)), self.y[r])
        self.hold_absent()
        self.xg += self.vg * self.AR.DT + 0.5 * ag * self.AR.DT ** 2
        self.vg += ag * self.AR.DT
        if t >= self.AR.WARMUP:
            self.start()

    def start(self):
        self.phase = "run"
        self.t0 = self.ar.t
        st0 = {r: c.state() for r, c in self.cars.items()}
        self.v0 = {r: s[3] for r, s in st0.items()}
        self.x0 = {r: s[0] for r, s in st0.items()}
        cs = self.cars
        self.ctx_real = {"d0_real": round((st0["A"][0] - cs["A"].length / 2) - (st0["B"][0] + cs["B"].length / 2), 2),
                         "dv_real": round(st0["A"][3] - st0["B"][3], 2)}
        self.pa, self.pb = self.job["pa"], self.job["pb"]
        self.decide = None
        self.dropping = False
        self.slot = self.queue[0]
        self.dec_state = None
        self.b_latched = False
        self.lat_seen = self.in_lane_at = self.b_brake_seen = None
        self.lead_a_at = {}
        self.contact = {r: False for r in cs}
        self.peak = {r: 0.0 for r in cs}
        self.vhist = {r: deque() for r in cs}
        self.s_h = {}
        self.prev = None
        mp_ = [("A", q) for q in self.queue] + [(self.queue[i - 1], f) for i, f in enumerate(self.queue[1:], 1)]
        self.meas = {p: measures.Measures(b_max=(C1.TYPES["car"].b_max,) * 2) for p in mp_}
        self.yA0, self.yT = self.y["A"], self.y["B"]

    def lat(self):
        y = self.cars["A"].state()[1]
        return max(0.0, min(1.0, (self.yA0 - y) / (self.yA0 - self.yT)))

    def step_run(self):
        AR = self.AR
        t = self.ar.t - self.t0
        cs = self.cars
        cap = C1.TYPES["car"]
        st_ = {r: c.state() for r, c in cs.items()}
        lat = self.lat()
        S = {}
        for key, ob in self.obs.items():
            u = key[1]
            x, _, _, spd = st_[u]
            s = ob.push(t, x, lat if u == "A" else 0.0, spd)
            S[key] = (s[1], s[2], s[3], s[4]) if s is not None else (x, lat if u == "A" else 0.0, spd, 0.0)
        acc = {}
        y_t = {r: self.y[r] for r in cs}
        # --- A (arena_n's rule, the queue of any length)
        A = cs["A"]
        xa, _, _, va = st_["A"]
        la = A.length
        pa = self.pa
        q0 = self.queue[0]
        if self.decide is None and not self.dropping:
            xt, _, vt, at = S[("A", q0)]
            gap_ahead = (xa - la / 2) - (xt + cs[q0].length / 2)
            need = pa["buffer"] + vt * pa["tau"] + max(0.0, vt ** 2 - va ** 2) / (2 * pa["a_rear"])
            if at <= -C1.YIELD_SEEN and self.b_brake_seen is None:
                self.b_brake_seen = t
            if gap_ahead >= need:
                self.decide = t
            elif (pa["use_yield"] and self.b_brake_seen is not None and t >= self.b_brake_seen + self.react["A"]
                  and gap_ahead > -la / 2):
                self.decide = t
            elif t >= pa["patience"] + (C1.WATCH if pa["use_yield"] else 0.0):
                self.dropping = True
        if self.dropping and self.decide is None:
            def slot_gaps(k):
                q = self.queue[k]
                xs, _, vs, _ = S[("A", q)]
                gb = (xs - cs[q].length / 2) - (xa + la / 2)
                short = False
                if k + 1 < len(self.queue):
                    nx = self.queue[k + 1]
                    xn, _, vn, _ = S[("A", nx)]
                    gn = (xa - la / 2) - (xn + cs[nx].length / 2)
                    short = gn < pa["buffer"] + vn * pa["tau"] + max(0.0, vn ** 2 - va ** 2) / (2 * pa["a_rear"])
                return gb, C1.S0 + vs * C1.HEADWAY, short, vs
            k = self.queue.index(self.slot)
            gap_behind, want, rear_short, vs = slot_gaps(k)
            while gap_behind >= want and rear_short:
                k += 1
                self.slot = self.queue[k]
                gap_behind, want, rear_short, vs = slot_gaps(k)
            if gap_behind >= want and not rear_short:
                self.decide = t
            aa = -C1.A_WAIT_DECEL if (gap_behind < want or rear_short) and va > 0.5 * vs else 0.0
            if self.decide is not None:
                aa = AR.follow(gap_behind, va, vs, cap, self.v0["A"])
        else:
            ahead = [(S[("A", r)][0], r) for r in self.queue if S[("A", r)][0] > xa]
            if lat > 0.05 and ahead:
                xs, r = min(ahead)
                vs = S[("A", r)][2]
                aa = AR.follow((xs - cs[r].length / 2) - (xa + la / 2), va, vs, cap, self.v0["A"])
            else:
                aa = max(-1.0, min(cap.a_max, 1.5 * (self.v0["A"] - va)))
        if self.decide is not None and self.dec_state is None:
            self.dec_state = {"dec_t": round(t, 2), "dec_vA": round(va, 3),
                              "dec_slot": self.slot if self.dropping else "ahead"}
        if self.decide is not None and t >= self.decide + pa["t_prep"]:
            s = min(1.0, (t - self.decide - pa["t_prep"]) / cap.t_lc)
            y_t["A"] = self.yA0 + (self.yT - self.yA0) * 0.5 * (1 - math.cos(math.pi * s))
        acc["A"] = aa
        # --- B
        xb, _, _, vb = st_["B"]
        lb = cs["B"].length
        ab = max(-1.0, min(cap.a_max, 1.5 * (self.v0["B"] - vb)))
        xa_s, lat_s, va_s, _ = S[("B", "A")]
        if lat_s > 0.08 and self.lat_seen is None:
            self.lat_seen = t
        if lat_s >= C1.LANE_LINE and xa_s > xb and self.in_lane_at is None:
            self.in_lane_at = t
        gap_b = (xa_s - la / 2) - (xb + lb / 2)
        a_ahead = xa_s > xb - lb
        pb = self.pb
        pre = t >= self.react["B"] and a_ahead and lat_s < C1.LANE_LINE and gap_b < pb["pre_band"]
        react = (self.lat_seen is not None and t >= self.lat_seen + self.react["B"] and a_ahead
                 and gap_b < pb["react_gap"])
        self.b_latched = self.b_latched or ((pre or react) and a_ahead)
        if self.b_latched and a_ahead:
            ab = -pb["b_y"] if gap_b < C1.S0 + vb * C1.HEADWAY else AR.follow(gap_b, vb, va_s, cap, self.v0["B"])
        if self.in_lane_at is not None and t >= self.in_lane_at + self.react["B"] and xa_s > xb:
            ab = min(ab, AR.follow(gap_b, vb, va_s, cap, self.v0["B"]))
        acc["B"] = ab
        # --- the followers: the predecessor (the nearest car ahead in a single lane) and A
        for i, f in enumerate(self.queue[1:], 1):
            xc, _, _, vc = st_[f]
            lc = cs[f].length
            ac = max(-1.0, min(cap.a_max, 1.5 * (self.v0[f] - vc)))
            q = self.queue[i - 1]
            xq_s, _, vq_s, _ = S[(f, q)]
            leads = [((xq_s - cs[q].length / 2) - (xc + lc / 2), vq_s)]
            xa_s, lat_s, va_s, _ = S[(f, "A")]
            if lat_s >= C1.LANE_LINE and xa_s > xc and f not in self.lead_a_at:
                self.lead_a_at[f] = t
            if f in self.lead_a_at and t >= self.lead_a_at[f] + self.react[f] and xa_s > xc:
                leads.append(((xa_s - la / 2) - (xc + lc / 2), va_s))
            leads = [g for g in leads if g[0] > -lc]
            if leads:
                g, vl = min(leads)
                ac = min(ac, C1.follow_accel(g, vc, vl, cap, C1.V_FREE))
            acc[f] = ac
        for r, c in cs.items():
            c.drive(acc[r], y_t[r])
        self.hold_absent()
        # --- truth
        for r in cs:
            h = self.vhist[r]
            h.append((t, st_[r][3]))
            while h and t - h[0][0] > AR.PEAK_WINDOW + 1e-9:
                h.popleft()
            if t - h[0][0] >= AR.PEAK_WINDOW - 1e-9:
                self.peak[r] = max(self.peak[r], (h[0][1] - st_[r][3]) / AR.PEAK_WINDOW)
        if self.prev is not None and not self.s_h and t >= H - 1e-9:  # progress along +x, at H exactly
            tp, xp = self.prev
            f = max(0.0, min(1.0, (H - tp) / (t - tp)))
            self.s_h = {r: xp[r] + f * (st_[r][0] - xp[r]) - self.x0[r] for r in cs}
        self.prev = (t, {r: st_[r][0] for r in cs})
        poses = {r: c.pose() for r, c in cs.items()}
        for p, m in self.meas.items():
            if p[0] == "A" and abs(poses["A"][0] - poses[p[1]][0]) > 80.0 and m._prev_pose is not None:
                m._prev_pose = (t, m._prev_pose[1], m._prev_pose[2])  # far apart: no risk to measure
                continue
            m.push_poses(t, poses[p[0]], poses[p[1]])
            if m.clear <= 0.0:
                self.contact[p[0]] = self.contact[p[1]] = True
        if max(st_[r][0] for r in cs) > X_MAX:
            raise RuntimeError(f"road {self.road}: a car passed x {X_MAX} m, near the parked cars")
        if any(self.contact.values()) or t >= T_RUN - 1e-9:
            self.finish(t, st_, lat)

    def finish(self, t, st_, lat):
        cs = self.cars
        cap = C1.TYPES["car"]
        rho = {}
        for p, m in self.meas.items():
            m.pet = m._pet()
            rho[p] = measures.rho(m.ttc2d, m.thw, m.pet, m.clear)
        latp = {r: 0.0 for r in cs}
        for p, m in self.meas.items():
            for k, r in enumerate(p):
                latp[r] = max(latp[r], m.lat_peak[k])
        got_in = lat >= 0.9
        rec = {**{k: v for k, v in self.job.items() if k not in ("pa", "pb")}, "a": self.pa["name"], "b": self.pb["name"],
               "road": self.road, **self.ctx_real, "t_run": round(t, 2), "n": len(cs)}
        lost_s = []
        for r in self.order:
            lost = measures.lost_progress(self.v0[r], self.s_h.get(r), H)
            if self.contact[r] or (r == "A" and not got_in):
                lost = 1.0
            risk = max([rho[p] for p in rho if r in p] or [0.0])
            u = measures.utility(lost, risk, self.peak[r], cap.b_max, latp[r], self.contact[r])
            if r in ("A", "B"):
                rec.update({f"u{r}": round(u, 4), f"lost{r}": round(lost, 4), f"rho{r}": round(risk, 4),
                            f"peak{r}": round(self.peak[r], 3), f"lat{r}": round(latp[r], 3),
                            f"contact{r}": int(self.contact[r]), f"v0{r}": round(self.v0[r], 3)})
            if r != "A":
                lost_s.append(round(lost, 4))
        rec["lost_stream"] = lost_s  # B first
        rec["rho_AB"] = round(rho[("A", "B")], 4)
        xa = st_["A"][0]
        rec["got_in"] = int(got_in)
        rec["decide"] = None if self.decide is None else round(self.decide, 2)
        rec["dropped"] = int(self.dropping)
        rec.update(self.dec_state or {})
        n_ahead = sum(1 for r in self.queue if st_[r][0] > xa) if got_in else None
        rec["a_pos"] = "none" if not got_in else ("ahead" if not n_ahead else "behind")
        rec["a_slot"] = n_ahead
        self.rec = rec
        self.phase = "done"


def run_groups(ar, cars, work, args, log=None):
    rows = []
    queue = deque(work)
    import carla
    t_run = time.monotonic()
    while queue:
        groups = []
        for r in range(len(cars)):
            if not queue:
                break
            g = Group(ar, r, cars[r], queue.popleft(), args)
            g.place()
            groups.append(g)
        for _ in range(3):
            for g in groups:
                for c in list(g.cars.values()) + list(g.absent.values()):
                    c.a.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
            ar.tick()
        for g in groups:
            g.tw = ar.t
        while any(g.phase != "done" for g in groups):
            for g in groups:
                if g.phase == "warmup":
                    g.step_warmup()
                elif g.phase == "run":
                    g.step_run()
                else:
                    for r, c in g.cars.items():
                        c.drive(-2.0, c.state()[1] if r == "A" else g.y[r])
                    g.hold_absent()
            ar.tick()
        rows += [g.rec for g in groups]
        if log:
            log(f"  {len(rows)}/{len(work)} runs, {time.monotonic() - t_run:.0f} s")
    return rows


def worker(port, k, inbox, outbox):
    import arena as AR
    AR.ROAD_LEN = ROAD_LEN
    ar = AR.Arena(port, k, AR.xodr(k, ROAD_LEN))
    try:
        cars = setup_cars(ar, k)
        while True:
            msg = inbox.get()
            if msg is None:
                break
            tag, work = msg
            try:
                outbox.put((tag, run_groups(ar, cars, work, N2.Args())))
            except RuntimeError as e:  # one batch refused (a car near the parked ones): report it, keep the pool
                print(f"  chunk {tag} failed: {e}", flush=True)
                outbox.put((tag, []))
    finally:
        ar.close()


def run_pool(ports, k, work, out_path, chunk):
    """n2_composition.run_pool with this scene's worker; chunks of one size class run together."""
    inbox, outbox = mp.Queue(), mp.Queue()
    procs = [mp.Process(target=worker, args=(p, k, inbox, outbox), daemon=True) for p in ports]
    for p in procs:
        p.start()
    work = sorted(work, key=lambda j: (j["roles"] != "AB", int(j["roles"][1:]) if j["roles"] != "AB" else 2))
    chunks = [work[i:i + chunk] for i in range(0, len(work), chunk)]
    for i, c in enumerate(chunks):
        inbox.put((i, c))
    for _ in procs:
        inbox.put(None)
    t0 = time.monotonic()
    n = 0
    with open(out_path, "a") as f:
        for _ in chunks:
            _, rows = outbox.get()
            for r in rows:
                f.write(json.dumps(r) + "\n")
            f.flush()
            n += len(rows)
            print(f"  {n}/{len(work)} runs, {time.monotonic() - t0:.0f} s", flush=True)
    for p in procs:
        p.join(timeout=120)


def smoke(port, out, k=6):
    """Wall time of one batch of K groups per N (one CARLA)."""
    import arena as AR
    AR.ROAD_LEN = ROAD_LEN
    ar = AR.Arena(port, k, AR.xodr(k, ROAD_LEN))
    res = {}
    try:
        cars = setup_cars(ar, k)
        base = jobs(ns=NS, draws=1)
        for roles in ("AB", *(f"N{n}" for n in NS)):
            work = [j for j in base if j["roles"] == roles][:k]
            t0 = time.monotonic()
            rows = run_groups(ar, cars, work, N2.Args())
            res[roles] = {"wall_s": round(time.monotonic() - t0, 1), "groups": len(rows),
                          "uA": [r["uA"] for r in rows], "a_slot": [r["a_slot"] for r in rows]}
            print(roles, res[roles], flush=True)
    finally:
        ar.close()
    json.dump(res, open(os.path.join(out, "smoke.json"), "w"), indent=1)


# --- the model ----------------------------------------------------------------------------------
def a_need(ab, pa, vb):
    """A's gap check for a slot, at its speed when it drops (n3_model.queue_walk, SLOT_AT_DECISION)."""
    vd = ab.get("dec_vA") or ab["v0A"]
    return (C1.S0 + vd * C1.HEADWAY + L_CAR + pa["buffer"] + vb * pa["tau"]
            + max(0.0, vb ** 2 - vd ** 2) / (2 * pa["a_rear"]))


def trunc_exp_mean(lo, hi, mean_excess):
    """E[h | lo <= h < hi] for h = lo + Exp(mean_excess)."""
    lam = 1.0 / mean_excess
    w = hi - lo
    if w <= 0:
        return lo
    return lo + 1.0 / lam - w * math.exp(-lam * w) / (1.0 - math.exp(-lam * w))


def far_params(hm, need, vb):
    """The far field as density and mean speed: p, a slot's chance to fit; g_short, the mean gap of a
    slot that does not (m)."""
    h_need = (need - C1.S0) / vb
    if h_need <= H_MIN:
        return 1.0, C1.S0 + vb * H_MIN
    p = math.exp(-(h_need - H_MIN) / (hm - H_MIN))
    return p, C1.S0 + vb * trunc_exp_mean(H_MIN, h_need, hm - H_MIN)


def _u_a(ab, pa, extra, t_fit):
    """A's composed utility: the A-B pair term with the walk's extra delay, the window cap; no slot
    by T_RUN (minus the lane change) is arena_n's lost = 1."""
    parts = {"lost": ab["lostA"], "rho": ab["rhoA"], "peak": ab["peakA"], "lat": ab["latA"], "contact": ab["contactA"]}
    if t_fit is None or t_fit + pa["t_prep"] + C1.TYPES["car"].t_lc > T_RUN:
        return N3.with_lost(parts, 1.0)[0]
    lost = min(ab["lostA"] + extra / H, N3.window_cap(ab, pa))
    return N3.with_lost(parts, lost)[0]


def walk(ab, pa, vb, gaps, n_slots):
    """The walk over realized gaps (m) of slots 1..: returns (extra s, t_fit s or None, slots
    skipped, done). Past the last car (n_slots) the road behind is free: A fits there."""
    need = a_need(ab, pa, vb)
    v = max(ab["v0A"], 0.1)
    vw = min(v, ab.get("dec_vA") or v)
    t = ab.get("dec_t") or 0.0
    extra = 0.0
    for j, g in enumerate(gaps[:n_slots]):
        if g >= need:
            return extra, t, j, True
        extra += (g + L_CAR) / v
        t += (g + L_CAR) / max(vb - vw, 0.5)
    if len(gaps) >= n_slots:
        return extra, t, n_slots, True  # behind the stream's last car
    return extra, t, len(gaps), False


def predict(ab, pa, vb, hw, n, hm, m_near=M_NEAR, mean_field=True):
    """U_A predicted. ab: the A-B pair's row. hw: the realized headways behind B. n: cars in the
    explicit run (A + n - 1 in the stream: n - 2 slots behind B). mean_field: the near field's
    m_near slots realized, the rest as the distribution (an infinite stream); else every realized
    slot of the n-car stream (the composition reference)."""
    if ab["a_pos"] == "ahead":
        return ab["uA"]  # A cuts in ahead of B: it meets only B
    gaps = [C1.S0 + vb * h for h in hw]
    if not mean_field:
        extra, t_fit, _, _ = walk(ab, pa, vb, gaps, n - 2)
        return _u_a(ab, pa, extra, t_fit)
    near = gaps[:m_near]
    extra, t_fit, _, done = walk(ab, pa, vb, near, 10 ** 6)
    if done:
        return _u_a(ab, pa, extra, t_fit)
    need = a_need(ab, pa, vb)
    p, g_short = far_params(hm, need, vb)
    v = max(ab["v0A"], 0.1)
    vw = min(v, ab.get("dec_vA") or v)
    c, tau = (g_short + L_CAR) / v, (g_short + L_CAR) / max(vb - vw, 0.5)
    u, mass, k = 0.0, 0.0, 0
    while mass < 1.0 - 1e-6 and k < 10 ** 4:
        pk = (1.0 - p) ** k * p
        tk = t_fit + k * tau
        u += pk * _u_a(ab, pa, extra + k * c, tk)
        mass += pk
        if tk + pa["t_prep"] + C1.TYPES["car"].t_lc > T_RUN:  # every later k: never in
            u += (1.0 - mass) * _u_a(ab, pa, 0.0, None)
            mass = 1.0
        k += 1
    return u


# --- analysis -----------------------------------------------------------------------------------
def load(out):
    p = os.path.join(out, "n4.jsonl")
    return [json.loads(l) for l in open(p) if l.strip()] if os.path.exists(p) else []


def boot_ci(xs, n=2000, seed=7):
    return N2.boot_ci(xs, n, seed) if len(xs) > 1 else (xs[0], xs[0])


def externality(lost_stream):
    """Followers' lost progress beyond the undisturbed level (the median follower's)."""
    f = lost_stream[1:]
    if not f:
        return 0.0
    base = st.median(f)
    return sum(max(0.0, x - base) for x in f)


def analyze(out):
    rows = load(out)
    by = {}
    for r in rows:
        by.setdefault((r["d0"], r["dv"], r["hm"], r["a"], r["seed"]), {})[r["roles"]] = r
    ns = sorted({int(r["roles"][1:]) for r in rows if r["roles"] != "AB"})
    cell = {}  # (d0, dv, hm, a, n) -> [(u, u_mf, u_comp, u_near)]
    for (d0, dv, hm, a, seed), g in by.items():
        if "AB" not in g:
            continue
        ab, pa = g["AB"], N2.PA[a]
        hw = headways(hm, seed)
        for n in ns:
            r = g.get(f"N{n}")
            if r is None:
                continue
            cell.setdefault((d0, dv, hm, a, n), []).append(
                {"u": r["uA"], "mf": predict(ab, pa, VB, hw, n, hm), "comp": predict(ab, pa, VB, hw, n, hm, mean_field=False),
                 "slot": r["a_slot"], "got_in": r["got_in"], "ext": externality(r["lost_stream"]), "ab": ab["uA"],
                 "pos_ab": ab["a_pos"]})
    L = [f"N4: {len(rows)} runs, {len(by)} (context, profile, draw) groups, N = {ns}; M_NEAR = {M_NEAR}, "
         f"h_min {H_MIN} s, h_mean {H_MEANS} s, v_B {VB} m/s", ""]
    L.append("per N, over the cells (context x profile; 16 draws each): the paired error e = U_A explicit - predicted per draw;")
    L.append("  'tol': cells with |mean e| <= max(0.02, the 95 % bootstrap half-width of mean e) (D2's tolerance, the noise")
    L.append("  being the draws'); '0.02': cells with |mean e| <= 0.02 (D2's floor alone); mean |cell e|; bias (mean e)")
    summ = {}
    for n in ns:
        for model in ("mf", "comp"):
            ok, tot, errs, strict = 0, 0, [], 0
            for key, xs in cell.items():
                if key[4] != n:
                    continue
                es = [x["u"] - x[model] for x in xs]
                lo, hi = boot_ci(es)
                e = st.mean(es)
                errs.append(e)
                ok += abs(e) <= max(0.02, (hi - lo) / 2)
                strict += abs(e) <= 0.02
                tot += 1
            summ[(n, model)] = (ok, tot, st.mean(abs(e) for e in errs), st.mean(errs), strict)
    L.append(f"  {'N':>3s} | {'mean field (near exact + density)':36s} | {'composition, every gap realized':36s}")
    for n in ns:
        a_, b_ = summ[(n, "mf")], summ[(n, "comp")]
        L.append(f"  {n:3d} | tol {a_[0]:2d}/{a_[1]:<2d} 0.02 {a_[4]:2d}/{a_[1]:<2d} |e| {a_[2]:.3f} bias {a_[3]:+.3f} "
                 f"| tol {b_[0]:2d}/{b_[1]:<2d} 0.02 {b_[4]:2d}/{b_[1]:<2d} |e| {b_[2]:.3f} bias {b_[3]:+.3f}")
    L.append("")
    L.append("explicit change with N, U_A(N) - U_A(N_max) per run, paired by draw (how much of the far field A feels),")
    L.append("  mean |.| / max |.| / runs differing by more than 0.02:")
    nmax = max(ns)
    for n in ns:
        ds = [g[f"N{n}"]["uA"] - g[f"N{nmax}"]["uA"] for g in by.values() if f"N{n}" in g and f"N{nmax}" in g]
        if ds:
            L.append(f"  N {n:3d}: {st.mean(abs(d) for d in ds):.3f} / {max(abs(d) for d in ds):.3f} / "
                     f"{sum(abs(d) > 0.02 for d in ds)} of {len(ds)}")
    L.append("")
    L.append("A's externality on the stream: sum over the followers behind B of lost progress above the stream's median"
             " (the undisturbed followers' own loss), per run:")
    for n in ns:
        xs = [x["ext"] for key, v in cell.items() if key[4] == n for x in v]
        L.append(f"  N {n:3d}: mean {st.mean(xs):.3f}, max {max(xs):.3f}")
    L.append("")
    L.append("cells (d0 dv h_mean profile N: U explicit [CI] | mean field | composition | A's slot mean | got in):")
    for key in sorted(cell):
        xs = cell[key]
        us = [x["u"] for x in xs]
        lo, hi = boot_ci(us)
        slots = [x["slot"] for x in xs if x["slot"] is not None]
        L.append(f"  {key[0]:4.0f} {key[1]:+3.0f} {key[2]:3.1f} {key[3]:11s} {key[4]:3d}: {st.mean(us):+.3f} [{lo:+.3f} {hi:+.3f}] | "
                 f"{st.mean(x['mf'] for x in xs):+.3f} | {st.mean(x['comp'] for x in xs):+.3f} | "
                 f"{st.mean(slots) if slots else float('nan'):.1f} | {sum(x['got_in'] for x in xs)}/{len(xs)}")
    txt = "\n".join(L)
    open(os.path.join(out, "n4_summary.txt"), "w").write(txt + "\n")
    with open(os.path.join(out, "n4_cells.jsonl"), "w") as f:
        for key, xs in sorted(cell.items()):
            f.write(json.dumps({"key": key, "runs": xs}) + "\n")
    print(txt)


def selftest():
    # draws are common random numbers across N, and the shifted exponential's mean is h_mean
    assert headways(3.0, 1)[:9] == headways(3.0, 1, n=11)
    hs = [h for d in range(200) for h in headways(3.0, d)]
    assert abs(st.mean(hs) - 3.0) < 0.1 and min(hs) >= H_MIN
    # truncated exponential mean, against sampling
    rng = random.Random(3)
    xs = [x for x in (1.0 + rng.expovariate(0.5) for _ in range(200000)) if x < 4.0]
    assert abs(trunc_exp_mean(1.0, 4.0, 2.0) - st.mean(xs)) < 0.01
    # the walk: a fitting second slot, then the stream's end
    pa = N2.PA["A_aw_perm"]
    ab = {"a_pos": "behind", "v0A": 12.0, "v0B": 12.0, "dec_vA": 5.0, "dec_t": 10.0, "uA": -0.2, "lostA": 0.15,
          "rhoA": 0.0, "peakA": 1.0, "latA": 1.0, "contactA": 0}
    need = a_need(ab, pa, VB)
    extra, t_fit, k, done = walk(ab, pa, VB, [20.0, need + 1.0, 20.0], 3)
    assert k == 1 and done and abs(extra - (20.0 + L_CAR) / 12.0) < 1e-9 and abs(t_fit - 10.0 - (20.0 + L_CAR) / 7.0) < 1e-9
    assert walk(ab, pa, VB, [20.0, 20.0], 2)[2:] == (2, True)  # behind the last car
    # the mean field: p = 1 is the near walk; a sparse far field costs less than a dense one
    hw = [1.0] * 48
    u_dense = predict(ab, pa, VB, hw, 50, 3.0)
    u_sparse = predict(ab, pa, VB, hw, 50, 8.0)
    assert u_sparse > u_dense, (u_sparse, u_dense)
    # the reference with every slot short: A is in only behind the last car, past T_RUN at N = 50
    assert predict(ab, pa, VB, hw, 50, 3.0, mean_field=False) <= -1.0
    assert predict(ab, pa, VB, hw, 5, 3.0, mean_field=False) > -1.0
    print("n4_far_field selftest ok")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", nargs="?", choices=("run", "smoke", "analyze"))
    ap.add_argument("--ports", default="2600,2610,2620")
    ap.add_argument("--port", type=int, default=2600)
    ap.add_argument("--out")
    ap.add_argument("--k", type=int, default=6)
    ap.add_argument("--ns", default=",".join(str(n) for n in NS))
    ap.add_argument("--draws", type=int, default=DRAWS)
    ap.add_argument("--resume", action="store_true", help="run: only the jobs not yet in <out>/n4.jsonl")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    os.makedirs(a.out, exist_ok=True)
    if a.mode == "run":
        ns = tuple(int(n) for n in a.ns.split(","))
        work = jobs(ns, a.draws)
        if a.resume:
            have = {(r["d0"], r["dv"], r["hm"], r["a"], r["seed"], r["roles"]) for r in load(a.out)}
            work = [j for j in work if (j["d0"], j["dv"], j["hm"], j["pa"]["name"], j["seed"], j["roles"]) not in have]
        print(f"[n4] {len(work)} group runs on {a.ports}", flush=True)
        run_pool([int(p) for p in a.ports.split(",")], a.k, work, os.path.join(a.out, "n4.jsonl"), chunk=a.k)
    elif a.mode == "smoke":
        smoke(a.port, a.out, a.k)
    else:
        analyze(a.out)


if __name__ == "__main__":
    mp.set_start_method("spawn")
    main()
