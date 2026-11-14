#!/usr/bin/env python3
"""Roadmap 074 N1: how many cars are in each car's conflict set, per time slot.

A car j is in car i's conflict set K_i during a slot (SLOT s) when, at some sample in it, by
measures' definitions and references (docs/design/cooperative-driving.md ①):
  - the 2D TTC of i and j is below TAU_REF (3 s), or
  - one follows the other at a time headway below H_REF (1 s), or
  - their paths cross (headings 30-150 deg apart, footprints overlapping) with a post-encroachment
    time below PET_REF (1.5 s); the pair is in conflict in every slot between the first car's
    and the second car's time at the spot.
An interaction event is a connected component (>= 2 cars) of the slot's conflict graph; its size
is what the literature reports (INTERACTION: 3.2 vehicles on average, 7 at most).

Sources:
  arena   N2's three-car groups (arena_n traces, groups.jsonl): scenes a and b
  ssv2    SSv2 scenes whose motion the xosc fixes without an ego: the NPC-only Town01 benches
          (scenarios/bench/town01_npc_{10,20,50}.xosc, scenarios/gap/town01_npc_shoulder.xosc).
          Each NPC starts at its LanePosition at 8 m/s (step) and follows its lanelets on
          Town01's lanelet2 map (projected as SSv2 projects it, gen_npc_benchmark.py), taking
          the `straight` successor at a fork, else the straightest; it keeps 8 m/s, capped by
          the IDM interaction term toward a car ahead on its path (SSv2 stops behind a front
          entity). Traffic lights are not modelled (the benches set none).

Usage:
  scripts/coop/n1_conflict_sets.py --groups <n2 dir>/groups.jsonl --out <dir>
"""

from __future__ import annotations

import argparse
import collections
import json
import math
import os
import re
import statistics as st
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "scripts"))
import cutin_game as C1  # noqa: E402
import measures as M  # noqa: E402

SLOT = 1.0  # s
DT = 0.1  # s, the sample step of every trajectory
LEN, WID = 4.79, 2.163  # the Model 3's box (sim_vehicle_spec), every car here
SSV2_SCENES = ["scenarios/bench/town01_npc_10.xosc", "scenarios/bench/town01_npc_20.xosc",
               "scenarios/bench/town01_npc_50.xosc", "scenarios/gap/town01_npc_shoulder.xosc"]
MAP = ROOT / "data/carla-autoware-bridge/Town01/lanelet2_map.osm"


# --- conflict sets ---------------------------------------------------------------------------------
def conflicts(trajs):
    """trajs: {car: {k: (x, y, yaw, vx, vy, L, W)}} on the common grid t = k DT.

    Returns {slot: set of frozenset({i, j})} and the pair-slots per criterion (the first that
    held: headway, then TTC, then PET)."""
    ks = sorted({k for tr in trajs.values() for k in tr})
    cars = sorted(trajs)
    out = collections.defaultdict(set)
    why = collections.Counter()
    slot_of = lambda k: int(k * DT / SLOT + 1e-9)
    # TTC and headway, sample by sample
    for k in ks:
        here = [(c, trajs[c][k]) for c in cars if k in trajs[c]]
        for a in range(len(here)):
            ci, pi = here[a]
            for b in range(a + 1, len(here)):
                cj, pj = here[b]
                pair = frozenset((ci, cj))
                if pair in out[slot_of(k)]:
                    continue
                d = math.hypot(pi[0] - pj[0], pi[1] - pj[1])
                reach = math.hypot(pi[5], pi[6]) / 2 + math.hypot(pj[5], pj[6]) / 2
                rv = math.hypot(pi[3] - pj[3], pi[4] - pj[4])
                vmax = max(math.hypot(pi[3], pi[4]), math.hypot(pj[3], pj[4]))
                if d - reach > max(rv * M.TAU_REF, vmax * M.H_REF) + 1.0:
                    continue
                hit = None
                h1, h2 = M.headway(pi, pj), M.headway(pj, pi)
                if (h1 is not None and h1 < M.H_REF) or (h2 is not None and h2 < M.H_REF):
                    hit = "thw"
                elif M.ttc2d(pi, pj) < M.TAU_REF:
                    hit = "ttc"
                if hit:
                    out[slot_of(k)].add(pair)
                    why[hit] += 1
    # PET: footprints on a grid, pairs of different cars at the same spot within PET_REF
    cell = 5.0
    grid = collections.defaultdict(list)
    for c in cars:
        for k, p in trajs[c].items():
            grid[(int(p[0] // cell), int(p[1] // cell))].append((c, k, p))
    seen = {}
    for (gx, gy), items in grid.items():
        near = [it for dx in (-1, 0, 1) for dy in (-1, 0, 1) for it in grid.get((gx + dx, gy + dy), ())]
        for ci, ka, pa in items:
            for cj, kb, pb in near:
                if cj <= ci or abs(ka - kb) * DT >= M.PET_REF:
                    continue
                if abs(math.cos(pa[2] - pb[2])) > math.cos(math.radians(30)):
                    continue
                if math.hypot(pa[0] - pb[0], pa[1] - pb[1]) > (math.hypot(pa[5], pa[6]) + math.hypot(pb[5], pb[6])) / 2:
                    continue
                key = (ci, cj, ka, kb)
                if key in seen:
                    continue
                seen[key] = True
                if M.overlap(M.corners(*pa[:3], *pa[5:]), M.corners(*pb[:3], *pb[5:])):
                    for s in range(slot_of(min(ka, kb)), slot_of(max(ka, kb)) + 1):
                        if frozenset((ci, cj)) not in out[s]:
                            out[s].add(frozenset((ci, cj)))
                            why["pet"] += 1
    return out, why


def sizes(trajs, conf):
    """Per car-slot |K_i| (cars present in the slot) and per slot the event sizes (components)."""
    present = collections.defaultdict(set)
    for c, tr in trajs.items():
        for k in tr:
            present[int(k * DT / SLOT + 1e-9)].add(c)
    k_sizes, events = [], []
    for s, cars in sorted(present.items()):
        adj = collections.defaultdict(set)
        for pr in conf.get(s, ()):
            i, j = tuple(pr)
            adj[i].add(j)
            adj[j].add(i)
        for c in cars:
            k_sizes.append(len(adj[c]))
        done = set()
        for c in cars:
            if c in done or not adj[c]:
                continue
            comp, todo = set(), [c]
            while todo:
                x = todo.pop()
                if x in comp:
                    continue
                comp.add(x)
                todo.extend(adj[x] - comp)
            done |= comp
            events.append(len(comp))
    return k_sizes, events


def describe(name, k_sizes, events, why=None):
    hist = collections.Counter(k_sizes)
    hk = {n: hist[n] for n in sorted(hist)}
    nz = [k for k in k_sizes if k > 0]
    ev = collections.Counter(events)
    return {"source": name, "car_slots": len(k_sizes), "K_hist": hk,
            "K_mean": round(st.mean(k_sizes), 3) if k_sizes else None,
            "K_mean_given_any": round(st.mean(nz), 3) if nz else None, "K_max": max(k_sizes) if k_sizes else None,
            "share_in_conflict": round(len(nz) / max(1, len(k_sizes)), 3),
            "events": len(events), "event_hist": {n: ev[n] for n in sorted(ev)},
            "event_mean": round(st.mean(events), 3) if events else None, "event_max": max(events) if events else None,
            "pair_slots_by_criterion": dict(why or {})}


# --- arena -----------------------------------------------------------------------------------------
def arena_trajs(groups_path):
    for line in open(groups_path):
        r = json.loads(line)
        if not isinstance(r.get("trace"), list):
            continue
        trajs = collections.defaultdict(dict)
        for row in r["trace"]:
            t = row[0]
            k = int(round(t / DT))
            for role, x, y, yaw, vx, vy in row[1:]:
                trajs[role][k] = (x, y, yaw, vx, vy, LEN, WID)
        yield r["scene"], r["a"], dict(trajs)


# --- SSv2 benches: lanelet replay -------------------------------------------------------------------
def load_map(path):
    import gen_npc_benchmark as G
    root = ET.parse(path).getroot()
    proj = G.projector()
    nodes = {n.get("id"): proj(float(n.get("lat")), float(n.get("lon"))) for n in root.iter("node")}
    ways = {w.get("id"): [nd.get("ref") for nd in w.iter("nd")] for w in root.iter("way")}
    lls = {}
    for rel in root.iter("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.iter("tag")}
        if tags.get("type") != "lanelet" or tags.get("subtype") == "walkway":
            continue
        mem = {m.get("role"): m.get("ref") for m in rel.iter("member") if m.get("type") == "way"}
        if "left" not in mem or "right" not in mem:
            continue
        lw, rw = ways[mem["left"]], ways[mem["right"]]
        left = [nodes[r] for r in lw if r in nodes]
        right = [nodes[r] for r in rw if r in nodes]
        center = G.centerline(left, right)
        if len(center) < 2:
            continue
        lls[int(rel.get("id"))] = {"center": resample(center), "turn": tags.get("turn_direction"),
                                   "start": (lw[0], rw[0]), "end": (lw[-1], rw[-1])}
    by_start = collections.defaultdict(list)
    for i, l in lls.items():
        by_start[l["start"]].append(i)
    for i, l in lls.items():
        nxt = [j for j in by_start.get(l["end"], []) if j != i]
        if not nxt:  # no shared nodes: by geometry
            e = l["center"][-1]
            he = heading(l["center"][-2], l["center"][-1])
            nxt = [j for j, m in lls.items() if j != i and math.dist(m["center"][0], e) < 1.0
                   and math.cos(heading(m["center"][0], m["center"][1]) - he) > 0.7]
        l["next"] = nxt
    return lls


def heading(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0])


def resample(pts, step=0.5):
    out = [pts[0]]
    for a, b in zip(pts, pts[1:]):
        d = math.dist(a, b)
        n = max(1, int(d / step))
        for i in range(1, n + 1):
            out.append((a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n))
    return out


def pick_next(lls, i):
    nxt = lls[i]["next"]
    if not nxt:
        return None
    straight = [j for j in nxt if lls[j]["turn"] == "straight"]
    if straight:
        return straight[0]
    c = lls[i]["center"]
    h = heading(c[-2], c[-1])
    return min(nxt, key=lambda j: abs(math.remainder(heading(lls[j]["center"][-2], lls[j]["center"][-1]) - h, math.tau)))


class Npc:
    def __init__(self, lls, ll, s, v):
        self.lls = lls
        self.path = list(lls[ll]["center"])
        self.cur = ll
        self.s = s
        self.v = v
        self.v0 = v
        self.done = False
        self.extend(120.0)

    def length(self):
        return (len(self.path) - 1) * 0.5  # resampled at <= 0.5 m: approximately

    def extend(self, ahead):
        cum = sum(math.dist(a, b) for a, b in zip(self.path, self.path[1:]))
        while cum < self.s + ahead:
            n = pick_next(self.lls, self.cur)
            if n is None:
                break
            c = self.lls[n]["center"]
            cum += sum(math.dist(a, b) for a, b in zip(c, c[1:])) + math.dist(self.path[-1], c[0])
            self.path += c
            self.cur = n
        self._cum = [0.0]
        for a, b in zip(self.path, self.path[1:]):
            self._cum.append(self._cum[-1] + math.dist(a, b))

    def at(self, s):
        import bisect
        i = min(max(1, bisect.bisect_left(self._cum, s)), len(self.path) - 1)
        a, b = self.path[i - 1], self.path[i]
        seg = self._cum[i] - self._cum[i - 1]
        f = 0.0 if seg <= 0 else min(1.0, max(0.0, (s - self._cum[i - 1]) / seg))
        return a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), heading(a, b) if seg > 0 else 0.0

    def pose(self):
        x, y, h = self.at(self.s)
        return (x, y, h, self.v * math.cos(h), self.v * math.sin(h), LEN, WID)

    def lead_gap(self, others):
        """The bumper gap and speed of the nearest car ahead on this car's path (60 m)."""
        best = None
        for o in others:
            if o is self:
                continue
            ox, oy, oh, _, _, _, _ = o.pose()
            x, y, _ = self.at(self.s)
            if math.hypot(ox - x, oy - y) > 65:
                continue
            for ds in range(1, 61):
                px, py, ph = self.at(self.s + ds)
                if math.hypot(px - ox, py - oy) < 1.2 and math.cos(ph - oh) > 0.7:
                    g = ds - LEN
                    if best is None or g < best[0]:
                        best = (g, o.v)
                    break
        return best

    def step(self, others, dt):
        self.extend(120.0)
        a = min(C1.TYPES["car"].a_max, 1.5 * (self.v0 - self.v))
        lg = self.lead_gap(others)
        if lg is not None:
            a = min(a, C1.follow_accel(max(lg[0], 0.1), self.v, lg[1], C1.TYPES["car"], 1e3))
        self.v = max(0.0, self.v + a * dt)
        self.s += self.v * dt
        if self.s > self._cum[-1] - 1.0:
            self.done = True  # the end of the mapped road


def ssv2_starts(xosc):
    txt = open(xosc).read()
    starts = re.findall(r'<Private entityRef="([^"]+)">.*?<LanePosition roadId="[^"]*" laneId="(\d+)" s="([\d.]+)".*?'
                        r'<AbsoluteTargetSpeed value="([\d.]+)"', txt, flags=re.S)
    dur = re.search(r'SimulationTimeCondition value="([\d.]+)"', txt)
    return [(n, int(l), float(s), float(v)) for n, l, s, v in starts], float(dur.group(1)) if dur else 60.0


def ssv2_trajs(lls, xosc):
    starts, dur = ssv2_starts(xosc)
    npcs = {n: Npc(lls, ll, s, v) for n, ll, s, v in starts}
    trajs = {n: {} for n in npcs}
    for k in range(int(dur / DT) + 1):
        for n, c in npcs.items():
            if not c.done:
                trajs[n][k] = c.pose()
        live = [c for c in npcs.values() if not c.done]
        for c in live:
            c.step(live, DT)
    return trajs, len(starts), dur


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--groups", help="N2's groups.jsonl (three-car arena traces)")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    res = []
    if a.groups:
        acc = collections.defaultdict(lambda: ([], [], collections.Counter()))
        for scene, rule, trajs in arena_trajs(a.groups):
            conf, why = conflicts(trajs)
            ks, ev = sizes(trajs, conf)
            for key in (f"arena scene {scene}", "arena, both scenes"):
                acc[key][0].extend(ks)
                acc[key][1].extend(ev)
                acc[key][2].update(why)
        for key, (ks, ev, why) in sorted(acc.items()):
            res.append(describe(key, ks, ev, why))
    lls = load_map(MAP)
    all_k, all_e, all_w = [], [], collections.Counter()
    for sc in SSV2_SCENES:
        trajs, n, dur = ssv2_trajs(lls, ROOT / sc)
        conf, why = conflicts(trajs)
        ks, ev = sizes(trajs, conf)
        all_w.update(why)
        d = describe(f"ssv2 {Path(sc).stem} ({n} NPCs, {dur:.0f} s)", ks, ev, why)
        d["npcs_reaching_map_end"] = sum(1 for tr in trajs.values() if len(tr) < int(dur / DT) + 1)
        res.append(d)
        all_k += ks
        all_e += ev
        with open(os.path.join(a.out, f"traj_{Path(sc).stem}.json"), "w") as f:
            json.dump({n: {str(k): [round(x, 3) for x in p[:5]] for k, p in tr.items()} for n, tr in trajs.items()}, f)
    res.append(describe("ssv2, all four benches", all_k, all_e, all_w))
    with open(os.path.join(a.out, "n1_sizes.json"), "w") as f:
        json.dump(res, f, indent=1)
    for d in res:
        print(json.dumps(d))


if __name__ == "__main__":
    main()
