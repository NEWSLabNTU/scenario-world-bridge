#!/usr/bin/env python3
"""Roadmap 074 N5's swb check: the 45 deg Y merge with three cars through swb, on Town03 j1901,
against the arena (ymerge3_arena.py) on D2's yardstick.

Each run: a scenario with three `agent` entities (no Autoware) on Town03 -- coop_a on the diagonal
(road 44), coop_b on the main road (road 21) and coop_c behind it, each where its warm-up starts
(ymerge3_town03's lanes, through the junctions before j1901) -- three ymerge3_agent.py processes
with the run's context and A's rule, `just scenario` against a stack started with
`ego_launch:=none` on Town03, then ymerge3.score3 on coop_a's log (all three cars' true states
each tick), as the arena scores. A run whose warm-up missed its context (dt or C's headway off
by more than 0.25 s, or a speed off by more than 10 %) is invalid and redone once.

Compared (--compare) with ymerge3_arena.py's runs: per (dt, A's rule) each swb run's payoffs
(uA, uB, uC) inside the arena's mean +- max(2 sd, 0.02), and the outcome (A's slot, contact) by
majority; the kinematic model (ymerge_game.simulate_stream, N = 3) beside both.

Usage (the stack up with the spec's physics on the agent cars, else they run the blueprint's tires:
  CARLA_PORT=<port> just run map_path:=<Town03> ego_launch:=none \
      vehicle_physics:=<repo>/src/sim_vehicle_spec/generated/tesla_model3/carla_physics.yaml):
  scripts/coop/n5_swb.py --out <dir> --lanes <lanes.json> --carla-port 2010 [--seeds 2]
  scripts/coop/n5_swb.py --out <dir> --lanes <lanes.json> --compare <arena dir>
"""

import argparse
import csv
import json
import math
import os
import statistics as st
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import crossing_game as G  # noqa: E402
import ymerge3 as Y3  # noqa: E402
import ymerge3_town03 as T  # noqa: E402
import ymerge_arena as YA  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
CARS = "ABC"
ENT = {"A": "coop_a", "B": "coop_b", "C": "coop_c"}
ORIGIN_AHEAD = 1.416  # the catalog's bounding-box centre ahead of the scenario's vehicle origin (m)


def ros_position(x, y, z, yaw, back=ORIGIN_AHEAD):
    """SSv2's WorldPosition for a car whose box centre is at CARLA (x, y, yaw)."""
    xo, yo = x - back * math.cos(yaw), y - back * math.sin(yaw)
    return f'<WorldPosition x="{xo:.3f}" y="{-yo:.3f}" z="{z + 0.3:.2f}" h="{-yaw:.4f}" p="0" r="0" />'


def scenario(path, lanes, dt, v, h, tag):
    target = Y3.start_u(dt, v, h)
    ents, inits = [], []
    for k in CARS:
        ln = lanes[k]
        x, y, z, yaw = ln.at(ln.s_of_u(T.WarmupPlan(k, target[k], v, YA.WARMUP).u_spawn))
        gx, gy, gz, gyaw = ln.at(ln.s_m + 120.0)
        ents.append(f'''		<ScenarioObject name="{ENT[k]}">
			<CatalogReference catalogName="vehicles" entryName="tesla_model3">
				<ParameterAssignments>
					<ParameterAssignment parameterRef="maxSpeed" value="50" />
					<ParameterAssignment parameterRef="maxAcceleration" value="15.0" />
					<ParameterAssignment parameterRef="maxDeceleration" value="15.0" />
					<ParameterAssignment parameterRef="maxSteering" value="0.5" />
				</ParameterAssignments>
			</CatalogReference>
			<ObjectController><Controller name="agent"><Properties /></Controller></ObjectController>
		</ScenarioObject>''')
        inits.append(f'''			<Private entityRef="{ENT[k]}">
				<PrivateAction><TeleportAction><Position>{ros_position(x, y, z, yaw)}</Position></TeleportAction></PrivateAction>
				<PrivateAction><RoutingAction><AcquirePositionAction><Position>{ros_position(gx, gy, gz - 0.3, gyaw)}</Position></AcquirePositionAction></RoutingAction></PrivateAction>
			</Private>''')
    nl = "\n"
    Path(path).write_text(f'''<?xml version="1.0"?>
<!-- Roadmap 074 N5 swb: the Y merge at Town03 j1901 with three light-policy agents, {tag}; written by scripts/coop/n5_swb.py -->
<OpenSCENARIO>
	<FileHeader author="scenario-world-bridge" date="2026-11-15T00:00:00+00:00" description="074 N5 swb {tag}" revMajor="1" revMinor="2" />
	<ParameterDeclarations />
	<CatalogLocations><VehicleCatalog><Directory path="$(find-pkg-share sim_vehicle_spec)/catalogs" /></VehicleCatalog></CatalogLocations>
	<RoadNetwork><LogicFile filepath="$(env CARLA_MAPS)/Town03" /></RoadNetwork>
	<Entities>
{nl.join(ents)}
	</Entities>
	<Storyboard>
		<Init><Actions>
{nl.join(inits)}
		</Actions></Init>
		<Story name="n5"><Act name="n5_act">
			<ManeuverGroup maximumExecutionCount="1" name="end"><Actors selectTriggeringEntities="false"><EntityRef entityRef="coop_a" /></Actors>
			<Maneuver name="end_m">
				<Event name="done" priority="parallel" maximumExecutionCount="1">
					<Action name="done_a"><UserDefinedAction><CustomCommandAction type="exitSuccess" /></UserDefinedAction></Action>
					<StartTrigger><ConditionGroup><Condition name="time" delay="0" conditionEdge="none"><ByValueCondition><SimulationTimeCondition value="{YA.WARMUP + G.HORIZON + 10}" rule="greaterThan" /></ByValueCondition></Condition></ConditionGroup></StartTrigger>
				</Event>
			</Maneuver></ManeuverGroup>
			<StartTrigger><ConditionGroup><Condition name="start" delay="0" conditionEdge="none"><ByValueCondition><SimulationTimeCondition value="0" rule="greaterThan" /></ByValueCondition></Condition></ConditionGroup></StartTrigger>
		</Act></Story>
		<StopTrigger />
	</Storyboard>
</OpenSCENARIO>
''')


# --- scoring ----------------------------------------------------------------------------------------
def score(log_a, log_others, dt, v, h):
    rows = [json.loads(line) for line in open(log_a) if line.strip()]
    st_a = next((r for r in rows if r.get("event") == "start"), None)
    run_a = next((r for r in rows if r.get("event") == "run"), None)
    if st_a is None or run_a is None:
        return {"error": "no run logged"}
    starts = {}
    for k, p in log_others.items():
        try:
            s = next((json.loads(line) for line in open(p) if '"event": "start"' in line), None)
        except FileNotFoundError:
            s = None
        if s is None:
            return {"error": f"{k} logged no start"}
        starts[k] = s["t0"]
    run = [r for r in rows if r.get("phase") == "run"]
    if not run:
        return {"error": "no run phase logged"}
    tr0 = run_a["t"]
    ticks = [(r["t"] - tr0, {k: tuple(r[k]) for k in CARS}) for r in run]
    u0, v0 = run_a["u"], run_a["v"]
    box = {k: tuple(st_a["box"][k]) for k in CARS}
    res = Y3.score3(ticks, {k: v0[k] for k in CARS}, box, eps=0.01)
    dt_real = -u0["A"] / max(v0["A"], 0.1) + u0["B"] / max(v0["B"], 0.1)
    h_real = (u0["B"] - u0["C"] - box["B"][0]) / max(v0["C"], 0.1)
    valid = abs(dt_real - dt) <= 0.25 and abs(h_real - h) <= 0.25 and all(abs(v0[k] - v) <= 0.1 * v for k in CARS)
    last = run[min(len(run) - 1, int(res["t_run"] / 0.05))].get("rec", {})
    return {**res, "dt_real": round(dt_real, 3), "h_real": round(h_real, 3), **{f"v{k}0": round(v0[k], 2) for k in CARS},
            "lag_b": round(starts["B"] - st_a["t0"], 3), "lag_c": round(starts["C"] - st_a["t0"], 3), "valid": int(valid),
            "A_facings": last.get("facings", ""), "friction": json.dumps(run_a.get("friction", {}))}


def run_one(out, lanes_path, lanes, dt, v, h, a, seed, port, relay):
    tag = f"dt{dt:g}_{a}_s{seed}"
    rd = Path(out) / tag
    rd.mkdir(parents=True, exist_ok=True)
    xosc = rd / "n5.xosc"
    scenario(xosc, lanes, dt, v, h, tag)
    procs = []
    for k in CARS:
        cfg = {"entity": ENT[k], "role": k, "roles": ENT, "dt": dt, "v": v, "h": h, "warmup": YA.WARMUP, "a": a, "seed": seed,
               "lanes": str(Path(lanes_path).resolve()), "log": str(rd / f"{ENT[k]}.jsonl"), "carla_port": port, "find_timeout": 60,
               "carla_pythonapi": os.path.join(os.environ.get("CARLA_DIR", os.path.expanduser("~/Downloads/CARLA_0.9.16")),
                                               "PythonAPI", "carla")}
        (rd / f"{ENT[k]}.json").write_text(json.dumps(cfg))
        procs.append(subprocess.Popen([sys.executable, str(ROOT / "scripts/coop/ymerge3_agent.py"), "--relay", relay,
                                       "--entity", ENT[k], "--config", str(rd / f"{ENT[k]}.json")],
                                      stdout=open(rd / f"{ENT[k]}.log", "w"), stderr=subprocess.STDOUT, start_new_session=True))
    time.sleep(3)
    env = dict(os.environ, CARLA_PORT=str(port))
    t_wall = time.monotonic()
    r = subprocess.run(["just", "scenario", str(xosc), f"output_directory:={rd}"], cwd=ROOT, env=env,
                       stdout=open(rd / "scenario.log", "w"), stderr=subprocess.STDOUT)
    wall = time.monotonic() - t_wall
    for p in procs:
        try:
            os.killpg(p.pid, 15)
        except ProcessLookupError:
            pass
        try:
            p.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, 9)
            p.wait()
    res = {"dt": dt, "v": v, "h": h, "a": a, "seed": seed, "scenario_exit": r.returncode, "wall_s": round(wall, 1)}
    try:
        res.update(score(rd / "coop_a.jsonl", {"B": rd / "coop_b.jsonl", "C": rd / "coop_c.jsonl"}, dt, v, h))
    except Exception as e:  # noqa: BLE001 -- recorded with the run
        res["error"] = repr(e)
    return res


# --- the comparison ---------------------------------------------------------------------------------
def compare(rows, arena_dir, v, h):
    lags = G.car_lags()
    arena = {}
    for r in csv.DictReader(open(os.path.join(arena_dir, "runs.csv"))):
        arena.setdefault((float(r["dt"]), r["a"]), []).append(r)
    swb = {}
    for r in rows:
        if r.get("valid") and "error" not in r:
            swb.setdefault((float(r["dt"]), r["a"]), []).append(r)
    lines, inside, total, agree_a, agree_k, n_out = [], 0, 0, 0, 0, 0
    cells = []
    maj = lambda rs, f: max(set(f(r) for r in rs), key=lambda x: sum(f(r) == x for r in rs))  # noqa: E731
    out_of = lambda r: (str(r["slot"]), int(r["contact"]))  # noqa: E731
    for dt in sorted({k[0] for k in arena}):
        for a in Y3.A_RULES:
            ss, ar = swb.get((dt, a), []), arena.get((dt, a), [])
            ko = Y3.kinematic(a, dt, v, h, lags)
            if not ss or not ar:
                lines.append(f"dt {dt:+4.1f} {a:>8} | {'no valid swb runs' if not ss else 'no arena runs'}")
                continue
            hits = []
            for key in ("uA", "uB", "uC"):
                xs = [float(r[key]) for r in ar]
                mu, sd = st.mean(xs), (st.pstdev(xs) if len(xs) > 1 else 0.0)
                for r in ss:
                    hit = abs(float(r[key]) - mu) <= max(2 * sd, 0.02)
                    inside += hit
                    total += 1
                    hits.append(hit)
            oa, os_ = maj(ar, out_of), maj(ss, out_of)
            ok = (str(ko["slot"]), ko["contact"])
            agree_a += oa == os_
            agree_k += ok == os_
            n_out += 1
            us = {k: st.mean(float(r[k]) for r in ss) for k in ("uA", "uB", "uC")}
            ua = {k: st.mean(float(r[k]) for r in ar) for k in ("uA", "uB", "uC")}
            cells.append({"dt": dt, "a": a, "swb": us, "arena": ua, "kinematic": {k: ko[k] for k in ("uA", "uB", "uC")},
                          "out_swb": os_, "out_arena": oa, "out_kinematic": ok, "inside": sum(hits), "of": len(hits)})
            lines.append(f"dt {dt:+4.1f} {a:>8} | swb ({len(ss)}) uA {us['uA']:+.3f} uB {us['uB']:+.3f} uC {us['uC']:+.3f} slot/contact {os_} "
                         f"| arena ({len(ar)}) uA {ua['uA']:+.3f} uB {ua['uB']:+.3f} uC {ua['uC']:+.3f} {oa} "
                         f"| kin uA {ko['uA']:+.3f} uB {ko['uB']:+.3f} uC {ko['uC']:+.3f} {ok} | payoffs in {sum(hits)}/{len(hits)}"
                         f"{'' if oa == os_ else ' (outcome differs from the arena)'}")
    lines.append(f"swb payoffs inside the arena's interval: {inside} of {total} ({100 * inside / max(total, 1):.0f} %); outcomes "
                 f"(A's slot, contact) agree with the arena in {agree_a} of {n_out}, with the kinematic model in {agree_k} of {n_out}")
    return lines, {"payoffs_inside": inside, "payoffs": total, "outcomes_arena": agree_a, "outcomes_kinematic": agree_k,
                   "pairs": n_out, "cells": cells}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--lanes", required=True, help="ymerge3_town03.py's JSON")
    ap.add_argument("--carla-port", type=int, default=2010)
    ap.add_argument("--relay", default=f"tcp://localhost:{os.environ.get('AGENT_PORT', '5560')}")
    ap.add_argument("--seeds", type=int, default=2)
    ap.add_argument("--v", type=float, default=Y3.V)
    ap.add_argument("--h", type=float, default=Y3.H_FOLLOW)
    ap.add_argument("--dts", default=",".join(str(d) for d in Y3.DTS))
    ap.add_argument("--only", help="dt:a:seed,... instead of every run")
    ap.add_argument("--redo-invalid", type=int, default=1)
    ap.add_argument("--compare", metavar="ARENA_DIR")
    ap.add_argument("--rescore", action="store_true")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    lanes = T.load(a.lanes)
    res_path = os.path.join(a.out, "n5_swb.jsonl")
    if a.rescore or a.compare:
        rows = [json.loads(line) for line in open(res_path) if line.strip()]
        if a.rescore:
            for r in rows:
                rd = Path(a.out) / f"dt{r['dt']:g}_{r['a']}_s{r['seed']}"
                r.update(score(rd / "coop_a.jsonl", {"B": rd / "coop_b.jsonl", "C": rd / "coop_c.jsonl"}, r["dt"], r["v"], r["h"]))
            with open(res_path, "w") as f:
                for r in rows:
                    f.write(json.dumps(r) + "\n")
        if a.compare:
            lines, summ = compare(rows, a.compare, a.v, a.h)
            Path(a.out, "compare.txt").write_text("\n".join(lines) + "\n")
            Path(a.out, "summary.json").write_text(json.dumps(summ, indent=1))
            print("\n".join(lines))
        return 0
    if a.only:
        runs = [(float(x.split(":")[0]), x.split(":")[1], int(x.split(":")[2])) for x in a.only.split(",")]
    else:
        runs = [(float(dt), r, s) for dt in a.dts.split(",") for r in Y3.A_RULES for s in range(a.seeds)]
    with open(res_path, "a") as f:
        for dt, rule, s in runs:
            for attempt in range(1 + a.redo_invalid):
                res = run_one(a.out, a.lanes, lanes, dt, a.v, a.h, rule, s, a.carla_port, a.relay)
                res["attempt"] = attempt
                print(json.dumps(res), flush=True)
                if res.get("valid") or attempt == a.redo_invalid:
                    break
            f.write(json.dumps(res) + "\n")
            f.flush()
    return 0


if __name__ == "__main__":
    sys.exit(main())
