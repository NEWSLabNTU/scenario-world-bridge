#!/usr/bin/env python3
"""Roadmap 074 C2b: run sampled cut-in cells through swb itself and score them as the arena does.

Each run: a scenario with two `agent` entities on Town03's east straight (coop_b in CARLA's
lane 3 of road 65, coop_a in lane 2 beside it; no Autoware), two cutin_agent.py
processes with the run's context and rules, `just scenario` against a stack started with
`ego_launch:=none` on Town03, then the payoffs from the agents' logs (their own and the other
car's true states each tick) with cutin_game's utility, the run judged as arena.py judges it.

Usage (the stack up: CARLA_PORT=<port> just run map_path:=<Town03> ego_launch:=none):
  scripts/coop/c2b.py --out <dir> --carla-port 2400 [--cells cells.json] [--seeds 1]
"""

import argparse
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import cutin_game as C1  # noqa: E402
import double_oracle as DO  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
H = math.atan2(1.0, 0.025)
LANES = {"coop_b": 246.0, "coop_a": 242.5}  # x at y = 16.7, +0.025 m per m of y
# (CARLA's lane at x = 239 is Bidirectional, not Driving, though Autoware's lanelet 68766 is a
# driving lane: the light agents use CARLA's two driving lanes, 2 and 3)
L = 4.79
VB, WARMUP = 8.0, 8.0

# Cells: the band, the contacts, a clear go; rule pairs: the double oracle's equilibrium and the
# Autoware-like pair.
A_EQ = {"name": "A_eq", "tau": 0.68, "a_rear": 2.07, "buffer": 4.22, "patience": 6.0, "use_yield": 0, "t_prep": 0.0}
B_EQ = dict(DO.B_REACT, name="B_eq")
CELLS = [(2.0, 2.0), (6.0, 0.0), (10.0, 0.0), (10.0, -2.0), (14.0, -3.0), (20.0, 1.0)]
PAIRS = [(A_EQ, B_EQ), (DO.A_AW_PERM, DO.B_AW)]


def xy(entity, y):
    return round(LANES[entity] + 0.025 * (y - 16.7), 2), round(y, 2)


def scenario(path, d0, dv):
    r0 = d0 - dv * WARMUP  # A's rear ahead of B's front at the warm-up's start
    yb = 24.0 + max(0.0, -(r0 + L))
    ya = yb + r0 + L
    pos = lambda e, y: f'<WorldPosition x="{xy(e, y)[0]}" y="{xy(e, y)[1]}" z="0.3" h="{H:.4f}" p="0" r="0" />'
    goal = lambda e: f'<WorldPosition x="{xy(e, 134.0)[0]}" y="134.0" z="0.0" h="{H:.4f}" p="0" r="0" />'
    ent = lambda e: f'''		<ScenarioObject name="{e}">
			<CatalogReference catalogName="vehicles" entryName="tesla_model3">
				<ParameterAssignments>
					<ParameterAssignment parameterRef="maxSpeed" value="50" />
					<ParameterAssignment parameterRef="maxAcceleration" value="15.0" />
					<ParameterAssignment parameterRef="maxDeceleration" value="15.0" />
					<ParameterAssignment parameterRef="maxSteering" value="0.5" />
				</ParameterAssignments>
			</CatalogReference>
			<ObjectController><Controller name="agent"><Properties /></Controller></ObjectController>
		</ScenarioObject>'''
    init = lambda e, y: f'''			<Private entityRef="{e}">
				<PrivateAction><TeleportAction><Position>{pos(e, y)}</Position></TeleportAction></PrivateAction>
				<PrivateAction><RoutingAction><AcquirePositionAction><Position>{goal(e)}</Position></AcquirePositionAction></RoutingAction></PrivateAction>
			</Private>'''
    Path(path).write_text(f'''<?xml version="1.0"?>
<!-- Roadmap 074 C2b: the cut-in with two light-policy agents, d0 {d0} dv {dv}; written by scripts/coop/c2b.py -->
<OpenSCENARIO>
	<FileHeader author="scenario-world-bridge" date="2026-11-13T00:00:00+00:00" description="074 C2b cut-in d0 {d0} dv {dv}" revMajor="1" revMinor="2" />
	<ParameterDeclarations />
	<CatalogLocations><VehicleCatalog><Directory path="$(find-pkg-share sim_vehicle_spec)/catalogs" /></VehicleCatalog></CatalogLocations>
	<RoadNetwork><LogicFile filepath="$(env CARLA_MAPS)/Town03" /></RoadNetwork>
	<Entities>
{ent("coop_a")}
{ent("coop_b")}
	</Entities>
	<Storyboard>
		<Init><Actions>
{init("coop_a", ya)}
{init("coop_b", yb)}
		</Actions></Init>
		<Story name="c2b"><Act name="c2b_act">
			<ManeuverGroup maximumExecutionCount="1" name="end"><Actors selectTriggeringEntities="false"><EntityRef entityRef="coop_b" /></Actors>
			<Maneuver name="end_m">
				<Event name="done" priority="parallel" maximumExecutionCount="1">
					<Action name="done_a"><UserDefinedAction><CustomCommandAction type="exitSuccess" /></UserDefinedAction></Action>
					<StartTrigger><ConditionGroup><Condition name="time" delay="0" conditionEdge="none"><ByValueCondition><SimulationTimeCondition value="{WARMUP + C1.T_MAX + 2}" rule="greaterThan" /></ByValueCondition></Condition></ConditionGroup></StartTrigger>
				</Event>
			</Maneuver></ManeuverGroup>
			<StartTrigger><ConditionGroup><Condition name="start" delay="0" conditionEdge="none"><ByValueCondition><SimulationTimeCondition value="0" rule="greaterThan" /></ByValueCondition></Condition></ConditionGroup></StartTrigger>
		</Act></Story>
		<StopTrigger />
	</Storyboard>
</OpenSCENARIO>
''')


def score(log_a, log_b, d0, dv, t_prep=0.0):
    """The arena's judgement from the agents' logs."""
    ra = [json.loads(line) for line in open(log_a) if line.strip()]
    rb = [json.loads(line) for line in open(log_b) if line.strip()]
    ta = [r for r in ra if "phase" in r and r["phase"] == "run"]
    tb = [r for r in rb if "phase" in r and r["phase"] == "run"]
    if not ta or not tb:
        return {"error": "no run phase logged"}
    t0 = ta[0]["t"]
    start = next((r for r in ra if r.get("event") == "start"), {})
    min_ttc = min_thw = math.inf
    contact = False
    end = ta[-1]
    peak = [0.0, 0.0]
    hist = []
    for r in ta:
        t = r["t"] - t0
        lat, along, v, ov = r["lat"], r["along"], r["v"], r["ov"]  # along: B's centre ahead of A
        if lat >= C1.LANE_LINE:
            if abs(along) < L:
                contact = True
            if along < 0:  # A ahead
                closing, vf, gap = ov - v, ov, -along - L
            else:
                closing, vf, gap = v - ov, v, along - L
            if closing > 1e-3 and gap > 0:
                min_ttc = min(min_ttc, gap / closing)
            if vf > 1.0 and gap > 0:
                min_thw = min(min_thw, gap / vf)
        # peak braking as arena.py measures it: the largest speed drop over any 0.5 s
        hist.append((t, v, ov))
        while hist and t - hist[0][0] > 0.5 + 1e-9:
            hist.pop(0)
        if t - hist[0][0] >= 0.5 - 0.051:
            span = t - hist[0][0]
            peak[0] = max(peak[0], (hist[0][1] - v) / span)
            peak[1] = max(peak[1], (hist[0][2] - ov) / span)
        dec = r.get("decide")
        merged = dec is not None and r["t"] - t0 >= (dec - 0.0) + t_prep + C1.TYPES["car"].t_lc
        if contact or t >= C1.T_MAX or (t >= C1.HORIZON and merged):
            end = r
            break
    T = end["t"] - t0
    va0, vb0 = ta[0]["v"], ta[0]["ov"]
    da = sum(r["v"] * 0.1 for r in ta if t0 <= r["t"] <= end["t"])
    db = sum(r["ov"] * 0.1 for r in ta if t0 <= r["t"] <= end["t"])
    lost_a = min(1.0, max(0.0, (va0 * T - da) / max(va0 * C1.HORIZON, 1.0)))
    lost_b = min(1.0, max(0.0, (vb0 * T - db) / max(vb0 * C1.HORIZON, 1.0)))
    # in: A's lane change is complete by its plan and A is in B's lane (the lateral measure on the
    # curve settles near 0.96, so a 0.9 threshold at the end misread a finished merge)
    dec = end.get("decide")
    got_in = dec is not None and end["t"] - t0 >= dec + t_prep + C1.TYPES["car"].t_lc and end["lat"] >= 0.5
    dropped = any(r.get("dropping") for r in ta)
    ahead = got_in and not dropped and end["along"] < 0
    if contact:
        lost_a = lost_b = 1.0
    elif not got_in:
        lost_a = 1.0
    o = C1.Outcome((lost_a, lost_b), min_ttc, min_thw, contact, (peak[0], peak[1]), ahead, "")
    pref, cap = C1.Preference(), C1.TYPES["car"]
    return {"d0_real": start.get("along") and round(-start["along"] - L, 2), "dv_real": start.get("v") and round(start["v"] - start["ov"], 2),
            "uA": round(C1.utility(o, 0, cap, pref), 4), "uB": round(C1.utility(o, 1, cap, pref), 4),
            "contact": int(contact), "ahead": int(ahead), "got_in": int(got_in), "t_run": round(T, 2),
            "ttc": round(min_ttc, 2) if math.isfinite(min_ttc) else "inf"}


def run_one(out, d0, dv, pa, pb, seed, port, relay):
    tag = f"d{d0:g}_v{dv:g}_{pa['name']}_{pb['name']}_s{seed}"
    rd = Path(out) / tag
    rd.mkdir(parents=True, exist_ok=True)
    xosc = rd / "c2b.xosc"
    scenario(xosc, d0, dv)
    procs = []
    for ent, other, side, rule in (("coop_a", "coop_b", "A", pa), ("coop_b", "coop_a", "B", pb)):
        cfg = {"entity": ent, "other": other, "side": side, "d0": d0, "dv": dv, "vb": VB, "warmup": WARMUP,
               "rule": rule, "seed": seed, "log": str(rd / f"{ent}.jsonl"), "carla_port": port, "find_timeout": 60,
               "carla_pythonapi": os.path.join(os.environ.get("CARLA_DIR", "/mnt/p5plus/aeon/carla/CARLA_0.9.16"), "PythonAPI", "carla")}
        (rd / f"{ent}.json").write_text(json.dumps(cfg))
        procs.append(subprocess.Popen([sys.executable, str(ROOT / "scripts/coop/cutin_agent.py"), "--relay", relay,
                                       "--entity", ent, "--config", str(rd / f"{ent}.json")],
                                      stdout=open(rd / f"{ent}.log", "w"), stderr=subprocess.STDOUT, start_new_session=True))
    time.sleep(3)
    env = dict(os.environ, CARLA_PORT=str(port))
    r = subprocess.run(["just", "scenario", str(xosc), f"output_directory:={rd}"], cwd=ROOT, env=env,
                       stdout=open(rd / "scenario.log", "w"), stderr=subprocess.STDOUT)
    for p in procs:
        try:
            os.killpg(p.pid, 15)
        except ProcessLookupError:
            pass
        p.wait(timeout=20)
    res = {"d0": d0, "dv": dv, "a": pa["name"], "b": pb["name"], "seed": seed, "scenario_exit": r.returncode}
    try:
        res.update(score(rd / "coop_a.jsonl", rd / "coop_b.jsonl", d0, dv, pa["t_prep"]))
    except Exception as e:  # noqa: BLE001 -- recorded with the run
        res["error"] = repr(e)
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--carla-port", type=int, default=2400)
    ap.add_argument("--relay", default="tcp://localhost:5560")
    ap.add_argument("--seeds", type=int, default=1)
    ap.add_argument("--only", type=int, default=0, help="run just the first N runs (a smoke test)")
    ap.add_argument("--arena-port", type=int, default=0, help="instead: the same cells in the arena on this CARLA")
    ap.add_argument("--rescore", action="store_true", help="instead: score the runs in --out again from their logs")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    if a.rescore:
        old = [json.loads(l) for l in open(os.path.join(a.out, "c2b.jsonl")) if l.strip()]
        with open(os.path.join(a.out, "c2b.jsonl"), "w") as f:
            for r in old:
                tag = f"d{r['d0']:g}_v{r['dv']:g}_{r['a']}_{r['b']}_s{r['seed']}"
                rd = Path(a.out) / tag
                pa = A_EQ if r["a"] == "A_eq" else DO.A_AW_PERM
                r.update(score(rd / "coop_a.jsonl", rd / "coop_b.jsonl", r["d0"], r["dv"], pa["t_prep"]))
                f.write(json.dumps(r) + "\n")
        return
    if a.arena_port:
        arena_compare(a.arena_port, a.out)
        return
    runs = [(d0, dv, pa, pb, s) for (d0, dv) in CELLS for pa, pb in PAIRS for s in range(a.seeds)]
    if a.only:
        runs = runs[: a.only]
    with open(os.path.join(a.out, "c2b.jsonl"), "a") as f:
        for d0, dv, pa, pb, s in runs:
            res = run_one(a.out, d0, dv, pa, pb, s, a.carla_port, a.relay)
            print(json.dumps(res), flush=True)
            f.write(json.dumps(res) + "\n")
            f.flush()


def arena_compare(port, out, seeds=4):
    """The same cells and rules in the arena, at C2b's settings (v_B 8 m/s, 8 s warm-up)."""
    import random
    import arena as AR

    class Args:
        k, obs_delay, obs_sigma_x, obs_sigma_v, reaction_jitter = 4, 0.19, 0.3, 0.2, 0.2
    AR.WARMUP = WARMUP
    ar = AR.Arena(port, 4)
    try:
        cars = AR.setup_cars(ar, 4)
        jobs = [{"d0": d0, "dv": dv, "vb": VB, "pa": pa, "pb": pb, "seed": s, "a": pa["name"], "b": pb["name"]}
                for (d0, dv) in CELLS for pa, pb in PAIRS for s in range(seeds)]
        rows = AR.run_jobs(ar, cars, jobs, random.Random(5), Args())
    finally:
        ar.close()
    keep = ("d0", "dv", "a", "b", "seed", "d0_real", "dv_real", "uA", "uB", "contact", "ahead", "got_in", "t_run", "ttc")
    with open(os.path.join(out, "arena.jsonl"), "w") as f:
        for r in rows:
            f.write(json.dumps({k: r[k] for k in keep}) + "\n")
    return rows


if __name__ == "__main__":
    main()
