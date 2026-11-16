#!/usr/bin/env python3
"""Roadmap 074 U2 swb: the roundabout entry through swb itself, on Town03's roundabout, scored as the arena
scores it.

Each run: a scenario with two `agent` entities (no Autoware) on Town03's roundabout -- coop_a at rest at
its yield line on the entry from the south, coop_b on the ring's outer lane upstream -- two
roundabout_agent.py processes with the run's context and rules, `just scenario-direct` against a stack started
with `ego_launch:=none` on Town03, then the payoffs from coop_a's log (both cars' true states each tick)
with roundabout_game's utility, the run judged as roundabout_arena.py judges it. B warms up to v and its
context, A holds; the scored window starts when the warm-up ends and stays on the ring (B circulates;
A joins it).

Compared (--compare) with roundabout_arena.py --lanes on the same lanes (Town03 in CARLA, no swb) on
D2's yardstick: the swb run's payoffs inside the arena's mean +- max(2 sd, 0.02), and the outcomes (A
ahead, A forced B, contact) by majority; and with the kinematic model on the same lanes.

Usage (the stack up: CARLA_PORT=<port> just run map_path:=<Town03> ego_launch:=none; the lanes from
roundabout_town03.py on that CARLA):
  scripts/coop/u2_swb.py --out <dir> --lanes <geom.json> --carla-port 2600 [--seeds 2]
  scripts/coop/u2_swb.py --out <dir> --lanes <geom.json> --compare <arena dir>
"""

import argparse
import json
import math
import os
import statistics as st
import subprocess
import sys
import time
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import crossing_game as X  # noqa: E402
import measures as M  # noqa: E402
import roundabout_game as G  # noqa: E402
import roundabout_town03 as T  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
V, WARMUP = 7.0, 10.0
TB = (1.75, 2.25, 3.0, 4.0, 6.5)  # B first; the forcing band; free joins (no tb equals a rule's G)
PAIRS = (("g2.5", "hold"), ("g2.5", "slow"), ("g8", "hold"))
ORIGIN_AHEAD = 1.416  # the catalog's bounding-box centre ahead of the scenario's vehicle origin (m)
CONTACT_EPS = 0.01
PEAK_W = 0.5


def carla_pose(fr, x, y, yaw):
    xc, yc = fr.to_c(x, y)
    return xc, yc, fr.yaw_c(yaw)


def ros_position(fr, pose, z=0.3):
    """SSv2's WorldPosition for a car whose centre is at the model pose (x, y, yaw)."""
    xc, yc, yawc = carla_pose(fr, *pose[:3])
    xc -= ORIGIN_AHEAD * math.cos(yawc)
    yc -= ORIGIN_AHEAD * math.sin(yawc)
    return f'<WorldPosition x="{xc:.3f}" y="{-yc:.3f}" z="{z}" h="{-yawc:.4f}" p="0" r="0" />'


def b_start(tb):
    import crossing_arena as XA

    return G.SZ0 - V * tb - XA.ramp(WARMUP, V)[0]


def scenario(path, fr, tb, tag):
    pa = G.a_path(G.S_YIELD)
    sb = b_start(tb)
    pb = G.ring(sb - G.L / 2)
    ga = G.a_path(G.A_JOINED + 60.0)
    gb = G.ring(sb + 60.0)
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
		</ScenarioObject>'''  # noqa: E731
    init = lambda e, p, g: f'''			<Private entityRef="{e}">
				<PrivateAction><TeleportAction><Position>{ros_position(fr, p)}</Position></TeleportAction></PrivateAction>
				<PrivateAction><RoutingAction><AcquirePositionAction><Position>{ros_position(fr, g, 0.0)}</Position></AcquirePositionAction></RoutingAction></PrivateAction>
			</Private>'''  # noqa: E731
    Path(path).write_text(f'''<?xml version="1.0"?>
<!-- Roadmap 074 U2 swb: the roundabout entry with two light-policy agents on Town03, {tag}; written by scripts/coop/u2_swb.py -->
<OpenSCENARIO>
	<FileHeader author="scenario-world-bridge" date="2026-11-15T00:00:00+00:00" description="074 U2 swb {tag}" revMajor="1" revMinor="2" />
	<ParameterDeclarations />
	<CatalogLocations><VehicleCatalog><Directory path="$(find-pkg-share sim_vehicle_spec)/catalogs" /></VehicleCatalog></CatalogLocations>
	<RoadNetwork><LogicFile filepath="$(env CARLA_MAPS)/Town03" /></RoadNetwork>
	<Entities>
{ent("coop_a")}
{ent("coop_b")}
	</Entities>
	<Storyboard>
		<Init><Actions>
{init("coop_a", pa, ga)}
{init("coop_b", pb, gb)}
		</Actions></Init>
		<Story name="u2"><Act name="u2_act">
			<ManeuverGroup maximumExecutionCount="1" name="end"><Actors selectTriggeringEntities="false"><EntityRef entityRef="coop_b" /></Actors>
			<Maneuver name="end_m">
				<Event name="done" priority="parallel" maximumExecutionCount="1">
					<Action name="done_a"><UserDefinedAction><CustomCommandAction type="exitSuccess" /></UserDefinedAction></Action>
					<StartTrigger><ConditionGroup><Condition name="time" delay="0" conditionEdge="none"><ByValueCondition><SimulationTimeCondition value="{WARMUP + G.HORIZON + 8}" rule="greaterThan" /></ByValueCondition></Condition></ConditionGroup></StartTrigger>
				</Event>
			</Maneuver></ManeuverGroup>
			<StartTrigger><ConditionGroup><Condition name="start" delay="0" conditionEdge="none"><ByValueCondition><SimulationTimeCondition value="0" rule="greaterThan" /></ByValueCondition></Condition></ConditionGroup></StartTrigger>
		</Act></Story>
		<StopTrigger />
	</Storyboard>
</OpenSCENARIO>
''')


# --- scoring ----------------------------------------------------------------------------------------
def _rows(path):
    return [json.loads(line) for line in open(path) if line.strip()]


def score(log_a, log_b, tb, b_rule):
    """roundabout_arena.Run's judgement, from the agents' logs."""
    ra, rb = _rows(log_a), _rows(log_b)
    st_a = next((r for r in ra if r.get("event") == "start"), None)
    run_a = next((r for r in ra if r.get("event") == "run"), None)
    st_b = next((r for r in rb if r.get("event") == "start"), None)
    if st_a is None or run_a is None or st_b is None:
        return {"error": "no run logged"}
    ta = [r for r in ra if r.get("phase") == "run"]
    if not ta:
        return {"error": "no run phase logged"}
    tr0 = run_a["t"]
    w0 = st_a["t0"] + tr0  # the run's start in world time
    real = {"a_off0": round(run_a["s"] - G.S_YIELD, 3), "a_v0": round(run_a["va"], 3), "tb_real": round(run_a["tb_real"], 3),
            "vB_real": round(run_a["vq"], 2), "b_lag": round(st_b["t0"] - st_a["t0"], 3)}
    valid = abs(run_a["vq"] - V) <= 0.1 * V and abs(run_a["tb_real"] - tb) <= 0.2
    la, wa, lb, wb = st_a["L"], st_a["W"], st_a["oL"], st_a["oW"]
    meas = M.Measures(b_max=(G.CAP.b_max, G.CAP.b_max), pet_sample=X.PET_SAMPLE)
    vh = [deque(), deque()]
    peaks = [0.0, 0.0]
    s0, sq0 = ta[0]["s"], ta[0]["os"]
    sa_h = sq_h = None
    contact = both = False
    st_both = 0.0
    joined_ahead = None
    t_prev = None
    end_t = ta[-1]["t"] - tr0
    end = ta[-1]
    for r in ta:
        t = r["t"] - tr0
        s, sq, va, vq = r["s"], r["os"], r["v"], r["ov"]
        dt = 0.05 if t_prev is None else t - t_prev
        t_prev = t
        if sa_h is None and t >= G.HORIZON - 1e-9:
            sa_h, sq_h = s - s0, [sq - sq0]
        for k, vv in enumerate((va, vq)):
            hq = vh[k]
            hq.append((t, vv))
            while t - hq[0][0] > PEAK_W + 1e-9:
                hq.popleft()
            if t - hq[0][0] >= PEAK_W - 0.026:
                peaks[k] = max(peaks[k], (hq[0][1] - vv) / (t - hq[0][0]))
        pa = (r["x"], r["y"], r["yaw"], va * math.cos(r["yaw"]), va * math.sin(r["yaw"]), la, wa)
        pq = (r["ox"], r["oy"], r["oyaw"], vq * math.cos(r["oyaw"]), vq * math.sin(r["oyaw"]), lb, wb)
        meas.push_poses(t, pa, pq, kappa=(r["kappa"], r["okappa"]))
        if meas.clear <= CONTACT_EPS:
            contact = True
        if joined_ahead is None and s >= G.A_JOINED:
            joined_ahead = G.a_ring_s(s) + G.L / 2 > sq
        if va < G.WAIT_V and vq < G.WAIT_V and s < G.A_IN and sq < G.SZ0:
            st_both += dt
            both = both or st_both >= 0.5
        else:
            st_both = 0.0
        if contact or t >= G.T_MAX - 1e-9 or (t >= G.HORIZON - 1e-9 and s >= G.A_JOINED + 5):
            end_t, end = t, r
            break
    if sa_h is None:
        sa_h, sq_h = end["s"] - s0, [end["os"] - sq0]
    merged = end["s"] >= G.A_JOINED
    lat = [meas.lat_peak[0], meas.lat_peak[1]]
    u, lost = G.score(1, V, contact, merged, sa_h, sq_h, peaks, lat, [meas])
    # B's safety layer over the scored window, from B's own log (aligned on world time)
    shield = any(r.get("shield") for r in rb if r.get("phase") == "run" and st_b["t0"] + r["t"] <= w0 + end_t + 1e-6)
    a_ahead = bool(joined_ahead) and not contact
    forced = a_ahead and (shield or (peaks[1] > X.YIELD_SEEN and b_rule != "slow"))
    pet = meas._pet()
    started = next((r["started"] for r in ta if r.get("started") is not None), None)
    return {"u0": round(u[0], 4), "u1": round(u[1], 4), "contact": int(contact), "a_ahead": int(a_ahead), "forced": int(forced),
            "both_stopped": int(both), "a_went": "" if started is None else round(started, 2),
            "pet": round(pet, 3) if math.isfinite(pet) else "inf", "clear": round(meas.clear, 3), "lostA": round(lost[0], 4),
            "lat_peakA": round(lat[0], 2), "lat_peakB": round(lat[1], 2), **real, "t_run": round(end_t, 2), "valid": int(valid)}


# --- runs ---------------------------------------------------------------------------------------------
def run_one(out, fr, lanes, tb, a, b, seed, port, relay):
    tag = f"tb{tb:g}_{a}_{b}_s{seed}"
    rd = Path(out) / tag
    rd.mkdir(parents=True, exist_ok=True)
    xosc = rd / "u2.xosc"
    scenario(xosc, fr, tb, tag)
    procs = []
    for ent, other, side in (("coop_a", "coop_b", "A"), ("coop_b", "coop_a", "B")):
        cfg = {"entity": ent, "other": other, "side": side, "tb": tb, "v": V, "warmup": WARMUP, "a": a, "b": b, "seed": seed,
               "lanes": str(Path(lanes).resolve()), "log": str(rd / f"{ent}.jsonl"), "carla_port": port, "find_timeout": 60,
               "carla_pythonapi": os.path.join(os.environ.get("CARLA_DIR", "/mnt/wd/aeon/carla-simulator/CARLA_0.9.16"),
                                               "PythonAPI", "carla")}
        (rd / f"{ent}.json").write_text(json.dumps(cfg))
        procs.append(subprocess.Popen([sys.executable, str(ROOT / "scripts/coop/roundabout_agent.py"), "--relay", relay,
                                       "--entity", ent, "--config", str(rd / f"{ent}.json")],
                                      stdout=open(rd / f"{ent}.log", "w"), stderr=subprocess.STDOUT, start_new_session=True))
    time.sleep(3)
    env = dict(os.environ, CARLA_PORT=str(port))
    t_wall = time.monotonic()
    r = subprocess.run(["just", "scenario-direct", str(xosc), f"output_directory:={rd}"], cwd=ROOT, env=env,
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
    res = {"n": 1, "h": 1.5, "tb": tb, "v": V, "a": a, "b": b, "seed": seed, "scenario_exit": r.returncode, "wall_s": round(wall, 1)}
    try:
        res.update(score(rd / "coop_a.jsonl", rd / "coop_b.jsonl", tb, b))
    except Exception as e:  # noqa: BLE001 -- recorded with the run
        res["error"] = repr(e)
    return res


# --- the comparison ---------------------------------------------------------------------------------
def compare(rows, arena_dir):
    import csv

    lags = X.car_lags()
    arena = {}
    for r in csv.DictReader(open(os.path.join(arena_dir, "runs.csv"))):
        arena.setdefault((float(r["tb"]), r["a"], r["b"]), []).append(r)
    swb = {}
    for r in rows:
        if r.get("valid") and "error" not in r:
            swb.setdefault((float(r["tb"]), r["a"], r["b"]), []).append(r)
    lines, inside, total, agree_a, agree_k, n_out = [], 0, 0, 0, 0, 0
    # constant offsets (the swb median minus the arena mean, per player), as roundabout_arena's report corrects
    diffs = ([], [])
    for key, ss in swb.items():
        ar = arena.get(key, [])
        for k in (0, 1):
            if ar:
                mu = st.mean(float(r[f"u{k}"]) for r in ar)
                diffs[k].extend(float(r[f"u{k}"]) - mu for r in ss)
    off = [st.median(d) if d else 0.0 for d in diffs]
    inside_c = 0
    for tb in TB:
        for a, b in PAIRS:
            ss, ar = swb.get((tb, a, b), []), arena.get((tb, a, b), [])
            if not ss or not ar:
                lines.append(f"tb {tb:4g} {a:>5}/{b:<5} | {'no valid swb runs' if not ss else 'no arena runs'}")
                continue
            ra = next(x for x in G.A_RULES if x[0] == a)
            ko = G.simulate(ra, b, 1, 1.5, V, tb, lags)
            maj = lambda rs, k: round(st.mean(int(r[k]) for r in rs))  # noqa: E731
            hits = []
            for k in (0, 1):
                xs = [float(r[f"u{k}"]) for r in ar]
                mu, sd = st.mean(xs), (st.pstdev(xs) if len(xs) > 1 else 0.0)
                for r in ss:
                    h = abs(float(r[f"u{k}"]) - mu) <= max(2 * sd, 0.02)
                    inside_c += abs(float(r[f"u{k}"]) - off[k] - mu) <= max(2 * sd, 0.02)
                    inside += h
                    total += 1
                    hits.append(h)
            oa = tuple(maj(ar, k) for k in ("a_ahead", "forced", "contact"))
            os_ = tuple(maj(ss, k) for k in ("a_ahead", "forced", "contact"))
            ok = (int(ko.a_ahead), int(ko.forced), int(ko.contact))
            agree_a += oa == os_
            agree_k += ok == os_
            n_out += 1
            us = [st.mean(float(r[f"u{k}"]) for r in ss) for k in (0, 1)]
            ua = [st.mean(float(r[f"u{k}"]) for r in ar) for k in (0, 1)]
            lines.append(f"tb {tb:4g} {a:>5}/{b:<5} | swb ({len(ss)}) uA {us[0]:+.3f} uB {us[1]:+.3f} out {os_} "
                         f"PET {st.median(float(r['pet']) for r in ss):.2f} | arena ({len(ar)}) uA {ua[0]:+.3f} uB {ua[1]:+.3f} "
                         f"out {oa} | kin uA {ko.u[0]:+.3f} uB {ko.u[1]:+.3f} out {ok} | payoffs in {sum(hits)}/{len(hits)}"
                         f"{'' if oa == os_ else ' (outcome differs from the arena)'}")
    lines.append(f"swb payoffs inside the arena's interval: {inside} of {total} ({100 * inside / max(total, 1):.0f} %); outcomes "
                 f"(A ahead, forced, contact) agree with the arena in {agree_a} of {n_out}, with the kinematic model in {agree_k} of {n_out}")
    lines.append(f"after constant offsets (A {off[0]:+.3f}, B {off[1]:+.3f}): {inside_c} of {total} "
                 f"({100 * inside_c / max(total, 1):.0f} %)")
    return lines, {"payoffs_inside": inside, "payoffs_inside_offset": inside_c, "offsets": off, "payoffs": total, "outcomes_arena": agree_a, "outcomes_kinematic": agree_k, "pairs": n_out}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--lanes", required=True, help="roundabout_town03.py's JSON")
    ap.add_argument("--carla-port", type=int, default=2600)
    ap.add_argument("--relay", default=f"tcp://localhost:{os.environ.get('AGENT_PORT', '5560')}")
    ap.add_argument("--seeds", type=int, default=2)
    ap.add_argument("--only", help="tb:a:b:seed,... instead of every run")
    ap.add_argument("--redo-invalid", type=int, default=1, help="times to redo a run whose warm-up missed its context")
    ap.add_argument("--compare", metavar="ARENA_DIR", help="instead: compare the runs in --out with this arena run")
    ap.add_argument("--rescore", action="store_true", help="instead: score the runs in --out again from their logs")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    _, _, fr = T.install(T.load(a.lanes))
    res_path = os.path.join(a.out, "u2_swb.jsonl")
    if a.rescore or a.compare:
        rows = [json.loads(line) for line in open(res_path) if line.strip()]
        if a.rescore:
            for r in rows:
                rd = Path(a.out) / f"tb{r['tb']:g}_{r['a']}_{r['b']}_s{r['seed']}"
                r.update(score(rd / "coop_a.jsonl", rd / "coop_b.jsonl", r["tb"], r["b"]))
            with open(res_path, "w") as f:
                for r in rows:
                    f.write(json.dumps(r) + "\n")
        if a.compare:
            lines, summ = compare(rows, a.compare)
            Path(a.out, "compare.txt").write_text("\n".join(lines) + "\n")
            Path(a.out, "summary.json").write_text(json.dumps(summ, indent=1))
            print("\n".join(lines))
        return 0
    if a.only:
        runs = [(float(x.split(":")[0]), x.split(":")[1], x.split(":")[2], int(x.split(":")[3])) for x in a.only.split(",")]
    else:
        runs = [(tb, pa, pb, s) for tb in TB for pa, pb in PAIRS for s in range(a.seeds)]
    with open(res_path, "a") as f:
        for tb, pa, pb, s in runs:
            for attempt in range(1 + a.redo_invalid):
                res = run_one(a.out, fr, a.lanes, tb, pa, pb, s, a.carla_port, a.relay)
                res["attempt"] = attempt
                print(json.dumps(res), flush=True)
                if res.get("valid") or attempt == a.redo_invalid:
                    break
            f.write(json.dumps(res) + "\n")
            f.flush()
    return 0


if __name__ == "__main__":
    sys.exit(main())
