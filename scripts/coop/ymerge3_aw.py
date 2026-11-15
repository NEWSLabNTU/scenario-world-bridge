#!/usr/bin/env python3
"""Roadmap 074 N5 with stock Autoware as B: the three-car Y merge at Town03 j1901 (078 T2's baseline).

B is the Autoware ego (the stack's own perception) on the main road, from rest at B_SPAWN, routed
straight through the junction. A, on the diagonal, and C, behind B at h, are ymerge3_agent.py light
agents (A's rule as N5 swb's). They are paced by B (the agent's "pace" mode): the run starts when
B's front reaches its context's u, and until then each agent's warm-up runs on B's predicted
arrival there, so that A is dt s from Z against B and C h s behind B, all at v, when it does.
Every run starts a fresh ego stack (c3.restart_ego).

Scored from A's log (all three cars' truth) by ymerge3.score3, as n5_swb.py. A run is valid when
dt, h and A's and C's speeds at its start are within n5_swb's tolerances and B's speed (Autoware's)
within 15 % (valid_strict: 10 %, n5_swb's). From the run's bag: when
B's stack first tracked A and C, every planning factor (module, behaviour; first and last time,
nearest stop distance) and B's yield: whether B slowed before A merged (its lowest speed and
hardest braking before M, and when it began).

  scripts/coop/ymerge3_aw.py --out <dir> --lanes <lanes.json> [--seeds 2] [--dts ...] [--only dt:a:seed,...]
  scripts/coop/ymerge3_aw.py --out <dir> --lanes <lanes.json> --rescore
  scripts/coop/ymerge3_aw.py --out <dir> --lanes <lanes.json> --compare <ymerge3_arena dir>

The stack: `just run map_path:=<Town03> vehicle_physics:=<spec's carla_physics.yaml>
ego_launch_args:="max_velocity:=10 lidar_detection_model:=centerpoint"` (ymerge3_aw_run.sh).
"""

import argparse
import bisect
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import c3 as C3  # noqa: E402
import n5_swb as N5  # noqa: E402
import s1_aw as S1  # noqa: E402
import ymerge3 as Y3  # noqa: E402
import ymerge3_town03 as T  # noqa: E402
import ymerge_arena as YA  # noqa: E402
import ymerge_game as Y  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
CARS = "ABC"
ENT = {"A": "coop_a", "B": "hero", "C": "coop_c"}
B_SPAWN = -130.0  # B's front at rest (u): past the main road's last curve (R < 60 m to u -156) and the
# walkway Autoware stops at near u -136 (the next is near u -94)
B_GOAL = 150.0
PERC_R = 2.5
AGENT = []
G_HORIZON = 20.0  # crossing_game.HORIZON
B_VTOL = 0.15
WAIT_V, WAIT_MIN = 0.5, 2.0  # a mutual-yield wait: A and B both below WAIT_V m/s for WAIT_MIN s, the zone empty
WAIT_CRAWL = 1.5  # m/s: the same with Autoware's creep at its stop line counted as waiting
BASE_RULE = "t1.5yp2"  # the baseline's runs carry a rule only for their context; A is parked


def _stop(signum, frame):
    for p in AGENT:
        try:
            os.killpg(p.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    sys.exit(128 + signum)


def scenario(path, lanes, dt, v, h, tag, t_end):
    target = Y3.start_u(dt, v, h)
    car = '''<CatalogReference catalogName="vehicles" entryName="tesla_model3">
				<ParameterAssignments>
					<ParameterAssignment parameterRef="maxSpeed" value="50" />
					<ParameterAssignment parameterRef="maxAcceleration" value="200.0" />
					<ParameterAssignment parameterRef="maxDeceleration" value="200.0" />
					<ParameterAssignment parameterRef="maxSteering" value="0.5" />
				</ParameterAssignments>
			</CatalogReference>'''
    ents, inits = [], []
    for k in CARS:
        ln = lanes[k]
        u0 = B_SPAWN if k == "B" else T.WarmupPlan(k, target[k], v, YA.WARMUP).u_spawn
        x, y, z, yaw = ln.at(ln.s_of_u(u0))
        gx, gy, gz, gyaw = ln.at(ln.s_of_u(B_GOAL if k == "B" else 120.0))
        name = "ego" if k == "B" else ENT[k]
        ctl = ('<Controller name="Autoware"><Properties><Property name="maxJerk" value="1.5" /><Property name="minJerk" value="-1.5" />'
               '</Properties></Controller>') if k == "B" else '<Controller name="agent"><Properties /></Controller>'
        ents.append(f'''		<ScenarioObject name="{name}">
			{car}
			<ObjectController>{ctl}</ObjectController>
		</ScenarioObject>''')
        inits.append(f'''			<Private entityRef="{name}">
				<PrivateAction><TeleportAction><Position>{N5.ros_position(x, y, z, yaw)}</Position></TeleportAction></PrivateAction>
				<PrivateAction><RoutingAction><AcquirePositionAction><Position>{N5.ros_position(gx, gy, gz - 0.3, gyaw)}</Position></AcquirePositionAction></RoutingAction></PrivateAction>
			</Private>''')
    nl = "\n"
    Path(path).write_text(f'''<?xml version="1.0"?>
<!-- Roadmap 074 N5 Autoware B: the Y merge at Town03 j1901, B the Autoware ego, A and C light agents, {tag}; written by scripts/coop/ymerge3_aw.py -->
<OpenSCENARIO>
	<FileHeader author="scenario-world-bridge" date="2026-11-15T00:00:00+00:00" description="074 N5 Autoware B {tag}" revMajor="1" revMinor="2" />
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
		<Story name="n5aw"><Act name="n5aw_act">
			<ManeuverGroup maximumExecutionCount="1" name="end"><Actors selectTriggeringEntities="false"><EntityRef entityRef="ego" /></Actors>
			<Maneuver name="end_m">
				<Event name="done" priority="parallel" maximumExecutionCount="1">
					<Action name="done_a"><UserDefinedAction><CustomCommandAction type="exitSuccess" /></UserDefinedAction></Action>
					<StartTrigger><ConditionGroup><Condition name="time" delay="0" conditionEdge="none"><ByValueCondition><SimulationTimeCondition value="{t_end:.0f}" rule="greaterThan" /></ByValueCondition></Condition></ConditionGroup></StartTrigger>
				</Event>
			</Maneuver></ManeuverGroup>
			<StartTrigger><ConditionGroup><Condition name="start" delay="0" conditionEdge="none"><ByValueCondition><SimulationTimeCondition value="0" rule="greaterThan" /></ByValueCondition></Condition></ConditionGroup></StartTrigger>
		</Act></Story>
		<StopTrigger />
	</Storyboard>
</OpenSCENARIO>
''')


# --- scoring -----------------------------------------------------------------------------------
def load(log):
    rows = [json.loads(line) for line in open(log) if line.strip()]
    ev = {r["event"]: r for r in rows if "event" in r}
    return [r for r in rows if "phase" in r], ev


def score(rd, dt, v, h, park=False):
    rows, ev = load(Path(rd) / "coop_a.jsonl")
    if "start" not in ev or "run" not in ev:
        return {"error": "no run logged" if "start" in ev else "no start logged"}
    run = [r for r in rows if r["phase"] == "run"]
    tr0 = ev["run"]["t"]
    ticks = [(r["t"] - tr0, {k: tuple(r[k]) for k in CARS}) for r in run]
    u0, v0 = ev["run"]["u"], ev["run"]["v"]
    box = {k: tuple(ev["start"]["box"][k]) for k in CARS}
    res = Y3.score3(ticks, {k: v0[k] for k in CARS}, box, eps=0.01)
    # B's time to Z on Autoware's launch from its start speed (T.B_LAUNCH), as A's pacing predicts it
    dt_real = -u0["A"] / max(v0["A"], 0.1) - T.t_reach(-u0["B"], v0["B"], v)
    dt_const = -u0["A"] / max(v0["A"], 0.1) + u0["B"] / max(v0["B"], 0.1)
    h_real = (u0["B"] - u0["C"] - box["B"][0]) / max(v0["C"], 0.1)
    # B's speed at the start is Autoware's (from rest, with its walkway stops before the run; 0.89 v at
    # dt -1 in the trials): 15 % for B, n5_swb's 10 % for A and C; valid_strict is n5_swb's test
    ctx = abs(dt_real - dt) <= 0.25 and abs(h_real - h) <= 0.25 and all(abs(v0[k] - v) <= 0.1 * v for k in "AC")
    if park:  # the baseline: A held at its spawn, so only B's start and C's h and speed count
        ctx = abs(h_real - h) <= 0.25 and abs(v0["C"] - v) <= 0.1 * v
    valid = ctx and abs(v0["B"] - v) <= B_VTOL * v
    strict = ctx and abs(v0["B"] - v) <= 0.1 * v
    # B's yield: before the first of A and B reaches M (u Z_LEN), B's lowest speed, its hardest braking (the speed's drop
    # over PEAK_W, as score3's peak) and when B's speed first fell 0.5 m/s below its start
    t_am = next((t for t, s in ticks if s["A"][0] >= Y.Z_LEN), None)
    t_bm = next((t for t, s in ticks if s["B"][0] >= Y.Z_LEN), None)
    t_pre = min([x for x in (t_am, t_bm) if x is not None], default=None)  # until the first of A and B is at M
    pre = [(t, s) for t, s in ticks if t_pre is None or t <= t_pre]
    vb = [(t, s["B"][1]) for t, s in pre]
    vmin = min(vb, key=lambda x: x[1]) if vb else (None, None)
    t_slow = next((t for t, x in vb if x < v0["B"] - 0.5), None)
    peak = 0.0
    for i, (t, x) in enumerate(vb):
        j = bisect.bisect_left([q[0] for q in vb], t - 1.0)
        if t - vb[j][0] >= 0.95:
            peak = max(peak, (vb[j][1] - x) / (t - vb[j][0]))
    last = run[min(len(run) - 1, int(res["t_run"] / 0.05))].get("rec", {})
    # mutual-yield waits: A and B both below WAIT_V for more than WAIT_MIN s with the zone empty and
    # neither through it yet (both fronts short of its near edge), over the scoring window; a stop
    # after the merge (B's walkway stop at u 53 with A behind it) is not one. Autoware creeps at its
    # intersection stop line (0.4-1.1 m/s), so the same count in a crawl band, WAIT_CRAWL, too
    def waits_below(vw):
        out, since = [], None
        for t, s in ticks:
            if t > G_HORIZON + 1e-9:
                break
            both = s["A"][1] < vw and s["B"][1] < vw and s["A"][0] <= 0.0 and s["B"][0] <= 0.0
            if both and since is None:
                since = t
            elif not both and since is not None:
                if t - since > WAIT_MIN:
                    out.append((round(since, 2), round(t - since, 2)))
                since = None
        if since is not None and ticks and min(ticks[-1][0], G_HORIZON) - since > WAIT_MIN:
            out.append((round(since, 2), round(min(ticks[-1][0], G_HORIZON) - since, 2)))
        return out
    waits, crawl = waits_below(WAIT_V), waits_below(WAIT_CRAWL)
    return {**res, "dt_real": round(dt_real, 3), "dt_const": round(dt_const, 3), "h_real": round(h_real, 3),
            "waits": len(waits), "wait_s": round(sum(w[1] for w in waits), 2), "wait_list": waits,
            "waits_crawl": len(crawl), "wait_crawl_s": round(sum(w[1] for w in crawl), 2), "wait_crawl_list": crawl, **{f"v{k}0": round(v0[k], 2) for k in CARS},
            "valid": int(valid), "valid_strict": int(strict), "A_facings": last.get("facings", ""), "friction": json.dumps(ev["run"].get("friction", {})),
            "t_A_M": None if t_am is None else round(t_am, 2), "t_B_M": None if t_bm is None else round(t_bm, 2), "vB_min_pre": None if vmin[1] is None else round(vmin[1], 2),
            "t_vB_min": None if vmin[0] is None else round(vmin[0], 2), "t_B_slow": None if t_slow is None else round(t_slow, 2),
            "peakB_pre": round(peak, 2), "t_warmup": round(tr0, 2)}


def bag_facts(rd, lanes):
    """From the bag: the clock offset (bag stamp - CARLA elapsed, by B's pose on the main road),
    when B's tracker first held A and C, and every planning factor in run time."""
    rows, ev = load(Path(rd) / "coop_a.jsonl")
    bag = Path(rd) / "bag"
    if not bag.exists() or "run" not in ev:
        return {}
    t0, tr0 = ev["start"]["t0"], ev["run"]["t"]
    by_y = sorted((r["B"][3], r["t"] + t0) for r in rows if r["B"][1] > 1.0 and abs(r["B"][2] + 149.0) < 1.0 and r["B"][3] < 60)
    stamp = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9  # noqa: E731
    msgs = list(S1._bag_rows(bag))
    offs = []
    for tp, m in msgs:
        if tp == "/localization/kinematic_state" and m.twist.twist.linear.x > 1.0 and by_y:
            y = -m.pose.pose.position.y  # CARLA y
            if abs(m.pose.pose.position.x + 149.0) > 1.0 or y >= 60:
                continue
            i = bisect.bisect_left(by_y, (y, -1e18))
            if 0 < i < len(by_y):
                (y0, e0), (y1, e1) = by_y[i - 1], by_y[i]
                if 1e-6 < y1 - y0 < 2.0:
                    offs.append(stamp(m.header) - (e0 + (e1 - e0) * (y - y0) / (y1 - y0)))
    if not offs:
        return {"bag_error": "no clock match"}
    offs.sort()
    off = offs[len(offs) // 2]
    to_run = lambda s: round(s - off - t0 - tr0, 2)  # noqa: E731
    truth = sorted((r["t"] + t0, {k: (r[k][2], -r[k][3]) for k in "AC"}) for r in rows)
    tt = [x[0] for x in truth]
    first, fac = {}, {}
    for tp, m in msgs:
        if tp in ("/perception/object_recognition/tracking/objects", "/perception/object_recognition/objects"):
            s = stamp(m.header)
            j = min(max(bisect.bisect_left(tt, s - off), 0), len(truth) - 1)
            for k, (ax, ay) in truth[j][1].items():
                for ob in m.objects:
                    p = ob.kinematics.pose_with_covariance.pose.position if hasattr(ob.kinematics, "pose_with_covariance") \
                        else ob.kinematics.initial_pose_with_covariance.pose.position
                    if math.hypot(p.x - ax, p.y - ay) < PERC_R:
                        first.setdefault(f"{tp.rsplit('/', 2)[-2]}:{k}", to_run(s))
                        break
        elif tp.startswith("/planning/planning_factors/") and m.factors:
            mod = tp.rsplit("/", 1)[-1]
            for f in m.factors:
                d = f.control_points[0].distance if f.control_points else None
                e = fac.setdefault(f"{mod}:{f.behavior}", {"first": to_run(stamp(m.header)), "d_min": None})
                e["last"] = to_run(stamp(m.header))
                if d is not None:
                    e["d_min"] = round(d if e["d_min"] is None else min(e["d_min"], d), 1)
    return {"clock_offset": round(off, 3), "clock_spread": round(offs[int(.9 * len(offs))] - offs[int(.1 * len(offs))], 3),
            **{f"perc:{k}": v for k, v in sorted(first.items())},
            "factors": {k: v for k, v in fac.items() if v["last"] > -20}}


# --- runs --------------------------------------------------------------------------------------
def tag_of(dt, a, seed, park=False):
    return f"{'base_' if park else ''}dt{dt:g}_{a}_s{seed}"


def run_one(out, lanes_path, lanes, dt, v, h, a, seed, port, relay, park=False):
    rd = Path(out) / tag_of(dt, a, seed, park)
    rd.mkdir(parents=True, exist_ok=True)
    for old in ("bag", "scenario_test_runner"):
        if (rd / old).exists():
            n = 1
            while (rd / f"{old}.failed{n}").exists():
                n += 1
            (rd / old).rename(rd / f"{old}.failed{n}")
    t_end = 40.0 + G_HORIZON + 10.0
    xosc = rd / "n5aw.xosc"
    scenario(xosc, lanes, dt, v, h, tag_of(dt, a, seed, park), t_end)
    base = {"dt": dt, "v": v, "h": h, "a": a, "seed": seed, **({"park": 1} if park else {})}
    if not C3.restart_ego():
        return {**base, "error": "ego not ready"}
    procs = []
    for k in "AC":
        cfg = {"entity": ENT[k], "role": k, "roles": ENT, "pace": "B", "dt": dt, "v": v, "h": h, "warmup": YA.WARMUP, "a": a,
               "seed": seed, "lanes": str(Path(lanes_path).resolve()), "log": str(rd / f"{ENT[k]}.jsonl"), "carla_port": port,
               "find_timeout": 90, "park": park and k == "A", "carla_pythonapi": os.path.join(os.environ.get("CARLA_DIR", ""), "PythonAPI", "carla")}
        (rd / f"{ENT[k]}.json").write_text(json.dumps(cfg))
        procs.append(subprocess.Popen([sys.executable, str(ROOT / "scripts/coop/ymerge3_agent.py"), "--relay", relay,
                                       "--entity", ENT[k], "--config", str(rd / f"{ENT[k]}.json")],
                                      stdout=open(rd / f"{ENT[k]}.log", "w"), stderr=subprocess.STDOUT, start_new_session=True))
    bag = subprocess.Popen(["ros2", "bag", "record", "-s", "mcap", "-o", str(rd / "bag"), "-e", S1.BAG_TOPICS],
                           stdout=open(rd / "bag.log", "w"), stderr=subprocess.STDOUT, start_new_session=True)
    AGENT[:] = procs + [bag]
    time.sleep(3)
    t_wall = time.monotonic()
    r = subprocess.run(["just", "scenario", str(xosc), f"output_directory:={rd}"], cwd=ROOT, env=dict(os.environ, CARLA_PORT=str(port)),
                       stdout=open(rd / "scenario.log", "w"), stderr=subprocess.STDOUT)
    wall = time.monotonic() - t_wall
    for p, sig in [(p, signal.SIGTERM) for p in procs] + [(bag, signal.SIGINT)]:
        try:
            os.killpg(p.pid, sig)
        except ProcessLookupError:
            pass
        try:
            p.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)
            p.wait()
    return finish(rd, lanes, {**base, "scenario_rc": r.returncode, "wall_s": round(wall, 1)})


def finish(rd, lanes, res):
    try:
        res.update(score(rd, res["dt"], res["v"], res["h"], bool(res.get("park"))))
    except Exception as e:  # noqa: BLE001 -- recorded with the run
        res["error"] = repr(e)[:200]
        return res
    try:
        res.update(bag_facts(rd, lanes))
    except Exception as e:  # noqa: BLE001 -- the bag is a diagnostic; the score stands without it
        res["bag_error"] = repr(e)[:200]
    return res


def relative(rows, base):
    """Payoffs net of Autoware's own drive: each car's progress over the window as a share of the
    baseline's (A parked, the same dt's start; mean over its valid runs), lost = 1 - that share,
    clipped to [0, 1]. B and C against the baseline's B and C; A against the baseline's C when A
    slots in behind B (it then follows B through its walkway stop as C does), else against a free
    road (lost as scored). Risk, comfort and contact as scored."""
    import crossing_game as G
    import measures as M
    import statistics as st
    ref = {}
    for b in base:
        if b.get("valid") and "error" not in b:
            ref.setdefault(float(b["dt"]), []).append(b)
    out = []
    for r in rows:
        bs = ref.get(float(r["dt"]))
        if not bs or "error" in r:
            continue
        p = {k: st.mean(1.0 - b[f"lost{k}"] for b in bs) for k in "BC"}
        rel = {}
        for k in "ABC":
            pk = p["C"] if k == "A" and str(r["slot"]) == "1" else (1.0 if k == "A" else p[k])
            rel[k] = 1.0 if r[f"lost{k}"] >= 1.0 else min(1.0, max(0.0, 1.0 - (1.0 - r[f"lost{k}"]) / max(pk, 1e-6)))
        hit = bool(r["contact"])
        q = {**r, **{f"lost{k}_rel": round(rel[k], 4) for k in "ABC"}}
        q["uA"] = round(M.utility(rel["A"], max(r["rhoB"], r["rhoC"]), r["peakA"], G.B_MAX, contact=hit), 4)
        q["uB"] = round(M.utility(rel["B"], r["rhoB"], r["peakB"], G.B_MAX, contact=hit), 4)
        q["uC"] = round(M.utility(rel["C"], r["rhoC"], r["peakC"], G.B_MAX, contact=hit or bool(r.get("rear"))), 4)
        out.append(q)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--lanes", required=True)
    ap.add_argument("--carla-port", type=int, default=int(os.environ.get("CARLA_PORT", 2000)))
    ap.add_argument("--relay", default=f"tcp://127.0.0.1:{os.environ.get('AGENT_PORT', '5560')}")
    ap.add_argument("--seeds", type=int, default=2)
    ap.add_argument("--v", type=float, default=Y3.V)
    ap.add_argument("--h", type=float, default=Y3.H_FOLLOW)
    ap.add_argument("--dts", default=",".join(str(d) for d in Y3.DTS))
    ap.add_argument("--only", help="dt:a:seed,... instead of every run")
    ap.add_argument("--redo-invalid", type=int, default=1)
    ap.add_argument("--rescore", action="store_true")
    ap.add_argument("--compare", metavar="ARENA_DIR")
    ap.add_argument("--baseline", action="store_true", help="the Autoware-alone baseline: A parked, BASE_RULE, every dt")
    a = ap.parse_args()
    signal.signal(signal.SIGTERM, _stop)
    os.makedirs(a.out, exist_ok=True)
    lanes = T.load(a.lanes)
    path = os.path.join(a.out, "n5aw_base.jsonl" if a.baseline else "n5aw.jsonl")
    bpath = os.path.join(a.out, "n5aw_base.jsonl")
    if a.rescore or a.compare:
        rows = [json.loads(line) for line in open(path) if line.strip()]
        if a.rescore:
            keep = ("dt", "v", "h", "a", "seed", "scenario_rc", "wall_s", "attempt")
            keep += ("park",)
            rows = [finish(Path(a.out) / tag_of(r["dt"], r["a"], r["seed"], bool(r.get("park"))), lanes, {k: r[k] for k in keep if k in r})
                    for r in rows]
            with open(path, "w") as f:
                for r in rows:
                    f.write(json.dumps(r) + "\n")
        if a.compare:
            lines, summ = N5.compare(rows, a.compare, a.v, a.h)
            Path(a.out, "compare.txt").write_text("\n".join(lines) + "\n")
            Path(a.out, "summary.json").write_text(json.dumps(summ, indent=1))
            print("\n".join(lines))
            if os.path.exists(bpath) and not a.baseline:
                base = [json.loads(line) for line in open(bpath) if line.strip()]
                lines, summ = N5.compare(relative(rows, base), a.compare, a.v, a.h)
                lines.insert(0, "# payoffs net of the Autoware-alone baseline (ymerge3_aw.relative)")
                Path(a.out, "compare_rel.txt").write_text("\n".join(lines) + "\n")
                Path(a.out, "summary_rel.json").write_text(json.dumps(summ, indent=1))
                print("\n".join(lines))
        return 0
    if a.baseline:
        runs = [(float(dt), BASE_RULE, s) for dt in a.dts.split(",") for s in range(a.seeds)]
    elif a.only:
        runs = [(float(x.split(":")[0]), x.split(":")[1], int(x.split(":")[2])) for x in a.only.split(",")]
    else:
        runs = [(float(dt), r, s) for dt in a.dts.split(",") for r in Y3.A_RULES for s in range(a.seeds)]
    with open(path, "a") as f:
        for dt, rule, s in runs:
            for attempt in range(3):  # an error (a stack that did not engage) is retried, an invalid run redone redo_invalid times
                res = run_one(a.out, a.lanes, lanes, dt, a.v, a.h, rule, s, a.carla_port, a.relay, a.baseline)
                res["attempt"] = attempt
                if "error" in res:
                    res["cause"] = C3.junit_message(Path(a.out) / tag_of(dt, rule, s, a.baseline))
                print(json.dumps(res), flush=True)
                if "error" not in res and (res.get("valid") or attempt >= a.redo_invalid):
                    break
            f.write(json.dumps(res) + "\n")
            f.flush()
    return 0


if __name__ == "__main__":
    sys.exit(main())
