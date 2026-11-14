#!/usr/bin/env python3
"""Roadmap 074 S1 on the real stack: the cut-out with B the Autoware ego (CenterPoint).

Town03's west straight, the left lane (heading -y): B (the ego) starts at y = 120, L
(cutout_agent.py) ahead of it at the cell's headway, O (an SSv2 vehicle, speed 0) stands at
y = Y_O past the junction. L leads B up to speed, then holds v and changes lanes to the right
when its time to O reaches T_L (cutout_game's rules; t = 0 when L's time to O is 8 s). Every
run starts a fresh ego stack, as c3.py does.

Scored from L's log (the three cars' truth, CARLA's frame) as cutout_game.simulate scores: S0's
utility over [0, H], B's risk against O and L, B's peak braking (speed drop over 0.5 s), contact.
From the run's bag: when B first perceives O (an object within 2.5 m of O's truth on the
detection, tracking and final object topics) and the planner's first obstacle factor naming it,
against t_reveal (L's body out of B's lane) and the cut-out's start. Each run is also simulated
in cutout_game at its achieved context (h0, v; look-through off, O stopped, the car's lags) for
B's gaps 0.8 / 1.5 / 2.5 s.

  scripts/coop/s1_aw.py --out <dir> [--v 6] [--h0 1.5,2.5] [--tl 6,2.5] [--seeds 2] [--seed0 0] [--y-o -70]
  scripts/coop/s1_aw.py --out <dir> --rescore
"""

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import c3 as C3  # noqa: E402
import crossing_game as X  # noqa: E402
import cutout_game as CG  # noqa: E402
import measures as M  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
Y_B, Y_O, Y_GOAL = 120.0, -70.0, -110.0
T_LC = 3.0
WARMUP_MIN = 8.0
PERC_R = 2.5  # m: an object this near O's truth is O
BAG_TOPICS = ("^/localization/kinematic_state$|^/control/command/control_cmd$|^/api/planning/velocity_factors$"
              "|^/perception/object_recognition/(detection/.*|tracking/)?objects$|^/planning/planning_factors/.*"
              "|^/system/fail_safe/mrm_state$")
AGENT = []


def _stop(signum, frame):
    for p in AGENT:
        try:
            os.killpg(p.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    sys.exit(128 + signum)


def scenario(path, v, h0, t_end, y_o=Y_O):
    y_l = round(Y_B - (CG.LEN + h0 * v), 2)
    wp = lambda x, y, z: f'<WorldPosition x="{x}" y="{y}" z="{z}" h="{C3.H:.4f}" p="0" r="0" />'
    xl = lambda y: C3.x_at(C3.X_LEFT, y)
    car = '''<CatalogReference catalogName="vehicles" entryName="tesla_model3">
				<ParameterAssignments>
					<ParameterAssignment parameterRef="maxSpeed" value="50" />
					<ParameterAssignment parameterRef="maxAcceleration" value="15.0" />
					<ParameterAssignment parameterRef="maxDeceleration" value="15.0" />
					<ParameterAssignment parameterRef="maxSteering" value="0.5" />
				</ParameterAssignments>
			</CatalogReference>'''
    Path(path).write_text(f'''<?xml version="1.0"?>
<!-- Roadmap 074 S1: the cut-out, B the Autoware ego, L a light agent, O stopped; v {v} h0 {h0}; written by scripts/coop/s1_aw.py -->
<OpenSCENARIO>
	<FileHeader author="scenario-world-bridge" date="2026-11-15T00:00:00+00:00" description="074 S1 cut-out v {v} h0 {h0}" revMajor="1" revMinor="2" />
	<ParameterDeclarations />
	<CatalogLocations><VehicleCatalog><Directory path="$(find-pkg-share sim_vehicle_spec)/catalogs" /></VehicleCatalog></CatalogLocations>
	<RoadNetwork><LogicFile filepath="$(env CARLA_MAPS)/Town03" /></RoadNetwork>
	<Entities>
		<ScenarioObject name="ego">
			{car}
			<ObjectController><Controller name="Autoware"><Properties>
				<Property name="maxJerk" value="1.5" /><Property name="minJerk" value="-1.5" />
			</Properties></Controller></ObjectController>
		</ScenarioObject>
		<ScenarioObject name="coop_l">
			{car}
			<ObjectController><Controller name="agent"><Properties /></Controller></ObjectController>
		</ScenarioObject>
		<ScenarioObject name="coop_o">
			<Vehicle name="coop_o" vehicleCategory="car">
				<BoundingBox><Center x="1.5" y="0.0" z="0.9" /><Dimensions width="2.1" height="1.8" length="4.5" /></BoundingBox>
				<Performance maxSpeed="20" maxAcceleration="15.0" maxDeceleration="15.0" />
				<Axles>
					<FrontAxle maxSteering="0.5" wheelDiameter="0.6" trackWidth="1.8" positionX="3.1" positionZ="0.3" />
					<RearAxle maxSteering="0.0" wheelDiameter="0.6" trackWidth="1.8" positionX="0.0" positionZ="0.3" />
				</Axles>
				<Properties />
			</Vehicle>
			<ObjectController><Controller name=""><Properties></Properties></Controller></ObjectController>
		</ScenarioObject>
	</Entities>
	<Storyboard>
		<Init><Actions>
			<Private entityRef="ego">
				<PrivateAction><TeleportAction><Position>{wp(xl(Y_B), Y_B, 0.0)}</Position></TeleportAction></PrivateAction>
				<PrivateAction><RoutingAction><AcquirePositionAction><Position>{wp(xl(Y_GOAL), Y_GOAL, 0.0)}</Position></AcquirePositionAction></RoutingAction></PrivateAction>
			</Private>
			<Private entityRef="coop_l">
				<PrivateAction><TeleportAction><Position>{wp(xl(y_l), y_l, 0.3)}</Position></TeleportAction></PrivateAction>
				<PrivateAction><RoutingAction><AcquirePositionAction><Position>{wp(C3.x_at(C3.X_RIGHT, Y_GOAL), Y_GOAL, 0.0)}</Position></AcquirePositionAction></RoutingAction></PrivateAction>
			</Private>
			<Private entityRef="coop_o">
				<PrivateAction><TeleportAction><Position>{wp(xl(y_o), y_o, 0.0)}</Position></TeleportAction></PrivateAction>
				<PrivateAction><LongitudinalAction><SpeedAction><SpeedActionDynamics dynamicsShape="step" value="0" dynamicsDimension="time" /><SpeedActionTarget><AbsoluteTargetSpeed value="0.0" /></SpeedActionTarget></SpeedAction></LongitudinalAction></PrivateAction>
			</Private>
		</Actions></Init>
		<Story name="s1"><Act name="s1_act">
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


# -- scoring ---------------------------------------------------------------------------------
def load_log(p):
    rows, ev = [], {}
    for line in open(p):
        r = json.loads(line)
        if "event" in r:
            ev.setdefault(r["event"], r)
        else:
            rows.append(r)
    return rows, ev


def score(log, v):
    """cutout_game.simulate's scoring on the truth in L's log (t = the run's tr)."""
    rows, ev = load_log(log)
    if "setup" not in ev:
        return {"error": "no setup"}
    ext = ev["setup"]["ext"]
    run = [r for r in rows if r.get("tr") is not None and r["tr"] <= CG.HORIZON + 1e-6]
    if "start" not in ev or len(run) < 10:
        return {"error": "no run phase", "rows": len(rows)}
    pairs = {k: M.Measures(b_max=(CG.B_MAX_B, CG.B_MAX_B)) for k in ("BO", "BL", "LO")}
    pose = lambda c, r: (*r[c], *ext[c])
    vh = {"L": deque(), "B": deque()}
    peak = {"L": 0.0, "B": 0.0}
    t_brake = None
    t_reveal = None
    vb_min = math.inf
    contact = False
    for r in run:
        t = r["tr"]
        for k in "LB":
            sp = math.hypot(r[k][3], r[k][4])
            h = vh[k]
            h.append((t, sp))
            while t - h[0][0] > CG.PEAK_W + 1e-6:
                h.popleft()
            if t - h[0][0] >= CG.PEAK_W - 0.06:
                dec = (h[0][1] - sp) / (t - h[0][0])
                peak[k] = max(peak[k], dec)
                if k == "B" and t_brake is None and dec >= M.BRAKE_START and ev.get("cutout") and t >= ev["cutout"]["t"]:
                    t_brake = h[0][0]
        vb_min = min(vb_min, math.hypot(r["B"][3], r["B"][4]))
        if t_reveal is None and r["lat"] >= CG.LAT_CLEAR:
            t_reveal = t
        pairs["BO"].push_poses(t, pose("B", r), pose("O", r))
        pairs["BL"].push_poses(t, pose("B", r), pose("L", r))
        pairs["LO"].push_poses(t, pose("L", r), pose("O", r))
        if any(m.clear <= 0.0 for m in pairs.values()):
            contact = True
    rr = {k: M.rho(m.ttc2d, m.thw, math.inf, m.clear) for k, m in pairs.items()}
    rho_b, rho_l = max(rr["BO"], rr["BL"]), max(rr["LO"], rr["BL"])
    sh = lambda m, i: m.s_h[i] if m.s_h[i] is not None else m.s[i]
    lost_b = 1.0 if contact else M.lost_progress(v, sh(pairs["BO"], 0))
    lost_l = 1.0 if contact else M.lost_progress(v, sh(pairs["LO"], 0))
    u_b = M.utility(lost_b, rho_b, peak["B"], CG.B_MAX_B, 0.0, contact)
    u_l = M.utility(lost_l, rho_l, peak["L"], CG.B_MAX, pairs["LO"].lat_peak[0], contact)
    st = ev["start"]
    last = run[-1]
    rnd = lambda x, n=3: None if x is None else (round(x, n) if math.isfinite(x) else None)
    return {"uB": rnd(u_b), "uL": rnd(u_l), "rhoB": rnd(rho_b), "rhoBO": rnd(rr["BO"]), "rhoBL": rnd(rr["BL"]),
            "contact": int(contact), "clearBO": rnd(pairs["BO"].clear, 2), "clearBL": rnd(pairs["BL"].clear, 2),
            "ttcBO": rnd(pairs["BO"].ttc2d, 2), "thwBO": rnd(pairs["BO"].thw, 2),
            "peakB": rnd(peak["B"], 2), "lostB": rnd(lost_b), "t_brakeB": rnd(t_brake, 2), "vB_min": rnd(vb_min, 2),
            "h0": rnd(st["h0_achieved"], 2), "vB0": rnd(st["vb"], 2), "vL0": rnd(st["v"], 2),
            "t_cutout": rnd(ev["cutout"]["t"], 2) if "cutout" in ev else None, "t_reveal": rnd(t_reveal, 2),
            "t_end": rnd(last["tr"], 2)}


def _bag_rows(bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    ty = {t.name: t.type for t in r.get_all_topics_and_types()}
    while r.has_next():
        tp, d, _ = r.read_next()
        yield tp, deserialize_message(d, get_message(ty[tp]))


def perception(rd, log):
    """When B's stack first perceived O, on each object topic, and the planner's first obstacle
    factor naming it; in the run's time (tr), matched to L's log by B's own pose. O is the object
    within PERC_R of O's truth; its tracked UUIDs name it in the planning factors."""
    import bisect
    rows, ev = load_log(log)
    bag = Path(rd) / "bag"
    if not bag.exists() or "start" not in ev:
        return {}
    r0 = next(r for r in rows if r.get("tr") is not None)
    el0 = r0["el"] - r0["tr"]
    by_y = sorted((r["B"][1], r["el"]) for r in rows if math.hypot(r["B"][3], r["B"][4]) > 1.0)
    o = rows[-1]["O"]
    ox, oy = o[0], -o[1]  # the map frame: CARLA's y flipped
    stamp = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
    msgs = list(_bag_rows(bag))
    offs = []
    for tp, m in msgs:  # the clock offset, ROS stamp - CARLA elapsed, from B's pose
        if tp == "/localization/kinematic_state" and m.twist.twist.linear.x > 1.0 and by_y:
            y = -m.pose.pose.position.y
            i = bisect.bisect_left(by_y, (y, -1e18))
            if 0 < i < len(by_y):
                (y0, e0), (y1, e1) = by_y[i - 1], by_y[i]
                if 1e-6 < y1 - y0 < 2.0:
                    offs.append(stamp(m.header) - (e0 + (e1 - e0) * (y - y0) / (y1 - y0)))
    if not offs:
        return {"perc_error": "no clock match"}
    offs.sort()
    off = offs[len(offs) // 2]
    to_tr = lambda st: round(st - off - el0, 2)
    first, uuids, fac = {}, set(), {}
    ego = []  # (stamp, x, y, yaw): detections come in base_link
    for tp, m in msgs:
        if tp == "/localization/kinematic_state":
            q = m.pose.pose.orientation
            ego.append((stamp(m.header), m.pose.pose.position.x, m.pose.pose.position.y,
                        math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))
    ego_t = [e[0] for e in ego]

    def to_map(frame, st, p):
        if frame == "map":
            return p.x, p.y
        if not ego:
            return math.inf, math.inf
        i = min(max(bisect.bisect_left(ego_t, st), 0), len(ego) - 1)
        _, ex, ey, eyaw = ego[i]
        return ex + p.x * math.cos(eyaw) - p.y * math.sin(eyaw), ey + p.x * math.sin(eyaw) + p.y * math.cos(eyaw)

    for tp, m in msgs:
        if tp.startswith("/perception/") and hasattr(m, "objects") and "/debug/" not in tp:
            for ob in m.objects:
                k = ob.kinematics
                p = (k.pose_with_covariance if hasattr(k, "pose_with_covariance") else k.initial_pose_with_covariance).pose.position
                mx, my = to_map(m.header.frame_id, stamp(m.header), p)
                if math.hypot(mx - ox, my - oy) < PERC_R:
                    first.setdefault(tp, to_tr(stamp(m.header)))
                    if hasattr(ob, "object_id"):
                        uuids.add(bytes(ob.object_id.uuid))
        elif tp.rsplit("/", 1)[-1] in ("obstacle_stop", "obstacle_cruise", "obstacle_slow_down") and uuids:
            for f in m.factors:
                if any(bytes(sf.object_id.uuid) in uuids for sf in f.safety_factors.factors):
                    d = f.control_points[0].distance if f.control_points else None
                    fac.setdefault(f.module, (to_tr(stamp(m.header)), None if d is None else round(d, 1)))
    res = {"clock_offset": round(off, 3),
           "clock_offset_spread": round(offs[int(.9 * len(offs))] - offs[int(.1 * len(offs))], 3)}
    for tp, t in sorted(first.items()):
        res["perc:" + tp.replace("/perception/object_recognition/", "")] = t
    res["factors"] = fac
    return res


def kinematic(h0, v, t_l, lags):
    """cutout_game at the run's achieved context, look-through off and on, B's three gaps."""
    out = {}
    for look in ("off", "on"):
        for tb in CG.B_GAPS:
            o = CG.simulate(t_l, tb, h0, v, "stopped", look, v, T_LC, lags=lags, peak_mode="window")
            out[f"{look}_{tb:g}"] = {"uB": round(o.uB, 3), "rhoB": round(o.rhoB, 3), "contact": int(o.contact),
                                     "clearBO": round(o.clearBO, 2) if math.isfinite(o.clearBO) else None,
                                     "peakB": round(o.peakB, 2)}
    return out


# -- running ---------------------------------------------------------------------------------
def run_one(out, v, h0, t_l, seed, port, relay, y_o):
    tag = f"v{v:g}_h{h0:g}_tl{t_l:g}_s{seed}"
    rd = Path(out) / tag
    rd.mkdir(parents=True, exist_ok=True)
    for old in ("bag", "scenario_test_runner"):
        if (rd / old).exists():
            n = 1
            while (rd / f"{old}.failed{n}").exists():
                n += 1
            (rd / old).rename(rd / f"{old}.failed{n}")
    dist = (Y_B - y_o) - CG.LEN - h0 * v
    t_end = 15.0 + max(0.0, dist - CG.T0_LO * v) / v + CG.HORIZON + 5.0
    xosc = rd / "s1.xosc"
    scenario(xosc, v, h0, t_end, y_o)
    base = {"v": v, "h0_set": h0, "t_l": t_l, "seed": seed, "y_o": y_o}
    if not C3.restart_ego():
        return {**base, "error": "ego not ready"}
    cfg = {"entity": "coop_l", "other": "hero", "obstacle": "coop_o", "v": v, "h0": h0, "t_l": t_l, "t_lc": T_LC,
           "warmup_min": WARMUP_MIN, "seed": seed, "log": str(rd / "coop_l.jsonl"), "carla_port": port,
           "find_timeout": 90,
           "carla_pythonapi": os.path.join(os.environ.get("CARLA_DIR", ""), "PythonAPI", "carla")}
    (rd / "coop_l.json").write_text(json.dumps(cfg))
    agent = subprocess.Popen([sys.executable, str(ROOT / "scripts/coop/cutout_agent.py"), "--relay", relay,
                              "--entity", "coop_l", "--config", str(rd / "coop_l.json")],
                             stdout=open(rd / "coop_l.log", "w"), stderr=subprocess.STDOUT, start_new_session=True)
    bag = subprocess.Popen(["ros2", "bag", "record", "-s", "mcap", "-o", str(rd / "bag"), "-e", BAG_TOPICS],
                           stdout=open(rd / "bag.log", "w"), stderr=subprocess.STDOUT, start_new_session=True)
    AGENT[:] = [agent, bag]
    time.sleep(3)
    env = dict(os.environ, CARLA_PORT=str(port))
    r = subprocess.run(["just", "scenario", str(xosc), f"output_directory:={rd}"], cwd=ROOT, env=env,
                       stdout=open(rd / "scenario.log", "w"), stderr=subprocess.STDOUT)
    for p, sig in ((agent, signal.SIGTERM), (bag, signal.SIGINT)):
        try:
            os.killpg(p.pid, sig)
        except ProcessLookupError:
            pass
        p.wait(timeout=30)
    return finish(rd, {**base, "scenario_rc": r.returncode})


def finish(rd, res):
    log = Path(rd) / "coop_l.jsonl"
    if not log.exists():
        res["error"] = "no agent log"
        return res
    res.update(score(log, res["v"]))
    if "error" in res:
        return res
    try:
        res.update(perception(rd, log))
    except Exception as e:  # noqa: BLE001 -- the bag is a diagnostic; the score stands without it
        res["perc_error"] = repr(e)[:200]
    if res.get("t_cutout") is not None:
        res["t_reveal_path"] = round(res["t_cutout"] + T_LC * math.acos(1 - 2 * CG.LAT_CLEAR) / math.pi, 2)
    res["kin"] = kinematic(res["h0"], res["v"], res["t_l"], X.car_lags())
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--carla-port", type=int, default=int(os.environ.get("CARLA_PORT", 2000)))
    ap.add_argument("--relay", default=f"tcp://127.0.0.1:{os.environ.get('AGENT_PORT', '5560')}")
    ap.add_argument("--v", type=float, default=float(os.environ.get("C3_VB", "6.0")))
    ap.add_argument("--h0", default="1.5,2.5")
    ap.add_argument("--tl", default="6,2.5")
    ap.add_argument("--seeds", type=int, default=2)
    ap.add_argument("--seed0", type=int, default=0)
    ap.add_argument("--only", type=int, default=0)
    ap.add_argument("--y-o", type=float, default=Y_O, help="O's y (SSv2's frame; B starts at 120, the goal is -110)")
    ap.add_argument("--rescore", action="store_true")
    a = ap.parse_args()
    signal.signal(signal.SIGTERM, _stop)
    os.makedirs(a.out, exist_ok=True)
    path = os.path.join(a.out, "s1.jsonl")
    if a.rescore:
        old = [json.loads(line) for line in open(path) if line.strip()]
        with open(path, "w") as f:
            for r in old:
                rd = Path(a.out) / f"v{r['v']:g}_h{r['h0_set']:g}_tl{r['t_l']:g}_s{r['seed']}"
                keep = {k: r[k] for k in ("v", "h0_set", "t_l", "seed", "y_o", "scenario_rc", "attempt") if k in r}
                f.write(json.dumps(finish(rd, keep)) + "\n")
        return
    runs = [(h0, tl, s) for h0 in map(float, a.h0.split(",")) for tl in map(float, a.tl.split(","))
            for s in range(a.seed0, a.seed0 + a.seeds)]
    if a.only:
        runs = runs[: a.only]
    with open(path, "a") as f:
        for h0, tl, s in runs:
            for attempt in range(3):
                res = run_one(a.out, a.v, h0, tl, s, a.carla_port, a.relay, a.y_o)
                res["attempt"] = attempt
                if "error" not in res:
                    break
                res["cause"] = C3.junit_message(Path(a.out) / f"v{a.v:g}_h{h0:g}_tl{tl:g}_s{s}")
                print(json.dumps(res), flush=True)
            print(json.dumps(res), flush=True)
            f.write(json.dumps(res) + "\n")
            f.flush()


if __name__ == "__main__":
    main()
