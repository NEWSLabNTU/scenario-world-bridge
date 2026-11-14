#!/usr/bin/env python3
"""Roadmap 074 S5 on the real stack: the unprotected left turn, B the Autoware ego (CenterPoint).

Town03's junction 861, unsignalized for Autoware (Town03's Lanelet2 map links none of its 38
traffic lights to a lanelet; E5). B starts 70 m north of the junction on road 4 lane 2 and is
routed left onto road 62 lane -1, crossing both northbound lanes. A (oncoming_agent.py) comes
north in the inner lane at v and goes straight; it times its arrival at the conflict so that it
reaches it `gap` s after B's front reaches B's stop line, then holds its speed (left_turn_game's
C1 "hold", with its safety layer). Every run starts a fresh ego stack, as c3.py does.

Scored from A's log (both cars' truth, CARLA's frame), with S0's utility (measures.push_poses,
B's route curvature as kappa) over [0, H] from the run's start (B 4 s from its stop line):
- who crossed first ("go": B's centre across A's lane centre before A's centre passed the
  crossing); whether B stopped before the junction. Autoware's decision itself (yield when the
  intersection module stops B) is s5_table.py's, from the bag's intersection factors
- the lag at B's decision: A's time to the conflict (distance over speed) when B's front reached
  its stop line, or when B stopped short of it, whichever was first
- PET, clearance, contact, each car's peak braking, uA, uB
From the run's bag: when B's stack first tracked A (an object within 2.5 m of A's truth) and the
intersection module's stop factors (first and last). Each run is also put to left_turn_game (U1)
at its lag and v: U1's best threshold G* against "hold", its decision at that lag, its PET.

  scripts/coop/s5_aw.py --out <dir> [--v 10] [--gap 2,3,4,5,6,7,8,9] [--seeds 2] [--seed0 0]
  scripts/coop/s5_aw.py --out <dir> --rescore
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
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import c3 as C3  # noqa: E402
import cutout_game as CG  # noqa: E402
import left_turn_game as LT  # noqa: E402
import measures as M  # noqa: E402
import s1_aw as S1  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
# SSv2's frame (x, -y_carla; heading -yaw): B 70 m before its stop line, its goal 25 m into road 62;
# A in the inner northbound lane, as far back as its gap needs (a_start), its goal on road 4 lane -1
B_START, B_GOAL = (-77.6, -98.5, 1.5743), (-122.7, 3.25, 3.1364)
A_GOAL = (-84.62, -88.6, -1.5673)
Y_C = 2.6  # SSv2 y of the conflict point C (C_BACK before the crossing of the centre lines)
A_HALF = 2.4  # m, half A's length


def a_start(v, gap):
    """A far enough back to launch while B approaches (run-up 25 m, then gap + 6 s at v, 10 m
    spare), at most 170 (the straight south of junction 730)."""
    y = min(170.0, Y_C + A_HALF + 25.0 + v * (gap + 6.0) + 10.0)
    if 125.0 < y < 157.0:  # not in junction 730: a start there took a turning lane (074 S5, gap 5)
        y = 157.0
    return (round(C3.x_at(C3.X_LEFT, y), 2), round(y, 1), -1.5673)
PERC_R = 2.5
# S1's topics, and the lidar detector's own timing (lidar_centerpoint or lidar_transfusion, debug/*_ms)
BAG_TOPICS = S1.BAG_TOPICS + "|^/perception/object_recognition/detection/(centerpoint|transfusion)/lidar_[a-z]+/debug/[a-z_]+_ms$"
AGENT = []


def _stop(signum, frame):
    for p in AGENT:
        try:
            os.killpg(p.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    sys.exit(128 + signum)


def scenario(path, v, gap, t_end):
    # the cars' acceleration limits are SSv2's sanity check on each entity; a contact here is a
    # result, so they are set past any impact (at 15 m/s^2 SSv2 ended the run on the first touch)
    wp = lambda p, z: f'<WorldPosition x="{p[0]}" y="{p[1]}" z="{z}" h="{p[2]:.4f}" p="0" r="0" />'
    car = '''<CatalogReference catalogName="vehicles" entryName="tesla_model3">
				<ParameterAssignments>
					<ParameterAssignment parameterRef="maxSpeed" value="50" />
					<ParameterAssignment parameterRef="maxAcceleration" value="200.0" />
					<ParameterAssignment parameterRef="maxDeceleration" value="200.0" />
					<ParameterAssignment parameterRef="maxSteering" value="0.5" />
				</ParameterAssignments>
			</CatalogReference>'''
    Path(path).write_text(f'''<?xml version="1.0"?>
<!-- Roadmap 074 S5: the unprotected left turn at Town03 junction 861, B the Autoware ego, A a light agent oncoming; v {v} gap {gap}; written by scripts/coop/s5_aw.py -->
<OpenSCENARIO>
	<FileHeader author="scenario-world-bridge" date="2026-11-15T00:00:00+00:00" description="074 S5 left turn v {v} gap {gap}" revMajor="1" revMinor="2" />
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
		<ScenarioObject name="coop_a">
			{car}
			<ObjectController><Controller name="agent"><Properties /></Controller></ObjectController>
		</ScenarioObject>
	</Entities>
	<Storyboard>
		<Init><Actions>
			<Private entityRef="ego">
				<PrivateAction><TeleportAction><Position>{wp(B_START, 0.0)}</Position></TeleportAction></PrivateAction>
				<PrivateAction><RoutingAction><AcquirePositionAction><Position>{wp(B_GOAL, 0.0)}</Position></AcquirePositionAction></RoutingAction></PrivateAction>
			</Private>
			<Private entityRef="coop_a">
				<PrivateAction><TeleportAction><Position>{wp(a_start(v, gap), 0.3)}</Position></TeleportAction></PrivateAction>
				<PrivateAction><RoutingAction><AcquirePositionAction><Position>{wp(A_GOAL, 0.0)}</Position></AcquirePositionAction></RoutingAction></PrivateAction>
			</Private>
		</Actions></Init>
		<Story name="s5"><Act name="s5_act">
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
def score(log):
    rows, ev = S1.load_log(log)
    if "setup" not in ev:
        return {"error": "no setup"}
    su = ev["setup"]
    ext = su["ext"]
    run = [r for r in rows if r.get("tr") is not None and r["tr"] <= M.HORIZON + 1e-6]
    if "start" not in ev or len(run) < 10:
        return {"error": "no run phase", "rows": len(rows)}
    lane = su["lane"]
    hx, hy = math.cos(math.radians(lane[4])), math.sin(math.radians(lane[4]))
    fwd = lambda x, y: (x - lane[2]) * hx + (y - lane[3]) * hy
    path = su["path"]
    k = min(range(1, len(path)), key=lambda i: abs(-(path[i][0] - lane[2]) * hy + (path[i][1] - lane[3]) * hx))
    c_cross = fwd(path[k][0], path[k][1])  # the crossing of the centre lines, along A's lane
    meas = M.Measures(b_max=(CG.B_MAX_B, CG.B_MAX_B))
    vh = {"A": deque(), "B": deque()}
    peak = {"A": 0.0, "B": 0.0}
    t_brake = {"A": None, "B": None}
    contact = False
    t_bx = t_ax = None  # B's centre across A's lane centre; A's centre past the crossing
    t_dec = lag = None
    stopped_before = False
    slow = None
    for r in run:
        t = r["tr"]
        for c in "AB":
            sp = math.hypot(r[c][3], r[c][4])
            h = vh[c]
            h.append((t, sp))
            while t - h[0][0] > CG.PEAK_W + 1e-6:
                h.popleft()
            if t - h[0][0] >= CG.PEAK_W - 0.06:
                dec = (h[0][1] - sp) / (t - h[0][0])
                peak[c] = max(peak[c], dec)
                if t_brake[c] is None and dec >= M.BRAKE_START:
                    t_brake[c] = h[0][0]
        meas.push_poses(t, (*r["A"], *ext["A"]), (*r["B"], *ext["B"]), kappa=(0.0, r.get("kB", 0.0)))
        if meas.clear <= 0.0:
            contact = True
        if t_bx is None and r["lat_b"] >= 0.0 and r["d_b"] < 0:  # B comes from the east: lat < 0
            t_bx = t
        if t_ax is None and fwd(r["A"][0], r["A"][1]) >= c_cross:
            t_ax = t
        vb = math.hypot(r["B"][3], r["B"][4])
        va = math.hypot(r["A"][3], r["A"][4])
        slow = (slow if slow is not None else t) if vb < 0.3 else None
        if t_dec is None and ((r["d_b"] <= 0) or (slow is not None and t - slow >= 0.5 and r["d_b"] < 25)):
            t_dec = t
            lag = r["d_a"] / max(va, 0.1) if r["d_a"] > 0 else -1.0
            stopped_before = r["d_b"] > 0
    meas.pet = meas._pet()
    go = t_bx is not None and (t_ax is None or t_bx < t_ax)
    rho = M.rho(meas.ttc2d, meas.thw, meas.pet, meas.clear)
    st = ev["start"]
    va0, vb0 = su["v"], st["vb"]  # A is scored against its own v, B against its speed at t = 0
    lost_a = 1.0 if contact else M.lost_progress(va0, meas.s_h[0] if meas.s_h[0] is not None else meas.s[0])
    lost_b = 1.0 if contact else M.lost_progress(max(vb0, 1.0), meas.s_h[1] if meas.s_h[1] is not None else meas.s[1])
    u_a = M.utility(lost_a, rho, peak["A"], CG.B_MAX_B, meas.lat_peak[0], contact)
    u_b = M.utility(lost_b, rho, peak["B"], CG.B_MAX_B, meas.lat_peak[1], contact)
    rnd = lambda x, n=3: None if x is None else (round(x, n) if math.isfinite(x) else None)
    vb_min = min(math.hypot(r["B"][3], r["B"][4]) for r in run)
    return {"go": int(go), "stopped_before": int(stopped_before), "lag": rnd(lag, 2), "t_dec": rnd(t_dec, 2),
            "gap_achieved": rnd(ev["lock"]["lag"], 2) if "lock" in ev else None, "vA0": rnd(va0, 2), "vB0": rnd(vb0, 2), "vB_min": rnd(vb_min, 2),
            "t_bx": rnd(t_bx, 2), "t_ax": rnd(t_ax, 2), "pet": rnd(meas.pet, 2), "clear": rnd(meas.clear, 2),
            "ttc2d": rnd(meas.ttc2d, 2), "contact": int(contact), "rho": rnd(rho), "peakA": rnd(peak["A"], 2),
            "peakB": rnd(peak["B"], 2), "latB": rnd(meas.lat_peak[1], 2), "t_brakeA": rnd(t_brake["A"], 2),
            "t_brakeB": rnd(t_brake["B"], 2), "lostA": rnd(lost_a), "lostB": rnd(lost_b), "uA": rnd(u_a), "uB": rnd(u_b),
            "t_end": rnd(run[-1]["tr"], 2)}


def perception(rd, log):
    """When B's stack first tracked A (an object within PERC_R of A's truth), on each object
    topic, and the intersection module's stop factors; in the run's time, matched to A's log by
    B's pose on its straight approach (CARLA y > 30)."""
    rows, ev = S1.load_log(log)
    bag = Path(rd) / "bag"
    if not bag.exists() or "start" not in ev:
        return {}
    r0 = next(r for r in rows if r.get("tr") is not None)
    el0 = r0["el"] - r0["tr"]
    by_y = sorted((-r["B"][1], r["el"]) for r in rows if math.hypot(r["B"][3], r["B"][4]) > 1.0 and r["B"][1] > 30)
    stamp = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
    msgs = list(S1._bag_rows(bag))
    offs = []
    for tp, m in msgs:
        if tp == "/localization/kinematic_state" and m.twist.twist.linear.x > 1.0 and by_y:
            y = m.pose.pose.position.y  # map y = -carla y; by_y holds -carla y
            if -y <= 30:
                continue
            i = bisect.bisect_left(by_y, (y, -1e18))
            if 0 < i < len(by_y):
                (y0, e0), (y1, e1) = by_y[i - 1], by_y[i]
                if 1e-6 < y1 - y0 < 2.0:
                    offs.append(stamp(m.header) - (e0 + (e1 - e0) * (y - y0) / (y1 - y0)))
    if not offs:
        return {"perc_error": "no clock match"}
    offs.sort()
    off = offs[len(offs) // 2]
    to_tr = lambda s: round(s - off - el0, 2)
    a_at = sorted((r["el"], r["A"][0], -r["A"][1]) for r in rows)  # A's truth in the map frame, by CARLA time
    a_t = [a[0] for a in a_at]
    ego = []
    for tp, m in msgs:
        if tp == "/localization/kinematic_state":
            q = m.pose.pose.orientation
            ego.append((stamp(m.header), m.pose.pose.position.x, m.pose.pose.position.y,
                        math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))
    ego_t = [e[0] for e in ego]

    def to_map(frame, s, p):
        if frame == "map":
            return p.x, p.y
        i = min(max(bisect.bisect_left(ego_t, s), 0), len(ego) - 1)
        _, ex, ey, eyaw = ego[i]
        return ex + p.x * math.cos(eyaw) - p.y * math.sin(eyaw), ey + p.x * math.sin(eyaw) + p.y * math.cos(eyaw)

    first, fac = {}, {}
    for tp, m in msgs:
        if tp.startswith("/perception/") and hasattr(m, "objects") and "/debug/" not in tp:
            s = stamp(m.header)
            j = min(max(bisect.bisect_left(a_t, s - off), 0), len(a_at) - 1)
            _, ax, ay = a_at[j]
            for ob in m.objects:
                k = ob.kinematics
                p = (k.pose_with_covariance if hasattr(k, "pose_with_covariance") else k.initial_pose_with_covariance).pose.position
                mx, my = to_map(m.header.frame_id, s, p)
                if math.hypot(mx - ax, my - ay) < PERC_R:
                    first.setdefault(tp, to_tr(s))
                    break
        elif tp.startswith("/planning/planning_factors/") and m.factors:
            mod = tp.rsplit("/", 1)[-1]
            for f in m.factors:
                d = f.control_points[0].distance if f.control_points else None
                e = fac.setdefault(f"{mod}:{f.behavior}", {"first": to_tr(stamp(m.header)), "d_first": None if d is None else round(d, 1)})
                e["last"] = to_tr(stamp(m.header))
    res = {"clock_offset": round(off, 3),
           "clock_offset_spread": round(offs[int(.9 * len(offs))] - offs[int(.1 * len(offs))], 3)}
    for tp, t in sorted(first.items()):
        res["perc:" + tp.replace("/perception/object_recognition/", "")] = t
    res["factors"] = {k: v for k, v in fac.items() if v["first"] > -30}
    return res


def kinematic(lag, v):
    """left_turn_game at the run's lag and v: A's best threshold against hold (n = 1), its
    decision at this lag, and the outcome."""
    res, eqs, opt, cost = LT.solve(1, 2.0, v)
    g_star = max(LT.GRID_G, key=lambda G: res[(G, "hold")][0])
    out = {"G_star": g_star}
    if lag is not None and lag > 0:
        o = LT.simulate(g_star, "hold", 1, 2.0, v, t_first=lag)
        ua, uc = LT.utilities(o)
        out.update({"go": int(lag >= g_star), "pet": round(o.pet, 2) if math.isfinite(o.pet) else None,
                    "a_delay": round(o.a_delay, 2), "contact": int(o.contact), "uA_u1": round(ua, 3), "uC_u1": round(uc[0], 3)})
    return out


# -- running ---------------------------------------------------------------------------------
def tag_of(v, gap, seed):
    return f"v{v:g}_g{gap:g}_s{seed}"


def run_one(out, v, gap, seed, port, relay):
    rd = Path(out) / tag_of(v, gap, seed)
    rd.mkdir(parents=True, exist_ok=True)
    for old in ("bag", "scenario_test_runner"):
        if (rd / old).exists():
            n = 1
            while (rd / f"{old}.failed{n}").exists():
                n += 1
            (rd / old).rename(rd / f"{old}.failed{n}")
    t_end = 40.0 + M.HORIZON
    xosc = rd / "s5.xosc"
    scenario(xosc, v, gap, t_end)
    base = {"v": v, "gap": gap, "seed": seed}
    if not C3.restart_ego():
        return {**base, "error": "ego not ready"}
    cfg = {"entity": "coop_a", "other": "hero", "v": v, "gap": gap, "seed": seed, "b_vmax": float(os.environ.get("C3_VB", "8.0")), "b_from": [4, 2], "b_to": [62, -1],
           "junction": 861, "a_ref": [-85.0, -30.0], "log": str(rd / "coop_a.jsonl"), "carla_port": port, "find_timeout": 90,
           "carla_pythonapi": os.path.join(os.environ.get("CARLA_DIR", ""), "PythonAPI", "carla")}
    (rd / "coop_a.json").write_text(json.dumps(cfg))
    agent = subprocess.Popen([sys.executable, str(ROOT / "scripts/coop/oncoming_agent.py"), "--relay", relay,
                              "--entity", "coop_a", "--config", str(rd / "coop_a.json")],
                             stdout=open(rd / "coop_a.log", "w"), stderr=subprocess.STDOUT, start_new_session=True)
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
    log = Path(rd) / "coop_a.jsonl"
    if not log.exists():
        res["error"] = "no agent log"
        return res
    res.update(score(log))
    if "error" in res:
        return res
    try:
        res.update(perception(rd, log))
    except Exception as e:  # noqa: BLE001 -- the bag is a diagnostic; the score stands without it
        res["perc_error"] = repr(e)[:200]
    res["kin"] = kinematic(res.get("lag"), res["v"])
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--carla-port", type=int, default=int(os.environ.get("CARLA_PORT", 2000)))
    ap.add_argument("--relay", default=f"tcp://127.0.0.1:{os.environ.get('AGENT_PORT', '5560')}")
    ap.add_argument("--v", type=float, default=10.0, help="A's speed")
    ap.add_argument("--gap", default="2,3,4,5,6,7,8,9")
    ap.add_argument("--seeds", type=int, default=2)
    ap.add_argument("--seed0", type=int, default=0)
    ap.add_argument("--only", type=int, default=0)
    ap.add_argument("--rescore", action="store_true")
    a = ap.parse_args()
    signal.signal(signal.SIGTERM, _stop)
    os.makedirs(a.out, exist_ok=True)
    path = os.path.join(a.out, "s5.jsonl")
    if a.rescore:
        old = [json.loads(line) for line in open(path) if line.strip()]
        with open(path, "w") as f:
            for r in old:
                keep = {k: r[k] for k in ("v", "gap", "seed", "scenario_rc", "attempt") if k in r}
                f.write(json.dumps(finish(Path(a.out) / tag_of(r["v"], r["gap"], r["seed"]), keep)) + "\n")
        return
    runs = [(g, s) for g in map(float, a.gap.split(",")) for s in range(a.seed0, a.seed0 + a.seeds)]
    if a.only:
        runs = runs[: a.only]
    with open(path, "a") as f:
        for g, s in runs:
            for attempt in range(3):
                res = run_one(a.out, a.v, g, s, a.carla_port, a.relay)
                res["attempt"] = attempt
                if "error" not in res:
                    break
                res["cause"] = C3.junit_message(Path(a.out) / tag_of(a.v, g, s))
                print(json.dumps(res), flush=True)
            print(json.dumps(res), flush=True)
            f.write(json.dumps(res) + "\n")
            f.flush()


if __name__ == "__main__":
    main()
