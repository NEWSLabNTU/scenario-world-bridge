#!/usr/bin/env python3
"""Roadmap 078 Phase 2: RTC on a real conflict at an unsignalled crossing, and the runs EV4 reads.

B is the Autoware ego (full CenterPoint, the spec's physics) on Town05 j979, from rest, straight on;
A is crossing_agent.py, a light agent on the crossing road, paced to reach Z dt s after B
(dt = t_A^Z - t_B^Z; dt > 0: the ego is nearer), with an S2 rule. Each run starts a fresh ego stack
(`just ego-av`, its own process group) with the interaction_strategy node on (arm "on", or "commit": on, with
the czp config directory --commit-config, e.g. the crossing row with `commit: on_move`;
interaction_strategy:=true, Town05's czp zones, its decision log) or off (arm "off": stock), records
a bag of the ego's domain, runs the scenario (`just scenario-direct`) against the long-lived simulation side
(`just run ego_launch:=none`, started by crossing_aw_run.sh), and scores the run from A's log (both
cars' truth): who entered Z first, contact, clearance, PET, 2D TTC, each car's wait before Z, both
stopped. The bag is read by crossing_aw_bag.py (RTC, the node, planning factors, EV4's tracks).

  scripts/coop/crossing_aw.py --out <dir> [--arms on,off] [--dts -1,-0.3,0,0.3,1] [--seeds 0,1]
                              [--rules t1.5p5] [--side left] [--only arm:dt:rule:seed,...]
  scripts/coop/crossing_aw.py --out <dir> --rescore
  scripts/coop/crossing_aw.py --out <dir> --summary   (final results; failed attempts apart, 082's signature per one)

Host load (issue 075's grid): --start-gate CMD runs CMD (a shell command) once the ego is ready and before A, the
bag and the scenario start; it should return when the host is quiet enough, and its last stdout line (JSON) is kept
as the run's "gate". --load-log FILE (lines "<epoch s> <1-min load> <PSI cpu some avg10>", written by the caller
every few seconds) gives each run its in-run load (from the scenario's start to its end); a run whose 1-min load
exceeded --load-max is invalid ("invalid_load": 1) and rerun as an invalid run is (--redo-invalid).

074 U3 on Autoware (the all-way stop): --map <dir> runs the ego, the scenario's LogicFile and czp's zones
(--zones, built from that map) on another Town05 map, the j979 stop-sign variant; A then takes one of
four_way_game's stop-sign rules (--rules stop1y_c1.5,fcfse_c0; crossing_agent.py). Each run also scores
both cars' rest at their stop lines (their order, tie distance), a deadlock (neither car in Z by the end) and
starved (A crossed, the ego never entered Z; an error instead when its bag shows issue 082).
"""

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import crossing_game as G  # noqa: E402
import measures as M  # noqa: E402
import n5_swb as N5  # noqa: E402
import four_way_game as FW  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
MAP = ROOT / "data/carla-autoware-bridge/Town05"
# Town05 j979, CARLA frame. B (the ego) south in ROS (+y in CARLA) on road 5/49 lane -2 (outer);
# A west on road 0..2 lane -2 (outer), from the ego's left ("left", czp zone j108210_108811_109526); or east on
# road 3..1 lane 2 (outer), from the ego's right ("right", zone j108210_108811_109235: lanelet 109235 at ROS
# y -6.34 at the zone; CARLA's lane 2 centre is y 6.61 at x -231, 6.44 at -160, 6.20 at -62; 078 P2b fixed the
# guessed y 7.6, which the goal projected onto a turning lane of the next junction)
SITES = {
    "left": {"b_spawn": (-131.74, -65.0, 90.4), "b_goal": (-131.37, 45.0, 89.5), "b_ref": (-132.0, -20.0),
             "a_spawn": (-31.0, -4.38, 179.9), "a_goal": (-175.0, -4.06, 179.9), "a_ref": (-100.0, -4.2)},
    "right": {"b_spawn": (-131.74, -65.0, 90.4), "b_goal": (-131.37, 45.0, 89.5), "b_ref": (-132.0, -20.0),
              "a_spawn": (-231.0, 6.61, 0.0), "a_goal": (-62.0, 6.2, 0.0), "a_ref": (-160.0, 6.44)},
}
B_VMAX = 8.0  # the ego's max_velocity (A's v too: from rest the ego reaches it before Z)
PHYSICS = ROOT / "src/sim_vehicle_spec/generated/tesla_model3/carla_physics.yaml"
T_END = 75.0  # s of scenario time
DT_TOL = 0.2  # s: a run is valid when its lock's dt is within this of the cell's
EGO_TIMEOUT = 900  # s for the ego stack to become ready
PROCS = []


def _stop(signum, frame):
    for p in PROCS:
        try:
            os.killpg(p.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    sys.exit(128 + signum)


def tag_of(arm, dt, rule, seed, side="left"):
    return f"{arm}_dt{dt:g}_{rule}_s{seed}" + ("" if side == "left" else f"_{side}")


def scenario(path, site, tag, logic=None):
    car = '''<CatalogReference catalogName="vehicles" entryName="tesla_model3">
				<ParameterAssignments>
					<ParameterAssignment parameterRef="maxSpeed" value="50" />
					<ParameterAssignment parameterRef="maxAcceleration" value="200.0" />
					<ParameterAssignment parameterRef="maxDeceleration" value="200.0" />
					<ParameterAssignment parameterRef="maxSteering" value="0.5" />
				</ParameterAssignments>
			</CatalogReference>'''
    ents, inits = [], []
    for name, sp, gl in (("ego", site["b_spawn"], site["b_goal"]), ("coop_a", site["a_spawn"], site["a_goal"])):
        ctl = ('<Controller name="Autoware"><Properties><Property name="maxJerk" value="1.5" /><Property name="minJerk" value="-1.5" />'
               '</Properties></Controller>') if name == "ego" else '<Controller name="agent"><Properties /></Controller>'
        ents.append(f'''		<ScenarioObject name="{name}">
			{car}
			<ObjectController>{ctl}</ObjectController>
		</ScenarioObject>''')
        x, y, yaw = sp[0], sp[1], math.radians(sp[2])
        gx, gy, gyaw = gl[0], gl[1], math.radians(gl[2])
        inits.append(f'''			<Private entityRef="{name}">
				<PrivateAction><TeleportAction><Position>{N5.ros_position(x, y, 0.0, yaw)}</Position></TeleportAction></PrivateAction>
				<PrivateAction><RoutingAction><AcquirePositionAction><Position>{N5.ros_position(gx, gy, -0.3, gyaw)}</Position></AcquirePositionAction></RoutingAction></PrivateAction>
			</Private>''')
    nl = "\n"
    Path(path).write_text(f'''<?xml version="1.0"?>
<!-- Roadmap 078 P2: the unsignalled crossing at Town05 j979, B the Autoware ego, A a light agent, {tag}; written by scripts/coop/crossing_aw.py -->
<OpenSCENARIO>
	<FileHeader author="scenario-world-bridge" date="2026-11-15T00:00:00+00:00" description="078 P2 crossing {tag}" revMajor="1" revMinor="2" />
	<ParameterDeclarations />
	<CatalogLocations><VehicleCatalog><Directory path="$(find-pkg-share sim_vehicle_spec)/catalogs" /></VehicleCatalog></CatalogLocations>
	<RoadNetwork><LogicFile filepath="{logic or '$(env CARLA_MAPS)/Town05'}" /></RoadNetwork>
	<Entities>
{nl.join(ents)}
	</Entities>
	<Storyboard>
		<Init><Actions>
{nl.join(inits)}
		</Actions></Init>
		<Story name="p2x"><Act name="p2x_act">
			<ManeuverGroup maximumExecutionCount="1" name="end"><Actors selectTriggeringEntities="false"><EntityRef entityRef="ego" /></Actors>
			<Maneuver name="end_m">
				<Event name="done" priority="parallel" maximumExecutionCount="1">
					<Action name="done_a"><UserDefinedAction><CustomCommandAction type="exitSuccess" /></UserDefinedAction></Action>
					<StartTrigger><ConditionGroup><Condition name="time" delay="0" conditionEdge="none"><ByValueCondition><SimulationTimeCondition value="{T_END:.0f}" rule="greaterThan" /></ByValueCondition></Condition></ConditionGroup></StartTrigger>
				</Event>
			</Maneuver></ManeuverGroup>
			<StartTrigger><ConditionGroup><Condition name="start" delay="0" conditionEdge="none"><ByValueCondition><SimulationTimeCondition value="0" rule="greaterThan" /></ByValueCondition></Condition></ConditionGroup></StartTrigger>
		</Act></Story>
		<StopTrigger />
	</Storyboard>
</OpenSCENARIO>
''')


# --- scoring from A's log (truth) ---------------------------------------------------------------
def load(log):
    rows = [json.loads(line) for line in open(log) if line.strip()]
    return [r for r in rows if "phase" in r], {r["event"]: r for r in rows if "event" in r}


def score(rd, dt):
    rows, ev = load(Path(rd) / "coop_a.jsonl")
    if "setup" not in ev:
        return {"error": "no setup logged"}
    if "lock" not in ev:
        return {"error": "no lock logged"}
    lk = ev["lock"]
    ext = ev["setup"]["ext"]
    t_lock = lk["t"]
    run = [r for r in rows if r["t"] >= t_lock - 1e-6]
    meas = M.Measures(b_max=(G.B_MAX, G.B_MAX), pet_sample=G.PET_SAMPLE)
    contact = False
    wait = {"A": 0.0, "B": 0.0}
    rest = {"A": 0.0, "B": 0.0}
    both, both_run = 0.0, 0.0
    first = {}
    vmin = {"A": 99.0, "B": 99.0}
    prev = None
    exit_b = G.W_BOX + ext["B"][0]
    exit_a = ev["setup"]["exit"]
    for r in run:
        tr = r["t"] - t_lock
        pa = (*r["A"], *ext["A"])
        pb = (*r["B"], *ext["B"])
        meas.push_poses(tr, pa, pb)
        if meas.clear <= 0.0:
            contact = True
        d = 0.0 if prev is None else tr - prev
        prev = tr
        for k, u, v in (("A", r["ua"], r["va"]), ("B", r["ub"], r["vb"])):
            if u >= 0 and k not in first:
                first[k] = tr
            if u < 0:
                vmin[k] = min(vmin[k], v)
                if v < G.WAIT_V:
                    wait[k] += d
                if v < 0.1:
                    rest[k] += d
        if r["va"] < G.WAIT_V and r["vb"] < G.WAIT_V and r["ua"] < 0 and r["ub"] < 0:
            both_run += d
            both = max(both, both_run)
        else:
            both_run = 0.0
    last = run[-1] if run else None
    fa, fb = first.get("A", math.inf), first.get("B", math.inf)
    who = "" if math.isinf(fa) and math.isinf(fb) else ("A" if fa < fb else "B")
    out_a = next((r["t"] - t_lock for r in run if r["ua"] >= exit_a), None)
    out_b = next((r["t"] - t_lock for r in run if r["ub"] >= exit_b), None)
    pet = meas._pet()
    dt_lock = lk.get("dt_lock")
    off = ev["setup"].get("rest_offset", 0.0)  # A aimed at dt + off (stop-sign rules; crossing_agent.py)
    res = {"first": who, "contact": int(contact), "clear": round(meas.clear, 3), "pet": round(pet, 3) if math.isfinite(pet) else None,
            "ttc2d": round(meas.ttc2d, 3) if math.isfinite(meas.ttc2d) else None,
            "tA_in": None if math.isinf(fa) else round(fa, 2), "tB_in": None if math.isinf(fb) else round(fb, 2),
            "tA_out": None if out_a is None else round(out_a, 2), "tB_out": None if out_b is None else round(out_b, 2),
            "waitA": round(wait["A"], 2), "waitB": round(wait["B"], 2), "restA": round(rest["A"], 2), "restB": round(rest["B"], 2),
            "vminA": round(vmin["A"], 2), "vminB": round(vmin["B"], 2), "both_stopped_s": round(both, 2),
            "dt_lock": dt_lock, "dt_const": lk.get("dt_const"), "vA_lock": lk.get("va"), "vB_lock": lk.get("vb"),
            "aB_lock": lk.get("ab"), "uB_lock": lk.get("ub"), "uA_lock": lk.get("ua"),
            "valid": int(dt_lock is not None and abs(dt_lock - dt - off) <= DT_TOL), "rest_offset": off,
            "both_cleared": int(out_a is not None and out_b is not None),
            "t_run": round(last["t"] - t_lock, 1) if last else None, "physics_off": ev["setup"].get("physics_off"),
            "deadlock": int(math.isinf(fa) and math.isinf(fb)), **rests(run, t_lock, ev["setup"].get("stops") or {})}
    # A crossed and the ego never entered Z: not a deadlock (A got through). A verdict, counted like a deadlock
    # (the strategy's own liveness, 243), unless the bag shows issue 082 -- then finish() makes it an error.
    res["starved"] = int(math.isinf(fb) and not math.isinf(fa))
    return res


def rests(run, t_lock, stops):
    """074 U3: each car's first rest (v < 0.1) within 3 m before its stop line, relative to the lock, and
    their order: rest_d = tA_rest - tB_rest (> 0: B, the ego, stopped first). Empty off the stop-sign map."""
    if not stops:
        return {}
    t = {}
    for r in run:
        for k, u, v in (("A", r["ua"], r["va"]), ("B", r["ub"], r["vb"])):
            if k not in t and v < 0.1 and stops[k] - 3.0 <= u < 0:
                t[k] = r["t"] - t_lock
    ta, tb = t.get("A"), t.get("B")
    return {"tA_rest": None if ta is None else round(ta, 2), "tB_rest": None if tb is None else round(tb, 2),
            "rest_d": None if ta is None or tb is None else round(ta - tb, 2),
            "stopA_u": round(stops["A"], 2), "stopB_u": round(stops["B"], 2)}


# --- runs ---------------------------------------------------------------------------------------
def ego_args(arm, rd, zones, extra, map_dir=MAP):
    a = [f"map_path:={map_dir}", "lidar_detection_model:=centerpoint", f"max_velocity:={B_VMAX:g}",
         f"carla_physics:={PHYSICS}"] + extra
    if arm in ("on", "commit"):
        a += ["interaction_strategy:=true", f"interaction_strategy_zones:={zones}",
              f"interaction_strategy_log:={rd / 'decisions.jsonl'}"]
    return a


def ego_ready(web_port):
    r = subprocess.run(["ros2", "run", "scensim_launch", "ego_ready", "--timeout", "10", "--web-port", str(web_port)],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return r.returncode == 0


def killpg(p, sig=signal.SIGTERM, wait=40):
    try:
        os.killpg(p.pid, sig)
    except ProcessLookupError:
        return
    try:
        p.wait(timeout=wait)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(p.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        p.wait()


BAG_TOPICS = ("^/localization/kinematic_state$|^/control/command/control_cmd$|^/api/planning/velocity_factors$"
              "|^/perception/object_recognition/(tracking/)?objects$|^/planning/planning_factors/(intersection|obstacle_stop|crosswalk|stop_line|blind_spot)$"
              "|^/planning/cooperate_status/.*|^/interaction_strategy/.*|^/system/fail_safe/mrm_state$"
              "|^/planning/scenario_planning/max_velocity_candidates$|^/planning/scenario_planning/current_max_velocity$"
              "|^/autoware/state$|^/api/operation_mode/state$")


def run_gate(cmd, rd):
    """--start-gate: (rc, its last stdout line as JSON or text, seconds waited)."""
    t = time.monotonic()
    r = subprocess.run(cmd, shell=True, cwd=ROOT, stdout=subprocess.PIPE, stderr=open(rd / "gate.log", "w"), text=True)
    lines = [x for x in r.stdout.splitlines() if x.strip()]
    (rd / "gate.out").write_text(r.stdout)
    try:
        last = json.loads(lines[-1]) if lines else None
    except ValueError:
        last = lines[-1]
    return {"rc": r.returncode, "out": last, "waited_s": round(time.monotonic() - t, 1)}


def in_run_load(path, t0, t1):
    """--load-log: the samples between t0 and t1 (epoch s): max and mean 1-min load, max PSI cpu some avg10."""
    xs = []
    try:
        for line in open(path):
            f = line.split()
            if len(f) >= 3:
                try:
                    t, ld, psi = float(f[0]), float(f[1]), float(f[2])
                except ValueError:
                    continue
                if t0 <= t <= t1:
                    xs.append((ld, psi))
    except OSError:
        return {}
    if not xs:
        return {"load_samples": 0}
    return {"load_samples": len(xs), "load_max": max(x[0] for x in xs),
            "load_mean": round(sum(x[0] for x in xs) / len(xs), 1), "psi_max": max(x[1] for x in xs)}


def run_one(out, arm, dt, rule, seed, side, a, gate_load=None):
    tag = tag_of(arm, dt, rule, seed, side)
    rd = Path(out) / tag
    rd.mkdir(parents=True, exist_ok=True)
    for old in ("bag", "scenario_test_runner", "decisions.jsonl", "coop_a.jsonl"):
        if (rd / old).exists():
            n = 1
            while (rd / f"{old}.failed{n}").exists():
                n += 1
            (rd / old).rename(rd / f"{old}.failed{n}")
    site = SITES[side]
    xosc = rd / "p2x.xosc"
    scenario(xosc, site, tag, logic=a.map if a.map != str(MAP) else None)
    base = {"arm": arm, "dt": dt, "rule": rule, "seed": seed, "side": side, "tag": tag}
    t0 = time.monotonic()
    env = dict(os.environ)
    if arm == "commit":  # the node on a czp config of its own (czp.config's CZP_CONFIG): --commit-config
        env["CZP_CONFIG"] = a.commit_config
    ego = subprocess.Popen(["just", "ego-av", *ego_args(arm, rd, a.zones, a.ego_extra.split() if a.ego_extra else [], a.map)],
                           cwd=ROOT, env=env, stdout=open(rd / "ego.log", "w"), stderr=subprocess.STDOUT, start_new_session=True)
    PROCS[:] = [ego]
    ok = False
    while time.monotonic() - t0 < EGO_TIMEOUT:
        time.sleep(10)
        if ego.poll() is not None:
            break
        if ego_ready(a.ego_web_port):
            ok = True
            break
    t_ready = round(time.monotonic() - t0, 1)
    if not ok:
        killpg(ego)
        return {**base, "error": "ego not ready", "t_ready": t_ready}
    time.sleep(5)
    # 074 U3 (133): the gate before the ego's start passed at < max_load, but the ego's own bring-up then took
    # the 1-min load to 117-132 by the scenario's start; so wait again here, ego up and idle, for its burst to decay
    t_gate = time.monotonic()
    # gate_load: the threshold this run waits for (the quiet rerun's is --quiet-load); 0: no gate, no load_gate_ok
    gate_load = a.max_load if gate_load is None else gate_load
    if gate_load:
        base["load_gate_ok"] = int(wait_load(gate_load, hold=30.0, poll=5.0, give_up=600.0))
    base["load_gate_s"] = round(time.monotonic() - t_gate, 1)
    base["mem_avail_gb"], base["swap_gb"] = meminfo()  # at the scenario's start, ego up
    gate = run_gate(a.start_gate, rd) if a.start_gate else None
    cfg = {"entity": "coop_a", "other": "hero", "dt": dt, "v": a.v, "rule": rule, "seed": seed, "b_vmax": B_VMAX,
           "a_ref": list(site["a_ref"]), "b_ref": list(site["b_ref"]), "log": str(rd / "coop_a.jsonl"),
           "carla_port": a.carla_port, "b_right": side == "left",
           "rest_offset": a.rest_offset if any(r[0] == rule for r in FW.RULES) else 0.0}
    (rd / "coop_a.json").write_text(json.dumps(cfg))
    agent = subprocess.Popen([sys.executable, str(ROOT / "scripts/coop/crossing_agent.py"), "--relay", a.relay,
                              "--entity", "coop_a", "--config", str(rd / "coop_a.json")],
                             stdout=open(rd / "coop_a.log", "w"), stderr=subprocess.STDOUT, start_new_session=True)
    bag = subprocess.Popen(["ros2", "bag", "record", "-s", "mcap", "-o", str(rd / "bag"), "-e", BAG_TOPICS],
                           stdout=open(rd / "bag.log", "w"), stderr=subprocess.STDOUT, start_new_session=True)
    PROCS[:] = [ego, agent, bag]
    time.sleep(3)
    tw, tw_epoch = time.monotonic(), time.time()
    sampler = LoadSampler()  # always (074 U3); --load-log's in-run load, when given, overrides its keys below
    r = subprocess.run(["just", "scenario-direct", str(xosc), f"output_directory:={rd}"], cwd=ROOT,
                       stdout=open(rd / "scenario.log", "w"), stderr=subprocess.STDOUT, start_new_session=True)
    wall = round(time.monotonic() - tw, 1)
    base.update(sampler.stop())
    load = in_run_load(a.load_log, tw_epoch, time.time()) if a.load_log else {}
    killpg(agent)
    killpg(bag, signal.SIGINT)
    killpg(ego, signal.SIGTERM, wait=60)
    PROCS[:] = []
    time.sleep(10)
    res = finish(rd, {**base, "scenario_rc": r.returncode, "wall_s": wall, "t_ready": t_ready})
    if gate is not None:
        res["gate"] = gate
    if load:
        res.update(load)
        if load.get("load_max") is not None and load["load_max"] > a.load_max:
            res["valid"], res["invalid_load"] = 0, 1
    return res


class LoadSampler(threading.Thread):
    """The host's 1-min load over a run, every 2 s: load_start, load_mean, load_max (074 U3: 133's rule is a
    gate at 1.5x the cores, the load logged per run, and a contact or deadlock rerun under a quieter load)."""

    def __init__(self):
        super().__init__(daemon=True)
        self.v = [os.getloadavg()[0]]
        self.ev = threading.Event()
        self.start()

    def run(self):
        while not self.ev.wait(2.0):
            self.v.append(os.getloadavg()[0])

    def stop(self):
        self.ev.set()
        self.join()
        return {"load_start": round(self.v[0], 1), "load_mean": round(sum(self.v) / len(self.v), 1),
                "load_max": round(max(self.v), 1)}


def meminfo():
    """(available, swap used) in GiB, from /proc/meminfo."""
    m = {}
    for line in open("/proc/meminfo"):
        k, v = line.split(":", 1)
        m[k] = int(v.split()[0]) / 2**20
    return round(m["MemAvailable"], 2), round(m["SwapTotal"] - m["SwapFree"], 2)


def wait_load(max_load, hold=60.0, poll=20.0, give_up=4 * 3600.0, min_mem=0.0, swap_rise=0.25):
    """Before a run: wait until the host's 1-min load has stayed below max_load for `hold` s (0: no wait).
    A shared host's build burst left the ego unready for 900 s (074 U3 on server139); a run started then
    is lost, so it waits instead (up to give_up, then runs anyway). Returns False when it gave up.
    With min_mem (GiB), the same `hold` also needs MemAvailable >= min_mem and swap use not climbing (up by
    no more than swap_rise GiB over the hold): 074 U3's czp heartbeat gaps came with 3-5 GiB available and
    swap at 42-48 of 48 GiB (133)."""
    if not max_load and not min_mem:
        return True
    t0 = time.monotonic()
    calm = 0.0
    swap0 = None
    while time.monotonic() - t0 < give_up:
        avail, swap = meminfo()
        ok = not max_load or os.getloadavg()[0] < max_load
        if min_mem:
            swap0 = swap if swap0 is None or calm == 0.0 else swap0
            ok = ok and avail >= min_mem and swap - swap0 <= swap_rise
        if ok:
            calm += poll
            if calm >= hold:
                return True
        else:
            calm = 0.0
        time.sleep(poll)
    return False


def limit0(msgs):
    """Issue 082's signature in /planning/scenario_planning/current_max_velocity, msgs [(t, max_velocity, sender)]:
    the selector publishing its value-initialised limit (0 m/s, sender '') after a real one. The time of the first
    such message, or None."""
    seen = False
    for t, v, sender in msgs:
        if v > 0.01:
            seen = True
        elif seen and sender == "":
            return t
    return None


def bag_limit0(bag):
    """limit0 over a run's bag, its time relative to the bag's first message: (1, t) with 082's signature, (0, None)
    without, (None, None) when the bag cannot be read (no ROS, no bag)."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        topic = "/planning/scenario_planning/current_max_velocity"
        r = rosbag2_py.SequentialReader()
        r.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
        t0 = r.get_metadata().starting_time.nanoseconds / 1e9
        r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
        cls = get_message("autoware_internal_planning_msgs/msg/VelocityLimit")
        msgs = []
        while r.has_next():
            _, data, ts = r.read_next()
            m = deserialize_message(data, cls)
            msgs.append((ts / 1e9 - t0, m.max_velocity, m.sender))
    except Exception:  # noqa: BLE001 -- unreadable: unknown, not "no 082"
        return None, None
    t = limit0(msgs)
    return (0, None) if t is None else (1, round(t, 1))


def finish(rd, res):
    try:
        res.update(score(rd, res["dt"]))
    except Exception as e:  # noqa: BLE001 -- recorded with the run
        res["error"] = repr(e)[:200]
    if ("error" in res or res.get("starved")) and (Path(rd) / "bag").exists():
        # 133: whether a failed or starved run was issue 082, in its row (the bag moves to bag.failed<n> when the
        # next attempt starts). A starved ego with 082's signature is a stack fault, not a verdict: an error, retried
        # (on 139's stage 1 one was scored ok). Without it -- or with an unreadable bag -- it stays a counted verdict,
        # so a strategy that holds the ego cannot drop out of the grid through retries.
        res["sig082"], res["sig082_t"] = bag_limit0(Path(rd) / "bag")
        if res.get("starved") and res["sig082"] == 1:
            res["error"] = "ego never entered"
    return res


def summary(path):
    """The stage's runs.jsonl, per tag: the final result (its last counted row) and, apart from it, every failed
    attempt by reason; "ego never entered" attempts listed one by one with whether their bag had 082's signature
    (sig082 0: no -- look at czp's liveness)."""
    rows = [json.loads(line) for line in open(path) if line.strip()]
    final, failed, never = {}, {}, []
    for r in rows:
        if r.get("counted", 1):
            final[r["tag"]] = r
    for r in rows:
        if "error" in r and r is not final.get(r["tag"]):
            failed[r["error"]] = failed.get(r["error"], 0) + 1
        if r.get("starved"):
            fin = final.get(r["tag"], {})
            never.append({"tag": r["tag"], "arm": r.get("arm"), "attempt": r.get("attempt"), "error": r.get("error"),
                          "sig082": r.get("sig082"), "sig082_t": r.get("sig082_t"), "counted": r.get("counted", 1),
                          "final": "this row" if fin is r else
                          fin.get("error") or ("starved" if fin.get("starved") else "ok")})
    errs = {}
    for r in final.values():
        if "error" in r:
            errs[r["error"]] = errs.get(r["error"], 0) + 1
    ok = [r for r in final.values() if "error" not in r]
    return {"tags": len(final), "final_ok": len(ok), "final_errors": errs,
            "contacts": sum(r.get("contact") or 0 for r in ok), "deadlocks": sum(r.get("deadlock") or 0 for r in ok),
            "starved": sum(r.get("starved") or 0 for r in ok),
            "failed_attempts": failed, "ego_never_entered": never,
            # 243: a cell that ran out of attempts with errors only has no verdict; listed so none drops out unseen
            "no_verdict": sorted(t for t, r in final.items() if "error" in r)}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--zones", default="/mnt/p5plus/aeon/coop/p1/zones/Town05/zones.yaml")
    ap.add_argument("--map", default=str(MAP), help="the ego's map_path and the scenario's LogicFile (074 U3: the stop-sign map)")
    ap.add_argument("--carla-port", type=int, default=int(os.environ.get("CARLA_PORT", 2000)))
    ap.add_argument("--relay", default=f"tcp://127.0.0.1:{os.environ.get('AGENT_PORT', '5560')}")
    ap.add_argument("--ego-web-port", type=int, default=int(os.environ.get("SCENSIM_WEB_PORT", 8080)) + 2)
    ap.add_argument("--ego-extra", default="", help="further ego-av arguments, space separated")
    ap.add_argument("--v", type=float, default=B_VMAX)
    ap.add_argument("--commit-config", default="", help="arm 'commit': a czp config directory (CZP_CONFIG) for the node")
    ap.add_argument("--arms", default="on,off")
    ap.add_argument("--dts", default="-1,-0.3,0,0.3,1")
    ap.add_argument("--seeds", default="0,1")
    ap.add_argument("--rules", default="t1.5p5")
    ap.add_argument("--side", default="left")
    ap.add_argument("--only", help="arm:dt:rule:seed[:side],... instead of the grid")
    ap.add_argument("--redo-invalid", type=int, default=1)
    ap.add_argument("--min-mem", type=float, default=0.0,
                    help="with --max-load's wait, also wait for this much MemAvailable (GiB) and swap not climbing (0: no memory gate)")
    ap.add_argument("--max-load", type=float, default=0.0, help="wait before each run until the 1-min load stays below this (0: no wait)")
    ap.add_argument("--rest-offset", type=float, default=1.4,
                    help="stop-sign rules: s added to A's pacing aim so rest_d lands on dt (074 U3 pace stage: 1.4)")
    ap.add_argument("--quiet-load", type=float, default=float(os.cpu_count() or 32),
                    help="a contact, deadlock or starved run whose load_max reached this is rerun once, gated below it; "
                         "only the rerun counts (counted 0 on the first)")
    ap.add_argument("--rescore", action="store_true")
    ap.add_argument("--summary", action="store_true", help="print the stage's summary (final results, failed attempts) as JSON")
    ap.add_argument("--start-gate", default="", help="a shell command run after the ego is ready, before the scenario")
    ap.add_argument("--load-log", default="", help="the host's load samples: <epoch s> <1-min load> <PSI cpu avg10>")
    ap.add_argument("--load-max", type=float, default=64.0, help="a run whose in-run 1-min load exceeds this is invalid")
    a = ap.parse_args()
    signal.signal(signal.SIGTERM, _stop)
    os.makedirs(a.out, exist_ok=True)
    path = Path(a.out) / "runs.jsonl"
    if a.summary:
        print(json.dumps(summary(path), indent=1))
        return 0
    if a.rescore:
        rows = [json.loads(line) for line in open(path) if line.strip()]
        keep = ("arm", "dt", "rule", "seed", "side", "tag", "scenario_rc", "wall_s", "t_ready", "attempt", "gate",
                "load_samples", "load_max", "load_mean", "psi_max", "invalid_load")
        rows = [finish(Path(a.out) / r["tag"], {k: r[k] for k in keep if k in r}) for r in rows]
        for r in rows:
            if r.get("invalid_load"):
                r["valid"] = 0
        path.write_text("".join(json.dumps(r) + "\n" for r in rows))
        return 0
    if a.only:
        runs = []
        for x in a.only.split(","):
            p = x.split(":")
            runs.append((p[0], float(p[1]), p[2], int(p[3]), p[4] if len(p) > 4 else a.side))
    else:
        # interleaved: per (dt, seed), the arms back to back, so drift in the host load hits both
        runs = [(arm, float(dt), rule, int(s), a.side) for s in a.seeds.split(",") for dt in a.dts.split(",")
                for rule in a.rules.split(",") for arm in a.arms.split(",")]
    done = set()
    if path.exists():
        for line in open(path):
            if line.strip():
                r = json.loads(line)
                if "error" not in r and r.get("counted", 1) and (r.get("valid") or r.get("attempt", 0) >= a.redo_invalid):
                    done.add(r["tag"])
    with open(path, "a") as f:
        for arm, dt, rule, s, side in runs:
            if tag_of(arm, dt, rule, s, side) in done:
                continue
            for attempt in range(3):
                pre_ok = wait_load(a.max_load, min_mem=a.min_mem)
                res = run_one(a.out, arm, dt, rule, s, side, a)
                res["attempt"] = attempt
                if a.max_load or a.min_mem:
                    res["load_pre_ok"] = int(pre_ok)
                if "error" not in res and (res.get("contact") or res.get("deadlock") or res.get("starved")) and \
                        res.get("load_max", 0.0) >= a.quiet_load:
                    res["counted"] = 0  # under load: its verdict waits for the quiet rerun
                    print(json.dumps(res), flush=True)
                    f.write(json.dumps(res) + "\n")
                    f.flush()
                    pre_ok = wait_load(a.quiet_load, min_mem=a.min_mem)
                    res = run_one(a.out, arm, dt, rule, s, side, a, gate_load=a.quiet_load)
                    res["attempt"], res["quiet_rerun"] = attempt, 1
                    if a.quiet_load or a.min_mem:
                        res["load_pre_ok"] = int(pre_ok)
                print(json.dumps(res), flush=True)
                f.write(json.dumps(res) + "\n")
                f.flush()
                if "error" not in res and (res.get("valid") or attempt >= a.redo_invalid):
                    break
    return 0


if __name__ == "__main__":
    sys.exit(main())
