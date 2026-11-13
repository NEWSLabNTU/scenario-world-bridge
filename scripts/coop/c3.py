#!/usr/bin/env python3
"""Roadmap 074 C3: the cut-in with B as Autoware, A a light agent, on Town03's west straight.

B is the ego: a full Autoware stack (`just run`, max_velocity 8 to match C2b's v_B), with
ground-truth objects (C3a: `publish_ground_truth_objects:=true`, planning and control
Autoware's, objects exact) or with its own perception (C3b: CenterPoint). A is
`cutin_agent.py` with side A and the CARLA role_name of the ego ("hero") as the other car: it
warms up to the context against whatever the ego does, then runs its rule. Scored from A's log
exactly as C2b scores (`c2b.score`), so the cells compare with the arena's B_AW (the
Autoware-like rule) and with C2b.

Site: Town03's west straight, two driving lanes with a dashed line, heading -y from y = 124 to
-146, no signals (one unsignalled junction at y 10..-28; E2 drove through it). B in the left
lane (lanelets 5518, 106662, 8209), A in the right lane (5750, 106746, 8395) cutting left.

  scripts/coop/c3.py --out <dir> [--seeds 2] [--only N]       one scenario per run on the stack
  scripts/coop/c3.py --out <dir> --rescore
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import c2b  # noqa: E402
import cutin_game as C1  # noqa: E402
import double_oracle as DO  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
L = c2b.L
VB, WARMUP = float(os.environ.get("C3_VB", "6.0")), 10.0  # 8 m/s ran A off the straight's end within HORIZON
H = -1.5679
# lane centres: x at y = 124 and the drift per m of y (from the lanelet centrelines)
X_LEFT, X_RIGHT, DRIFT = -85.2, -88.7, -0.00287
Y_EGO = 120.0  # the straight starts at 124; the cut-in should be done before the junction at y 10
Y_GOAL = -110.0  # in lanelet 8209; -140 is in a junction lanelet and Autoware refused the route
CELLS = c2b.CELLS
# what Autoware saw and why it slowed, per run (the ego domain; c3.py runs with it sourced)
BAG_TOPICS = ("^/localization/kinematic_state$|^/control/command/control_cmd$|^/api/planning/velocity_factors$"
              "|^/perception/object_recognition/objects$|^/planning/planning_factors/.*|^/system/fail_safe/mrm_state$")
AGENT = []  # the running agent, killed with this process: a stray one keeps the entity name


def _stop(signum, frame):
    for p in AGENT:
        try:
            os.killpg(p.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    sys.exit(128 + signum)
RULES = [c2b.A_EQ, DO.A_AW_PERM]


def x_at(lane_x, y):
    return round(lane_x + DRIFT * (y - 124.0), 2)


def scenario(path, d0, dv):
    r0 = d0 - dv * WARMUP  # A's rear ahead of B's front at the warm-up's start
    ya = round(Y_EGO - (r0 + L), 2)  # ahead is -y
    ya = min(ya, 117.0)  # at y 123 A sat at the junction's edge and the agent's lane lookup took another road
    wp = lambda x, y, z: f'<WorldPosition x="{x}" y="{y}" z="{z}" h="{H:.4f}" p="0" r="0" />'
    Path(path).write_text(f'''<?xml version="1.0"?>
<!-- Roadmap 074 C3: the cut-in, B the Autoware ego, A a light agent; d0 {d0} dv {dv}; written by scripts/coop/c3.py -->
<OpenSCENARIO>
	<FileHeader author="scenario-world-bridge" date="2026-11-13T00:00:00+00:00" description="074 C3 cut-in d0 {d0} dv {dv}" revMajor="1" revMinor="2" />
	<ParameterDeclarations />
	<CatalogLocations><VehicleCatalog><Directory path="$(find-pkg-share sim_vehicle_spec)/catalogs" /></VehicleCatalog></CatalogLocations>
	<RoadNetwork><LogicFile filepath="$(env CARLA_MAPS)/Town03" /></RoadNetwork>
	<Entities>
		<ScenarioObject name="ego">
			<CatalogReference catalogName="vehicles" entryName="tesla_model3">
				<ParameterAssignments>
					<ParameterAssignment parameterRef="maxSpeed" value="50" /><!-- 12 tripped SSv2's range check on a one-frame spike while the ego drove 5.3 m/s -->
					<ParameterAssignment parameterRef="maxAcceleration" value="15.0" />
					<ParameterAssignment parameterRef="maxDeceleration" value="15.0" />
					<ParameterAssignment parameterRef="maxSteering" value="0.5" />
				</ParameterAssignments>
			</CatalogReference>
			<ObjectController><Controller name="Autoware"><Properties>
				<Property name="maxJerk" value="1.5" /><Property name="minJerk" value="-1.5" />
			</Properties></Controller></ObjectController>
		</ScenarioObject>
		<ScenarioObject name="coop_a">
			<CatalogReference catalogName="vehicles" entryName="tesla_model3">
				<ParameterAssignments>
					<ParameterAssignment parameterRef="maxSpeed" value="50" />
					<ParameterAssignment parameterRef="maxAcceleration" value="15.0" />
					<ParameterAssignment parameterRef="maxDeceleration" value="15.0" />
					<ParameterAssignment parameterRef="maxSteering" value="0.5" />
				</ParameterAssignments>
			</CatalogReference>
			<ObjectController><Controller name="agent"><Properties /></Controller></ObjectController>
		</ScenarioObject>
	</Entities>
	<Storyboard>
		<Init><Actions>
			<Private entityRef="ego">
				<PrivateAction><TeleportAction><Position>{wp(x_at(X_LEFT, Y_EGO), Y_EGO, 0.0)}</Position></TeleportAction></PrivateAction>
				<PrivateAction><RoutingAction><AcquirePositionAction><Position>{wp(x_at(X_LEFT, Y_GOAL), Y_GOAL, 0.0)}</Position></AcquirePositionAction></RoutingAction></PrivateAction>
			</Private>
			<Private entityRef="coop_a">
				<PrivateAction><TeleportAction><Position>{wp(x_at(X_RIGHT, ya), ya, 0.3)}</Position></TeleportAction></PrivateAction>
				<PrivateAction><RoutingAction><AcquirePositionAction><Position>{wp(x_at(X_RIGHT, Y_GOAL), Y_GOAL, 0.0)}</Position></AcquirePositionAction></RoutingAction></PrivateAction>
			</Private>
		</Actions></Init>
		<Story name="c3"><Act name="c3_act">
			<ManeuverGroup maximumExecutionCount="1" name="end"><Actors selectTriggeringEntities="false"><EntityRef entityRef="ego" /></Actors>
			<Maneuver name="end_m">
				<Event name="done" priority="parallel" maximumExecutionCount="1">
					<Action name="done_a"><UserDefinedAction><CustomCommandAction type="exitSuccess" /></UserDefinedAction></Action>
					<StartTrigger><ConditionGroup><Condition name="time" delay="0" conditionEdge="none"><ByValueCondition><SimulationTimeCondition value="{WARMUP + C1.T_MAX + 8}" rule="greaterThan" /></ByValueCondition></Condition></ConditionGroup></StartTrigger>
				</Event>
			</Maneuver></ManeuverGroup>
			<StartTrigger><ConditionGroup><Condition name="start" delay="0" conditionEdge="none"><ByValueCondition><SimulationTimeCondition value="0" rule="greaterThan" /></ByValueCondition></Condition></ConditionGroup></StartTrigger>
		</Act></Story>
		<StopTrigger />
	</Storyboard>
</OpenSCENARIO>
''')


def _ego_launches():
    """The ego's nested play_launch processes (the Python wrapper and the binary), by argv."""
    found = []
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        try:
            argv = open(f"/proc/{d}/cmdline", "rb").read().split(b"\0")
        except OSError:
            continue
        if any(a.endswith(b"play_launch") for a in argv[:2]) and b"carla_simulator.launch.xml" in argv \
                and any(a.endswith(b"/play_log/ego") for a in argv):
            found.append(int(d))
    return found


def restart_ego(timeout=600):
    """A fresh ego stack per run: after a scenario the next one found Autoware stuck in PLANNING
    (planning_validator: trajectory too short, deceleration too large), while a fresh stack's
    first run engaged. `just run` respawns the ego when its play_launch ends."""
    old = _ego_launches()
    for pid in old:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    t0 = time.time()
    while time.time() - t0 < timeout:
        time.sleep(10)
        new = [p for p in _ego_launches() if p not in old]
        if not new:
            continue
        r = subprocess.run(["ros2", "run", "swb_launch", "ego_ready", "--timeout", "10"],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if r.returncode == 0:
            time.sleep(10)
            return True
    return False


def junit_message(rd):
    """The scenario's failure message from its JUnit file, if any."""
    import re
    for f in Path(rd).rglob("result.junit.xml"):
        m = re.search(r'message="([^"]*)"', f.read_text(errors="replace"))
        if m:
            return m.group(1)[:200]
    return None


def run_one(out, d0, dv, pa, seed, port, relay):
    tag = f"d{d0:g}_v{dv:g}_{pa['name']}_aw_s{seed}"
    rd = Path(out) / tag
    rd.mkdir(parents=True, exist_ok=True)
    for old in ("bag", "scenario_test_runner"):  # a retry: keep the failed attempt's beside it
        if (rd / old).exists():
            n = 1
            while (rd / f"{old}.failed{n}").exists():
                n += 1
            (rd / old).rename(rd / f"{old}.failed{n}")
    xosc = rd / "c3.xosc"
    scenario(xosc, d0, dv)
    if not restart_ego():
        return {"d0": d0, "dv": dv, "a": pa["name"], "b": "autoware", "seed": seed, "error": "ego not ready"}
    cfg = {"entity": "coop_a", "other": "hero", "side": "A", "d0": d0, "dv": dv, "vb": VB, "warmup": WARMUP,
           "rule": pa, "seed": seed, "log": str(rd / "coop_a.jsonl"), "carla_port": port, "find_timeout": 90,
           "carla_pythonapi": os.path.join(os.environ.get("CARLA_DIR", "/mnt/p5plus/aeon/carla/CARLA_0.9.16"),
                                           "PythonAPI", "carla")}
    (rd / "coop_a.json").write_text(json.dumps(cfg))
    agent = subprocess.Popen([sys.executable, str(ROOT / "scripts/coop/cutin_agent.py"), "--relay", relay,
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
    res = {"d0": d0, "dv": dv, "a": pa["name"], "b": "autoware", "seed": seed, "scenario_rc": r.returncode}
    log = rd / "coop_a.jsonl"
    if log.exists():
        res.update(c2b.score(log, log, d0, dv, pa["t_prep"]))
    else:
        res["error"] = "no agent log"
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--carla-port", type=int, default=int(os.environ.get("CARLA_PORT", 2000)))
    ap.add_argument("--relay", default=f"tcp://127.0.0.1:{os.environ.get('AGENT_PORT', '5560')}")
    ap.add_argument("--seeds", type=int, default=1)
    ap.add_argument("--seed0", type=int, default=0, help="first seed (to split seeds across hosts)")
    ap.add_argument("--cells", default="", help="only these cells, e.g. '6,0;14,-3'")
    ap.add_argument("--only", type=int, default=0, help="run just the first N runs (a smoke test)")
    ap.add_argument("--rescore", action="store_true")
    ap.add_argument("--arena-port", type=int, default=0, help="instead: C2b's arena cells at this VB on this CARLA")
    a = ap.parse_args()
    signal.signal(signal.SIGTERM, _stop)
    os.makedirs(a.out, exist_ok=True)
    path = os.path.join(a.out, "c3.jsonl")
    if a.arena_port:
        c2b.VB = VB
        c2b.PAIRS = [(c2b.A_EQ, DO.B_AW), (DO.A_AW_PERM, DO.B_AW)]  # B as the Autoware-like rule
        c2b.arena_compare(a.arena_port, a.out)
        return
    if a.rescore:
        old = [json.loads(line) for line in open(path) if line.strip()]
        with open(path, "w") as f:
            for r in old:
                rd = Path(a.out) / f"d{r['d0']:g}_v{r['dv']:g}_{r['a']}_aw_s{r['seed']}"
                pa = c2b.A_EQ if r["a"] == "A_eq" else DO.A_AW_PERM
                r.update(c2b.score(rd / "coop_a.jsonl", rd / "coop_a.jsonl", r["d0"], r["dv"], pa["t_prep"]))
                f.write(json.dumps(r) + "\n")
        return
    cells = [tuple(float(v) for v in c.split(",")) for c in a.cells.split(";") if c] or CELLS
    runs = [(d0, dv, pa, s) for (d0, dv) in cells for pa in RULES for s in range(a.seed0, a.seed0 + a.seeds)]
    if a.only:
        runs = runs[: a.only]
    with open(path, "a") as f:
        for d0, dv, pa, s in runs:
            for attempt in range(3):
                res = run_one(a.out, d0, dv, pa, s, a.carla_port, a.relay)
                res["attempt"] = attempt
                if "error" not in res:
                    break
                # a run with no run phase is the stack's failure, not the game's (on a loaded host
                # Autoware stayed INITIALIZING after its restart): restart the ego and run it again
                res["cause"] = junit_message(Path(a.out) / f"d{d0:g}_v{dv:g}_{pa['name']}_aw_s{s}")
                print(json.dumps(res), flush=True)
            print(json.dumps(res), flush=True)
            f.write(json.dumps(res) + "\n")
            f.flush()


if __name__ == "__main__":
    main()
