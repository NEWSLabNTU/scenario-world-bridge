#!/usr/bin/env python3
"""Roadmap 074 C3c: the cut-in with both cars Autoware, on Town03's west straight (C3's site).

B is the ego, the stack's Autoware (`just run ... ego_launch_args:="max_velocity:=<VB> ..."`).
A is a second, background Autoware (`just bg-av`, entity bg_av_1), started fresh for each run
with max_velocity v_B + dv. A starts in the right lane with its goal in B's lane, so its route
needs a lane change. Its RTC stays off: Autoware's own lane-change module decides when it is
safe to go, which is A's real strategy here. Nothing warms the pair up to the cell's context:
the cell sets each stack's speed limit and A's start offset, and the context actually reached
is recorded twice, at the run's start and at the onset of A's lane change (d0_real / dv_real
and d0_lc / dv_lc).

A CARLA client logs both cars' true poses at 10 Hz (A as the agent's log names "me", B as
"other"), with the route curvature at each, so `c2b.score` scores the run as it scores C2b and
C3. The run starts when both cars have reached 90 % of their speed limits, or 1 s before A's
lane change starts, whichever is first, but no more than 10 s before the lane change, so the
lane change falls in the 20 s window. A's decision is taken to be the lane change's onset, with no
prepare time.

  scripts/coop/c3c.py --out <dir> [--seeds 2] [--cells '6,0;10,0;14,-3;2,2'] [--only N]
  scripts/coop/c3c.py --out <dir> --rescore
  scripts/coop/c3c.py --out <dir> --arena-port <port>   A_aw_perm against B_aw_react in the arena

The stack must be up on Town03 with the ego at max_velocity VB (C3_VB, default 6) and, for
ground truth, publish_ground_truth_objects:=true occupancy_grid_map_method:=laserscan_based. The
same perception switches go to A through EXTRA_BG (default: the ground-truth ones).
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
import c2b  # noqa: E402
import c3  # noqa: E402
import cutin_game as C1  # noqa: E402
import double_oracle as DO  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
L = c2b.L
VB = c3.VB
CELLS = [(6.0, 0.0), (10.0, 0.0), (14.0, -3.0), (2.0, 2.0)]
ENTITY = "bg_av_1"
EXTRA_BG = os.environ.get("EXTRA_BG", "publish_ground_truth_objects:=true occupancy_grid_map_method:=laserscan_based").split()
PARAMS = """
				<ParameterAssignments>
					<ParameterAssignment parameterRef="maxSpeed" value="50" />
					<ParameterAssignment parameterRef="maxAcceleration" value="15.0" />
					<ParameterAssignment parameterRef="maxDeceleration" value="15.0" />
					<ParameterAssignment parameterRef="maxSteering" value="0.5" />
				</ParameterAssignments>
			"""  # as c3.py's: SSv2's catalog entry needs them
T_END = 80.0  # s of simulation: start from rest, the 5 s engage lag, and the window
# B (the ego) moves about 5 s before A (the background AV engages after its route arrives):
# smoke run, 6/0: A fell 29 m back relative to B by the time both were at speed. A starts that
# much further ahead; the context actually reached is recorded either way
START_LEAD = float(os.environ.get("C3C_START_LEAD", "29"))
START_TOL = float(os.environ.get("C3C_START_TOL", 25.0))  # m: |d0 at the start - the cell's d0| above it, retried
INIT_S = float(os.environ.get("C3C_INIT_S", "300"))  # s, the scenario's Autoware init-through-engage budget  # s of simulation per run: the start from rest, then the 20 s window
BG = []  # the background AV's play_launch (a process group), stopped with this process


def _stop(signum, frame):
    stop_bg()
    sys.exit(128 + signum)


def scenario(path, d0, dv):
    """B (the ego) in the left lane at Y_EGO; A (bg_av_1, a vehicle agent) in the right lane with
    its rear d0 m ahead of B's front, its goal in B's lane. Ahead is -y."""
    ya = round(c3.Y_EGO - (d0 + L + START_LEAD), 2)
    ya = min(ya, 117.0)
    x_at, XL, XR, YG, H = c3.x_at, c3.X_LEFT, c3.X_RIGHT, c3.Y_GOAL, c3.H
    wp = lambda x, y, z: f'<WorldPosition x="{x}" y="{y}" z="{z}" h="{H:.4f}" p="0" r="0" />'
    Path(path).write_text(f'''<?xml version="1.0"?>
<!-- Roadmap 074 C3c: the cut-in, both cars Autoware; d0 {d0} dv {dv}; written by scripts/coop/c3c.py -->
<OpenSCENARIO>
	<FileHeader author="scenario-world-bridge" date="2026-11-15T00:00:00+00:00" description="074 C3c cut-in d0 {d0} dv {dv}" revMajor="1" revMinor="2" />
	<ParameterDeclarations />
	<CatalogLocations><VehicleCatalog><Directory path="$(find-pkg-share sim_vehicle_spec)/catalogs" /></VehicleCatalog></CatalogLocations>
	<RoadNetwork><LogicFile filepath="$(env CARLA_MAPS)/Town03" /></RoadNetwork>
	<Entities>
		<ScenarioObject name="ego">
			<CatalogReference catalogName="vehicles" entryName="tesla_model3">{PARAMS}</CatalogReference>
			<ObjectController><Controller name="Autoware"><Properties>
				<Property name="maxJerk" value="1.5" /><Property name="minJerk" value="-1.5" />
			</Properties></Controller></ObjectController>
		</ScenarioObject>
		<ScenarioObject name="{ENTITY}">
			<CatalogReference catalogName="vehicles" entryName="tesla_model3">{PARAMS}</CatalogReference>
			<ObjectController><Controller name="agent"><Properties /></Controller></ObjectController>
		</ScenarioObject>
	</Entities>
	<Storyboard>
		<Init><Actions>
			<Private entityRef="ego">
				<PrivateAction><TeleportAction><Position>{wp(x_at(XL, c3.Y_EGO), c3.Y_EGO, 0.0)}</Position></TeleportAction></PrivateAction>
				<PrivateAction><RoutingAction><AcquirePositionAction><Position>{wp(x_at(XL, YG), YG, 0.0)}</Position></AcquirePositionAction></RoutingAction></PrivateAction>
			</Private>
			<Private entityRef="{ENTITY}">
				<PrivateAction><TeleportAction><Position>{wp(x_at(XR, ya), ya, 0.3)}</Position></TeleportAction></PrivateAction>
				<PrivateAction><RoutingAction><AcquirePositionAction><Position>{wp(x_at(XL, YG), YG, 0.0)}</Position></AcquirePositionAction></RoutingAction></PrivateAction>
			</Private>
		</Actions></Init>
		<Story name="c3c"><Act name="c3c_act">
			<ManeuverGroup maximumExecutionCount="1" name="end"><Actors selectTriggeringEntities="false"><EntityRef entityRef="ego" /></Actors>
			<Maneuver name="end_m">
				<Event name="done" priority="parallel" maximumExecutionCount="1">
					<Action name="done_a"><UserDefinedAction><CustomCommandAction type="exitSuccess" /></UserDefinedAction></Action>
					<StartTrigger><ConditionGroup><Condition name="time" delay="0" conditionEdge="none"><ByValueCondition><SimulationTimeCondition value="{T_END}" rule="greaterThan" /></ByValueCondition></Condition></ConditionGroup></StartTrigger>
				</Event>
			</Maneuver></ManeuverGroup>
			<StartTrigger><ConditionGroup><Condition name="start" delay="0" conditionEdge="none"><ByValueCondition><SimulationTimeCondition value="0" rule="greaterThan" /></ByValueCondition></Condition></ConditionGroup></StartTrigger>
		</Act></Story>
		<StopTrigger />
	</Storyboard>
</OpenSCENARIO>
''')


# --- the background AV -----------------------------------------------------------------------
def start_bg(rd, va):
    log = open(rd / "bg.console", "w")
    args = ["just", "bg-av", f"map_path:={ROOT}/data/carla-autoware-bridge/Town03", f"max_velocity:={va:g}",
            f"carla_physics:={ROOT}/src/sim_vehicle_spec/generated/tesla_model3/carla_physics.yaml", *EXTRA_BG]
    p = subprocess.Popen(args, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    BG[:] = [p]
    return p


def stop_bg():
    for p in BG:
        try:
            os.killpg(p.pid, signal.SIGTERM)
        except ProcessLookupError:
            continue
        try:
            p.wait(timeout=60)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)
    BG[:] = []


def bg_ready(timeout=600):
    """The background AV's Autoware ready, in its own ROS domain."""
    env = dict(os.environ, ROS_DOMAIN_ID=os.environ.get("BG_ROS_DOMAIN_ID", "2"))
    t0 = time.time()
    while time.time() - t0 < timeout:
        r = subprocess.run(["ros2", "run", "swb_launch", "ego_ready", "--timeout", "10"], env=env,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if r.returncode == 0:
            return True
        time.sleep(10)
    return False


# --- the truth logger ------------------------------------------------------------------------
class Logger(threading.Thread):
    """Both cars' true states from CARLA at 10 Hz of simulation time, A first."""

    def __init__(self, port, path):
        super().__init__(daemon=True)
        self.port, self.path, self.stop = port, path, threading.Event()

    def run(self):
        import carla
        import route

        c = carla.Client("localhost", self.port)
        c.set_timeout(30.0)
        w = c.get_world()
        cmap = w.get_map()
        a = b = None
        t0 = last = None
        with open(self.path, "w") as f:
            while not self.stop.is_set():
                try:
                    snap = w.wait_for_tick(10.0)
                except RuntimeError:  # a synchronous CARLA pauses while the stack initialises
                    continue
                if a is None or b is None:
                    for act in w.get_actors().filter("vehicle.*"):
                        rn = act.attributes.get("role_name")
                        if rn == ENTITY:
                            a = act
                        elif rn == "hero":
                            b = act
                    continue
                t = snap.timestamp.elapsed_seconds
                if t0 is None:
                    t0 = t
                if last is not None and t - last < 0.1 - 1e-6:
                    continue
                last = t
                try:
                    ta, tb = a.get_transform(), b.get_transform()
                    va, vb = a.get_velocity(), b.get_velocity()
                except RuntimeError:
                    break
                la, lb = ta.location, tb.location
                # the scenario frame's y is CARLA's -y; A's progress from its lane to B's
                ys = -la.y
                xr, xl = c3.x_at(c3.X_RIGHT, ys), c3.x_at(c3.X_LEFT, ys)
                lat = max(0.0, min(1.0, (la.x - xr) / (xl - xr)))
                yaw_a, yaw_b = math.radians(ta.rotation.yaw), math.radians(tb.rotation.yaw)
                along = (lb.x - la.x) * math.cos(yaw_a) + (lb.y - la.y) * math.sin(yaw_a)
                f.write(json.dumps({"t": round(t - t0, 3), "x": round(la.x, 2), "y": round(la.y, 2),
                                    "ox": round(lb.x, 2), "oy": round(lb.y, 2), "yaw": round(yaw_a, 4),
                                    "oyaw": round(yaw_b, 4), "v": round(math.hypot(va.x, va.y), 3),
                                    "ov": round(math.hypot(vb.x, vb.y), 3), "lat": round(lat, 3),
                                    "along": round(along, 2),
                                    "kappa": round(route.kappa_at(cmap, la.x, la.y), 5),
                                    "okappa": round(route.kappa_at(cmap, lb.x, lb.y), 5)}) + "\n")
                f.flush()


def phase_log(truth, out, va_lim, vb_lim):
    """The truth log as c2b.score reads an agent's log: phases, the start event, decide. Returns
    the start and lane-change contexts."""
    rows = [json.loads(l) for l in open(truth) if l.strip()]
    if not rows:
        return None
    t_lc = next((r["t"] for r in rows if r["lat"] > 0.05), None)
    t_fast = next((r["t"] for r in rows if r["v"] >= 0.9 * va_lim and r["ov"] >= 0.9 * vb_lim), None)
    if t_lc is not None:  # the lane change inside the window: from up to 10 s before it
        t0 = min(max(t_fast if t_fast is not None else 0.0, t_lc - 10.0), max(0.0, t_lc - 1.0))
    elif t_fast is not None:
        t0 = t_fast
    else:
        return None
    ctx = lambda r: {"d0": round(-r["along"] - L, 2), "dv": round(r["v"] - r["ov"], 2), "vA": r["v"], "vB": r["ov"]}
    start = next(r for r in rows if r["t"] >= t0)
    lc = next((r for r in rows if t_lc is not None and r["t"] >= t_lc), None)
    decide = None if t_lc is None else round(max(0.0, t_lc - t0), 2)
    with open(out, "w") as f:
        f.write(json.dumps({"event": "start", "t": t0, "along": start["along"], "v": start["v"], "ov": start["ov"]}) + "\n")
        for r in rows:
            ph = "run" if r["t"] >= t0 else "warmup"
            dec = decide if (ph == "run" and t_lc is not None and r["t"] >= t_lc) else None
            f.write(json.dumps(dict(r, phase=ph, decide=dec, dropping=False)) + "\n")
    return {"t_start": t0, "t_lc": t_lc, "start": ctx(start), "lc": None if lc is None else ctx(lc)}


def run_one(out, d0, dv, seed, port):
    tag = f"d{d0:g}_v{dv:g}_aw_aw_s{seed}"
    rd = Path(out) / tag
    rd.mkdir(parents=True, exist_ok=True)
    va = VB + dv
    xosc = rd / "c3c.xosc"
    scenario(xosc, d0, dv)
    base = {"d0": d0, "dv": dv, "a": "autoware", "b": "autoware", "seed": seed, "vA_limit": va, "vB_limit": VB}
    stop_bg()
    if not c3.restart_ego():
        return dict(base, error="ego not ready")
    start_bg(rd, va)
    if not bg_ready():
        stop_bg()
        return dict(base, error="A's Autoware not ready")
    time.sleep(5)
    bag = subprocess.Popen(["ros2", "bag", "record", "-s", "mcap", "-o", str(rd / "bag"), "-e", c3.BAG_TOPICS],
                           stdout=open(rd / "bag.log", "w"), stderr=subprocess.STDOUT, start_new_session=True)
    lg = Logger(port, rd / "truth.jsonl")
    lg.start()
    env = dict(os.environ, CARLA_PORT=str(port))
    # two Autoware stacks on a shared host: the ego's initialisation (its pose seeded on attach)
    # took longer than the scenario's default 120 s at load 80-100
    r = subprocess.run(["just", "scenario-direct", str(xosc), f"output_directory:={rd}",
                        f"initialize_duration:={INIT_S:g}"], cwd=ROOT, env=env,
                       stdout=open(rd / "scenario.log", "w"), stderr=subprocess.STDOUT)
    lg.stop.set()
    lg.join(timeout=20)
    try:
        os.killpg(bag.pid, signal.SIGINT)
        bag.wait(timeout=30)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass
    stop_bg()
    return dict(base, scenario_rc=r.returncode, **score_dir(rd, d0, dv, va))


def score_dir(rd, d0, dv, va):
    ctx = phase_log(rd / "truth.jsonl", rd / "coop_a.jsonl", va, VB) if (rd / "truth.jsonl").exists() else None
    if ctx is None:
        return {"error": "no usable truth log", "cause": c3.junit_message(rd)}
    res = {"t_start": ctx["t_start"], "t_lc": ctx["t_lc"],
           "d0_start": ctx["start"]["d0"], "dv_start": ctx["start"]["dv"],
           "d0_lc": ctx["lc"] and ctx["lc"]["d0"], "dv_lc": ctx["lc"] and ctx["lc"]["dv"],
           "vA_lc": ctx["lc"] and ctx["lc"]["vA"], "vB_lc": ctx["lc"] and ctx["lc"]["vB"]}
    res.update(c2b.score(rd / "coop_a.jsonl", rd / "coop_a.jsonl", d0, dv, 0.0))
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--carla-port", type=int, default=int(os.environ.get("CARLA_PORT", 2000)))
    ap.add_argument("--seeds", type=int, default=2)
    ap.add_argument("--cells", default="", help="only these cells, e.g. '6,0;14,-3'")
    ap.add_argument("--only", type=int, default=0, help="run just the first N runs (a smoke test)")
    ap.add_argument("--rescore", action="store_true")
    ap.add_argument("--arena-port", type=int, default=0, help="instead: the cells in the arena, A_aw_perm against B_aw_react")
    a = ap.parse_args()
    signal.signal(signal.SIGTERM, _stop)
    os.makedirs(a.out, exist_ok=True)
    cells = [tuple(float(v) for v in c.split(",")) for c in a.cells.split(";") if c] or CELLS
    path = os.path.join(a.out, "c3c.jsonl")
    if a.arena_port:
        c2b.VB, c2b.CELLS = VB, cells
        c2b.PAIRS = [(DO.A_AW_PERM, DO.B_AW_REACT)]
        c2b.arena_compare(a.arena_port, a.out)
        return
    if a.rescore:
        old = [json.loads(line) for line in open(path) if line.strip()]
        with open(path, "w") as f:
            for r in old:
                rd = Path(a.out) / f"d{r['d0']:g}_v{r['dv']:g}_aw_aw_s{r['seed']}"
                r.update(score_dir(rd, r["d0"], r["dv"], VB + r["dv"]))
                f.write(json.dumps(r) + "\n")
        return
    runs = [(d0, dv, s) for (d0, dv) in cells for s in range(a.seeds)]
    if a.only:
        runs = runs[: a.only]
    try:
        with open(path, "a") as f:
            for d0, dv, s in runs:
                for attempt in range(2):
                    res = run_one(a.out, d0, dv, s, a.carla_port)
                    res["attempt"] = attempt
                    # A's Autoware engaged 20-30 s after B's in some runs at load ~100 (START_LEAD
                    # covers 5 s): the cars never met. Such a run keeps its data and is retried.
                    if res.get("d0_start") is not None and abs(res["d0_start"] - d0) > START_TOL:
                        res["error"] = "start misaligned"
                    if "error" not in res:
                        break
                    print(json.dumps(res), flush=True)
                print(json.dumps(res), flush=True)
                f.write(json.dumps(res) + "\n")
                f.flush()
    finally:
        stop_bg()


if __name__ == "__main__":
    main()
