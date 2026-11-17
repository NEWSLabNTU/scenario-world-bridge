"""``scensim drive``: drive an entity's pilot on the project's stack without a scenario
(roadmap 077, G14).

    scensim drive --spawn x,y,z,yaw [--goal x,y,z,yaw] [--map Town01] [--entity ego]
        [--poses <file: initial_pose, goal_pose>]
        [--timeout 600] [--realtime true|false] [--start-stack] [--stop-stack]

In SSv2's place, ``scensim_free_drive`` (the bridge's second binary, roadmap 073) initializes the
episode on the map, spawns the entity from its catalog entry at ``--spawn`` and steps the
world synchronously. When the entity's pilot is up on the episode, ``--goal`` is sent to it
through the relay as a commander (``set_goal``: route, control and engage, for every pilot
kind), and the run ends ``passed`` when the pilot reports ARRIVED, ``failed`` at
``--timeout``. Without a goal the world steps until Ctrl-C or the timeout (a pilot driven by
hand, e.g. a goal from RViz), and the run ends ``passed``. Poses are the map frame's
(base_link; yaw in rad, counter-clockwise from east), as a scenario's WorldPosition.

The run is a run like a scenario's: one at a time per stack, its own directory with
``resolved.yaml``, ``result.json`` and ``logs/free_drive/console.log``.

Exit codes as ``scensim run``'s: 0 passed, 1 failed, 2 error, 3 refused.
"""
import argparse
import json
import math
import os
import signal
import sys
import time
from pathlib import Path

import yaml

from scensim import launcher as launchermod
from scensim import config, results, run as runmod, signals as signalsmod, stack as stackmod

DEFAULT_TIMEOUT = 600.0
UP_TIMEOUT = 300.0     # s for the entity's pilot to be on the episode after the spawn
POLL = 0.5
# agent phases (scensim_agent.protocol)
UP_PHASES = ("IDLE", "PLANNING", "READY", "DRIVING", "ARRIVED", "STOPPED")
AT_SPAWN_M = 3.0   # the pilot is on this drive's episode once its pose is this near the spawn
ARRIVED = "ARRIVED"


def parse_pose(text: str, what: str) -> tuple:
    """``x,y,z,yaw`` (or ``x,y,yaw``, z 0) as four floats."""
    try:
        v = [float(t) for t in text.split(",")]
    except ValueError:
        raise ValueError(f"{what} {text!r}: numbers x,y,z,yaw") from None
    if len(v) == 3:
        v = [v[0], v[1], 0.0, v[2]]
    if len(v) != 4 or not all(math.isfinite(x) for x in v):
        raise ValueError(f"{what} {text!r}: four numbers x,y,z,yaw (or three, x,y,yaw)")
    return tuple(v)


def read_poses(path) -> tuple:
    """``(spawn, goal)`` from a poses file of acb's pilot (``initial_pose``, ``goal_pose``:
    x, y, z and a quaternion; roadmap 017, item 47), each None when absent."""
    doc = yaml.safe_load(Path(path).read_text()) or {}

    def one(key):
        q = doc.get(key)
        if not isinstance(q, dict):
            return None
        try:
            yaw = math.atan2(2.0 * (float(q.get("qw", 1)) * float(q.get("qz", 0))
                                    + float(q.get("qx", 0)) * float(q.get("qy", 0))),
                             1.0 - 2.0 * (float(q.get("qy", 0)) ** 2 + float(q.get("qz", 0)) ** 2))
            return (float(q["x"]), float(q["y"]), float(q.get("z", 0.0)), yaw)
        except (KeyError, TypeError, ValueError):
            raise ValueError(f"{path}: {key} needs numbers x, y (z, qx, qy, qz, qw)") from None
    return one("initial_pose"), one("goal_pose")


def pose_dict(p: tuple) -> dict:
    """The protocol's pose (position and a quaternion of the yaw)."""
    x, y, z, yaw = p
    return {"x": x, "y": y, "z": z, "qx": 0.0, "qy": 0.0,
            "qz": math.sin(yaw / 2.0), "qw": math.cos(yaw / 2.0)}


def catalog_path() -> str:
    """sim_vehicle_spec's catalog, as the free-drive launch takes it by default."""
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory("sim_vehicle_spec"), "catalogs",
                        "vehicles.xosc")


def free_drive_argv(rec: dict, map_path: str, entity: str, vehicle: str, spawn: tuple,
                    realtime: bool, catalog: str = None) -> list:
    return ["ros2", "run", "scenario_world_bridge", "scensim_free_drive", "--map", map_path,
            "--catalog", catalog or catalog_path(), "--entry", vehicle,
            "--pose", ",".join(f"{v:g}" for v in spawn), "--port", str(rec["ports"]["ssv2"]),
            "--name", entity, "--realtime", "true" if realtime else "false"]


def commander(rec: dict):
    from scensim_agent.commander import Commander
    return Commander(f"tcp://localhost:{rec['ports']['relay']}", "scensim_drive")


def _phase(c, entity):
    """The entity's agent's phase and its pose (a dict, or None), from the relay;
    (None, None) when it is not registered."""
    reply = c.query(entity)
    state = (reply.get("state") or {}) if isinstance(reply, dict) else {}
    if not reply.get("registered", state != {}):
        return None, None
    return state.get("phase"), state.get("pose")


def at_spawn(pose, spawn) -> bool:
    """The pose is the spawn's (a pilot left up by an earlier run reports its old pose until
    the entity is spawned for this drive)."""
    if not pose:
        return False
    return math.hypot(float(pose["x"]) - spawn[0], float(pose["y"]) - spawn[1]) <= AT_SPAWN_M


def drive(ctx, entity: str, spawn: tuple, goal: tuple = None, map_name: str = None,
          timeout: float = DEFAULT_TIMEOUT, realtime: bool = True, argv_fn=None,
          commander_fn=None, up_timeout: float = UP_TIMEOUT) -> results.RunResult:
    """One free drive on ``ctx``'s stack; its result.json written."""
    runmod.take_turn(ctx)
    rec, project = ctx.stack, ctx.project
    out = ctx.out or sys.stdout
    pilot = next((p for p in rec.get("pilots", []) if p["entity"] == entity), None)
    if pilot is None:
        raise runmod.Refused(f"entity {entity} has no pilot in the stack {rec.get('stack_id')}"
                             f" (its pilots: {', '.join(p['entity'] for p in rec.get('pilots', [])) or 'none'})")
    if pilot.get("lifetime") == "run":
        raise runmod.Refused(f"pilot {entity} is lifetime: run; a free drive needs a stack "
                             "pilot (lifetime: stack)")
    name = map_name or rec.get("map") or (rec.get("maps") or [None])[0]
    if not name:
        raise runmod.Refused("no map: give --map (one of the project's maps)")
    infra = []
    msg = runmod.ensure_map(ctx, name)
    if msg:
        infra.append(msg)
    rec = ctx.stack
    map_path = str(stackmod.map_dir(project, name))
    vehicle = (pilot.get("spec") or {}).get("vehicle") or stackmod.DEFAULT_VEHICLE

    run_dir = runmod.new_run_dir(ctx.runs_root, f"drive_{entity}")
    logs = run_dir / "logs" / "free_drive"
    logs.mkdir(parents=True)
    argv = (argv_fn or free_drive_argv)(rec, map_path, entity, vehicle, spawn, realtime)
    events = []
    t0 = time.time()
    resolved = {
        "run_id": run_dir.name, "project": str(project),
        "drive": {"entity": entity, "map": name, "spawn": list(spawn),
                  "goal": list(goal) if goal else None, "timeout_s": timeout,
                  "realtime": realtime, "argv": argv},
        "stack": {k: rec.get(k) for k in ("stack_id", "simulator", "map", "seat", "domains",
                                           "ports", "fingerprint")},
        "pilots": {entity: {"kind": pilot["kind"], "domain": pilot["domain"]}},
    }
    (run_dir / "resolved.yaml").write_text(yaml.safe_dump(resolved, sort_keys=False))
    verdict, reason, proc = results.ERROR, "", None
    try:
        if infra:
            verdict, reason = results.ERROR, "; ".join(infra)
        else:
            proc = runmod._start(argv, stackmod.ros_env(rec, rec["domains"]["scenario"]),
                                 logs / "console.log", cwd=str(project))
            events.append({"t_wall": time.time(), "type": "drive.start", "entity": entity})
            out.write(f"drive    {entity} at {spawn} on {name}" +
                      (f", goal {goal}" if goal else ", no goal (Ctrl-C ends it)") + "\n")
            out.flush()
            verdict, reason = _follow(proc, rec, entity, goal, timeout, up_timeout, events,
                                      commander_fn or commander, out, spawn)
    except KeyboardInterrupt:
        verdict = results.PASSED if goal is None else results.FAILED
        reason = "ended by hand" + ("" if goal is None else " before the goal")
    finally:
        if proc is not None:
            runmod._stop(proc)
    duration = time.time() - t0
    prov = results.provenance(ctx.workspace or stackmod.workspace_root() or project, project,
                              build=results.installed_build())
    prov.update(simulator=rec.get("simulator"), stack_id=rec.get("stack_id"),
                launcher=launchermod.of(rec),  # 079: which launcher ran the stack
                spec_hash=rec.get("spec_hash") or {})
    if pilot.get("overlays"):
        prov["overlays"] = {entity: pilot["overlays"]}   # G60
    # the drive's steps go to its timeline; result.json keeps the schema's events
    with open(run_dir / "timeline.jsonl", "w") as f:
        for e in events:
            f.write(json.dumps({"t_wall": e["t_wall"], "event": e["type"],
                                "entity": e.get("entity")}) + "\n")
    result = results.RunResult(
        run_id=run_dir.name, verdict=verdict, reason=reason,
        events=[{"t_sim": None, "t_wall": time.strftime("%Y-%m-%dT%H:%M:%S",
                                                       time.localtime(e["t_wall"])),
                 "type": e["type"], "entity": e["entity"]}
                for e in events if e["type"] in results.EVENT_TYPES],
        metrics={"duration_s": round(duration, 1)},
        pilots={entity: {"kind": pilot["kind"], "vehicle": vehicle,
                         "inputs": (pilot.get("spec") or {}).get("inputs") or []}},
        provenance=prov)
    (run_dir / "result.json").write_text(result.to_json())
    out.write(runmod.line(result, run_dir) + "\n")
    out.flush()
    return result


def _follow(proc, rec, entity, goal, timeout, up_timeout, events, commander_fn, out,
            spawn=None):
    """``(verdict, reason)``: wait for the pilot on the episode, send the goal, wait for it."""
    t0 = time.time()
    sent = False
    with commander_fn(rec) as c:
        while True:
            code = proc.poll()
            if code is not None:
                return results.ERROR, f"scensim_free_drive ended ({code}) during the drive"
            now = time.time()
            phase, pose = _phase(c, entity)
            if not sent:
                if phase in UP_PHASES and (at_spawn(pose, spawn) if spawn else pose):
                    events.append({"t_wall": now, "type": "drive.up", "entity": entity})
                    if goal is None:
                        sent = True
                        out.write(f"up       {entity}'s pilot is on the episode\n")
                        out.flush()
                    else:
                        reply = c.command(entity, "set_goal", goal=pose_dict(goal),
                                          waypoints=[])
                        if reply.get("status") != "OK":
                            return results.FAILED, (f"the pilot refused the goal: "
                                                    f"{reply.get('status')} {reply.get('message', '')}")
                        sent = True
                        events.append({"t_wall": time.time(), "type": "drive.goal",
                                       "entity": entity})
                        out.write(f"goal     sent to {entity}\n")
                        out.flush()
                elif now - t0 > up_timeout:
                    return results.ERROR, (f"{entity}'s pilot was not on the episode within "
                                           f"{up_timeout:.0f} s (phase {phase})")
            elif goal is not None and phase == ARRIVED:
                events.append({"t_wall": now, "type": "route_complete", "entity": entity})
                return results.PASSED, ""
            if now - t0 > timeout:
                if goal is None:
                    return results.PASSED, f"no goal: ended at the timeout ({timeout:.0f} s)"
                return results.FAILED, (f"{entity} did not arrive within {timeout:.0f} s "
                                        f"(phase {phase})")
            time.sleep(POLL)


def main(argv=None, out=None) -> int:
    ap = argparse.ArgumentParser(prog="scensim drive", description=__doc__.split("\n\n")[0])
    ap.add_argument("--spawn", help="x,y,z,yaw in the map frame (or x,y,yaw)")
    ap.add_argument("--poses", help="a poses file (initial_pose, goal_pose) as acb's pilot "
                                    "takes: the spawn and the goal, unless --spawn/--goal")
    ap.add_argument("--goal", help="x,y,z,yaw: the pilot drives there; the run passes on arrival")
    ap.add_argument("--map", help="one of the project's maps (default: the stack's)")
    ap.add_argument("--entity", default="ego", help="the pilot's entity (default ego)")
    ap.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT)
    ap.add_argument("--realtime", default="true", choices=("true", "false"))
    ap.add_argument("--project")
    ap.add_argument("--seat")
    ap.add_argument("--start-stack", action="store_true")
    ap.add_argument("--stop-stack", action="store_true")
    ap.add_argument("--wait", type=float, default=0.0)
    args = ap.parse_args(argv)
    out = out or sys.stdout
    try:
        spawn = parse_pose(args.spawn, "--spawn") if args.spawn else None
        goal = parse_pose(args.goal, "--goal") if args.goal else None
        if args.poses:
            p_spawn, p_goal = read_poses(args.poses)
            spawn, goal = spawn or p_spawn, goal or p_goal
        if spawn is None:
            raise ValueError("give --spawn, or --poses with an initial_pose")
    except (ValueError, OSError) as e:
        print(f"scensim drive: refused: {e}", file=sys.stderr)
        return runmod.EXIT_REFUSED
    signal.signal(signal.SIGTERM, runmod._on_term)
    signalsmod.take_sigint()   # even inherited ignored (a script's `&` job)
    ctx = None
    try:
        ctx = runmod.context(args.project, seat=args.seat, wait=args.wait,
                             start=args.start_stack or args.stop_stack, out=out)
        result = drive(ctx, args.entity, spawn, goal, args.map, args.timeout,
                       args.realtime == "true")
        code = runmod.outcome(result)
    except runmod.Refused as e:
        print(f"scensim drive: refused: {e}", file=sys.stderr)
        code = runmod.EXIT_REFUSED
    except runmod.StackFailed as e:
        print(f"scensim drive: {e}", file=sys.stderr)
        code = runmod.EXIT_INFRA
    finally:
        if ctx is not None:
            runmod.release_turn(ctx)
            if args.stop_stack and ctx.started is not None:
                out.write(f"stack    stopping the stack this run started (pid "
                          f"{ctx.started.proc.pid})\n")
                ctx.started.stop()
    return code


if __name__ == "__main__":
    sys.exit(main())
