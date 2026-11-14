"""swb.stack: the stack's plan (seat -> arguments, pilots bound, refusals) and its keeper."""
import importlib.util
import json
import os
import time
from pathlib import Path

import pytest
import yaml

from swb import assets, config, project, seat, stack

LAUNCH = Path(__file__).resolve().parents[2] / "swb_launch" / "launch"


@pytest.fixture
def where(host):
    """Seat locks and logs under tmp (the system host.yaml), seat 2."""
    tmp = host["tmp"]
    (tmp / "system" / "host.yaml").write_text(yaml.safe_dump(
        {"seat_locks": str(tmp / "locks"), "logs": str(tmp / "logs"), "seat": 2}))
    return host


def make(where, template="multi-pilot", swb_yaml=None, maps_hub=None):
    d = where["tmp"] / "proj"
    project.new_project(d, template)
    if swb_yaml is not None:
        (d / "swb.yaml").write_text(swb_yaml)
    assets.fetch(d, out=lambda s: None)
    return d


def fake_pilot(rec, p, entry, log_dir):
    """A pilot that only sleeps (the keeper's pilot_command in tests)."""
    import sys
    return [sys.executable, "-c", "import time; time.sleep(60)"]


def test_plan_multi_pilot(where, maps_hub):
    d = make(where)
    p = stack.plan(project_arg=str(d))
    # lazy on CARLA: the pilots start for the first run's map
    assert p.seat.index == 2 and p.simulator == "carla" and p.map is None
    assert p.record["maps"] == ["Town01"] and p.record["map"] is None
    assert "map_path" not in p.record["pilot_contract"]
    a = p.launch_args
    assert (a["scenario_domain"], a["ego_domain"]) == ("17", "18")
    assert (a["ssv2_port"], a["agent_port"], a["world_port"]) == ("5755", "5760", "5791")
    assert a["ego_launch"] == "none" and a["nest_side"] == "true"
    assert a["map_path"] == "" and "pilots" not in a  # the keeper runs the pilots
    # the world server writes the signal table into the state dir, the bridge reads it there
    assert a["sim_args"] == f"state_dir:={d / 'state'}"
    assert a["side_args"] == f"state_dir:={d / 'state'}"
    assert a["log_dir"] == str(where["tmp"] / "logs" / "proj" / "stacks" / p.stack_id / "proc")
    pilots = p.record["pilot_entries"]
    assert [(x["entity"], x["kind"], x["domain"]) for x in pilots] == [
        ("ego", "autoware", 18), ("follower", "command", 19), ("lead", "carla_basic", 20)]
    follower = pilots[1]
    assert follower["run"] == ["python3", str((d / "pilots" / "follower.py").resolve())]
    assert pilots[0]["vehicle"] == "tesla_model3" and pilots[0]["seed"] == 0
    assert p.env["SWB_MAPS"] == str(d / "assets" / "maps")
    assert p.env["SWB_CARLA_USD"] == str(d / "assets" / "carla-usd")
    assert p.env["CYCLONEDDS_URI"].endswith("cyclonedds-localhost.xml")
    assert "CUDA_VISIBLE_DEVICES" not in p.env
    r = p.record
    assert r["ports"]["web"] == 8100 and r["domains"]["pilots"]["lead"] == 20
    assert r["pilots"][0]["ready"] == stack.EGO_READY
    # a carla_basic pilot is ready when its agent registered the entity with the relay
    assert r["pilots"][1]["ready"][-4:] == ["registered", "--relay", "tcp://localhost:5760",
                                           r["pilots"][1]["entity"]]
    assert r["fingerprint"]["swb_yaml"].startswith("sha256:")
    assert r["pilot_contract"]["release_notify_endpoint"] == "tcp://127.0.0.1:5756"


def test_plan_map_given(where, maps_hub):
    d = make(where)
    p = stack.plan(project_arg=str(d), map_name="Town01")
    town = d / "assets" / "maps" / "Town01"
    assert p.map == "Town01" and p.record["map_path"] == str(town)
    c = p.record["pilot_contract"]
    assert c["map_path"] == str(town)
    assert c["traffic_light_state_path"] == str(d / "state" / "Town01" / "carla" /
                                                "traffic_lights.yaml")
    with pytest.raises(stack.StackRefused, match="map Town05 is not fetched"):
        stack.plan(project_arg=str(d), map_name="Town05")


MAPS_YAML = """simulator: carla
assets:
  - dataset: carla-town-autoware-maps
    take: [Town01, Town05]
maps: [Town05, Town01]
pilots:
  ego: {kind: autoware, vehicle: tesla_model3}
  lead: {kind: carla_basic}
  follower: {kind: command, run: [python3, pilots/follower.py]}
"""


def test_maps_key(where, maps_hub):
    d = make(where, swb_yaml=MAPS_YAML)
    assert project.check_project(d) == []
    p = stack.plan(project_arg=str(d))
    assert p.record["maps"] == ["Town05", "Town01"] and p.map is None
    (d / "swb.yaml").write_text(MAPS_YAML.replace("[Town05, Town01]", "[Town05, Town03]"))
    f = project.check_project(d)
    assert [x.what for x in f] == ["maps: Town03 is not fetched (assets/maps/Town03)"]
    assert f[0].refused
    (d / "swb.yaml").write_text(MAPS_YAML.replace("[Town05, Town01]", "Town05"))
    assert [x.what for x in project.check_project(d)] == ["maps: is not a list of map names"]


def test_switch_problem(where, maps_hub):
    d = make(where, swb_yaml=MAPS_YAML.replace("maps: [Town05, Town01]\n", ""))
    rec = stack.plan(project_arg=str(d)).record
    assert rec["maps"] == ["Town01"]
    assert stack.switch_problem(rec, "Town01") is None
    assert "not one of the stack's maps (Town01)" in stack.switch_problem(rec, "Town05")
    assert "map Town03 is not fetched" in stack.switch_problem(rec, "Town03")
    rec = dict(rec, simulator="isaac", map="Town01", maps=["Town01", "Town05"])
    assert "restart the stack with map:=Town05" in stack.switch_problem(rec, "Town05")


def test_plan_args_pass_stack_launch(where, maps_hub):
    pytest.importorskip("launch")
    spec = importlib.util.spec_from_file_location("stack_launch", LAUNCH / "stack.launch.py")
    sl = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sl)
    p = stack.plan(project_arg=str(make(where)))
    a = p.launch_args
    parsed = sl.parse_pilots(json.dumps(p.record["pilot_entries"]), a["sim"], a["ego_domain"],
                             a["scenario_domain"])
    assert [p["domain"] for p in parsed] == ["18", "19", "20"]
    assert stack.PILOT_FILES == sl.PILOT_KINDS


def test_refused_on_project_check(where, maps_hub):
    d = where["tmp"] / "proj"
    project.new_project(d, "carla-first-drive")  # not fetched
    with pytest.raises(stack.StackRefused, match="project check") as e:
        stack.plan(project_arg=str(d))
    assert any(f.refused for f in e.value.findings)


def test_refused_on_taken_seat(where, maps_hub):
    d = make(where)
    s = seat.resolve(config.host_settings(d))
    with seat.acquire(s, "other"):
        with pytest.raises(stack.StackRefused, match="seat 2 is taken"):
            stack.plan(project_arg=str(d))
    assert stack.plan(project_arg=str(d)).seat.index == 2


def test_seat_auto_skips_held(where, maps_hub):
    d = make(where)
    settings = config.host_settings(d, {"seat": "auto"})
    with seat.acquire(seat.table(settings)[0], "other"):
        assert stack.plan(project_arg=str(d), seat_arg="auto").seat.index == 1


def test_param_in_stack_pilot_refused(where, maps_hub):
    d = make(where, swb_yaml="""simulator: carla
assets:
  - dataset: carla-town-autoware-maps
    take: [Town01]
pilots:
  ego: {kind: autoware}
  lead: {kind: carla_basic}
  follower: {kind: command, run: [python3, pilots/follower.py, "{param:policy}"]}
""")
    with pytest.raises(stack.StackRefused, match=r"\{param:policy\}.*lifetime: run"):
        stack.plan(project_arg=str(d))
    # as a run pilot it is the runner's
    (d / "swb.yaml").write_text((d / "swb.yaml").read_text().replace(
        '"{param:policy}"]}', '"{param:policy}"], lifetime: run}'))
    p = stack.plan(project_arg=str(d))
    assert [x["entity"] for x in p.record["pilot_entries"]] == ["ego", "lead"]
    assert [x["lifetime"] for x in p.pilots] == ["stack", "run", "stack"]


def test_pattern_binds_each_entity():
    sc = project.Scenario(Path("s.xosc"), entities=[
        project.Entity("bg_av_1", "agent"), project.Entity("bg_av_2", "agent"),
        project.Entity("npc", ""), project.Entity("ego", "Autoware")])
    bound = stack.bind_pilots({"pilots": {"ego": {"kind": "autoware"},
                                          "bg_av_*": {"kind": "autoware"}}}, [(sc.path, sc)])
    assert [(b.entity, b.key) for b in bound] == [
        ("ego", "ego"), ("bg_av_1", "bg_av_*"), ("bg_av_2", "bg_av_*")]


def test_domains_run_out_and_isaac_rules():
    s = seat.table(config.merge(config.DEFAULTS, {"seats": {"domains": {"base": 1, "stride": 2}}}))[0]
    two = [stack.Bound("ego", "ego", {"kind": "autoware"}),
           stack.Bound("b", "b", {"kind": "carla_basic"})]
    with pytest.raises(stack.StackRefused, match="1 pilot domains"):
        stack.assign_domains(two, s, "carla")
    s = seat.table(config.DEFAULTS)[0]
    with pytest.raises(stack.StackRefused, match="one pilot has a vehicle side"):
        stack.assign_domains([stack.Bound("a", "a", {"kind": "autoware"}),
                              stack.Bound("b", "b", {"kind": "autoware"})], s, "isaac")
    bound = stack.assign_domains([stack.Bound("a", "a", {"kind": "command", "run": ["x"]}),
                                  stack.Bound("ego", "ego", {"kind": "autoware"})], s, "isaac")
    assert {b.entity: b.domain for b in bound} == {"ego": 2, "a": 3}


def test_isaac_plan(where, maps_hub):
    d = make(where, template="isaac")
    p = stack.plan(project_arg=str(d))
    assert p.launch_args["sim"] == "isaac"
    assert p.launch_args["sim_args"] == (f"world:={d / 'assets' / 'maps' / 'Town01'} ego:=default"
                                         f" state_dir:={d / 'state'}")
    assert p.launch_args["side_args"] == f"state_dir:={d / 'state'}"


def test_fingerprint_diff_names_parts(where, maps_hub):
    d = make(where)
    a = stack.fingerprint(d)
    assert stack.fingerprint_diff(a, stack.fingerprint(d)) == []
    (d / "swb.yaml").write_text((d / "swb.yaml").read_text() + "\n# edited\n")
    b = stack.fingerprint(d)
    diff = stack.fingerprint_diff(a, b)
    assert len(diff) == 1 and diff[0].startswith("swb.yaml (stack sha256:")
    b["build"] = {"swb": "abc", "submodules": {"src/x": "1"}}
    a["build"] = {"swb": "abd", "submodules": {"src/x": "2"}}
    diff = stack.fingerprint_diff(a, b)
    assert "the build (stack swb abd, now abc)" in diff
    assert "the build's submodules (src/x)" in diff


def test_substitute():
    ctx = {"seed": 7, "run_dir": "/r", "run_id": "id", "params": {"p": 3}}
    assert stack.substitute(["--s", "{seed}", "{run_dir}/x", "{param:p}", "{other}"], ctx, "w") \
        == ["--s", "7", "/r/x", "3", "{other}"]


def test_keeper_marks_ready_and_links(where, maps_hub):
    d = make(where)
    p = stack.plan(project_arg=str(d), map_name="Town01")
    for x in p.record["pilots"]:
        x["ready"] = ["true"]
    stack.write_plan(p)
    assert stack.keep(p.stack_dir, poll=0.01, once=True, pilot_command=fake_pilot) == 0
    rec = json.loads(p.stack_json.read_text())
    assert rec["state"] == "ready" and rec["pid"] == os.getpid()
    assert all(x["ready_at"] for x in rec["pilots"])
    # the keeper ran the pilots (kind none would not be), and stopped them on its way out
    assert all(x["map"] == "Town01" and x["started_at"] for x in rec["pilots"])
    assert all(x["pid"] for x in rec["pilots"])
    for x in rec["pilots"]:
        assert not seat.pid_alive(x["pid"]) or _zombie(x["pid"])
    link = where["tmp"] / "logs" / "proj" / "seat-2-current"
    assert link.resolve() == p.stack_dir.resolve()
    ev = [json.loads(l)["event"] for l in (p.stack_dir / "events.jsonl").read_text().splitlines()]
    assert ev[0] == "stack.start" and ev[-1] == "stack.ready"
    assert seat.holder(2, str(where["tmp"] / "locks")) is None  # once: released


def _zombie(pid):
    try:
        return Path(f"/proc/{pid}/stat").read_text().split()[2] == "Z"
    except OSError:
        return True


def test_lazy_keeper_ready_without_pilots(where, maps_hub):
    d = make(where)
    p = stack.plan(project_arg=str(d))
    stack.write_plan(p)
    started = []

    def cmd(*a):
        started.append(a)
        return fake_pilot(*a)
    assert stack.keep(p.stack_dir, poll=0.01, once=True, pilot_command=cmd) == 0
    rec = json.loads(p.stack_json.read_text())
    assert rec["state"] == "ready" and rec["map"] is None and started == []


def test_pilot_argv_has_the_maps_contract(where, maps_hub):
    d = make(where)
    rec = stack.plan(project_arg=str(d), map_name="Town01").record
    ego = rec["pilots"][0]
    argv = stack.pilot_argv(rec, ego, rec["pilot_entries"][0], Path("/l"))
    assert argv[:5] == ["play_launch", "launch", "--disable-web-ui", "--log-dir", "/l"]
    assert argv[argv.index("--composable-respawn") + 2].endswith("pilots/autoware.launch.xml")
    words = dict(w.split(":=", 1) for w in argv if ":=" in w)
    assert words["map_path"] == str(d / "assets" / "maps" / "Town01")
    assert words["entity"] == "ego" and words["relay"] == "tcp://localhost:5760"


def test_keeper_refuses_a_held_seat(where, maps_hub):
    d = make(where)
    p = stack.plan(project_arg=str(d))
    stack.write_plan(p)
    with seat.acquire(p.seat, "other"):
        assert stack.keep(p.stack_dir, once=True) == 3
    assert json.loads(p.stack_json.read_text())["state"] == "stopped"


def test_source_id_ignores_docs_and_commits(tmp_path):
    import subprocess as sp
    r = tmp_path / "ws"
    (r / "src").mkdir(parents=True)
    (r / "src" / "a.py").write_text("x = 1\n")
    (r / "README.md").write_text("doc\n")
    g = lambda *a: sp.run(["git", "-c", "user.name=t", "-c", "user.email=t@t", *a], cwd=r,  # noqa: E731
                          check=True, capture_output=True)
    g("init", "-q")
    g("add", "-A")
    g("commit", "-qm", "1")
    base = stack.source_id(r)
    assert base.startswith("src:") and "+" not in base
    (r / "README.md").write_text("doc 2\n")
    assert stack.source_id(r) == base          # a docs edit
    g("commit", "-qam", "docs")
    assert stack.source_id(r) == base          # its commit
    (r / "src" / "a.py").write_text("x = 2\n")
    dirty = stack.source_id(r)
    assert dirty.startswith(base + "+")
    g("commit", "-qam", "code")
    assert stack.source_id(r) not in (base, dirty)


def test_carla_root_gives_carla_basic_its_agents(where, maps_hub):
    d = make(where, maps_hub=maps_hub)
    sysf = where["tmp"] / "system" / "host.yaml"
    h = yaml.safe_load(sysf.read_text())
    h["carla"] = {"root": str(where["tmp"] / "CARLA_0.9.16"), "start": "external"}
    sysf.write_text(yaml.safe_dump(h))
    p = stack.plan(project_arg=str(d), map_name="Town01")
    assert p.record["pilot_contract"]["carla_pythonapi"] == str(
        where["tmp"] / "CARLA_0.9.16" / "PythonAPI" / "carla")


def test_registered_asks_the_relay():
    import socket
    import sys as _sys
    root = Path(__file__).resolve().parents[2]
    for sub in ("swb_agent", "scenario_agent_relay"):
        _sys.path.insert(0, str(root / sub))
    server = pytest.importorskip("scenario_agent_relay.server")
    P = pytest.importorskip("swb_agent.protocol")
    relay = server.AgentServer(host="127.0.0.1", port=0)
    relay.start()
    url = f"tcp://127.0.0.1:{relay.port}"
    try:
        assert not stack.is_registered(url, "cut_in")
        agent = socket.create_connection(("127.0.0.1", relay.port), timeout=5)
        agent.sendall(P.encode(P.register("cut_in", agent="test")))
        deadline = time.time() + 5
        while time.time() < deadline and not stack.is_registered(url, "cut_in"):
            time.sleep(0.1)
        assert stack.is_registered(url, "cut_in")
        assert stack.main(["registered", "--relay", url, "cut_in"]) == 0
        assert stack.main(["registered", "--relay", url, "lead"]) == 1
        agent.close()
    finally:
        relay.stop()
    assert not stack.is_registered("tcp://127.0.0.1:1", "cut_in")


def test_isaac_root_reaches_the_world_server(where, maps_hub):
    d = make(where, template="isaac")
    sysf = where["tmp"] / "system" / "host.yaml"
    h = yaml.safe_load(sysf.read_text())
    h["isaac"] = {"root": str(where["tmp"] / "isaac-sim")}
    sysf.write_text(yaml.safe_dump(h))
    assert stack.plan(project_arg=str(d)).env["ISAAC_ROOT"] == str(where["tmp"] / "isaac-sim")


def test_retired_args_refused_by_name(where, maps_hub):
    """roadmap 076 step 10: the old stack's arguments are refused, naming what replaced them."""
    assert stack.retired_args(["side_args", "seat"]) == []
    [line] = stack.retired_args(["ego_launch"])
    assert line.startswith("ego_launch:= is retired") and "pilots:" in line
    assert "swb.launch.xml" in line
    d = make(where)
    for name in ("ego_launch", "ego_launch_args", "agent", "map_path"):
        with pytest.raises(stack.StackRefused, match=f"stack_args: {name}:= is retired"):
            stack.plan(project_arg=str(d), stack_args={name: "x"})
    # the stack's own arguments still pass
    p = stack.plan(project_arg=str(d), stack_args={"side_args": "weather:=ClearNoon"})
    assert p.launch_args["side_args"] == "weather:=ClearNoon"
