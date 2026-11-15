"""swb.stack: the stack's plan (seat -> arguments, pilots bound, refusals) and its keeper."""
import importlib.util
import json
import os
import time
from pathlib import Path

import pytest
import yaml

from conftest import fake_isaac
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
    assert "SWB_VEHICLE_USD" not in p.env   # a CARLA project fetches no USD set (G36)
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
    contacts = where["tmp"] / "logs" / "proj" / "stacks" / p.stack_id / "episodes" / "contacts"
    # each run's contact reports: the bridge writes them under the stack, the run moves them
    assert a["side_args"] == f"state_dir:={d / 'state'} contact_report_dir:={contacts}"
    assert p.record["episodes"] == {"gap_trace": None, "contacts": str(contacts)}
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
    assert p.launch_args["side_args"].startswith(f"state_dir:={d / 'state'} contact_report_dir:=")


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
    argv = argv[argv.index("--") + 1:]   # after the autoware environment's prelude
    assert argv[:5] == ["play_launch", "launch", "--disable-web-ui", "--log-dir", "/l"]
    assert argv[argv.index("--composable-respawn") + 2].endswith("pilots/autoware.launch.xml")
    words = dict(w.split(":=", 1) for w in argv if ":=" in w)
    assert words["map_path"] == str(d / "assets" / "maps" / "Town01")
    assert words["entity"] == "ego" and words["relay"] == "tcp://localhost:5760"


SIMPLE_CARLA = """simulator: carla
assets:
  - dataset: carla-town-autoware-maps
    take: [Town01]
pilots:
  ego: {kind: simple, vehicle: tesla_model3, args: {max_speed_kmh: "30"}}
"""


def test_plan_simple_pilot(where, maps_hub):
    d = make(where, template="carla-first-drive", swb_yaml=SIMPLE_CARLA)
    rec = stack.plan(project_arg=str(d), map_name="Town01").record
    ego = rec["pilots"][0]
    assert (ego["entity"], ego["kind"], ego["domain"]) == ("ego", "simple", 18)
    # ready when simple_pilot registered the entity with the relay
    assert ego["ready"][-4:] == ["registered", "--relay", "tcp://localhost:5760", "ego"]
    argv = stack.pilot_argv(rec, ego, rec["pilot_entries"][0], Path("/l"))
    assert argv[argv.index("--composable-respawn") + 2].endswith("pilots/simple.launch.py")
    words = dict(w.split(":=", 1) for w in argv if ":=" in w)
    assert words["map_path"] == str(d / "assets" / "maps" / "Town01")
    assert words["sim"] == "carla" and words["max_speed_kmh"] == "30"
    assert words["carla_port"] and words["release_notify_endpoint"]
    assert stack.Bound("ego", "ego", {"kind": "simple"}).sided()
    assert (Path(stack.swb_launch_share()) / "launch" / "pilots" / "simple.launch.py").is_file() \
        or (LAUNCH / "pilots" / "simple.launch.py").is_file()


def test_isaac_plan_simple_pilot_turns_ground_truth_on(where, maps_hub, tmp_path):
    base = tmp_path / "aib" / "ego.yaml"
    base.parent.mkdir()
    base.write_text("rig: tesla_model3.rig.yaml\ndomain: 1\nnode_name: isaac_vehicle\n")
    d = make(where, template="isaac")
    text = (d / "swb.yaml").read_text().replace("kind: autoware", "kind: simple")
    (d / "swb.yaml").write_text(text)
    real = stack.aib_ego_config
    stack.aib_ego_config = lambda: base
    try:
        p = stack.plan(project_arg=str(d))
    finally:
        stack.aib_ego_config = real
    ego = p.stack_dir / stack.ISAAC_EGO
    assert p.launch_args["sim_args"] == (f"world:={d / 'assets' / 'maps' / 'Town01'} ego:={ego}"
                                         f" state_dir:={d / 'state'}")
    cfg = p.record["isaac_ego"]["config"]
    assert cfg["ground_truth"] == {"topic": "/ground_truth/odom"}
    assert cfg["rig"] == str(base.parent / "tesla_model3.rig.yaml")
    assert cfg["node_name"] == "isaac_vehicle"
    stack.write_plan(p)
    assert yaml.safe_load(ego.read_text()) == cfg
    # one vehicle side on Isaac: two simple pilots are refused
    s = seat.table(config.DEFAULTS)[0]
    with pytest.raises(stack.StackRefused, match="one pilot has a vehicle side"):
        stack.assign_domains([stack.Bound("a", "a", {"kind": "simple"}),
                              stack.Bound("b", "b", {"kind": "simple"})], s, "isaac")


def test_isaac_ego_config_needs_aib(tmp_path):
    with pytest.raises(stack.StackRefused, match="aib"):
        stack.isaac_ego_with_ground_truth(tmp_path / "missing.yaml")


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


def test_isaac_root_reaches_the_world_server(where, maps_hub, monkeypatch):
    d = make(where, template="isaac")
    root, setup = fake_isaac(where["tmp"], monkeypatch)
    sysf = where["tmp"] / "system" / "host.yaml"
    h = yaml.safe_load(sysf.read_text())
    h["isaac"] = {"root": str(root)}
    h["autoware"] = {"setup": str(setup)}
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
    side = stack._named(p.launch_args["side_args"])
    assert side["weather"] == "ClearNoon" and side["state_dir"] == str(d / "state")


def test_seat_carla_port_is_the_stacks(where, maps_hub):
    d = make(where, maps_hub=maps_hub)
    sysf = where["tmp"] / "system" / "host.yaml"
    h = yaml.safe_load(sysf.read_text())
    h["seats"] = {"carla": {"base": 2000, "stride": 10}}
    h["carla"] = {"start": "external", "port": 2000}
    sysf.write_text(yaml.safe_dump(h))
    p = stack.plan(project_arg=str(d), map_name="Town01")   # seat 2 (the fixture's)
    assert p.launch_args["carla_port"] == "2020"
    assert p.record["pilot_contract"]["carla_port"] == "2020"
    assert stack.seat_of(p.record).carla_port == 2020


def test_autoware_pilot_runs_in_the_autoware_environment(where, maps_hub, tmp_path):
    d = make(where, maps_hub=maps_hub)
    p = stack.plan(project_arg=str(d), map_name="Town01")
    env = p.record["autoware_env"]
    assert env["setup"] == "/opt/autoware/1.5.0/setup.bash" and env["use_overlay"] == "true"
    rec = dict(p.record, pilots_dir="/x")
    ego = next(x for x in rec["pilots"] if x["kind"] == "autoware")
    lead = next(x for x in rec["pilots"] if x["kind"] == "carla_basic")
    entry = {"entity": "ego", "kind": "autoware", "domain": ego["domain"]}
    argv = stack.pilot_argv(rec, ego, entry, tmp_path)
    assert argv[:2] == ["bash", str(stack.AUTOWARE_ENV)]
    assert argv[argv.index("--") + 1:][:2] == ["play_launch", "launch"]
    plain = stack.pilot_argv(rec, lead, {"entity": "lead", "kind": "carla_basic",
                                         "domain": lead["domain"]}, tmp_path)
    assert plain[0] == "play_launch"
    # the prelude runs the command after sourcing; with stand-ins for every setup
    ws = tmp_path / "ws"
    (ws / "scripts").mkdir(parents=True)
    (ws / "install").mkdir()
    (ws / "install" / "setup.bash").write_text("export SWB_WS_SOURCED=1\n")
    (ws / "scripts" / "autoware_overlay.bash").write_text(
        "swb_autoware_overlay_source() { export OVERLAY_ARGS=\"$1 $2\"; }\n")
    (ws / "scripts" / "autoware_tensorrt.bash").write_text(
        "swb_tensorrt_source() { export LD_LIBRARY_PATH=\"$1:$LD_LIBRARY_PATH\"; }\n")
    setup = tmp_path / "aw.bash"
    setup.write_text("export AW_SOURCED=1 CYCLONEDDS_URI=file:///opt/aw.xml\n")
    import subprocess as sp
    out = sp.run(["bash", str(stack.AUTOWARE_ENV), str(ws), str(setup), "", "false", "", "",
                  "--", "bash", "-c", "echo $AW_SOURCED $SWB_WS_SOURCED $OVERLAY_ARGS "
                  "${LD_LIBRARY_PATH%%:*}"], capture_output=True, text=True).stdout.split()
    assert out == ["1", "1", f"{ws}/third_party/autoware_overlay", "false",
                   f"{ws}/third_party/tensorrt/lib"]
    # the stack's DDS profile survives Autoware's setup (which sets its own)
    env = dict(os.environ, CYCLONEDDS_URI="file:///stack.xml", ROS_DOMAIN_ID="7")
    out = sp.run(["bash", str(stack.AUTOWARE_ENV), str(ws), str(setup), "", "false", "", "",
                  "--", "bash", "-c", "echo $CYCLONEDDS_URI $ROS_DOMAIN_ID"],
                 capture_output=True, text=True, env=env).stdout.split()
    assert out == ["file:///stack.xml", "7"]


def test_check_ready_keeps_the_checks_reason():
    import sys as _sys
    why = []
    cmd = [_sys.executable, "-c", "print('[ego-health] not ready: nothing publishes /x'); "
           "import sys; sys.exit(1)"]
    assert not stack.check_ready(cmd, dict(os.environ), why=why)
    assert why == ["[ego-health] not ready: nothing publishes /x"]
    assert stack.check_ready([_sys.executable, "-c", "pass"], dict(os.environ), why=why)




# --- roadmap 077: world:, isaac:, the gap trace, the weather --------------------------------

MULTI = (project.TEMPLATES / "multi-pilot" / "swb.yaml").read_text()


def test_world_reaches_the_side_beside_swbs(where, maps_hub):
    """G10: world: goes to the bridge and the world server, merged with swb's own arguments;
    stack_args merge too, and a name swb owns is refused."""
    d = make(where)
    (d / "config").mkdir()
    (d / "config" / "bridge.yaml").write_text("{}\n")
    (d / "config" / "carla.yaml").write_text("{}\n")
    (d / "swb.yaml").write_text(MULTI + "world:\n  weather: {preset: WetNoon, cloudiness: 30}\n"
                                "  npc_motion: kinematic\n  config_file: config/bridge.yaml\n"
                                "  world_config_file: config/carla.yaml\n"
                                "logging: {gap_trace: true}\n")
    p = stack.plan(project_arg=str(d))
    side, sim = stack._named(p.launch_args["side_args"]), stack._named(p.launch_args["sim_args"])
    stack_dir = where["tmp"] / "logs" / "proj" / "stacks" / p.stack_id
    assert side == {"state_dir": str(d / "state"),
                    "gap_trace_dir": str(stack_dir / "episodes" / "gap_trace"),
                    "contact_report_dir": str(stack_dir / "episodes" / "contacts"),
                    "weather": "WetNoon", "npc_motion": "kinematic",
                    "config_file": str((d / "config" / "bridge.yaml").resolve())}
    assert sim == {"state_dir": str(d / "state"),
                   "config_file": str((d / "config" / "carla.yaml").resolve())}
    assert p.record["weather"] == {"parameters": {"cloudiness": 30.0}, "state": "pending"}
    stack.write_plan(p)
    assert (stack_dir / "episodes" / "gap_trace").is_dir()
    # stack_args: merged by name, the developer's value wins; swb's own names are refused
    p = stack.plan(project_arg=str(d), stack_args={"side_args": "weather:=ClearNoon",
                                                   "sim_args": "vehicle_physics:=/p.yaml"})
    assert stack._named(p.launch_args["side_args"])["weather"] == "ClearNoon"
    assert stack._named(p.launch_args["sim_args"]) == {
        "state_dir": str(d / "state"), "config_file": str((d / "config" / "carla.yaml").resolve()),
        "vehicle_physics": "/p.yaml"}
    assert p.record["pilot_contract"]["carla_physics"] == "/p.yaml"
    for args, name in (({"side_args": "state_dir:=/elsewhere"}, "state_dir"),
                       ({"side_args": "gap_trace_dir:=/t"}, "gap_trace_dir"),
                       ({"sim_args": "port:=1"}, "port")):
        with pytest.raises(stack.StackRefused, match=f"{name}:= is swb's"):
            stack.plan(project_arg=str(d), stack_args=args)
    # contact_report_dir of the user's: theirs, not collected per run
    (d / "swb.yaml").write_text(MULTI + "world: {contact_report_dir: off}\n")
    p = stack.plan(project_arg=str(d))
    assert stack._named(p.launch_args["side_args"])["contact_report_dir"] == "off"
    assert p.record["episodes"] == {"gap_trace": None, "contacts": None}


def test_a_judge_that_needs_the_trace_turns_it_on(where, maps_hub):
    d = make(where)
    (d / "swb.yaml").write_text(MULTI + "judges:\n  - {name: gap, run: [python3, -m, swb.gap,"
                                " oracle], needs: [gap_trace]}\n")
    p = stack.plan(project_arg=str(d))
    assert p.record["episodes"]["gap_trace"].endswith("episodes/gap_trace")
    assert "gap_trace_dir:=" in p.launch_args["side_args"]


ISAAC = """simulator: isaac
assets:
  - dataset: carla-town-autoware-maps
    take: [Town01, Town05]
  - dataset: carla-0.9.16-isaac-usd
    take: [Town01, vehicles]
pilots:
  ego: {kind: autoware, vehicle: tesla_model3}
isaac: {ego: config/ego_lockstep.yaml, render: headless, follow: false, physics_hz: 240,
        preload: [Town05], wheels_topic: false}
"""


def test_isaac_options_and_preload(where, maps_hub):
    """G6: isaac: reaches the world server beside swb's world, ego and state_dir; a preloaded
    map is one the stack can switch to."""
    d = make(where, template="isaac")
    (d / "config").mkdir()
    (d / "config" / "ego_lockstep.yaml").write_text("lockstep: true\n")
    (d / "swb.yaml").write_text(ISAAC)
    assets.fetch(d, out=lambda s: None)
    p = stack.plan(project_arg=str(d))
    sim = stack._named(p.launch_args["sim_args"])
    maps = d / "assets" / "maps"
    assert sim == {"world": f"{maps / 'Town01'},{maps / 'Town05'}",
                   "ego": str((d / "config" / "ego_lockstep.yaml").resolve()),
                   "state_dir": str(d / "state"), "render": "headless", "physics_hz": "240",
                   "follow": "false", "wheels_topic": "false"}
    assert p.record["maps"] == ["Town01", "Town05"] and p.record["preload"] == ["Town05"]
    assert stack.switch_problem(p.record, "Town05") is None
    rec = dict(p.record, preload=[])
    assert "add Town05 to isaac.preload" in stack.switch_problem(rec, "Town05")
    with pytest.raises(stack.StackRefused, match="world:= is swb's"):
        stack.plan(project_arg=str(d), stack_args={"sim_args": "world:=/other"})
    # the developers' ego:= still wins over isaac.ego
    p = stack.plan(project_arg=str(d), stack_args={"sim_args": "ego:=none"})
    assert stack._named(p.launch_args["sim_args"])["ego"] == "none"


def test_keeper_sets_the_weather_before_ready(where, maps_hub):
    d = make(where)
    (d / "swb.yaml").write_text(MULTI + "world: {weather: {cloudiness: 30}}\n")
    p = stack.plan(project_arg=str(d))
    stack.write_plan(p)
    answers = iter([(None, "waiting for service"), (False, "no world server yet"),
                    (True, "ok")])
    calls = []

    def call(rec):
        calls.append(rec["weather"]["parameters"])
        return next(answers)
    old = stack.WEATHER_RETRY
    stack.WEATHER_RETRY = 0.01
    try:
        assert stack.keep(p.stack_dir, poll=0.01, once=True, pilot_command=fake_pilot,
                          weather_call=call) == 0
    finally:
        stack.WEATHER_RETRY = old
    rec = json.loads(p.stack_json.read_text())
    assert rec["state"] == "ready" and len(calls) == 3
    assert rec["weather"]["state"] == "applied" and rec["weather"]["attempts"] == 3
    ev = [json.loads(l)["event"] for l in (p.stack_dir / "events.jsonl").read_text().splitlines()]
    assert ev.index("weather.applied") < ev.index("stack.ready")


def test_weather_that_never_takes_fails_the_stack(where, maps_hub):
    d = make(where)
    (d / "swb.yaml").write_text(MULTI + "world: {weather: {cloudy: 30}}\n")
    p = stack.plan(project_arg=str(d))
    stack.write_plan(p)
    old = stack.WEATHER_RETRY, stack.WEATHER_DEADLINE
    stack.WEATHER_RETRY, stack.WEATHER_DEADLINE = 0.01, 0.05
    try:
        code = stack.keep(p.stack_dir, poll=0.01, once=True, pilot_command=fake_pilot,
                          weather_call=lambda rec: (False, "unknown parameter cloudy"))
    finally:
        stack.WEATHER_RETRY, stack.WEATHER_DEADLINE = old
    rec = json.loads(p.stack_json.read_text())
    assert code == 2 and rec["state"] == "stopped" and rec["weather"]["state"] == "failed"
    assert "unknown parameter cloudy" in rec["error"]


def test_a_stopping_keeper_says_so_before_its_pilots_stop(where, maps_hub, monkeypatch):
    """play_launch gives the keeper seconds before SIGKILL, less than the pilots' grace: the
    record must not still say ready while the pilots stop."""
    d = make(where)
    (d / "swb.yaml").write_text(MULTI + "world: {weather: {cloudy: 30}}\n")
    p = stack.plan(project_arg=str(d))
    stack.write_plan(p)
    seen = []
    real = stack.Pilots.stop_all

    def stop_all(self, rec=None):
        seen.append(json.loads(p.stack_json.read_text())["state"])
        return real(self, rec)
    monkeypatch.setattr(stack.Pilots, "stop_all", stop_all)
    monkeypatch.setattr(stack, "WEATHER_RETRY", 0.01)
    monkeypatch.setattr(stack, "WEATHER_DEADLINE", 0.05)
    assert stack.keep(p.stack_dir, poll=0.01, once=True, pilot_command=fake_pilot,
                      weather_call=lambda rec: (False, "no")) == 2
    assert seen == ["stopping"]
    assert json.loads(p.stack_json.read_text())["state"] == "stopped"


def test_the_stack_is_ready_only_once_the_bridge_serves(where, maps_hub):
    """An Isaac world server answers minutes after it starts, and the bridge binds SSv2's port
    only then: a stack marked ready before that sent the first run into SSv2's
    SIMULATOR_INITIALIZE_TIMEOUT (seen 2026-11-15, Town10HD)."""
    d = make(where)
    p = stack.plan(project_arg=str(d), map_name="Town01")
    for x in p.record["pilots"]:
        x["ready"] = ["true"]
    stack.write_plan(p)
    answers = iter([False, False, True])
    seen = []

    def probe(rec):
        seen.append(json.loads(p.stack_json.read_text())["state"])
        return next(answers)

    assert stack.keep(p.stack_dir, poll=0.01, once=True, pilot_command=fake_pilot,
                      world_probe=probe) == 0
    assert seen == ["starting", "starting", "starting"]
    assert json.loads(p.stack_json.read_text())["state"] == "ready"
    events = [json.loads(x)["event"] for x in (p.stack_dir / "events.jsonl").read_text().splitlines()]
    assert events.index("world.ready") < events.index("stack.ready")


def test_a_named_seat_whose_ports_are_in_use_is_refused(where, maps_hub):
    """roadmap 077, G30: the stack refuses seat 2 when something already serves its SSv2
    port, naming the port, rather than starting a bridge that cannot bind it."""
    import socket
    d = make(where)
    s2 = seat.table(config.host_settings(d), seat.seats_source(d))[2]
    sock = socket.socket()
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("127.0.0.1", s2.ssv2_port))
    sock.listen()
    try:
        with pytest.raises(stack.StackRefused, match=f"SSv2 {s2.ssv2_port}"):
            stack.plan(project_arg=str(d))
    finally:
        sock.close()
    assert stack.plan(project_arg=str(d)).seat.index == 2


def test_an_own_car_reaches_the_bridge_the_world_server_and_its_pilot(where, maps_hub,
                                                                       monkeypatch):
    """roadmap 077, G5: vehicles: is loaded: the merged catalog to the bridge and the
    scenarios (SWB_VEHICLES), the car's physics to CARLA's world server, its files to the
    pilot, its spec_hash recorded."""
    from swb import vehicles
    from test_vehicles import INSTALLED, fake_gen, own_spec
    monkeypatch.setattr(vehicles, "run_gen", fake_gen)
    monkeypatch.setattr(vehicles, "installed_catalog", lambda: INSTALLED)
    d = make(where)
    own_spec(d)
    s = yaml.safe_load((d / "swb.yaml").read_text())
    s["vehicles"] = ["vehicles/my_lincoln.yaml"]
    s["pilots"] = {"ego": {"kind": "simple", "vehicle": "my_lincoln"},
                   "*": {"kind": "none"}}
    (d / "swb.yaml").write_text(yaml.safe_dump(s))
    p = stack.plan(project_arg=str(d), check=None)
    own = d / "state" / "vehicles"
    assert p.launch_args["vehicle_catalogs"] == str(own / "catalogs" / "vehicles.xosc")
    assert p.env["SWB_VEHICLES"] == str(own / "catalogs")
    assert f"vehicle_physics:={own / 'my_lincoln' / 'carla' / 'carla_physics.yaml'}" in \
        p.launch_args["sim_args"]
    [ego] = [e for e in p.record["pilot_entries"] if e["entity"] == "ego"]
    assert ego["args"]["spec"] == str(d / "vehicles" / "my_lincoln.yaml")
    assert ego["args"]["vehicle_config"].endswith("my_lincoln/carla/carla_sensors.yaml")
    assert ego["args"]["description"].endswith("my_lincoln/description.urdf")
    assert p.record["spec_hash"]["ego"] == "hash-of-my_lincoln"
    assert p.record["vehicles"]["my_lincoln"]["spec_hash"] == "hash-of-my_lincoln"


def test_isaac_ground_truth_for_a_pilot_that_says_it_drives_on_it(where, maps_hub, tmp_path,
                                                                   monkeypatch):
    """A launch pilot running simple_pilot on Isaac (inputs: [ground_truth]) gets the
    world server's /ground_truth/odom, as a simple pilot does (seen live 2026-11-16: without
    it the pilot never had a pose and the run ended at SSv2's Autoware wait)."""
    base = tmp_path / "aib" / "ego.yaml"
    base.parent.mkdir()
    base.write_text("rig: tesla_model3.rig.yaml\ndomain: 1\nnode_name: isaac_vehicle\n")
    monkeypatch.setattr(stack, "aib_ego_config", lambda: base)
    d = make(where, template="isaac")
    text = (d / "swb.yaml").read_text().replace(
        "kind: autoware", 'kind: launch, launch: "swb_launch simple.launch.py", agent: own, '
        "inputs: [map, ground_truth]")
    (d / "swb.yaml").write_text(text)
    p = stack.plan(project_arg=str(d), check=None)
    assert p.record["isaac_ego"]["config"]["ground_truth"] == {"topic": "/ground_truth/odom"}



def test_the_isaac_world_server_finds_the_cars_in_the_projects_usd_set(where, maps_hub):
    """roadmap 077, G36: SWB_VEHICLE_USD defaulted to a host path; on a host without it
    every car was a box. The project's fetched set has them."""
    d = make(where, template="isaac")   # the fake set has vehicles/vehicles.json
    cat = d / "assets" / "carla-usd" / "vehicles"
    assert stack.plan(project_arg=str(d), check=None).env["SWB_VEHICLE_USD"] == str(cat)
