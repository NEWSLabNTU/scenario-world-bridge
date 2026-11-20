"""scensim.stack: the stack's plan (seat -> arguments, pilots bound, refusals) and its keeper."""
import importlib.util
import json
import os
import time
from pathlib import Path

import pytest
import yaml

from conftest import fake_isaac
from scensim import assets, config, project, seat, stack
from scensim import vehicles as vehiclesmod
from scensim import simulator as simmod

LAUNCH = Path(__file__).resolve().parents[2] / "scensim_launch" / "launch"


@pytest.fixture
def where(host):
    """Seat locks and logs under tmp (the system host.yaml), seat 2."""
    tmp = host["tmp"]
    (tmp / "system" / "host.yaml").write_text(yaml.safe_dump(
        {"seat_locks": str(tmp / "locks"), "logs": str(tmp / "logs"), "seat": 2}))
    return host


def make(where, template="multi-pilot", testbench_yaml=None, maps_hub=None):
    d = where["tmp"] / "proj"
    project.new_project(d, template)
    if testbench_yaml is not None:
        (d / "testbench.yaml").write_text(testbench_yaml)
    assets.fetch(d, out=lambda s: None)
    return d


def fake_pilot(rec, p, entry, log_dir):
    """A pilot that only sleeps (the keeper's pilot_command in tests)."""
    import sys
    return [sys.executable, "-c", "import time; time.sleep(60)"]


def test_plan_multi_pilot(where, maps_hub):
    d = make(where)
    p = stack.plan(project_arg=str(d))
    assert "SCENSIM_VEHICLE_USD" not in p.env   # a CARLA project fetches no USD set (G36)
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
    # and an installed car runs its spec's physics (G55), which the pilots read back
    physics = vehiclesmod.installed_physics("tesla_model3")
    assert a["sim_args"] == f"state_dir:={d / 'state'} vehicle_physics:={physics}"
    assert p.record["pilot_contract"]["carla_physics"] == str(physics)
    assert p.record["pilot_contract"]["carla_physics_mode"] == "verify"
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
    assert p.env["SCENSIM_MAPS"] == str(d / "assets" / "maps")
    assert p.env["SCENSIM_CARLA_USD"] == str(d / "assets" / "carla-usd")
    assert p.env["CYCLONEDDS_URI"].endswith("cyclonedds-localhost.xml")
    assert "CUDA_VISIBLE_DEVICES" not in p.env
    r = p.record
    assert r["ports"]["web"] == 8100 and r["domains"]["pilots"]["lead"] == 20
    assert r["pilots"][0]["ready"] == stack.EGO_READY
    # a carla_basic pilot is ready when its agent registered the entity with the relay
    assert r["pilots"][1]["ready"][-4:] == ["registered", "--relay", "tcp://localhost:5760",
                                           r["pilots"][1]["entity"]]
    assert r["fingerprint"]["testbench_yaml"].startswith("sha256:")
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
    d = make(where, testbench_yaml=MAPS_YAML)
    assert project.check_project(d) == []
    p = stack.plan(project_arg=str(d))
    assert p.record["maps"] == ["Town05", "Town01"] and p.map is None
    (d / "testbench.yaml").write_text(MAPS_YAML.replace("[Town05, Town01]", "[Town05, Town03]"))
    f = project.check_project(d)
    assert [x.what for x in f] == ["maps: Town03 is not fetched (assets/maps/Town03)"]
    assert f[0].refused
    (d / "testbench.yaml").write_text(MAPS_YAML.replace("[Town05, Town01]", "Town05"))
    assert [x.what for x in project.check_project(d)] == ["maps: is not a list of map names"]


def test_switch_problem(where, maps_hub):
    d = make(where, testbench_yaml=MAPS_YAML.replace("maps: [Town05, Town01]\n", ""))
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
    d = make(where, testbench_yaml="""simulator: carla
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
    (d / "testbench.yaml").write_text((d / "testbench.yaml").read_text().replace(
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
    (d / "testbench.yaml").write_text((d / "testbench.yaml").read_text() + "\n# edited\n")
    b = stack.fingerprint(d)
    diff = stack.fingerprint_diff(a, b)
    assert len(diff) == 1 and diff[0].startswith("testbench.yaml (stack sha256:")
    b["build"] = {"scensim": "abc", "submodules": {"src/x": "1"}}
    a["build"] = {"scensim": "abd", "submodules": {"src/x": "2"}}
    diff = stack.fingerprint_diff(a, b)
    assert "the build (stack scensim abd, now abc)" in diff
    assert "the build's submodules (src/x)" in diff


def test_a_failed_submodule_probe_is_unknown_and_not_a_different_build(
        where, maps_hub, monkeypatch, tmp_path, capsys):
    # 241, 2026-11-18: the keeper's `git submodule status` failed at a stack's start under
    # load; its fingerprint said {} and the run's had 11 pins, so the run was refused
    from scensim import results
    ws = tmp_path / "ws"
    (ws / ".git").mkdir(parents=True)

    def slow(*a, **k):
        raise results.subprocess.TimeoutExpired(a[0], k.get("timeout"))
    monkeypatch.setattr(results.subprocess, "run", slow)
    assert results.submodule_pins(ws) is None
    err = capsys.readouterr().err
    assert "WARN" in err and "git --no-optional-locks submodule status" in err \
        and "no answer within 30 s" in err
    d = make(where)
    keeper = stack.fingerprint(d, build={"scensim": "abc", "submodules": None})
    runner = stack.fingerprint(d, build={"scensim": "abc", "submodules": {"src/x": "1"}})
    notes = []
    assert stack.fingerprint_diff(keeper, runner, notes) == []
    assert notes == ["the build's submodules were not compared (unknown on the stack's side: "
                     "its `git submodule status` failed)"]
    # a known difference still refuses
    runner["build"]["scensim"] = "abd"
    assert stack.fingerprint_diff(keeper, runner) == ["the build (stack scensim abc, now abd)"]


def test_substitute():
    ctx = {"seed": 7, "run_dir": "/r", "run_id": "id", "params": {"p": 3}}
    assert stack.substitute(["--s", "{seed}", "{run_dir}/x", "{param:p}", "{other}"], ctx, "w") \
        == ["--s", "7", "/r/x", "3", "{other}"]


def test_map_path_is_a_runs_only():
    """swb 241: a command pilot had no map path; a lifetime: stack one outlives a switch."""
    ctx = {"seed": 0, "run_dir": "/r", "run_id": "id", "params": {}, "map_path": "/p/maps/Town01"}
    assert stack.substitute(["--map", "{map_path}"], ctx, "w") == ["--map", "/p/maps/Town01"]
    with pytest.raises(stack.StackRefused, match="lifetime: run"):
        stack.substitute("{map_path}", dict(ctx, map_path=""), "pilots.b (b)")


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
    # G59: no unbounded diagnostics.csv per nested launch
    assert argv[:6] == ["play_launch", "launch", "--disable-web-ui", "--disable-diagnostics",
                        "--log-dir", "/l"]
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
    d = make(where, template="carla-first-drive", testbench_yaml=SIMPLE_CARLA)
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
    assert (Path(stack.scensim_launch_share()) / "launch" / "pilots" / "simple.launch.py").is_file() \
        or (LAUNCH / "pilots" / "simple.launch.py").is_file()


def test_isaac_plan_simple_pilot_turns_ground_truth_on(where, maps_hub, tmp_path):
    base = tmp_path / "aib" / "ego.yaml"
    base.parent.mkdir()
    base.write_text("rig: tesla_model3.rig.yaml\ndomain: 1\nnode_name: isaac_vehicle\n")
    d = make(where, template="isaac")
    text = (d / "testbench.yaml").read_text().replace("kind: autoware", "kind: simple")
    (d / "testbench.yaml").write_text(text)
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


def test_isaac_wheel_query_goes_into_the_ego_config(where, maps_hub, tmp_path):
    """isaac.wheel_query (issue 077): the stack's ego config carries vehicle.wheel_query, over
    the ground truth a simple pilot turns on and the ego config's own vehicle overrides."""
    base = tmp_path / "aib" / "ego.yaml"
    base.parent.mkdir()
    base.write_text("rig: tesla_model3.rig.yaml\ndomain: 1\nvehicle: {mass: 1900.0}\n")
    d = make(where, template="isaac")
    text = (d / "testbench.yaml").read_text().replace("kind: autoware", "kind: simple")
    (d / "testbench.yaml").write_text(text + "isaac: {wheel_query: sweep}\n")
    real = stack.aib_ego_config
    stack.aib_ego_config = lambda: base
    try:
        p = stack.plan(project_arg=str(d))
    finally:
        stack.aib_ego_config = real
    cfg = p.record["isaac_ego"]["config"]
    assert cfg["vehicle"] == {"mass": 1900.0, "wheel_query": "sweep"}
    assert cfg["ground_truth"] == {"topic": "/ground_truth/odom"}
    assert f"ego:={p.stack_dir / stack.ISAAC_EGO}" in p.launch_args["sim_args"]
    # an autoware ego (aib's default config): the stack writes one for the wheel query alone
    (d / "testbench.yaml").write_text(text.replace("kind: simple", "kind: autoware")
                                      + "isaac: {wheel_query: sweep}\n")
    stack.aib_ego_config = lambda: base
    try:
        cfg = stack.plan(project_arg=str(d)).record["isaac_ego"]["config"]
    finally:
        stack.aib_ego_config = real
    assert cfg["vehicle"]["wheel_query"] == "sweep" and "ground_truth" not in cfg


def test_an_own_car_on_isaac_drives_its_rig(where, maps_hub, tmp_path, monkeypatch):
    """Roadmap 091 W5: an own car's ego on Isaac is its rig, made from its spec; the installed
    car's body does not fit it."""
    base = tmp_path / "aib" / "ego.yaml"
    base.parent.mkdir()
    base.write_text("rig: tesla_model3.rig.yaml\nbody: model3/entry.json\ndomain: 1\n")
    d = make(where, template="isaac")
    spec = Path(__file__).resolve().parents[2] / "sim_vehicle_spec" / "vehicles" / "tesla_model3.yaml"
    if not spec.is_file():
        pytest.skip("sim_vehicle_spec not checked out")
    doc = yaml.safe_load(spec.read_text())
    doc["name"], doc["body"]["catalogue_model"] = "my_racer", "vehicle.my.racer"
    for b in (doc.get("bodies") or {}).values():
        b["catalogue_model"] = "vehicle.my.racer"
    (d / "vehicles").mkdir(exist_ok=True)
    (d / "vehicles" / "my_racer.yaml").write_text(yaml.safe_dump(doc, sort_keys=False))
    rig = d / "state" / "vehicles" / "my_racer" / "isaac" / "my_racer.rig.yaml"
    car = {"spec": str(d / "vehicles" / "my_racer.yaml"), "dir": str(rig.parents[1]),
           "catalog": "", "spec_hash_file": "/h.yaml", "spec_hash": "h", "carla_physics": "",
           "carla_sensors": "", "description": "/d.urdf", "pedal_maps": {},
           "isaac_rig": str(rig)}
    monkeypatch.setattr(stack.vehiclesmod, "generate",
                        lambda *a, **k: {"my_racer": dict(car), "__catalog__": None})
    text = (d / "testbench.yaml").read_text().replace("kind: autoware", "kind: simple")
    text = text.replace("vehicle: tesla_model3", "vehicle: my_racer")
    (d / "testbench.yaml").write_text(text + "vehicles: [vehicles/my_racer.yaml]\n")
    monkeypatch.setattr(stack, "aib_ego_config", lambda: base)
    p = stack.plan(project_arg=str(d))
    cfg = p.record["isaac_ego"]["config"]
    assert cfg["rig"] == str(rig) and "body" not in cfg
    assert cfg["ground_truth"] == {"topic": "/ground_truth/odom"}
    assert f"ego:={p.stack_dir / stack.ISAAC_EGO}" in p.launch_args["sim_args"]


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
    for sub in ("scensim_agent", "scenario_agent_relay"):
        _sys.path.insert(0, str(root / sub))
    server = pytest.importorskip("scenario_agent_relay.server")
    P = pytest.importorskip("scensim_agent.protocol")
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
    # an Isaac project's own world server gets the host's ISAAC_ROOT too (own_world: the host's
    # environment still reaches it), but none of the stock server's arguments
    tb = d / "testbench.yaml"
    tb.write_text(tb.read_text() + "world: {server: [my_isaac_world, my_server]}\n")
    p = stack.plan(project_arg=str(d))
    assert p.env["ISAAC_ROOT"] == str(where["tmp"] / "isaac-sim")
    assert p.launch_args["world_server"] == "my_isaac_world my_server"
    assert p.launch_args["sim_args"] == "" and p.record["isaac_ego"] is None


def test_retired_args_refused_by_name(where, maps_hub):
    """roadmap 076 step 10: the old stack's arguments are refused, naming what replaced them."""
    assert stack.retired_args(["side_args", "seat"]) == []
    [line] = stack.retired_args(["ego_launch"])
    assert line.startswith("ego_launch:= is retired") and "pilots:" in line
    assert "scensim.launch.xml" in line
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
    (ws / "install" / "setup.bash").write_text("export SCENSIM_WS_SOURCED=1\n")
    (ws / "scripts" / "autoware_overlay.bash").write_text(
        "scensim_autoware_overlay_source() { export OVERLAY_ARGS=\"$1 $2\"; }\n")
    (ws / "scripts" / "autoware_tensorrt.bash").write_text(
        "scensim_tensorrt_source() { export LD_LIBRARY_PATH=\"$1:$LD_LIBRARY_PATH\"; }\n")
    setup = tmp_path / "aw.bash"
    setup.write_text("export AW_SOURCED=1 CYCLONEDDS_URI=file:///opt/aw.xml\n")
    import subprocess as sp
    out = sp.run(["bash", str(stack.AUTOWARE_ENV), str(ws), str(setup), "", "false", "", "",
                  "--", "bash", "-c", "echo $AW_SOURCED $SCENSIM_WS_SOURCED $OVERLAY_ARGS "
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

MULTI = (project.TEMPLATES / "multi-pilot" / "testbench.yaml").read_text()


def test_world_reaches_the_side_beside_swbs(where, maps_hub):
    """G10: world: goes to the bridge and the world server, merged with scensim's own arguments;
    stack_args merge too, and a name scensim owns is refused."""
    d = make(where)
    (d / "config").mkdir()
    (d / "config" / "bridge.yaml").write_text("{}\n")
    (d / "config" / "carla.yaml").write_text("{}\n")
    (d / "testbench.yaml").write_text(MULTI + "world:\n  weather: {preset: WetNoon, cloudiness: 30}\n"
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
                   "config_file": str((d / "config" / "carla.yaml").resolve()),
                   "vehicle_physics": str(vehiclesmod.installed_physics("tesla_model3"))}
    assert p.record["weather"] == {"parameters": {"cloudiness": 30.0}, "state": "pending"}
    stack.write_plan(p)
    assert (stack_dir / "episodes" / "gap_trace").is_dir()
    # stack_args: merged by name, the developer's value wins; scensim's own names are refused
    p = stack.plan(project_arg=str(d), stack_args={"side_args": "weather:=ClearNoon",
                                                   "sim_args": "vehicle_physics:=/p.yaml"})
    assert stack._named(p.launch_args["side_args"])["weather"] == "ClearNoon"
    assert stack._named(p.launch_args["sim_args"]) == {
        "state_dir": str(d / "state"), "config_file": str((d / "config" / "carla.yaml").resolve()),
        "vehicle_physics": "/p.yaml"}
    assert p.record["pilot_contract"]["carla_physics"] == "/p.yaml"
    # none keeps CARLA's blueprint: nothing for the pilots to read back
    p = stack.plan(project_arg=str(d), stack_args={"sim_args": "vehicle_physics:=none"})
    assert stack._named(p.launch_args["sim_args"])["vehicle_physics"] == "none"
    assert "carla_physics" not in p.record["pilot_contract"]
    for args, name in (({"side_args": "state_dir:=/elsewhere"}, "state_dir"),
                       ({"side_args": "gap_trace_dir:=/t"}, "gap_trace_dir"),
                       ({"sim_args": "port:=1"}, "port")):
        with pytest.raises(stack.StackRefused, match=f"{name}:= is scensim's"):
            stack.plan(project_arg=str(d), stack_args=args)
    # contact_report_dir of the user's: theirs, not collected per run
    (d / "testbench.yaml").write_text(MULTI + "world: {contact_report_dir: off}\n")
    p = stack.plan(project_arg=str(d))
    assert stack._named(p.launch_args["side_args"])["contact_report_dir"] == "off"
    assert p.record["episodes"] == {"gap_trace": None, "contacts": None}


def test_a_judge_that_needs_the_trace_turns_it_on(where, maps_hub):
    d = make(where)
    (d / "testbench.yaml").write_text(MULTI + "judges:\n  - {name: gap, run: [python3, -m, scensim.gap,"
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
    """G6: isaac: reaches the world server beside scensim's world, ego and state_dir; a preloaded
    map is one the stack can switch to."""
    d = make(where, template="isaac")
    (d / "config").mkdir()
    (d / "config" / "ego_lockstep.yaml").write_text("lockstep: true\n")
    (d / "testbench.yaml").write_text(ISAAC)
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
    with pytest.raises(stack.StackRefused, match="world:= is scensim's"):
        stack.plan(project_arg=str(d), stack_args={"sim_args": "world:=/other"})
    # the developers' ego:= still wins over isaac.ego
    p = stack.plan(project_arg=str(d), stack_args={"sim_args": "ego:=none"})
    assert stack._named(p.launch_args["sim_args"])["ego"] == "none"


def test_keeper_sets_the_weather_before_ready(where, maps_hub):
    d = make(where)
    (d / "testbench.yaml").write_text(MULTI + "world: {weather: {cloudiness: 30}}\n")
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
    (d / "testbench.yaml").write_text(MULTI + "world: {weather: {cloudy: 30}}\n")
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
    (d / "testbench.yaml").write_text(MULTI + "world: {weather: {cloudy: 30}}\n")
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
    scenarios (SCENSIM_VEHICLES), the car's physics to CARLA's world server, its files to the
    pilot, its spec_hash recorded."""
    from scensim import vehicles
    from test_vehicles import INSTALLED, fake_gen, own_spec
    monkeypatch.setattr(vehicles, "run_gen", fake_gen)
    monkeypatch.setattr(vehicles, "installed_catalog", lambda body=None: INSTALLED)
    d = make(where)
    own_spec(d)
    s = yaml.safe_load((d / "testbench.yaml").read_text())
    s["vehicles"] = ["vehicles/my_lincoln.yaml"]
    s["pilots"] = {"ego": {"kind": "simple", "vehicle": "my_lincoln"},
                   "*": {"kind": "none"}}
    (d / "testbench.yaml").write_text(yaml.safe_dump(s))
    p = stack.plan(project_arg=str(d), check=None)
    own = d / "state" / "vehicles"
    assert p.launch_args["vehicle_catalogs"] == str(own / "catalogs" / "vehicles.xosc")
    assert p.env["SCENSIM_VEHICLES"] == str(own / "catalogs")
    assert f"vehicle_physics:={own / 'my_lincoln' / 'carla' / 'carla_physics.yaml'}" in \
        p.launch_args["sim_args"]
    [ego] = [e for e in p.record["pilot_entries"] if e["entity"] == "ego"]
    assert ego["args"]["spec"] == str(d / "vehicles" / "my_lincoln.yaml")
    assert ego["args"]["vehicle_config"].endswith("my_lincoln/carla/carla_sensors.yaml")
    assert ego["args"]["description"].endswith("my_lincoln/description.urdf")
    assert p.record["spec_hash"]["ego"] == "hash-of-my_lincoln"
    assert p.record["vehicles"]["my_lincoln"]["spec_hash"] == "hash-of-my_lincoln"


def test_what_carla_does_not_apply_is_named_and_recorded(where, maps_hub, monkeypatch, capsys):
    """roadmap 091 W24: an own car whose CARLA file names what CARLA does not apply (its
    body.contact): the stack's start says so, and stack.json keeps it for the runs."""
    from scensim import vehicles
    from test_vehicles import INSTALLED, fake_gen, own_spec

    def gen(spec, d):
        fake_gen(spec, d)
        (d / "carla" / "carla_physics.yaml").write_text(
            "not_applied: ['body.contact: CARLA has no per-actor physics material']\n")
    monkeypatch.setattr(vehicles, "run_gen", gen)
    monkeypatch.setattr(vehicles, "installed_catalog", lambda body=None: INSTALLED)
    d = make(where)
    own_spec(d)
    s = yaml.safe_load((d / "testbench.yaml").read_text())
    s["vehicles"] = ["vehicles/my_lincoln.yaml"]
    s["pilots"] = {"ego": {"kind": "simple", "vehicle": "my_lincoln"}, "*": {"kind": "none"}}
    (d / "testbench.yaml").write_text(yaml.safe_dump(s))
    p = stack.plan(project_arg=str(d), check=None)
    assert p.record["not_applied"] == {
        "my_lincoln": ["body.contact: CARLA has no per-actor physics material"]}
    assert "does not apply these of my_lincoln's spec: body.contact" in capsys.readouterr().err
    # a car that names nothing: nothing said, nothing recorded
    monkeypatch.setattr(vehicles, "run_gen", fake_gen)
    assert stack.plan(project_arg=str(d), check=None).record["not_applied"] == {}


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
    text = (d / "testbench.yaml").read_text().replace(
        "kind: autoware", 'kind: launch, launch: "scensim_launch simple.launch.py", agent: own, '
        "inputs: [map, ground_truth]")
    (d / "testbench.yaml").write_text(text)
    p = stack.plan(project_arg=str(d), check=None)
    assert p.record["isaac_ego"]["config"]["ground_truth"] == {"topic": "/ground_truth/odom"}



def test_the_isaac_world_server_finds_the_cars_in_the_projects_usd_set(where, maps_hub):
    """roadmap 077, G36: SCENSIM_VEHICLE_USD defaulted to a host path; on a host without it
    every car was a box. The project's fetched set has them."""
    d = make(where, template="isaac")   # the fake set has vehicles/vehicles.json
    cat = d / "assets" / "carla-usd" / "vehicles"
    assert stack.plan(project_arg=str(d), check=None).env["SCENSIM_VEHICLE_USD"] == str(cat)


def test_the_keeper_tees_its_lines_to_its_launchers_stdout(tmp_path, monkeypatch):
    """G41: play_launch keeps a node's output in its log; the keeper's progress goes to the
    launcher's stdout too, unless that is the keeper's own."""
    import io
    import sys as _sys
    fake = tmp_path / "proc" / "42" / "fd"
    fake.mkdir(parents=True)
    terminal = tmp_path / "terminal"
    terminal.write_text("")
    (fake / "1").symlink_to(terminal)
    own = tmp_path / "own"

    with open(own, "w") as f:
        monkeypatch.setattr(_sys, "stdout", f)
        monkeypatch.setattr(_sys, "stderr", io.StringIO())
        other = stack.tee_to_launcher(ppid=42, proc=tmp_path / "proc")
        assert other is not None
        print("scensim stack s1: ready", flush=True)
        _sys.stdout.flush()
        other.close()
    assert terminal.read_text() == "scensim stack s1: ready\n"
    assert own.read_text() == "scensim stack s1: ready\n"
    # the launcher's stdout is the keeper's own: nothing doubled
    (fake / "1").unlink()
    (fake / "1").symlink_to(own)
    with open(own, "a") as f:
        monkeypatch.setattr(_sys, "stdout", f)
        assert stack.tee_to_launcher(ppid=42, proc=tmp_path / "proc") is None
    assert stack.tee_to_launcher(ppid=43, proc=tmp_path / "proc") is None


def test_the_web_ui_line_names_the_address_the_launcher_was_given(tmp_path):
    """G45: no 'pass --web-addr' hint when it was passed."""
    d = tmp_path / "proc" / "42"
    d.mkdir(parents=True)
    (d / "cmdline").write_bytes(b"play_launch\x00launch\x00--web-addr\x00127.0.0.1:8080\x00scensim_launch\x00")
    assert stack.launcher_web_addr(42, tmp_path / "proc") == "127.0.0.1:8080"
    (d / "cmdline").write_bytes(b"play_launch\x00launch\x00--web-addr=0.0.0.0:8090\x00")
    assert stack.launcher_web_addr(42, tmp_path / "proc") == "0.0.0.0:8090"
    (d / "cmdline").write_bytes(b"play_launch\x00launch\x00scensim_launch\x00")
    assert stack.launcher_web_addr(42, tmp_path / "proc") is None


def test_isaac_renders_on_the_cuda_card():
    """swb 241: CUDA on GPU 1 and the renderer on GPU 0 left the sensors silent."""
    assert stack.isaac_kit_args("", 1) == \
        "--/renderer/multiGpu/enabled=false --/renderer/activeGpu=1"
    # the user's own come first; the stack's activeGpu, the last one, wins
    assert stack.isaac_kit_args("--/x=1 --/renderer/activeGpu=0", 1).endswith(
        "--/renderer/activeGpu=0 --/renderer/multiGpu/enabled=false --/renderer/activeGpu=1")


OWN_SERVER = """simulator: carla
assets:
  - dataset: carla-town-autoware-maps
    take: [Town01]
pilots:
  ego: {kind: simple, vehicle: tesla_model3}
world:
  server: [my_world, my_world_server, --level, "two words"]
"""


def test_a_world_server_of_the_projects_replaces_the_stock_one(where, maps_hub):
    """081 step 5: world.server goes to stack.launch.py as world_server (package, executable)
    and world_server_args (its own arguments); the stock server's arguments are not its."""
    d = make(where, template="carla-first-drive", testbench_yaml=OWN_SERVER)
    p = stack.plan(project_arg=str(d))
    a = p.launch_args
    assert a["sim"] == "carla" and a["world_port"] == "5791"
    assert a["world_server"] == "my_world my_world_server"
    assert a["world_server_args"] == "--level 'two words'"
    assert a["sim_args"] == ""   # no state_dir, no spec physics: those are the stock server's
    assert "carla_physics" not in p.record["pilot_contract"]
    assert p.record["world_server"] == ["my_world", "my_world_server", "--level", "two words"]
    assert p.record["carla"] is not None   # CARLA is still the keeper's to start
    with pytest.raises(stack.StackRefused, match="world.server itself"):
        stack.plan(project_arg=str(d), stack_args={"sim_args": "vehicle_physics:=none"})


NONE_SERVER = """simulator: none
assets:
  - dataset: carla-town-autoware-maps
    take: [Town01]
pilots:
  ego: {kind: command, run: [python3, -c, "import time; time.sleep(60)"]}
world:
  server: [my_world, my_world_server]
"""


def test_simulator_none_starts_no_simulator(where, maps_hub):
    """simulator: none with world.server: the user's server owns the simulator, so the stack
    decides no CARLA and gives no Isaac world or ego; without world.server it is refused."""
    d = make(where, template="carla-first-drive", testbench_yaml=NONE_SERVER)
    p = stack.plan(project_arg=str(d))
    a = p.launch_args
    assert p.simulator == "none" and a["sim"] == "none" and a["ego_launch"] == "none"
    assert a["world_server"] == "my_world my_world_server" and a["world_server_args"] == ""
    assert a["sim_args"] == ""
    assert p.record["carla"] is None and p.record["isaac_ego"] is None
    assert "ISAAC_ROOT" not in p.env
    (d / "testbench.yaml").write_text(NONE_SERVER.replace(
        "world:\n  server: [my_world, my_world_server]\n", ""))
    with pytest.raises(stack.StackRefused, match="simulator: none needs world.server"):
        stack.plan(project_arg=str(d))
    with pytest.raises(stack.StackRefused, match="or none with world.server"):
        stack.plan(project_arg=str(d), check=None)


def test_simulator_none_with_its_own_server_passes_check_and_plan(where, maps_hub):
    """docs/how-to/your-world-server.md's simulator: none: `testbench check` finds nothing, the
    plan starts no simulator, and the CARLA and Isaac blocks are refused beside it."""
    d = make(where, template="carla-first-drive", testbench_yaml=NONE_SERVER)
    assert [str(f) for f in project.check_project(d)] == []
    assert [str(f) for f in project.check_stack(d)] == []
    p = stack.plan(project_arg=str(d))
    assert p.record["simulator"] == "none" and p.record["simulator_start"] == simmod.WORLD_SERVER
    assert p.record["carla"] is None and p.launch_args["world_server"] == "my_world my_world_server"
    for block, match in (({"carla": {"quality": "low"}}, "carla: is for simulator carla"),
                         ({"isaac": {"render": "headless"}}, "isaac")):
        tb = {**yaml.safe_load(NONE_SERVER), **block}
        (d / "testbench.yaml").write_text(yaml.safe_dump(tb))
        found = [str(f) for f in project.check_project(d)]
        assert any(match in f for f in found), found


def _autoware_setup(where, setup):
    sysf = where["tmp"] / "system" / "host.yaml"
    h = yaml.safe_load(sysf.read_text())
    h["autoware"] = {"setup": setup}
    sysf.write_text(yaml.safe_dump(h))


def _ego_args(p):
    ego = next(x for x in p.record["pilots"] if x["kind"] == "autoware")
    return ego["spec"].get("args") or {}


@pytest.fixture
def any_install(monkeypatch):
    """The workspaces' own build (their chained Autoware) left out: these tests name
    releases the checkout running them was not built against."""
    monkeypatch.setattr(stack.autowaremod, "install_problems", lambda host, ws: [])


def _profiled(d):
    """The testbench's autoware pilots on autoware_sim_profile (autoware_profile: scensim)."""
    f = d / "testbench.yaml"
    tb = yaml.safe_load(f.read_text())
    for p in tb["pilots"].values():
        if p.get("kind") == "autoware":
            p["autoware_profile"] = "scensim"
    f.write_text(yaml.safe_dump(tb))
    return d


def test_a_stock_autoware_pilot_gets_no_release_switch(where, maps_hub, any_install):
    """The default, stock: the user's autoware_launch as it is; only the profile has a
    release switch, and a release it lacks does not stop a stock pilot."""
    d = make(where, maps_hub=maps_hub)
    _autoware_setup(where, "/opt/autoware/1.9.0/setup.bash")
    assert "autoware_release" not in _ego_args(stack.plan(project_arg=str(d), map_name="Town01"))


def test_the_autoware_pilot_runs_the_release_of_the_hosts_autoware(where, maps_hub,
                                                                   any_install):
    d = _profiled(make(where, maps_hub=maps_hub))
    p = stack.plan(project_arg=str(d), map_name="Town01")
    assert _ego_args(p)["autoware_release"] == "1.5.0"      # the default setup's
    _autoware_setup(where, "/opt/autoware/2.1.0/setup.bash")
    p = stack.plan(project_arg=str(d), map_name="Town01")
    assert _ego_args(p)["autoware_release"] == "2.1.0"
    # the carla_basic pilot gets nothing of Autoware's
    lead = next(x for x in p.record["pilots"] if x["kind"] == "carla_basic")
    assert "autoware_release" not in (lead["spec"].get("args") or {})


def test_a_pilot_naming_another_release_than_the_hosts_is_refused(where, maps_hub,
                                                                  any_install):
    tb = ("simulator: carla\nassets:\n  - dataset: carla-town-autoware-maps\n"
          "    take: [Town01]\npilots:\n  ego: {kind: autoware, autoware_profile: scensim, "
          "vehicle: tesla_model3, args: {autoware_release: 2.0.0}}\n")
    d = make(where, maps_hub=maps_hub)
    (d / "testbench.yaml").write_text(tb)
    _autoware_setup(where, "/opt/autoware/2.1.0/setup.bash")
    with pytest.raises(stack.StackRefused, match=r"pilots.ego \(ego\): autoware_release:=2.0.0, "
                       r"but the host's Autoware is 2.1.0 \(/opt/autoware/2.1.0/setup.bash\)"):
        stack.plan(project_arg=str(d), map_name="Town01")
    _autoware_setup(where, "/opt/autoware/2.0.0/setup.bash")
    assert _ego_args(stack.plan(project_arg=str(d), map_name="Town01"))["autoware_release"] \
        == "2.0.0"


def test_a_release_the_profile_does_not_have_is_refused(where, maps_hub, any_install):
    d = _profiled(make(where, maps_hub=maps_hub))
    _autoware_setup(where, "/opt/autoware/3.0.0/setup.bash")
    with pytest.raises(stack.StackRefused, match="the host's Autoware is 3.0.0"):
        stack.plan(project_arg=str(d), map_name="Town01")


def test_the_1_5_0_overlay_is_never_sourced_over_another_release(tmp_path):
    """scripts/autoware_overlay.bash: the overlay is built for 1.5.0; with another Autoware
    sourced it says so and sources nothing (a built overlay would shadow that release)."""
    import subprocess as sp
    ws = stack.workspace_root()
    if ws is None:
        pytest.skip("the scensim checkout is not known")
    built = tmp_path / "overlay" / "install"
    built.mkdir(parents=True)
    (built / "local_setup.bash").write_text("export OVERLAY_SOURCED=1\n")
    script = (f'source "{ws}/scripts/autoware_overlay.bash"; '
              f'scensim_autoware_overlay_source "{tmp_path / "overlay"}" true; '
              'echo "sourced=${OVERLAY_SOURCED:-0}"')
    for prefix, sourced in (("/opt/autoware/2.1.0:/opt/ros/humble", "0"),
                            ("/opt/autoware/1.5.0:/opt/ros/humble", "1")):
        r = sp.run(["bash", "-c", script], capture_output=True, text=True,
                   env=dict(os.environ, AMENT_PREFIX_PATH=prefix))
        assert r.stdout.split()[-1] == f"sourced={sourced}", r.stderr
    assert "Autoware 2.1.0: the overlay carries fixes for Autoware 1.5.0 and is not sourced" \
        in sp.run(["bash", "-c", script], capture_output=True, text=True,
                  env=dict(os.environ, AMENT_PREFIX_PATH="/opt/autoware/2.1.0")).stderr


def test_a_workspace_built_against_another_autoware_is_refused(where, maps_hub, monkeypatch):
    d = make(where, maps_hub=maps_hub)
    seen = []
    monkeypatch.setattr(stack.autowaremod, "install_problems",
                        lambda host, ws: seen.append(host["autoware"]["setup"]) or ["mixed"])
    _autoware_setup(where, "/opt/autoware/2.1.0/setup.bash")
    with pytest.raises(stack.StackRefused, match="^mixed$"):
        stack.plan(project_arg=str(d), map_name="Town01")
    assert seen == ["/opt/autoware/2.1.0/setup.bash"]


def test_ros_env_puts_the_servers_carla_client_first(monkeypatch):
    monkeypatch.setenv("PYTHONPATH", "/opt/ros/lib:/client")
    env = stack.ros_env({"env": {simmod.CLIENT_ENV: "/client"}}, 3)
    assert env["PYTHONPATH"] == "/client:/opt/ros/lib" and env["ROS_DOMAIN_ID"] == "3"
    monkeypatch.delenv("PYTHONPATH")
    assert stack.ros_env({"env": {simmod.CLIENT_ENV: "/client"}}, 3)["PYTHONPATH"] == "/client"
    assert "PYTHONPATH" not in stack.ros_env({"env": {}}, 3)


def test_plan_refuses_carla_basic_on_another_carla_client(where, maps_hub, monkeypatch):
    """The 1.0 server, 0.9.16's client, no carla.root to take 1.0's from (T3 on 1.0)."""
    d = make(where, maps_hub=maps_hub)   # multi-pilot: lead is carla_basic
    monkeypatch.setattr(stack, "CARLA_DECIDE", lambda c, m, p, launch=None: simmod.decide(
        {**c, "start": "external"}, "external", lambda h, port: (simmod.CARLA, "1.0")))
    monkeypatch.setattr(simmod, "installed_client", lambda: "0.9.16")
    with pytest.raises(stack.StackRefused, match=r"pilots lead \(carla_basic\).*is 1\.0, "
                       r"and this Python's carla client is 0\.9\.16"):
        stack.plan(project_arg=str(d), map_name="Town01")


def test_a_pilot_stuck_on_a_lost_load_reply_is_restarted(where, maps_hub, monkeypatch, tmp_path):
    """Issue 088: a container's reply to the launch's load_node is lost and launch_ros waits for it
    for ever; nothing failed, so the start sat in the ready timeout. The keeper now restarts that
    pilot, naming the container, and the next start becomes ready."""
    import sys
    from scensim import timeline as timelinemod
    monkeypatch.setattr(timelinemod, "LOST_GRACE", 0.0)
    d = make(where)
    p = stack.plan(project_arg=str(d), map_name="Town01")
    marker = tmp_path / "second-start"
    for x in p.record["pilots"]:
        x["ready"] = ["test", "-e", str(marker)]
    stack.write_plan(p)
    starts = []
    bp = "planning.behavior_planning_container"
    stalled = ("import time; t = time.time(); print('[INFO] [launch]: All log files can be found "
               "below /x'); print('[c-1] [INFO] [%.3f] [" + bp + "]: Instantiate class: Glog' % t); "
               "print('[c-1] [WARN] [%.3f] [" + bp + ".rclcpp]: failed to send response to "
               "/planning/behavior_planning_container/_container/load_node (timeout)' % t, "
               "flush=True); time.sleep(60)")

    def cmd(rec, pilot, entry, log_dir):
        starts.append(pilot["entity"])
        if starts.count(pilot["entity"]) == 1:
            return [sys.executable, "-c", stalled]
        return [sys.executable, "-c", "import pathlib, time; print('[INFO] [launch]: All log files "
                f"can be found below /y', flush=True); pathlib.Path({str(marker)!r}).touch(); "
                "time.sleep(60)"]

    monkeypatch.setattr(stack, "STOP_GRACE", 1.0)
    assert stack.keep(p.stack_dir, poll=0.05, once=True, pilot_command=cmd, respawn_delay=0.1,
                      once_timeout=30) == 0
    rec = json.loads(p.stack_json.read_text())
    assert rec["state"] == "ready"
    events = [json.loads(x) for x in (p.stack_dir / "events.jsonl").read_text().splitlines()]
    stalls = [e for e in events if e["event"] == "pilot.stalled"]
    assert stalls and all("/planning/behavior_planning_container (its reply to the launch's "
                          "load_node was lost" in e["containers"] for e in stalls)
    assert {e["entity"] for e in stalls} <= set(starts)
    assert all(x.get("restarts", 0) >= 1 for x in rec["pilots"] if x["entity"] in
               {e["entity"] for e in stalls})


def _composables_pilot(marker, first, then=None):
    """A pilot command whose first start writes ``first`` (console lines) and sleeps; later ones
    write ``then`` and touch ``marker`` (its ready check), or repeat ``first`` when ``then`` is None."""
    import sys
    starts = []

    def cmd(rec, pilot, entry, log_dir):
        if pilot["entity"] != "ego":
            return fake_pilot(rec, pilot, entry, log_dir)
        starts.append(1)
        lines = first if len(starts) == 1 or then is None else then
        touch = "" if len(starts) == 1 or then is None else f"pathlib.Path({str(marker)!r}).touch(); "
        # {T}: this start's wall time, as the container would stamp its lines
        return [sys.executable, "-c", "import pathlib, time; "
                f"print({chr(10).join(lines)!r}.replace('{{T}}', '%.3f' % time.time()), flush=True); "
                f"{touch}time.sleep(60)"]
    return cmd, starts


def _ego_ready_on(p, marker):
    for x in p.record["pilots"]:
        x["ready"] = ["test", "-e", str(marker)] if x["entity"] == "ego" else ["true"]


def test_a_play_launch_start_summarised_with_composables_missing_is_restarted(where, maps_hub, tmp_path):
    """088 under play_launch: with its container frozen early, play_launch wrote a STARTUP_SUMMARY
    naming every composable missing (064's check); the keeper restarts that start too."""
    d = make(where)
    p = stack.plan(project_arg=str(d), map_name="Town01")
    marker = tmp_path / "second-start"
    _ego_ready_on(p, marker)
    stack.write_plan(p)
    summary = ('2026-11-20T20:01:36.749409Z  INFO STARTUP_SUMMARY {"composable_loaded":0,'
               '"composable_shortfall":[{"container":"/planning/stall_container","declared":3,'
               '"loaded":0,"missing":["/listener","/server","/talker"]}],"composable_total":3}')
    cmd, starts = _composables_pilot(marker, ["INFO Step 1/3: Parsing launch file...", summary],
                                     ["INFO Step 1/3: Parsing launch file..."])
    assert stack.keep(p.stack_dir, poll=0.05, once=True, pilot_command=cmd, respawn_delay=0.1,
                      once_timeout=30) == 0
    events = [json.loads(x) for x in (p.stack_dir / "events.jsonl").read_text().splitlines()]
    stalls = [e for e in events if e["event"] == "pilot.stalled"]
    assert len(stalls) == 1 and "/listener (not loaded in /planning/stall_container" in stalls[0]["containers"]
    assert len(starts) == 2   # the old start's summary stays in the console: not read again


def test_a_start_that_never_loads_is_restarted_twice_then_left_not_ready(where, maps_hub, tmp_path,
                                                                          monkeypatch):
    from scensim import timeline as timelinemod
    monkeypatch.setattr(timelinemod, "LOST_GRACE", 0.0)
    d = make(where)
    p = stack.plan(project_arg=str(d), map_name="Town01")
    marker = tmp_path / "never"
    _ego_ready_on(p, marker)
    stack.write_plan(p)
    bp = "planning.behavior_planning_container"
    lost = ["[INFO] [launch]: All log files can be found below /x",
            f"[c-1] [INFO] [{{T}}] [{bp}]: Instantiate class: Glog",
            f"[c-1] [WARN] [{{T}}] [{bp}.rclcpp]: failed to send response to "
            "/planning/behavior_planning_container/_container/load_node (timeout)"]
    cmd, starts = _composables_pilot(marker, lost)
    assert stack.keep(p.stack_dir, poll=0.05, once=True, pilot_command=cmd, respawn_delay=0.1,
                      once_timeout=8) == 3
    events = [json.loads(x) for x in (p.stack_dir / "events.jsonl").read_text().splitlines()]
    assert [e["restart"] for e in events if e["event"] == "pilot.stalled"] == [1, 2]
    assert len(starts) == 3
    missing = [e for e in events if e["event"] == "pilot.missing"]
    assert len(missing) == 1 and missing[0]["composables"].startswith(
        "(stalled) /planning/behavior_planning_container (its reply")
