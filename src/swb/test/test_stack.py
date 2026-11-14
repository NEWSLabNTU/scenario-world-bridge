"""swb.stack: the stack's plan (seat -> arguments, pilots bound, refusals) and its keeper."""
import importlib.util
import json
import os
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


def test_plan_multi_pilot(where, maps_hub):
    d = make(where)
    p = stack.plan(project_arg=str(d))
    assert p.seat.index == 2 and p.simulator == "carla" and p.map == "Town01"
    a = p.launch_args
    assert (a["scenario_domain"], a["ego_domain"]) == ("17", "18")
    assert (a["ssv2_port"], a["agent_port"], a["world_port"]) == ("5755", "5760", "5791")
    assert a["ego_launch"] == "none" and a["nest_side"] == "true"
    assert a["map_path"] == str(d / "assets" / "maps" / "Town01")
    assert a["log_dir"] == str(where["tmp"] / "logs" / "proj" / "stacks" / p.stack_id / "proc")
    pilots = json.loads(a["pilots"])
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
    assert r["pilots"][0]["ready"] == stack.EGO_READY and r["pilots"][1]["ready"] is None
    assert r["fingerprint"]["swb_yaml"].startswith("sha256:")
    assert r["pilot_contract"]["release_notify_endpoint"] == "tcp://127.0.0.1:5756"


def test_plan_args_pass_stack_launch(where, maps_hub):
    pytest.importorskip("launch")
    spec = importlib.util.spec_from_file_location("stack_launch", LAUNCH / "stack.launch.py")
    sl = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sl)
    a = stack.plan(project_arg=str(make(where))).launch_args
    parsed = sl.parse_pilots(a["pilots"], a["sim"], a["ego_domain"], a["scenario_domain"])
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
    assert [x["entity"] for x in json.loads(p.launch_args["pilots"])] == ["ego", "lead"]
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
    p = stack.plan(project_arg=str(d))
    for x in p.record["pilots"]:
        x["ready"] = ["true"]
    stack.write_plan(p)
    assert stack.keep(p.stack_dir, poll=0.01, once=True) == 0
    rec = json.loads(p.stack_json.read_text())
    assert rec["state"] == "ready" and rec["pid"] == os.getpid()
    assert all(x["ready_at"] for x in rec["pilots"])
    link = where["tmp"] / "logs" / "proj" / "seat-2-current"
    assert link.resolve() == p.stack_dir.resolve()
    ev = [json.loads(l)["event"] for l in (p.stack_dir / "events.jsonl").read_text().splitlines()]
    assert ev[0] == "stack.start" and ev[-1] == "stack.ready"
    assert seat.holder(2, str(where["tmp"] / "locks")) is None  # once: released


def test_keeper_refuses_a_held_seat(where, maps_hub):
    d = make(where)
    p = stack.plan(project_arg=str(d))
    stack.write_plan(p)
    with seat.acquire(p.seat, "other"):
        assert stack.keep(p.stack_dir, once=True) == 3
    assert json.loads(p.stack_json.read_text())["state"] == "stopped"
