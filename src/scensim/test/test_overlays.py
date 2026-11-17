"""A pilot's own workspaces, ``overlays:`` (roadmap 077, G60)."""
import os
import subprocess as sp

import yaml

from scensim import assets, overlays, project, stack
from test_project import BASE, make, scenario_checks
from test_stack import make as make_stack, where  # noqa: F401 (a fixture)


def host_file(host, **settings):
    """Add ``settings`` to the test's user host.yaml."""
    f = host["tmp"] / "user" / "host.yaml"
    data = yaml.safe_load(f.read_text()) or {}
    data.update(settings)
    f.write_text(yaml.safe_dump(data))


def workspace(tmp, name="ws", body="export WS_SOURCED=1\n", git=False):
    ws = tmp / name
    (ws / "install").mkdir(parents=True)
    (ws / "install" / "setup.bash").write_text(body)
    if git:
        run = lambda *a: sp.run(["git", "-C", str(ws), *a], check=True, capture_output=True)  # noqa: E731
        run("init", "-q")
        run("-c", "user.email=t@t", "-c", "user.name=t", "add", ".")
        run("-c", "user.email=t@t", "-c", "user.name=t", "commit", "-q", "-m", "x")
    return ws


def pilot_yaml(entries, kind="autoware"):
    extra = ", launch: pilots/own.launch.py" if kind == "launch" else ""
    return BASE.replace("{kind: autoware, vehicle: tesla_model3}",
                        f"{{kind: {kind}, vehicle: tesla_model3{extra}, overlays: {entries}}}")


def test_overlays_is_a_key_of_autoware_and_launch_pilots_only(host):
    d = make(host["tmp"], BASE.replace("ego: {kind: autoware, vehicle: tesla_model3}",
                                       "ego: {kind: simple, vehicle: tesla_model3, overlays: [a]}"))
    assert any("unknown key overlays for kind simple" in w for w in
               (f.what for f in scenario_checks(d)))
    d = make(host["tmp"], pilot_yaml("x.bash"))
    assert "pilots.ego.overlays: is not a list of setup files" in [
        f.what for f in scenario_checks(d)]


def test_a_workspace_entry_resolves_through_the_host_settings(host):
    ws = workspace(host["tmp"])
    host_file(host, workspaces={"drift": str(ws)})
    d = make(host["tmp"], pilot_yaml("['{workspace:drift}/install/setup.bash']"))
    assert scenario_checks(d) == []
    r = overlays.resolve(d, "{workspace:drift}/install/setup.bash",
                         {"workspaces": {"drift": str(ws)}})
    assert r["path"] == str(ws / "install" / "setup.bash") and r["problem"] is None
    # a project-relative entry
    (d / "env").mkdir()
    (d / "env" / "setup.bash").write_text("")
    assert overlays.resolve(d, "env/setup.bash", {})["path"] == str(d / "env" / "setup.bash")


def test_an_unknown_workspace_and_a_missing_file_are_refused(host):
    d = make(host["tmp"], pilot_yaml("['{workspace:drift}/install/setup.bash', nope.bash]"))
    found = {f.what: f for f in scenario_checks(d)}
    unknown = found["pilots.ego.overlays: {workspace:drift}/install/setup.bash: this host names"
                    " no workspace drift"]
    assert unknown.refused and "workspaces: {drift: <dir>}" in unknown.fix
    missing = next(f for w, f in found.items() if w.startswith("pilots.ego.overlays: nope.bash:"
                                                              " no such file"))
    assert missing.refused
    # a named workspace whose setup file is not built
    ws = host["tmp"] / "empty_ws"
    ws.mkdir()
    host_file(host, workspaces={"drift": str(ws)})
    assert any("no such file" in w and "{workspace:drift}" in w for w in
               (f.what for f in scenario_checks(d)))
    assert project.main(["check", str(d)]) == 3


def test_a_host_path_in_the_project_file_warns(host, capsys):
    ws = workspace(host["tmp"])
    entry = str(ws / "install" / "setup.bash")
    d = make(host["tmp"], pilot_yaml(f"['{entry}']"))
    assert scenario_checks(d) == []   # it works: a warning, not a problem
    notes = overlays.notes(yaml.safe_load((d / "swb.yaml").read_text()))
    assert len(notes) == 1 and "a host path in a project file; name it as a host workspace" in notes[0]
    project.main(["check", str(d)])
    assert "warning: pilots.ego.overlays: " + entry in capsys.readouterr().out
    assert overlays.notes({"pilots": {"ego": {"overlays": ["~/ws/setup.bash"]}}})
    assert not overlays.notes({"pilots": {"ego": {"overlays": ["{workspace:w}/setup.bash"]}}})


def test_the_overlays_are_sourced_in_order_after_the_installs_before_tensorrt(tmp_path):
    ws = tmp_path / "ws"
    (ws / "scripts").mkdir(parents=True)
    (ws / "install").mkdir()
    (ws / "install" / "setup.bash").write_text("export ORDER=ws LD_LIBRARY_PATH=/ws\n")
    (ws / "scripts" / "autoware_overlay.bash").write_text("scensim_autoware_overlay_source() { :; }\n")
    (ws / "scripts" / "autoware_tensorrt.bash").write_text(
        "scensim_tensorrt_source() { export LD_LIBRARY_PATH=\"$1:$LD_LIBRARY_PATH\"; }\n")
    setup = tmp_path / "aw.bash"
    setup.write_text("export ORDER=aw\n")
    a, b = tmp_path / "a.bash", tmp_path / "b.bash"
    a.write_text('export ORDER="$ORDER a" LD_LIBRARY_PATH=/a:$LD_LIBRARY_PATH\n')
    b.write_text('export ORDER="$ORDER b"\n')
    env = {"recorded": [{"path": str(a)}, {"path": str(b)}]}
    extra = overlays.flags(env["recorded"])
    assert extra == ["--overlay", str(a), "--overlay", str(b)]
    out = sp.run(["bash", str(stack.AUTOWARE_ENV), str(ws), str(setup), "", "false", "", "/trt",
                  *extra, "--", "bash", "-c", "echo $ORDER; echo ${LD_LIBRARY_PATH%%:*}"],
                 capture_output=True, text=True).stdout.splitlines()
    assert out == ["ws a b", "/trt"]
    # a launch pilot: overlay_env.bash, the stack's DDS settings kept
    c = tmp_path / "c.bash"
    c.write_text("export CYCLONEDDS_URI=file:///ws.xml ORDER=c\n")
    out = sp.run(["bash", str(stack.OVERLAY_ENV), "--overlay", str(c), "--overlay", str(b), "--",
                  "bash", "-c", "echo $ORDER $CYCLONEDDS_URI"], capture_output=True, text=True,
                 env=dict(os.environ, CYCLONEDDS_URI="file:///stack.xml")).stdout.split()
    assert out == ["c", "b", "file:///stack.xml"]


def test_the_stack_records_each_overlay_and_wraps_the_pilot(where, maps_hub, tmp_path):
    ws = workspace(where["tmp"], git=True)
    host_file(where, workspaces={"drift": str(ws)})
    d = make_stack(where, maps_hub=maps_hub)
    text = (d / "swb.yaml").read_text().replace(
        "ego: {kind: autoware, vehicle: tesla_model3}",
        "ego: {kind: autoware, vehicle: tesla_model3,"
        " overlays: ['{workspace:drift}/install/setup.bash']}")
    (d / "swb.yaml").write_text(text)
    assets.fetch(d, out=lambda s: None)
    p = stack.plan(project_arg=str(d), map_name="Town01")
    ego = next(x for x in p.record["pilots"] if x["entity"] == "ego")
    commit = sp.run(["git", "-C", str(ws), "rev-parse", "HEAD"], capture_output=True,
                    text=True).stdout.strip()
    assert ego["overlays"] == [{"entry": "{workspace:drift}/install/setup.bash",
                                "path": str(ws / "install" / "setup.bash"), "commit": commit}]
    lead = next(x for x in p.record["pilots"] if x["entity"] == "lead")
    assert lead["overlays"] == []
    rec = dict(p.record, pilots_dir="/x")
    argv = stack.pilot_argv(rec, ego, {"entity": "ego", "kind": "autoware",
                                       "domain": ego["domain"]}, tmp_path)
    cut = argv.index("--")
    assert argv[0:2] == ["bash", str(stack.AUTOWARE_ENV)]
    assert argv[cut - 2:cut] == ["--overlay", str(ws / "install" / "setup.bash")]
    # a launch pilot with overlays runs under overlay_env.bash; without, as before
    rec["autoware_env"] = None
    launch = dict(ego, kind="launch")
    argv = stack.pilot_argv(rec, launch, {"entity": "ego", "kind": "launch",
                                          "domain": ego["domain"]}, tmp_path)
    assert argv[:4] == ["bash", str(stack.OVERLAY_ENV), "--overlay",
                        str(ws / "install" / "setup.bash")]
    assert stack.pilot_argv(rec, dict(launch, overlays=[]), {"entity": "ego", "kind": "launch",
                            "domain": ego["domain"]}, tmp_path)[0] == "play_launch"
    # no git: no commit
    plain = workspace(where["tmp"], name="plain")
    assert overlays.resolved(d, {"overlays": ["{workspace:p}/install/setup.bash"]},
                             {"workspaces": {"p": str(plain)}})[0]["commit"] is None


def test_a_workspace_substitutes_in_args_and_an_unknown_one_is_refused(host):
    ctx = {"workspaces": {"drift": "/w/drift"}}
    assert stack.substitute({"extra_launch": "{workspace:drift}/install/x.launch.xml"}, ctx,
                            "pilots.ego") == {"extra_launch": "/w/drift/install/x.launch.xml"}
    import pytest
    with pytest.raises(stack.StackRefused, match="this host names no workspace nope"):
        stack.substitute("{workspace:nope}/x", ctx, "pilots.ego")
    d = make(host["tmp"], BASE.replace(
        "{kind: autoware, vehicle: tesla_model3}",
        "{kind: autoware, vehicle: tesla_model3, args: {extra_launch: '{workspace:drift}/x.xml'}}"))
    f = next(f for f in scenario_checks(d) if f.what.startswith("pilots.ego.args:"))
    assert f.refused and "this host names no workspace drift" in f.what
    host_file(host, workspaces={"drift": str(host["tmp"])})
    assert not [f for f in scenario_checks(d) if f.what.startswith("pilots.ego.args:")]
