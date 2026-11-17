"""081 step 3: testbenches found by name, their work directory, the version guard, the run
target, `testbench new|list|check`, `assets fetch <tb>`, `runs list`'s workspace, `stack`,
and an autoware pilot's car packages."""
import argparse
import io
import json
import os
from pathlib import Path

import pytest

from scensim import assets, cli, config, project, run, runs, stack_cli, testbench_cli
from scensim import testbench as tbmod
from scensim import testbenches

TB_YAML = ("simulator: carla\nassets: []\npilots:\n  ego: {kind: autoware, vehicle: tesla_model3}\n"
           "default: first\n")


def source_tb(root: Path, name="tb_one", yaml_text=TB_YAML, suites=("first",)) -> Path:
    src = root / "ws" / "src" / name
    (src / "scenarios").mkdir(parents=True)
    (src / "scenarios" / "drive.xosc").write_text(
        (project.TEMPLATES / "carla-first-drive" / "scenarios" / "drive_to_goal.xosc").read_text())
    (src / "suites").mkdir()
    for s in suites:
        (src / "suites" / f"{s}.yaml").write_text("scenarios: [scenarios/]\n")
    (src / tbmod.TESTBENCH_FILE).write_text(yaml_text)
    (src / "package.xml").write_text(testbench_cli.PACKAGE_XML.format(
        name=name, maintainer="m", email="m@example.com"))
    return src


def install(src: Path, built: dict, symlink=True) -> Path:
    """What colcon-scensim installs: share/<pkg> (a link per file under --symlink-install)
    and the index entry, in <ws>/install/<pkg>."""
    name = src.name
    prefix = src.parent.parent / "install" / name
    share = prefix / "share" / name
    for f in sorted(src.rglob("*")):
        if f.is_file():
            out = share / f.relative_to(src)
            out.parent.mkdir(parents=True, exist_ok=True)
            if symlink:
                os.symlink(f, out)
            else:
                out.write_bytes(f.read_bytes())
    idx = prefix / "share" / "ament_index" / "resource_index"
    for kind, content in (("packages", ""), (testbenches.INDEX, json.dumps(built))):
        (idx / kind).mkdir(parents=True, exist_ok=True)
        (idx / kind / name).write_text(content)
    return prefix


NOW = {"scensim": "0.1.0", "prefix": "/opt/scensim", "build": {"commit": "a" * 40}}


@pytest.fixture
def ws(tmp_path, monkeypatch, host):
    src = source_tb(tmp_path)
    prefix = install(src, NOW)
    monkeypatch.setenv("AMENT_PREFIX_PATH",
                       f"{prefix}:{os.environ.get('AMENT_PREFIX_PATH', '')}")
    monkeypatch.setattr(tbmod, "built_over", lambda: dict(NOW))
    logs = tmp_path / "logs"
    (host["tmp"] / "user" / "host.yaml").write_text(
        f"cache: {host['cache']}\nhub_url: {host['hub'].url}\nlogs: {logs}\n")
    return {"src": src, "prefix": prefix, "logs": logs, "tmp": tmp_path}


# --- (1) by name, through the index ----------------------------------------------------------

def test_a_testbench_is_found_by_name_and_its_workspace_named(ws):
    tb = testbenches.find("tb_one")
    assert tb.share == ws["prefix"] / "share" / "tb_one"
    assert tb.source() == ws["src"]          # through --symlink-install's link
    assert tb.workspace() == ws["tmp"] / "ws"
    assert testbenches.names()[-1] == "tb_one" or "tb_one" in testbenches.names()


def test_a_name_not_built_is_refused_naming_scensim_and_colcon(ws):
    with pytest.raises(testbenches.TestbenchRefused) as e:
        testbenches.split("tb_never_built")
    assert "source scensim's setup.bash, then colcon build" in str(e.value)
    assert "skips a testbench with only a warning" in str(e.value)


def test_paths_and_a_projects_own_directories_are_no_testbench(ws, tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    assert testbenches.split("./x.xosc") is None
    assert testbenches.split("suites/first.yaml") is None
    assert testbenches.split("scenarios") is None
    assert testbenches.split("tb_one/first") == ("tb_one", "first")


# --- (2) the version guard -------------------------------------------------------------------

def test_a_testbench_built_over_another_scensim_is_refused_naming_colcon_build(ws):
    tb = testbenches.find("tb_one")
    assert testbenches.version_problem(tb, dict(NOW)) == ""
    newer = dict(NOW, scensim="0.2.0")
    why = testbenches.version_problem(tb, newer)
    assert "built over scensim 0.1.0" in why and "this is scensim 0.2.0" in why
    assert why.endswith(f"(cd {ws['tmp'] / 'ws'} && colcon build)")
    rebuilt = dict(NOW, build={"commit": "b" * 40})
    assert "colcon build" in testbenches.version_problem(tb, rebuilt)


def test_an_entry_without_what_it_was_built_over_is_another_scensim(ws):
    tb = testbenches.find("tb_one")
    tb.built = {}
    assert "colcon build" in testbenches.version_problem(tb, dict(NOW))


# --- the work directory ----------------------------------------------------------------------

def test_the_work_directory_links_the_testbench_and_holds_the_hosts_own(ws):
    tb = testbenches.find("tb_one")
    d = testbenches.work_dir(tb, {"logs": str(ws["logs"])})
    assert d == ws["logs"] / ".testbenches" / "tb_one" and d.resolve().name == "tb_one"
    assert os.readlink(d / "scenarios") == str(tb.share / "scenarios")
    assert os.readlink(d / tbmod.LOCK_FILE) == str(ws["src"] / tbmod.LOCK_FILE)  # the source's
    (d / "assets").mkdir()
    (d / tbmod.LOCK_FILE).write_text("version: 1\n")       # writes the source, to commit
    assert (ws["src"] / tbmod.LOCK_FILE).read_text() == "version: 1\n"
    # a removed entry's link goes; the host's own stay
    os.unlink(tb.share / "package.xml")
    testbenches.work_dir(tb, {"logs": str(ws["logs"])})
    assert not (d / "package.xml").exists() and (d / "assets").is_dir()


def test_a_copy_install_links_the_lock_it_installed(ws):
    src = source_tb(ws["tmp"] / "copy", name="tb_copy")
    prefix = install(src, NOW, symlink=False)
    tb = testbenches.Testbench("tb_copy", prefix / "share" / "tb_copy", prefix, NOW)
    assert tb.source() is None
    d = testbenches.work_dir(tb, {"logs": str(ws["logs"])})
    assert os.readlink(d / tbmod.LOCK_FILE) == str(tb.share / tbmod.LOCK_FILE)


# --- the run target --------------------------------------------------------------------------

def args_for(target, logs=None):
    return argparse.Namespace(target=target, project=None, logs=logs)


def test_a_testbench_alone_runs_its_default_suite_in_its_work_directory(ws):
    a = args_for("tb_one")
    info = run.resolve_target(a)
    assert info == {"name": "tb_one", "workspace": str(ws["tmp"] / "ws")}
    assert a.project == str(ws["logs"] / ".testbenches" / "tb_one")
    assert a.target == str(Path(a.project) / "suites" / "first.yaml")


def test_a_suite_is_found_before_a_path_and_a_path_is_in_the_testbench(ws):
    a = args_for("tb_one/first")
    run.resolve_target(a)
    assert a.target.endswith("suites/first.yaml")
    a = args_for("tb_one/scenarios/drive.xosc")
    run.resolve_target(a)
    assert a.target.endswith("scenarios/drive.xosc")
    with pytest.raises(run.Refused) as e:
        run.resolve_target(args_for("tb_one/nothing"))
    assert "no suite nothing" in str(e.value) and "suites: first" in str(e.value)


def test_without_default_and_with_two_suites_the_run_is_refused_listing_them(ws, monkeypatch):
    src = source_tb(ws["tmp"] / "two", name="tb_two", yaml_text=TB_YAML.replace(
        "default: first\n", ""), suites=("a", "b"))
    prefix = install(src, NOW)
    monkeypatch.setenv("AMENT_PREFIX_PATH", f"{prefix}:{os.environ['AMENT_PREFIX_PATH']}")
    with pytest.raises(run.Refused) as e:
        run.resolve_target(args_for("tb_two"))
    assert "has no default: suite; name one: tb_two/a, tb_two/b" in str(e.value)


def test_a_testbench_directory_not_yet_built_runs_in_place(ws):
    a = args_for(str(ws["src"]))
    info = run.resolve_target(a)
    assert info["name"] == "tb_one" and a.project == str(ws["src"])
    assert a.target == str(ws["src"] / "suites" / "first.yaml")


def test_a_directory_with_swb_yaml_is_refused_by_name(ws):
    (ws["src"] / tbmod.TESTBENCH_FILE).rename(ws["src"] / "swb.yaml")
    with pytest.raises(run.Refused) as e:
        run.resolve_target(args_for(str(ws["src"])))
    assert "swb.yaml is renamed testbench.yaml" in str(e.value)


def test_a_stale_testbench_is_refused_before_anything_starts(ws, monkeypatch):
    monkeypatch.setattr(tbmod, "built_over", lambda: dict(NOW, scensim="9.9.9"))
    with pytest.raises(run.Refused) as e:
        run.resolve_target(args_for("tb_one"))
    assert "colcon build" in str(e.value)


# --- (5) testbench new | list | check --------------------------------------------------------

def test_new_refuses_a_name_that_is_no_ros_package_before_writing(tmp_path, monkeypatch, capsys):
    monkeypatch.chdir(tmp_path)
    assert testbench_cli.main(["new", "my-first-drive"]) == 3
    assert "my-first-drive is not a valid ROS package name (use my_first_drive)" in \
        capsys.readouterr().err
    assert not list(tmp_path.iterdir())


def test_new_writes_a_testbench_package_with_a_default_suite(tmp_path, monkeypatch, capsys):
    (tmp_path / "ws" / "src").mkdir(parents=True)
    monkeypatch.chdir(tmp_path / "ws" / "src")
    assert testbench_cli.main(["new", "my_first_drive"]) == 0
    assert capsys.readouterr().out.startswith("wrote src/my_first_drive (carla-first-drive)")
    d = tmp_path / "ws" / "src" / "my_first_drive"
    xml = (d / "package.xml").read_text()
    assert "<build_type>scensim_testbench</build_type>" in xml
    assert "<license>TODO: License declaration</license>" in xml
    assert config.read_yaml(d / tbmod.TESTBENCH_FILE)["default"] == "first"
    assert (d / "suites" / "first.yaml").is_file()
    # what colcon build checks of it: nothing
    assert tbmod.build_findings(d, d / "package.xml") == []


def test_list_names_each_testbench_its_suites_and_workspace(ws, capsys):
    assert testbench_cli.main(["list"]) == 0
    out = capsys.readouterr().out
    assert "tb_one" in out and "first (default)" in out and str(ws["tmp"] / "ws") in out


def test_check_reports_the_package_and_this_host_apart(ws):
    out = io.StringIO()
    code = testbench_cli.check("tb_one", logs=str(ws["logs"]), out=out)
    text = out.getvalue()
    assert code != 0 and "problem: testbench.yaml" not in text      # the package is clean
    assert "problem (this host): " in text                          # its map is not fetched


def test_check_without_any_finding_says_ok(ws, monkeypatch):
    monkeypatch.setattr(testbench_cli, "host_findings", lambda tb, d: [])
    out = io.StringIO()
    assert testbench_cli.check("tb_one", logs=str(ws["logs"]), out=out) == 0
    assert out.getvalue() == "testbench: ok\n"


def test_check_live_runs_the_own_world_servers_conformance(ws, monkeypatch):
    seen = {}

    def fake(server, port, **kw):
        seen.update(server=server, port=port, log=kw.get("log"))
        return []
    from scensim import own_world
    monkeypatch.setattr(own_world, "conformance_findings", fake)
    d = testbenches.work_dir(testbenches.find("tb_one"), {"logs": str(ws["logs"])})
    os.unlink(d / tbmod.TESTBENCH_FILE)
    (d / tbmod.TESTBENCH_FILE).write_text(TB_YAML + "world: {server: [my_world, server]}\n")
    tb = testbenches.find("tb_one")
    assert testbench_cli.live_findings(tb, d, {"logs": str(ws["logs"])}) == []
    assert seen["server"] == ["my_world", "server"] and seen["port"] > 0
    assert str(seen["log"]).endswith("tb_one/world_server.log")


# --- (6) assets fetch <tb> -------------------------------------------------------------------

def test_assets_of_a_testbench_work_in_its_work_directory(ws):
    d = assets.testbench_dir("tb_one", fetching=True)
    assert d == ws["logs"] / ".testbenches" / "tb_one"


def test_a_copy_install_fetch_is_refused_naming_source(ws, monkeypatch):
    src = source_tb(ws["tmp"] / "copy", name="tb_copy")
    prefix = install(src, NOW, symlink=False)
    monkeypatch.setenv("AMENT_PREFIX_PATH", f"{prefix}:{os.environ['AMENT_PREFIX_PATH']}")
    with pytest.raises(config.ConfigError) as e:
        assets.testbench_dir("tb_copy", fetching=True)
    assert "scensim assets fetch tb_copy --source src/tb_copy" in str(e.value)
    d = assets.testbench_dir("tb_copy", source=str(src), fetching=True)
    assert os.readlink(d / tbmod.LOCK_FILE) == str(src / tbmod.LOCK_FILE)


# --- (3) runs by package, with their workspace -----------------------------------------------

def test_runs_list_of_a_testbench_shows_its_workspace(ws, capsys):
    r = ws["logs"] / "tb_one" / "runs" / "2026-11-17T10-00-00_drive_r0"
    r.mkdir(parents=True)
    (r / "result.json").write_text(json.dumps({
        "schema": "swb.result/1", "run_id": r.name, "verdict": "passed",
        "scenario": {"file": "scenarios/drive.xosc"}, "metrics": {"duration_s": 3},
        "provenance": {"testbench": {"name": "tb_one",
                                     "workspace": str(Path.home() / "my_ws")}}}))
    out = io.StringIO()
    assert runs.main(["list", "tb_one"], out=out) == 0
    row = out.getvalue().splitlines()[1].split()
    assert row[0] == r.name and row[1] == "passed" and row[-1] == "~/my_ws"


# --- (4) stack ---------------------------------------------------------------------------------

def test_stack_list_without_stacks_and_a_name_not_built(ws, capsys):
    out = io.StringIO()
    assert stack_cli.main(["--list", "--logs", str(ws["logs"])], out=out) == 0
    assert out.getvalue() == "no live stack\n"
    assert stack_cli.main(["tb_never_built", "--stop"], out=io.StringIO()) == run.EXIT_REFUSED
    assert "colcon build" in capsys.readouterr().err


def test_stack_stop_with_none_up_says_so(ws):
    out = io.StringIO()
    assert stack_cli.main(["tb_one", "--stop", "--logs", str(ws["logs"])], out=out) == 0
    assert out.getvalue() == "stack    tb_one: none is up\n"


# --- (8) an autoware pilot's car ---------------------------------------------------------------

def test_an_autoware_pilots_car_without_its_packages_is_refused_by_name():
    settings = {"pilots": {"ego": {"kind": "autoware", "vehicle": "golf_cart"},
                           "npc": {"kind": "simple", "vehicle": "golf_cart"}}}
    [f] = project.vehicle_package_problems(settings, installed=lambda p: p == "golf_cart_launch")
    assert f.refused and f.host
    assert f.what == "pilots.ego: vehicle golf_cart: package golf_cart_description not installed"
    assert project.vehicle_package_problems(settings, installed=lambda p: True) == []


# --- the front end -----------------------------------------------------------------------------

def test_the_scensim_front_end_dispatches_and_refuses_an_unknown_command(capsys):
    assert cli.main(["nope"]) == 3
    assert "scensim: no command nope" in capsys.readouterr().err
    assert cli.main(["--help"]) == 0
    assert "testbench" in capsys.readouterr().out


# --- the stack in the foreground, and testbench:= -----------------------------------------

def test_project_for_is_the_work_directory_and_keeps_the_guard(ws, monkeypatch):
    assert testbenches.project_for("tb_one") == ws["logs"] / ".testbenches" / "tb_one"
    monkeypatch.setattr(tbmod, "built_over", lambda: dict(NOW, scensim="9.9.9"))
    with pytest.raises(testbenches.TestbenchRefused) as e:
        testbenches.project_for("tb_one")
    assert "colcon build" in str(e.value)


def test_the_foreground_stack_names_the_testbench_never_its_work_directory(ws):
    from types import SimpleNamespace as NS
    tb = testbenches.find("tb_one")
    seat = NS(index=2, web_port=8100)
    host = {"logs": str(ws["logs"]), "launcher": "ros2"}
    argv = stack_cli.foreground_argv(tb, Path("/w"), host, seat, ws["tmp"] / "l")
    text = " ".join(map(str, argv))
    assert "scensim_launch" in text and "scensim.launch.xml" in text
    assert "testbench:=tb_one" in text and "seat:=2" in text and ".testbenches" not in text
    path_tb = testbenches.at_path(ws["src"])
    text = " ".join(map(str, stack_cli.foreground_argv(path_tb, ws["src"], host, seat,
                                                       ws["tmp"] / "l")))
    assert f"project:={ws['src']}" in text and "testbench:=" not in text


def test_ctrl_c_stops_the_foreground_stack_by_its_process_group(monkeypatch):
    import signal as sig
    sent = []

    class Proc:
        pid = 4242
        calls = 0

        def wait(self, timeout=None):
            Proc.calls += 1
            if Proc.calls == 1:
                raise KeyboardInterrupt()
            return 0
    monkeypatch.setattr(stack_cli.os, "killpg", lambda pgid, s: sent.append((pgid, s)))
    out = io.StringIO()
    rc = stack_cli.run_foreground(["x"], {}, out, popen=lambda *a, **k: Proc())
    assert rc == 130 and sent == [(4242, sig.SIGINT)]
    assert "stopping (Ctrl-C)" in out.getvalue()


# --- 243's review of 76f5a2b -------------------------------------------------------------------

def test_new_twice_says_it_already_exists(tmp_path, monkeypatch, capsys):
    (tmp_path / "src").mkdir()
    monkeypatch.chdir(tmp_path / "src")
    assert testbench_cli.main(["new", "tb_x"]) == 0
    capsys.readouterr()
    assert testbench_cli.main(["new", "tb_x"]) == 3
    assert "src/tb_x already exists" in capsys.readouterr().err


def test_new_writes_a_testbench_readme_and_no_gitignore(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    assert testbench_cli.main(["new", "tb_x"]) == 0
    readme = (tmp_path / "tb_x" / "README.md").read_text()
    assert "scensim run tb_x" in readme and "colcon build --symlink-install" in readme
    assert "project" not in readme and "ros2 run" not in readme
    assert not (tmp_path / "tb_x" / ".gitignore").exists()
    assert not (tmp_path / "tb_x" / "gitignore").exists()


def test_check_names_files_in_the_testbench_and_the_fetch_with_its_name(ws):
    out = io.StringIO()
    testbench_cli.check("tb_one", logs=str(ws["logs"]), out=out)
    text = out.getvalue()
    assert ".testbenches" not in text and str(ws["logs"]) not in text
    if "assets fetch" in text:
        assert "scensim assets fetch tb_one)" in text


def test_runs_outside_a_testbench_name_the_testbench_form(ws, tmp_path, monkeypatch, capsys):
    monkeypatch.chdir(tmp_path)
    assert runs.main(["list"]) == 3
    assert "name a testbench: scensim runs list <testbench>" in capsys.readouterr().err
    assert runs.main(["show", "drive"]) == 3
    assert "no run drive in any testbench's runs" in capsys.readouterr().err
    out = io.StringIO()
    assert runs.main(["list", "tb_one"], out=out) == 0
    assert out.getvalue() == "no runs yet\n"
    r = ws["logs"] / "tb_one" / "runs" / "2026-11-17T10-00-00_drive_r0"
    r.mkdir(parents=True)
    (r / "result.json").write_text(json.dumps({"run_id": r.name, "verdict": "passed"}))
    out = io.StringIO()
    assert runs.main(["show", "drive"], out=out) == 0
    assert out.getvalue().startswith(f"run       {r.name}\n")


def test_testbench_list_shows_the_workspace_with_a_tilde(ws, capsys, monkeypatch):
    monkeypatch.setenv("HOME", str(ws["tmp"]))
    assert testbench_cli.main(["list"]) == 0
    assert "(~/ws)" in capsys.readouterr().out


def test_scensim_alone_prints_the_commands_and_exits_0(capsys):
    assert cli.main([]) == 0
    out = capsys.readouterr().out
    assert "testbench  new, list and check testbenches" in out


# --- 118's journey run (F1-F4) ---------------------------------------------------------------

def test_two_workspaces_of_one_testbench_take_turns_and_a_live_stack_holds_it(ws, monkeypatch):
    other = source_tb(ws["tmp"] / "clone")
    prefix2 = install(other, NOW)
    host = {"logs": str(ws["logs"])}
    d = testbenches.work_dir(testbenches.find("tb_one"), host)
    assert (d / testbenches.WORKSPACE_FILE).read_text().strip() == str(ws["prefix"])
    tb2 = testbenches.Testbench("tb_one", prefix2 / "share" / "tb_one", prefix2, dict(NOW))
    # no stack up: the sourced workspace's links win
    testbenches.work_dir(tb2, host)
    assert os.readlink(d / "scenarios") == str(tb2.share / "scenarios")
    # a stack from the first workspace is up: the second is refused, naming both and the stop
    testbenches.work_dir(testbenches.find("tb_one"), host)
    monkeypatch.setattr(testbenches, "_live_stack", lambda name, h: {"seat": {"seat": 0}})
    with pytest.raises(testbenches.TestbenchRefused) as e:
        testbenches.work_dir(tb2, host)
    msg = str(e.value)
    assert str(ws["prefix"]) in msg and str(prefix2) in msg
    assert "scensim stack tb_one --stop" in msg
    assert os.readlink(d / "scenarios") == str(ws["prefix"] / "share" / "tb_one" / "scenarios")


def test_a_scenario_refusal_of_a_testbench_names_its_fetch_and_files(ws, monkeypatch):
    from scensim.findings import Finding
    d = testbenches.work_dir(testbenches.find("tb_one"), {"logs": str(ws["logs"])})
    monkeypatch.setattr(project, "scenario_problems", lambda p, files: [
        Finding(f"{d}/scenarios/drive.xosc", "map Town01 is not fetched",
                fix="scensim assets fetch", refused=True)])
    with pytest.raises(run.Refused) as e:
        run.check_scenarios(d, [], testbench="tb_one")
    msg = str(e.value)
    assert msg.startswith("testbench check found problems")
    assert "scensim assets fetch tb_one)" in msg and str(d) not in msg


def test_a_started_stack_is_named_and_stopped_by_its_testbench(ws, tmp_path):
    d = testbenches.work_dir(testbenches.find("tb_one"), {"logs": str(ws["logs"])})
    assert run.stack_name(d) == "tb_one" and run.stack_target(d) == "tb_one"
    assert run.stack_target(ws["src"]) == str(ws["src"])


def test_runs_refuses_a_renamed_variable_before_asking_for_a_testbench(ws, monkeypatch, capsys):
    monkeypatch.setenv("SWB_LOG_ROOT", "x")
    assert runs.main(["list"]) == 3
    err = capsys.readouterr().err
    assert "SWB_LOG_ROOT was renamed" in err and "name a testbench" not in err


# --- 118's pip journey (P2-P4) --------------------------------------------------------------

def test_the_front_ends_commands_go_to_it_when_it_is_installed(monkeypatch):
    got = []
    monkeypatch.setattr(cli, "_front_end", lambda: lambda argv: got.append(argv) or 0)
    assert cli.main(["--version"]) == 0 and cli.main(["install", "--force"]) == 0
    assert got == [["--version"], ["install", "--force"]]


def test_without_the_front_end_a_source_build_answers_version_and_refuses_install(
        monkeypatch, tmp_path, capsys):
    monkeypatch.setattr(cli, "_front_end", lambda: None)
    monkeypatch.setattr(tbmod, "built_over", lambda: {
        "scensim": "0.1.0+abcdef123456", "prefix": str(tmp_path), "build": None})
    assert cli.main(["--version"]) == 0
    assert capsys.readouterr().out == f"scensim 0.1.0+abcdef123456 (source build at {tmp_path})\n"
    assert cli.main(["install"]) == 3
    assert "this scensim is a source build; update it with git and colcon" in \
        capsys.readouterr().err


def test_a_release_is_known_by_its_own_version_and_a_source_build_by_its_commit(tmp_path):
    assert tbmod.scensim_version(str(tmp_path), "0.1.0", {"commit": "c" * 40}) == \
        "0.1.0+cccccccccccc"
    (tmp_path / tbmod.RELEASE_FILE).write_text('version: "0.2.0"\nside: sim\n')
    assert tbmod.scensim_version(str(tmp_path), "0.1.0", {"commit": "c" * 40}) == "0.2.0"


def test_an_upgrade_behind_current_is_another_scensim(monkeypatch, tmp_path):
    # ~/.local/share/scensim/current -> 0.1.0, then -> 0.2.0: the same prefix through
    # current, the same package version (0.1.0); the release's version and the resolved
    # prefix differ, and the guard fires
    root = tmp_path / "share"
    for v in ("0.1.0", "0.2.0"):
        (root / v / "sim").mkdir(parents=True)
        (root / v / "sim" / tbmod.RELEASE_FILE).write_text(f'version: "{v}"\n')
    cur = root / "current"
    os.symlink(root / "0.1.0", cur)
    import ament_index_python.packages as aip
    monkeypatch.setattr(aip, "get_package_prefix", lambda name: str(cur / "sim"))
    before = tbmod.built_over()
    assert before["scensim"] == "0.1.0" and before["prefix"] == str(root / "0.1.0" / "sim")
    os.unlink(cur)
    os.symlink(root / "0.2.0", cur)
    after = tbmod.built_over()
    tb = testbenches.Testbench("tb_one", tmp_path / "tb", tmp_path / "ws", before)
    problem = testbenches.version_problem(tb, after)
    assert "was built over scensim 0.1.0" in problem and "this is scensim 0.2.0" in problem


def test_a_pilot_launch_whose_package_is_missing_ends_the_wait_at_once(tmp_path):
    console = tmp_path / "console.log"
    console.write_text("old line\n")
    w = run.CrashWatch(console)
    assert w.launch_failed() is None
    with console.open("a") as f:
        f.write("[ERROR] [launch]: Caught exception in launch (see debug for traceback): "
                "\"package 'acb_launch' not found, searching: ['/opt/ros/humble']\"\n")
    msg = w.launch_failed()
    assert "package acb_launch not found" in msg and "vehicle side" in msg
    console.write_text("[ERROR] [launch]: Caught exception in launch (see debug for "
                       "traceback): file not found: x.launch.xml\n")
    w2 = run.CrashWatch(tmp_path / "none.log")
    w2.path, w2.offset = console, 0
    assert w2.launch_failed() == "its launch fails at once: file not found: x.launch.xml"


def test_waiting_for_stack_pilots_names_a_launch_that_cannot_start(tmp_path, monkeypatch):
    proc = tmp_path / "proc"
    (proc / "pilot-ego").mkdir(parents=True)
    console = proc / "pilot-ego" / "console.log"
    console.write_text("")
    rec = {"proc": str(proc), "pilots": [{"entity": "ego", "domain": 2, "ready": {}}]}
    ctx = run.Context.__new__(run.Context)
    ctx.stack, ctx.ready_timeout = rec, 600

    def not_ready(*a, **k):
        console.write_text("Package 'acb_launch' not found\n")
        return False
    monkeypatch.setattr(run.stackmod, "check_ready", not_ready)
    monkeypatch.setattr(run.time, "sleep", lambda s: None)
    msg = run.wait_pilots_ready(ctx)
    assert msg.startswith("pilot ego is not coming up: its launch fails at once: package "
                          "acb_launch not found")
