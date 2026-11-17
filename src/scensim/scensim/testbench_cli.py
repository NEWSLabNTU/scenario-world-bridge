"""``scensim testbench new|list|check``: testbench packages.

- ``new <name> [--template T | --from pkg://...]``: a testbench package in ``./<name>``: the
  076 starter (or a copy of a project), a ``package.xml`` of build type ``scensim_testbench``
  and a ``default:`` suite. The name must be a ROS package name; nothing is written otherwise.
- ``list``: the testbenches of the sourced chain, each with its suites and workspace.
- ``check [<testbench>|<dir>] [--live]``: the package's findings (what ``colcon build``
  checks) and this host's (its tools, fetched assets, simulator); ``--live`` also starts the
  testbench's own world server (``world: {server: ...}``) and runs the conformance check.
"""
import argparse
import os
import socket
import subprocess
import sys
from pathlib import Path


from scensim import config
from scensim import project as projectmod
from scensim import testbench as tbmod
from scensim import testbenches
from scensim.findings import Finding, exit_code

PACKAGE_XML = """<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{name}</name>
  <version>0.0.1</version>
  <description>A scensim testbench: {name}</description>
  <maintainer email="{email}">{maintainer}</maintainer>
  <license>TODO: License declaration</license>

  <exec_depend>scensim</exec_depend>

  <export>
    <build_type>scensim_testbench</build_type>
  </export>
</package>
"""
DEFAULT_SUITE = "first"
README = """# {name}

A scensim testbench: scenarios, suites and the pilots that drive them, in testbench.yaml.

From the workspace (the directory holding src/), with scensim's setup.bash sourced:

    colcon build --symlink-install
    source install/setup.bash
    scensim assets fetch {name}      # this host's maps and datasets; writes scensim.lock here
    scensim testbench check {name}
    scensim run {name}               # its default suite; {name}/<suite> or {name}/<path> for others

Commit scensim.lock. With --symlink-install an edit here takes effect without a rebuild;
adding or removing a file needs one.
"""


def _git(key: str) -> str:
    try:
        return subprocess.run(["git", "config", key], capture_output=True, text=True,
                              timeout=5).stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        return ""


def suggested_name(name: str) -> str:
    s = "".join(c if c.isalnum() else "_" for c in name.lower()).strip("_")
    return s if s and s[0].isalpha() else f"tb_{s}"


def new_testbench(name: str, where: Path = None, template: str = None, src: str = None) -> Path:
    """Write the testbench package ``<where>/<name>``; ConfigError when it cannot be."""
    if not testbenches.PACKAGE_NAME.match(name):
        raise config.ConfigError(f"{name} is not a valid ROS package name (use "
                                 f"{suggested_name(name)})")
    dest = Path(where or Path.cwd()) / name
    if dest.exists():
        raise config.ConfigError(f"{_shown(dest)} already exists (fix: another name, or move it "
                                 "aside)")
    if src:
        projectmod.copy_project(src, dest)
    else:
        projectmod.new_project(dest, template or projectmod.DEFAULT_TEMPLATE)
    tb = dest / tbmod.TESTBENCH_FILE
    settings = config.read_yaml(tb)
    suites = sorted(p.stem for p in (dest / "suites").glob("*.y*ml"))
    if "default" not in settings:
        if not suites:
            (dest / "suites").mkdir(exist_ok=True)
            (dest / "suites" / f"{DEFAULT_SUITE}.yaml").write_text(
                "# every scenario of the testbench, once (scensim run <testbench> runs this)\n"
                "scenarios: [scenarios/]\n")
            suites = [DEFAULT_SUITE]
        with tb.open("a") as f:
            f.write(f"default: {suites[0]}   # the suite `scensim run {name}` runs\n")
    # a testbench's README and no .gitignore: assets, state and runs never live in src/
    (dest / "gitignore").unlink(missing_ok=True)
    (dest / ".gitignore").unlink(missing_ok=True)
    if not src:   # a copy keeps its own README
        (dest / "README.md").write_text(README.format(name=name))
    if not (dest / "package.xml").exists():
        maintainer = _git("user.name") or "Maintainer"
        email = _git("user.email") or "maintainer@example.com"
        (dest / "package.xml").write_text(PACKAGE_XML.format(name=name, maintainer=maintainer,
                                                             email=email))
    return dest


def _shown(dest: Path) -> str:
    """``dest`` as the user's workspace names it: ``src/<name>`` inside a workspace's src/."""
    dest = Path(dest)
    if dest.parent.name == "src":
        return os.path.relpath(dest, dest.parent.parent)
    return str(dest)


def resolve(target, logs=None):
    """``(testbench, its directory to check, the host settings)`` for a name or a path."""
    over = {"logs": logs} if logs else {}
    t = str(target or ".")
    sp = testbenches.split(t)
    if sp is not None:
        tb = testbenches.find(sp[0])
        host = config.host_settings(None, over)
        return tb, testbenches.work_dir(tb, host), host
    tb = testbenches.at_path(t)
    return tb, tb.share, config.host_settings(tb.share, over)


def package_findings(tb) -> list:
    share = tb.share
    if (share / "package.xml").is_file():
        return tbmod.build_findings(share, share / "package.xml")
    # a project directory without package.xml: today's rules, the package's part
    return [f for f in projectmod.check_project(share) if not tbmod.is_host_finding(f)]


def host_findings(tb, d: Path) -> list:
    out = [f for f in projectmod.check_project(d) if tbmod.is_host_finding(f)]
    problem = testbenches.version_problem(tb)
    if problem:
        out.insert(0, Finding("host", problem, refused=True, host=True))
    return out


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def live_findings(tb, d: Path, host: dict) -> list:
    """The testbench's own world server against the conformance check, when it has one."""
    world = config.project_settings(d).get("world")
    if not (isinstance(world, dict) and world.get("server")):
        return []
    from scensim import own_world
    log = Path(host["logs"]).expanduser() / tb.name / "world_server.log"
    log.parent.mkdir(parents=True, exist_ok=True)
    return own_world.conformance_findings(list(world["server"]), _free_port(), log=log)


def check(target=None, live=False, logs=None, out=None) -> int:
    out = out or sys.stdout
    try:
        tb, d, host = resolve(target, logs)
    except (testbenches.TestbenchRefused, config.ConfigError) as e:
        out.write(f"problem: {target or '.'}: {e}\n")
        return 3
    pkg = package_findings(tb)
    here = host_findings(tb, d)
    if live and not pkg:
        here += live_findings(tb, d, host)
    def shown(f) -> str:
        # names relative to the testbench (the files the user edits are src/<tb>/...), and the
        # asset fix with the testbench, so it runs from the workspace
        line = f"{f}\n".replace(f"{tb.share}/", "")
        if tb.installed:
            return testbenches.shown_finding(line, tb.name, d)
        return line.replace(f"{d}/", "")
    for f in pkg:
        out.write(shown(f))
    for f in here:
        out.write(shown(f).replace("problem: ", "problem (this host): ", 1))
    if not pkg and not here:
        out.write("testbench: ok\n")
    return exit_code(pkg + here)


def list_testbenches(out=None) -> int:
    out = out or sys.stdout
    found = testbenches.names()
    if not found:
        out.write("no testbench in the sourced workspace (colcon build it, then source its "
                  "install/setup.bash)\n")
        return 0
    for name in found:
        tb = testbenches.find(name)
        suites = testbenches.suites(tb.share)
        try:
            default = config.read_yaml(tb.share / tbmod.TESTBENCH_FILE).get("default")
        except config.ConfigError:
            default = None
        shown = ", ".join(f"{s} (default)" if s == default else s for s in suites) or "no suite"
        stale = "  [built over another scensim: colcon build]" if \
            testbenches.version_problem(tb) else ""
        from scensim.assets import _home
        out.write(f"{name:24} {shown}  ({_home(tb.workspace())}){stale}\n")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="scensim testbench", description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    n = sub.add_parser("new", help="write a testbench package in ./<name>")
    n.add_argument("name", help="a ROS package name: lower case letters, digits, underscores")
    how = n.add_mutually_exclusive_group()
    how.add_argument("--template", help=f"one of {', '.join(projectmod.templates())} "
                                        f"(default {projectmod.DEFAULT_TEMPLATE})")
    how.add_argument("--from", dest="src", help="a project to copy (a directory or pkg://...)")
    sub.add_parser("list", help="the testbenches of the sourced workspace")
    c = sub.add_parser("check", help="the testbench's findings and this host's")
    c.add_argument("target", nargs="?", help="a testbench name or directory (default: .)")
    c.add_argument("--live", action="store_true",
                   help="also run its own world server against the conformance check")
    c.add_argument("--logs", help="the log root (default: the host settings' logs:)")
    try:
        args = ap.parse_args(argv)
    except SystemExit as e:
        return 0 if e.code == 0 else 3
    if args.cmd == "new":
        try:
            dest = new_testbench(args.name, template=args.template, src=args.src)
        except config.ConfigError as e:
            sys.stderr.write(f"scensim testbench new: refused: {e}\n")
            return 3
        what = args.src or args.template or projectmod.DEFAULT_TEMPLATE
        print(f"wrote {_shown(dest)} ({what}); next: colcon build --symlink-install, then "
              f"scensim assets fetch {args.name}")
        return 0
    if args.cmd == "list":
        return list_testbenches()
    return check(args.target, args.live, args.logs)


if __name__ == "__main__":
    sys.exit(main())
