"""``scensim testbench new|list|check``: testbench packages.

- ``new <name> [--template T | --from pkg://...] [--default S] [--maintainer M] [--license L]``:
  a testbench package in ``./<name>``: the 076 starter (or a copy of a project), a
  ``package.xml`` of build type ``scensim_testbench`` and a ``default:`` suite. The name must be
  a ROS package name; nothing is written otherwise. A copy's paths that leave the source are
  rewritten to ``pkg://`` where they can be, and anything left to the user is printed (055).
- ``list``: the testbenches of the sourced chain, each with its suites and workspace.
- ``check [<testbench>|<dir>] [--live]``: the package's findings (what ``colcon build``
  checks) and this host's (its tools, fetched assets, simulator); ``--live`` also starts the
  testbench's own world server (``world: {server: ...}``) and runs the conformance check.
"""
import argparse
import os
import re
import shutil
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
  <license>{license}</license>

  <exec_depend>scensim</exec_depend>
{depends}
  <export>
    <build_type>scensim_testbench</build_type>
  </export>
</package>
"""
DEFAULT_SUITE = "first"
PLACEHOLDER_MAINTAINER = ("Maintainer", "maintainer@example.com")
PLACEHOLDER_LICENSE = "TODO: License declaration"
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


def _package_of(target: str):
    """``(package, path in it)`` for a file or directory: an installed share's, else the
    nearest source package.xml's; None when it is in no package."""
    pkg = config.package_project(target)
    if pkg:
        for prefix in (x for x in os.environ.get("AMENT_PREFIX_PATH", "").split(":") if x):
            share = os.path.join(prefix, "share", pkg)
            if target == share or target.startswith(share + os.sep):
                return pkg, os.path.relpath(target, share)
    for d in Path(target).parents:
        if (d / "package.xml").is_file():
            name = tbmod.package_name(d / "package.xml")
            return (name, os.path.relpath(target, d)) if name else None
    return None


def _same(a: str, b: Path) -> bool:
    """The installed ``b`` is the file ``a`` names: the same file (a symlink install), or the
    same content (a copying one, or another checkout's install of the package)."""
    import filecmp
    try:
        if os.path.isdir(a):
            return b.is_dir()
        return os.path.samefile(a, b) or filecmp.cmp(a, b, shallow=False)
    except OSError:
        return False


def rewrite_leaving_paths(root: Path, dest: Path) -> tuple:
    """In the copy ``dest`` of the project ``root``: each path that leaves the source
    (testbench.yaml, the suites, the Isaac ego config) made ``pkg://<package>/<path>`` when
    that resolves to the same file (issue 055). ``(packages named by pkg://, notes)``."""
    root = Path(root).resolve()
    names, notes = set(), []
    for f, base, doc in tbmod._reference_files(dest):
        rel = f.relative_to(dest)
        src_base = root / rel.parent
        text = f.read_text()
        for s in sorted(set(tbmod._strings(doc))):
            names.update(m.group(1) for m in tbmod.PKG_REF.finditer(s))
            if "://" in s or "$(" in s or not re.search(r"(^|/)\.\.(/|$)", s):
                continue
            target = os.path.normpath(os.path.join(root, s))
            if not os.path.exists(target):
                target = os.path.normpath(os.path.join(src_base, s))
            if target == str(root) or target.startswith(str(root) + os.sep):
                continue   # inside the source: the same relative path holds in the copy
            found = _package_of(target) if os.path.exists(target) else None
            ref = f"pkg://{found[0]}/{found[1]}".rstrip("/.") if found else None
            if s.endswith("/") and ref:
                ref += "/"
            try:
                ok = ref is not None and _same(target, config.pkg_path(ref))
            except config.ConfigError:
                ok = False
            if not ok:
                notes.append(f"{rel}: {s} leaves the testbench and is no installed package's "
                             "file: put it in a package and name it pkg://<package>/<path>")
                continue
            text = text.replace(s, ref)
            names.add(found[0])
        if text != f.read_text():
            f.write_text(text)
    return names, notes


def _choose_default(settings: dict, suites: list, wanted: str = None):
    """``(suite, note)``: ``--default``, the source's default:, a lone suite or one named
    ``default``; else the first, with a note naming the others (issue 055)."""
    if wanted:
        if wanted not in suites:
            raise config.ConfigError(f"--default {wanted}: no suites/{wanted}.yaml (the suites: "
                                     f"{', '.join(suites) or 'none'})")
        return wanted, ""
    if len(suites) == 1 or "default" in suites:
        return ("default" if "default" in suites else suites[0]), ""
    return suites[0], (f"default: {suites[0]}, the first of {len(suites)} suites "
                       f"({', '.join(suites)}): name another with --default <suite>, or edit "
                       "testbench.yaml")


def new_testbench(name: str, where: Path = None, template: str = None, src: str = None,
                  default: str = None, maintainer: str = None, license: str = None,
                  notes: list = None) -> Path:
    """Write the testbench package ``<where>/<name>``; ConfigError when it cannot be. What
    the user still has to do goes to ``notes``."""
    notes = [] if notes is None else notes
    if not testbenches.PACKAGE_NAME.match(name):
        raise config.ConfigError(f"{name} is not a valid ROS package name (use "
                                 f"{suggested_name(name)})")
    dest = Path(where or Path.cwd()) / name
    if dest.exists():
        raise config.ConfigError(f"{_shown(dest)} already exists (fix: another name, or move it "
                                 "aside)")
    m = re.fullmatch(r"\s*(.*?)\s*<([^<>\s]+@[^<>\s]+)>\s*", maintainer or "")
    if maintainer and not m:
        raise config.ConfigError(f"--maintainer {maintainer!r}: give \"Name <email>\"")
    try:
        return _fill(dest, name, template, src, default, m, license, notes)
    except BaseException:
        shutil.rmtree(dest, ignore_errors=True)   # nothing half-written is left behind
        raise


def _fill(dest, name, template, src, default, m, license, notes) -> Path:
    depends = set()
    if src:
        # a project of before roadmap 080 (swb.yaml) is copied and renamed (issue 061)
        old = not str(src).startswith("pkg://") and projectmod.is_old_project(src)
        projectmod.copy_project(src, dest, old_name=old)
        if old:
            notes += projectmod.rename_old(dest)
        depends, left = rewrite_leaving_paths(projectmod.source_project(src, old), dest)
        notes += left
    else:
        projectmod.new_project(dest, template or projectmod.DEFAULT_TEMPLATE)
    tb = dest / tbmod.TESTBENCH_FILE
    settings = config.read_yaml(tb)
    suites = sorted(p.stem for p in (dest / "suites").glob("*.y*ml"))
    if default and "default" in settings:
        raise config.ConfigError(f"--default {default}: the source names its own default: "
                                 f"{settings['default']} (edit its testbench.yaml instead)")
    if "default" not in settings:
        if not suites:
            (dest / "suites").mkdir(exist_ok=True)
            (dest / "suites" / f"{DEFAULT_SUITE}.yaml").write_text(
                "# every scenario of the testbench, once (scensim run <testbench> runs this)\n"
                "scenarios: [scenarios/]\n")
            suites = [DEFAULT_SUITE]
        chosen, note = _choose_default(settings, suites, default)
        if note:
            notes.append(note)
        with tb.open("a") as f:
            f.write(f"default: {chosen}   # the suite `scensim run {name}` runs\n")
    # a testbench's README and no .gitignore: assets, state and runs never live in src/
    (dest / "gitignore").unlink(missing_ok=True)
    (dest / ".gitignore").unlink(missing_ok=True)
    if not src:   # a copy keeps its own README
        (dest / "README.md").write_text(README.format(name=name))
    if not (dest / "package.xml").exists():
        who = (m.group(1), m.group(2)) if m else (_git("user.name") or PLACEHOLDER_MAINTAINER[0],
                                                  _git("user.email") or PLACEHOLDER_MAINTAINER[1])
        lic = license or PLACEHOLDER_LICENSE
        extra = "".join(f"  <exec_depend>{d}</exec_depend>\n"
                        for d in sorted(depends - {name, "scensim"}))
        (dest / "package.xml").write_text(PACKAGE_XML.format(
            name=name, maintainer=who[0], email=who[1], license=lic, depends=extra))
        left = [x for x, ph in (("maintainer", who == PLACEHOLDER_MAINTAINER),
                                ("license", lic == PLACEHOLDER_LICENSE)) if ph]
        if left:
            notes.append(f"package.xml: the {' and '.join(left)} "
                         f"{'are placeholders' if len(left) > 1 else 'is a placeholder'}: "
                         "edit it, or give --maintainer \"Name <email>\" / --license <SPDX id>")
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


def live_findings(tb, d: Path, host: dict, note=None) -> list:
    """``--live``: the testbench's own world server (``world.server``) started on a free port
    and run through the full conformance check; else the stock world server of its live stack
    probed with ``Hello`` (the stack's bridge owns it, so a conformance run would be refused).
    No stack up: nothing to probe, said through ``note``."""
    from scensim import own_world
    world = config.project_settings(d).get("world")
    if not (isinstance(world, dict) and world.get("server")):
        from scensim import run as runmod
        try:
            rec = runmod.find_stack(d, host)
        except runmod.NoStack:
            if note:
                note(f"no stack of {tb.name} is up, so its world server was not probed "
                     f"(start one: scensim stack {tb.name})")
            return []
        except runmod.Refused as e:   # up but not ready, or another seat's
            if note:
                note(f"{e}; its world server was not probed")
            return []
        return own_world.hello_findings(rec["ports"]["world"])
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
        here += live_findings(tb, d, host,
                              note=lambda text: out.write(f"note (this host): {text}\n"))
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
    for n in projectmod.autoware_side_notes(config.project_settings(d)):
        out.write(f"note (this host): {n}\n")
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
    n.add_argument("--default", help="the suite `scensim run <name>` runs (default: the source's, "
                                     "a lone suite, or one named default)")
    n.add_argument("--maintainer", help='package.xml\'s maintainer, "Name <email>" (default: git '
                                        "config's user)")
    n.add_argument("--license", help="package.xml's license, an SPDX id (e.g. Apache-2.0)")
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
            notes = []
            dest = new_testbench(args.name, template=args.template, src=args.src,
                                 default=args.default, maintainer=args.maintainer,
                                 license=args.license, notes=notes)
        except config.ConfigError as e:
            sys.stderr.write(f"scensim testbench new: refused: {e}\n")
            return 3
        what = args.src or args.template or projectmod.DEFAULT_TEMPLATE
        print(f"wrote {_shown(dest)} ({what}); next: colcon build --symlink-install, then "
              f"scensim assets fetch {args.name}")
        for note in notes:
            print(f"note: {note}")
        return 0
    if args.cmd == "list":
        return list_testbenches()
    return check(args.target, args.live, args.logs)


if __name__ == "__main__":
    sys.exit(main())
