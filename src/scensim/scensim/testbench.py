"""The testbench check at build (roadmap 081 step 2): what colcon-scensim's
``scensim_testbench`` build type runs on an installed testbench, ``share/<pkg>/``.

A testbench is a ROS package holding ``testbench.yaml`` (a project file's schema) and the files
it names. ``build_findings`` is the package's own findings only: what holds on any host, before
any asset is fetched. So a build on a fresh host or a CI box fails only on the package itself.
``scensim testbench check`` (step 3) is these plus the host's findings (its tools, fetched assets
and maps, its CARLA or Isaac install).

On top of today's project rules (``scensim.project``), two of 081's:

- a ``pkg://<package>/...`` reference names a package declared in package.xml
  (``<depend>`` or ``<exec_depend>``), or the testbench itself;
- no path leaves the package: a relative path that climbs out of it (``../``) is refused,
  naming the ``pkg://`` form.

Built from today's public ``scensim.project`` functions; splitting ``check_project`` into the
package's findings and the host's is 241's, after the landing, and then this module calls that.
"""
import os
import re
import xml.etree.ElementTree as ET
from dataclasses import replace
from pathlib import Path

import yaml

from scensim import config
from scensim import own_world
from scensim import project as projectmod
from scensim.findings import Finding

TESTBENCH_FILE = "testbench.yaml"
LOCK_FILE = "scensim.lock"
# written by the host at run time, never part of a package (colcon-scensim installs none)
HOST_DIRS = frozenset({"assets", "state", "runs", ".scensim", ".swb"})
PKG_REF = re.compile(r"pkg://([A-Za-z0-9_]+)(/[^\s\"']*)?")
DEPEND_TAGS = ("depend", "exec_depend")


def package_name(package_xml: Path) -> str:
    return (ET.parse(package_xml).getroot().findtext("name") or "").strip()


def declared_packages(package_xml: Path) -> set:
    """The packages package.xml declares a testbench may name (``<depend>``,
    ``<exec_depend>``)."""
    root = ET.parse(package_xml).getroot()
    return {(e.text or "").strip() for tag in DEPEND_TAGS for e in root.iter(tag)} - {""}


def _strings(node):
    """Every string in a YAML document, anywhere (keys of mappings too)."""
    if isinstance(node, str):
        yield node
    elif isinstance(node, dict):
        for k, v in node.items():
            yield from _strings(k)
            yield from _strings(v)
    elif isinstance(node, (list, tuple)):
        for v in node:
            yield from _strings(v)


def _reference_files(share: Path):
    """``(file, its directory, document)`` for testbench.yaml and every suite: where the
    package names other files and packages."""
    out = []
    for f in [share / TESTBENCH_FILE] + sorted((share / "suites").glob("*.y*ml")):
        if not f.is_file():
            continue
        try:
            doc = yaml.safe_load(f.read_text())
        except (OSError, yaml.YAMLError):
            continue   # the schema check names an unreadable file
        out.append((f, f.parent, doc))
    return out


LEAVES_FIX = ("put the file in a package and name it as pkg://<package>/<path>, declared in "
              "package.xml")


def _file_values(f: Path, doc) -> list:
    """The values that name files: a suite's scenarios and distribution; testbench.yaml's
    vehicles, ground files, world config files and Isaac ego config. (Not every string: a ROS
    topic, e.g. a logging topic, is absolute and no path.)"""
    if not isinstance(doc, dict):
        return []
    if f.name != TESTBENCH_FILE:
        items, _ = projectmod.suite_items(doc.get("scenarios") or [])
        return [str(p) for p, _ in items] + ([str(doc["distribution"])]
                                             if doc.get("distribution") else [])
    out = [v for v in doc.get("vehicles") or [] if isinstance(v, str)]
    out += list(_strings(doc.get("ground") or {}))
    world, isaac = doc.get("world") or {}, doc.get("isaac") or {}
    if isinstance(world, dict):
        out += [world[k] for k in ("config_file", "world_config_file")
                if isinstance(world.get(k), str)]
    if isinstance(isaac, dict) and isinstance(isaac.get("ego"), str):
        out.append(isaac["ego"])
    return out


def _leaves(share: Path, base: Path, s: str) -> bool:
    """Whether path ``s`` leaves the package: absolute, or climbing out of it. Lexical
    (``normpath``), not ``resolve()``: under --symlink-install each installed file is a link
    into the source, and following it would put every file outside ``share``. Resolved as
    scensim resolves it (collect_scenarios): from the testbench's root, else from the suite's
    directory when only that one exists."""
    if os.path.isabs(s) or s.startswith("~"):
        return True
    root = os.path.normpath(share)
    target = os.path.normpath(os.path.join(share, s))
    if not os.path.exists(target) and os.path.exists(os.path.join(base, s)):
        target = os.path.normpath(os.path.join(base, s))
    return target != root and not target.startswith(root + os.sep)


def reference_findings(share: Path, package_xml: Path) -> list:
    """081's two rules: ``pkg://`` names a declared package; no path leaves the package
    (``../`` out of it anywhere; an absolute path in a value that names a file)."""
    own = package_name(package_xml)
    declared = declared_packages(package_xml) | {own}
    out = []
    for f, base, doc in _reference_files(share):
        rel = f.relative_to(share).as_posix()
        seen = set()
        for s in _strings(doc):
            for m in PKG_REF.finditer(s):
                if m.group(1) not in declared:
                    out.append(Finding(rel, f"{m.group(0)}: package {m.group(1)} is not declared",
                                       f"add <exec_depend>{m.group(1)}</exec_depend> to "
                                       "package.xml"))
            climbs = re.search(r"(^|/)\.\.(/|$)", s)
            if climbs and "://" not in s and "$(" not in s and _leaves(share, base, s):
                out.append(Finding(rel, f"{s}: leaves the testbench", LEAVES_FIX))
                seen.add(s)
        for s in _file_values(f, doc):
            if s not in seen and "://" not in s and "$(" not in s and \
                    (os.path.isabs(s) or s.startswith("~")):
                out.append(Finding(rel, f"{s}: leaves the testbench", LEAVES_FIX))
                seen.add(s)
    return out


# The findings today's project rules make about the host, not the package, that the rules
# build_findings runs can make: check_scenario's map not fetched here (project.py: f"map {name}
# is not fetched (assets/maps/{name})"). Assets are fetched per host and never installed. Exact,
# so no package finding that mentions fetching is dropped; a test holds this list to
# project.py's wording. In 241's split of check_project this becomes a field of Finding.
HOST_FINDINGS = (re.compile(r"map \S+ is not fetched \(assets/maps/\S+\)"),)


def is_host_finding(f: Finding) -> bool:
    """A finding about this host (an asset not fetched here), not about the package."""
    return any(rx.fullmatch(f.what) for rx in HOST_FINDINGS)


def split_findings(findings) -> tuple:
    """``(the package's, the host's)``: what `testbench check` reports in two parts."""
    package = [f for f in findings if not is_host_finding(f)]
    return package, [f for f in findings if is_host_finding(f)]


def _as_testbench(f: Finding) -> Finding:
    return replace(f, file=TESTBENCH_FILE) if f.file == config.PROJECT_FILE else f


def build_findings(share, package_xml) -> list:
    """The installed testbench's own findings (``share``: ``share/<pkg>/``): today's project
    rules that hold on any host, then 081's reference rules. Empty: it builds."""
    share, package_xml = Path(share), Path(package_xml)
    tb = share / TESTBENCH_FILE
    if not tb.is_file():
        if (share / config.PROJECT_FILE).is_file():
            return [Finding(config.PROJECT_FILE, f"a testbench's file is {TESTBENCH_FILE}, not "
                            f"{config.PROJECT_FILE}",
                            f"git mv {config.PROJECT_FILE} {TESTBENCH_FILE}", refused=True)]
        return [Finding(TESTBENCH_FILE, "no such file",
                        "a scensim_testbench package holds testbench.yaml", refused=True)]
    try:
        settings = config.read_yaml(tb)
    except config.ConfigError as e:
        return [Finding(TESTBENCH_FILE, str(e).split(": ", 1)[-1], refused=True)]
    refs = reference_findings(share, package_xml)
    escaping = {f.what.split(": ", 1)[0] for f in refs}
    # world.server (step 5) is checked here; today's world rules know only the stock keys
    world = settings.get("world")
    out = [Finding(TESTBENCH_FILE, f"{where}: {what}")
           for where, what in own_world.server_problems(world)
           + own_world.simulator_problems(settings)]
    if isinstance(world, dict) and own_world.KEY in world:
        settings = dict(settings, world={k: v for k, v in world.items() if k != own_world.KEY})
    out += projectmod.check_schema(settings)
    for v in settings.get("vehicles") or []:
        if isinstance(v, str) and not (share / v).is_file() and v not in escaping:
            out.append(Finding(TESTBENCH_FILE, f"vehicles: {v}: no such file"))
    files, found = projectmod.collect_scenarios(share)
    # a suite's pkg:// item is resolved when the testbench runs (step 3); here it is named by
    # the declaration rule, and a ../ item by the leaving rule, not as a missing file too
    out += [f for f in found if not ("pkg://" in f.what or
                                     any(f"scenario {e}:" in f.what for e in escaping))]
    if not files and not found:
        out.append(Finding("scenarios/", "no scenario", "add one, e.g. scenarios/drive.xosc"))
    out += projectmod.scenario_problems(share, files, settings)
    out += refs
    return [_as_testbench(f) for f in split_findings(out)[0]]


def build_notes(share) -> list:
    """What the build says without failing: only that the testbench has no lock yet. (Assets
    are not checked at build; the docs say so, rather than a line on every build.)"""
    share = Path(share)
    if (share / LOCK_FILE).is_file():
        return []
    pkg = share.name
    return [f"no {LOCK_FILE} yet: run `scensim assets fetch {pkg}` and commit "
            f"src/{pkg}/{LOCK_FILE}"]


def built_over() -> dict:
    """The scensim this testbench was built over: its version, its prefix and its install's
    build stamp (G29) when it has one. ``scensim run`` refuses a testbench built over another
    (241, later), naming ``colcon build``."""
    out: dict = {"scensim": None, "prefix": None, "build": None}
    try:
        from importlib.metadata import version
        out["scensim"] = version("scensim")
    except Exception:  # noqa: BLE001 (not installed as a distribution: a source tree)
        pass
    try:
        from ament_index_python.packages import get_package_prefix
        out["prefix"] = get_package_prefix("scensim")
    except Exception:  # noqa: BLE001 (no ROS environment, or scensim not in the index)
        pass
    try:
        from scensim import results
        stamp = results.installed_build(out["prefix"]) if out["prefix"] else None
        if stamp:
            out["build"] = {"commit": stamp.get("swb") or stamp.get("scensim"),
                            "built": stamp.get("built")}
    except Exception:  # noqa: BLE001
        pass
    return out
