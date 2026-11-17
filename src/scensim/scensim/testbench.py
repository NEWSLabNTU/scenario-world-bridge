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
import re
import xml.etree.ElementTree as ET
from dataclasses import replace
from pathlib import Path

import yaml

from scensim import config
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


def reference_findings(share: Path, package_xml: Path) -> list:
    """081's two rules: ``pkg://`` names a declared package; no path leaves the package."""
    own = package_name(package_xml)
    declared = declared_packages(package_xml) | {own}
    root = share.resolve()
    out = []
    for f, base, doc in _reference_files(share):
        rel = f.relative_to(share).as_posix()
        for s in _strings(doc):
            for m in PKG_REF.finditer(s):
                if m.group(1) not in declared:
                    out.append(Finding(rel, f"{m.group(0)}: package {m.group(1)} is not declared",
                                       f"add <exec_depend>{m.group(1)}</exec_depend> to "
                                       "package.xml"))
            if "pkg://" in s or "://" in s or "$(" in s or not re.search(r"(^|/)\.\.(/|$)", s):
                continue
            # as scensim resolves it (collect_scenarios): from the testbench's root, else from
            # the suite's directory when only that one exists
            target = (share / s).resolve()
            if not target.exists() and (base / s).exists():
                target = (base / s).resolve()
            if target != root and root not in target.parents:
                out.append(Finding(rel, f"{s}: leaves the testbench",
                                   "put the file in a package and name it as "
                                   "pkg://<package>/<path>, declared in package.xml"))
    return out


def _host_state(f: Finding) -> bool:
    """A finding about this host, not the package: an asset or map not fetched here (assets are
    fetched per host and never installed)."""
    return "not fetched" in f.what


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
                            f"{config.PROJECT_FILE} (roadmap 081)",
                            f"git mv {config.PROJECT_FILE} {TESTBENCH_FILE}", refused=True)]
        return [Finding(TESTBENCH_FILE, "no such file",
                        "a scensim_testbench package holds testbench.yaml", refused=True)]
    try:
        settings = config.read_yaml(tb)
    except config.ConfigError as e:
        return [Finding(TESTBENCH_FILE, str(e).split(": ", 1)[-1], refused=True)]
    refs = reference_findings(share, package_xml)
    escaping = {f.what.split(": ", 1)[0] for f in refs}
    out = list(projectmod.check_schema(settings))
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
    return [_as_testbench(f) for f in out if not _host_state(f)]


def build_notes(share) -> list:
    """What the build says without failing: no committed lock (an installed testbench without
    its source cannot write one: ``scensim assets fetch --source <dir>``), and that assets are
    not checked at build."""
    share = Path(share)
    out = ["assets are not checked at build: they are fetched per host "
           "(`scensim assets fetch`); `scensim testbench check` checks them"]
    if not (share / LOCK_FILE).is_file():
        out.insert(0, f"no {LOCK_FILE}: run `scensim assets fetch` in the testbench's source "
                      "(or `--source <dir>`) and commit the lock")
    return out


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
