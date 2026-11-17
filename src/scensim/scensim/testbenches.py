"""Testbenches by name (roadmap 081 step 3): finding an installed testbench through the ament
index, the scensim it was built over, its work directory on this host, and the targets
``scensim run`` takes.

A testbench installed by colcon-scensim is ``<prefix>/share/<pkg>/`` and is registered as
``share/ament_index/resource_index/scensim_testbenches/<pkg>``, whose content is the JSON
``scensim.testbench.built_over()`` wrote at build time ({scensim, prefix, build}). The installed
tree is read-only by design; what a host writes for it (fetched assets, ground state, the
stack's local settings) lives in its **work directory**, ``<logs>/.testbenches/<pkg>/``: a
directory whose entries link to the installed testbench's, beside the host's own ``assets/``,
``state/`` and ``.scensim/``. Every command that takes a project works on it unchanged, and
its runs go to ``<logs>/<pkg>/runs`` (``run.runs_dir``). The lock is linked to the testbench's
source (``--symlink-install``) so ``assets fetch`` writes it there, to be committed.

A testbench given as a path (``scensim run ./src/tb_courses``, built or not) runs in place,
as a project does.
"""
import json
import os
import re
from dataclasses import dataclass, field
from pathlib import Path

from scensim import config
from scensim import testbench as tbmod

INDEX = "scensim_testbenches"
WORK = ".testbenches"
# written by this host in the work directory; never linked to the installed testbench
OWN = frozenset({"assets", "state", "runs", ".scensim"})
PACKAGE_NAME = re.compile(r"^[a-z][a-z0-9_]*$")
# a project's own directories: a target starting with one is a path, never a testbench
PROJECT_DIRS = frozenset({"scenarios", "suites", "pilots", "judges", "config", "ground"})


class TestbenchRefused(RuntimeError):
    """A testbench that cannot be used as asked; the message names the fix."""


@dataclass
class Testbench:
    name: str
    share: Path                 # <prefix>/share/<pkg>, or the directory of a path testbench
    prefix: Path = None         # None: a testbench given as a path
    built: dict = field(default_factory=dict)   # the index entry: what it was built over

    @property
    def installed(self) -> bool:
        return self.prefix is not None

    def source(self) -> Path:
        """The testbench's source directory: itself for a path testbench; for an installed one,
        where ``--symlink-install`` links its testbench.yaml; None after a copy install."""
        if not self.installed:
            return self.share
        tb = self.share / tbmod.TESTBENCH_FILE
        if tb.is_symlink():
            return Path(os.path.realpath(tb)).parent
        return None

    def workspace(self) -> Path:
        """The colcon workspace it was built in (``<ws>/install[/<pkg>]``), or its own
        directory for a path testbench."""
        if not self.installed:
            return self.share
        p = self.prefix
        if p.parent.name == "install":
            return p.parent.parent
        if p.name == "install":
            return p.parent
        return p


def not_indexed(name: str) -> str:
    return (f"testbench {name} is not in the sourced workspace: source scensim's setup.bash, "
            "then colcon build (without scensim sourced, colcon skips a testbench with only a "
            "warning and exits 0), then source the workspace's install/setup.bash")


def names() -> list:
    """The testbenches of the sourced chain, by name."""
    try:
        from ament_index_python.resources import get_resources
        return sorted(get_resources(INDEX))
    except Exception:  # noqa: BLE001 (no ROS environment)
        return []


def find(name: str) -> Testbench:
    """An installed testbench by package name; TestbenchRefused naming the build when the
    sourced chain has none of that name."""
    try:
        from ament_index_python.resources import get_resource
        content, prefix = get_resource(INDEX, name)
    except Exception:  # noqa: BLE001 (LookupError: not indexed; ImportError: no ROS)
        raise TestbenchRefused(not_indexed(name)) from None
    try:
        built = json.loads(content) if content.strip() else {}
    except ValueError:
        built = {}
    prefix = Path(prefix)
    return Testbench(name, prefix / "share" / name, prefix, built if isinstance(built, dict) else {})


def at_path(path) -> Testbench:
    """A testbench given as a directory (built or not): it runs in place."""
    d = Path(path).expanduser().resolve()
    old = config.old_project_file(d)
    if old:
        raise TestbenchRefused(old)
    if not (d / tbmod.TESTBENCH_FILE).is_file():
        raise TestbenchRefused(f"{d}: no {tbmod.TESTBENCH_FILE}")
    name = d.name
    if (d / "package.xml").is_file():
        name = tbmod.package_name(d / "package.xml") or name
    return Testbench(name, d)


def _short(built: dict) -> str:
    b = built.get("build") or {}
    commit = (b.get("commit") or "")[:12]
    return (f"scensim {built.get('scensim') or '?'}"
            + (f" at {built.get('prefix')}" if built.get("prefix") else "")
            + (f" ({commit})" if commit else ""))


def version_problem(tb: Testbench, now: dict = None) -> str:
    """Why the installed testbench was built over another scensim than this one, or "". A
    testbench built before its index entry held anything is "another" too."""
    if not tb.installed:
        return ""
    now = now if now is not None else tbmod.built_over()
    was = tb.built or {}
    keys = [k for k in ("scensim", "prefix") if now.get(k) or was.get(k)]
    differ = [k for k in keys if (was.get(k) or None) != (now.get(k) or None)]
    a, b = (was.get("build") or {}).get("commit"), (now.get("build") or {}).get("commit")
    if a and b and a != b:
        differ.append("build")
    if not was:
        differ = differ or ["entry"]
    if not differ:
        return ""
    return (f"testbench {tb.name} was built over {_short(was)}, but this is {_short(now)}: "
            f"rebuild the workspace (cd {tb.workspace()} && colcon build)")


def work_dir(tb: Testbench, host: dict) -> Path:
    """The directory commands run an installed testbench in (made or refreshed here): a link
    per entry of the installed testbench, and this host's own assets/, state/ and .scensim/.
    The lock links to the source's when the source is known (assets fetch writes it there);
    else to the installed copy (read only)."""
    if not tb.installed:
        return tb.share
    d = Path(host["logs"]).expanduser() / WORK / tb.name
    d.mkdir(parents=True, exist_ok=True)
    want = {}
    for e in sorted(tb.share.iterdir()):
        if e.name in OWN or e.name == tbmod.LOCK_FILE:
            continue
        want[e.name] = e
    src = tb.source()
    want[tbmod.LOCK_FILE] = (src if src is not None else tb.share) / tbmod.LOCK_FILE
    for e in list(d.iterdir()):
        if e.is_symlink() and (e.name not in want or os.readlink(e) != str(want[e.name])):
            e.unlink()
    for name, target in want.items():
        link = d / name
        if not link.is_symlink():
            if link.exists():
                raise TestbenchRefused(f"{link}: not a link this command made; remove it")
            os.symlink(target, link)
    return d


def suites(tb_dir: Path) -> list:
    return sorted(p.stem for p in (Path(tb_dir) / "suites").glob("*.y*ml"))


def default_target(tb: Testbench, tb_dir: Path) -> Path:
    """``testbench.yaml``'s ``default:`` suite; refused, listing the suites, without one."""
    settings = config.project_settings(tb_dir)
    default = settings.get("default")
    have = suites(tb_dir)
    if default:
        p = Path(tb_dir) / "suites" / f"{default}.yaml"
        if not p.is_file():
            raise TestbenchRefused(f"testbench {tb.name}: default: {default} is no suite "
                                   f"(suites: {', '.join(have) or 'none'})")
        return p
    if len(have) == 1:
        return Path(tb_dir) / "suites" / f"{have[0]}.yaml"
    if have:
        raise TestbenchRefused(f"testbench {tb.name} has no default: suite; name one: "
                               + ", ".join(f"{tb.name}/{s}" for s in have))
    raise TestbenchRefused(f"testbench {tb.name} has no suite; name a scenario: "
                           f"{tb.name}/scenarios/<file>")


def in_testbench(tb: Testbench, tb_dir: Path, rest: str) -> Path:
    """What ``<tb>/<rest>`` names: a suite first (``suites/<rest>.yaml``), then a path."""
    if not rest:
        return default_target(tb, tb_dir)
    suite = Path(tb_dir) / "suites" / f"{rest}.yaml"
    if "/" not in rest and suite.is_file():
        return suite
    p = Path(tb_dir) / rest
    if p.exists():
        return p
    raise TestbenchRefused(f"{tb.name}/{rest}: no suite {rest} and no such file in the "
                           f"testbench (suites: {', '.join(suites(tb_dir)) or 'none'})")


def split(target: str):
    """``(testbench name, rest)`` when ``target`` names an indexed testbench and no local
    path; else None (a path, run as before)."""
    t = str(target)
    if os.path.exists(t) or t.startswith(("/", "./", "../", "~")):
        return None
    name, _, rest = t.partition("/")
    if not PACKAGE_NAME.match(name) or name in PROJECT_DIRS:
        return None
    if name not in names():
        raise TestbenchRefused(not_indexed(name))
    return name, rest.strip("/")


def project_for(name: str, logs=None, now: dict = None) -> Path:
    """The directory a stack of testbench ``name`` runs in (its work directory): what
    ``testbench:=<name>`` on scensim.launch.xml and ``scensim stack <name>`` resolve, so no one
    types it. TestbenchRefused for a name not built in the sourced chain, or built over
    another scensim."""
    tb = find(name)
    problem = version_problem(tb, now)
    if problem:
        raise TestbenchRefused(problem)
    return work_dir(tb, config.host_settings(None, {"logs": logs} if logs else {}))
