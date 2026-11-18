"""The Autoware side of an autoware pilot: the car's Autoware packages in the user's workspace.

A release of scensim ships the simulation side only. The car's Autoware packages (its vehicle
launch, its sensor kit's launch, ``individual_params``) are source, imported with the release's
``autoware-side.repos`` and built in the user's own Autoware workspace
(docs/how-to/connect-autoware.md). Each declares the Autoware releases it supports, and those
it was tested on, in its package.xml::

    <export>
      <scensim_autoware supported="1.5.0..2.1.0" tested="1.5.0/humble 2.0.0/humble 2.1.0/humble"/>
    </export>

``problems`` refuses an autoware pilot whose packages are missing, or whose sourced Autoware is
outside a package's supported range, naming the package, the release and the fix. ``notes``
warns, without refusing, when the sourced Autoware and ROS are supported but untested, or when
the release cannot be told.
"""
import os
import re
import xml.etree.ElementTree as ET
from pathlib import Path

GUIDE = "docs/how-to/connect-autoware.md"
IMPORT_FIX = ("import and build the car's Autoware packages in your Autoware workspace: "
              "`vcs import src < autoware-side.repos`, then `colcon build --packages-up-to "
              "{pkgs}` with your Autoware sourced, and source the workspace (" + GUIDE + ")")

# autoware_launch's version in each Autoware release; 2.0.0 and 2.1.0 share 0.53.0, so a
# workspace's own build of either is told apart only by its install path
LAUNCH_TO_RELEASES = {
    "0.48.0": ("1.5.0",),
    "0.50.0": ("1.7.1",),
    "0.52.0": ("1.9.0",),
    "0.53.0": ("2.0.0", "2.1.0"),
}
IN_PATH = re.compile(r"/autoware/(\d+\.\d+\.\d+)(?=/|$)")


def vtuple(v: str) -> tuple:
    return tuple(int(x) for x in v.split("."))


def required(pilot: dict) -> list:
    """The Autoware-side packages an autoware pilot's launch looks up: the car's vehicle launch
    and its sensor kit's launch (the descriptions come with scensim), and with
    ``autoware_profile: scensim`` autoware_sim_profile."""
    args = pilot.get("args") if isinstance(pilot.get("args"), dict) else {}
    car = pilot.get("vehicle") or "tesla_model3"
    kit = args.get("sensor_model") or f"{car}_sensor_kit"
    out = [f"{car}_launch", f"{kit}_launch"]
    if pilot.get("autoware_profile") == "scensim":
        out.append("autoware_sim_profile")   # the optional import
    return out


def _share(pkg: str):
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory(pkg))
    except Exception:  # noqa: BLE001 (not found, or no ROS environment)
        return None


def _version(package_xml: Path):
    try:
        return ET.parse(package_xml).getroot().findtext("version")
    except (OSError, ET.ParseError):
        return None


def sourced_releases(env=None, share=_share) -> tuple:
    """``(releases, how)``: the Autoware releases the terminal's Autoware can be, and how that
    was read. An install path naming a version (/opt/autoware/2.1.0) gives one; otherwise
    autoware_launch's version gives one or two (2.0.0 and 2.1.0 share it); none when no
    Autoware is found or its version is unknown."""
    env = os.environ if env is None else env
    for var in ("AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH"):
        for prefix in env.get(var, "").split(":"):
            m = IN_PATH.search(prefix)
            if m:
                return (m.group(1),), f"its install path ({prefix})"
    d = share("autoware_launch")
    if d is None:
        return (), "no autoware_launch found: is your Autoware sourced?"
    v = _version(d / "package.xml")
    rel = LAUNCH_TO_RELEASES.get(v or "")
    if not rel:
        return (), f"autoware_launch {v} is not a release scensim knows"
    return rel, f"autoware_launch {v}"


def declared(pkg_share: Path) -> dict:
    """The package's ``<scensim_autoware>`` export: ``{"supported": (lo, hi), "tested":
    {(release, distro), ...}}``, or {} when it declares none."""
    try:
        root = ET.parse(pkg_share / "package.xml").getroot()
    except (OSError, ET.ParseError):
        return {}
    el = root.find("export/scensim_autoware")
    if el is None:
        return {}
    lo, _, hi = (el.get("supported") or "").partition("..")
    out = {"supported": (lo.strip(), (hi or lo).strip()) if lo.strip() else None,
           "tested": set()}
    for t in (el.get("tested") or "").split():
        rel, _, distro = t.partition("/")
        out["tested"].add((rel, distro))
    return out


def in_range(release: str, supported) -> bool:
    lo, hi = supported
    return vtuple(lo) <= vtuple(release) <= vtuple(hi)


def assess(pilots: dict, env=None, share=_share) -> tuple:
    """``(problems, notes)`` for every autoware pilot: each problem is ``(where, what, fix)``,
    each note a sentence. Problems refuse the run; notes are printed and the run goes on."""
    env = os.environ if env is None else env
    distro = env.get("ROS_DISTRO", "")
    releases, how = sourced_releases(env, share)
    problems, notes, seen = [], [], set()
    for key, p in (pilots or {}).items():
        if not isinstance(p, dict) or p.get("kind") != "autoware":
            continue
        where = f"pilots.{key}"
        pkgs = required(p)
        shares = {n: share(n) for n in pkgs}
        missing = [n for n, d in shares.items() if d is None]
        if missing:
            problems.append((where, f"the car's Autoware package {' and '.join(missing)} not "
                             "found in the sourced workspaces",
                             IMPORT_FIX.format(pkgs=" ".join(pkgs))))
            continue
        if not releases:
            if ("release", how) not in seen:
                seen.add(("release", how))
                notes.append(f"which Autoware release is sourced cannot be told ({how}); the "
                             "car's Autoware packages are not checked against it")
            continue
        for n, d in shares.items():
            dec = declared(d)
            if not dec or not dec["supported"]:
                if (n, "none") not in seen:
                    seen.add((n, "none"))
                    notes.append(f"{n} declares no supported Autoware releases ({d}/package."
                                 "xml); it is not checked against your Autoware")
                continue
            lo, hi = dec["supported"]
            if not any(in_range(r, dec["supported"]) for r in releases):
                problems.append((where, f"{n} supports Autoware {lo} to {hi}, but the sourced "
                                 f"Autoware is {' or '.join(releases)} (from {how})",
                                 f"source an Autoware from {lo} to {hi}, or import the "
                                 f"Autoware-side packages of a scensim release that supports "
                                 f"{releases[0]} ({GUIDE})"))
                continue
            ok = [r for r in releases if in_range(r, dec["supported"])]
            if not any((r, distro) in dec["tested"] for r in ok) and (n, "untested") not in seen:
                seen.add((n, "untested"))
                notes.append(f"{n} supports Autoware {' or '.join(ok)} on ROS 2 {distro or '?'}"
                             f" but has not been tested on it (tested: "
                             f"{', '.join(sorted(r + ' on ' + t for r, t in dec['tested'])) or 'none'})")
    return problems, notes
