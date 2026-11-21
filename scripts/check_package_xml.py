#!/usr/bin/env python3
"""Every package.xml colcon would read parses as XML, and colcon sees its package with the
build type it declares.

colcon does not fail on a package.xml it cannot parse: it takes the package for whatever else
it finds there (a setup.py: a plain Python package), builds it, and leaves it out of the ament
index, so `ros2 run <pkg>` says "Package not found" and nothing in a build says why. Seen
2026-11-21: a comment holding `--` (not allowed in an XML comment) in scensim's package.xml,
from 2b0f78a7 to ad246294 on scensim main; the build and `just test` passed throughout.
Checks src/, skipping the deps/ copies (COLCON_IGNORE) and the build trees. With colcon on
PATH, `colcon list --base-paths src` must list each such package as ros.<its build_type>: a
package it demoted (to `python`, say) fails.
"""
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def ignored(p: Path) -> bool:
    for parent in p.parents:
        if parent == ROOT:
            return False
        if (parent / "COLCON_IGNORE").exists() or parent.name in ("build", "install", "log"):
            return True
    return False


def colcon_types(declared):
    """What colcon makes of each package that declares a build type, against what it declares."""
    if shutil.which("colcon") is None:
        print("package.xml: colcon not on PATH, build types not checked", file=sys.stderr)
        return []
    r = subprocess.run(["colcon", "list", "--base-paths", "src"], cwd=ROOT, text=True,
                       capture_output=True, timeout=300)
    if r.returncode != 0:
        return [f"colcon list failed: {r.stderr.strip()[-300:]}"]
    seen = {}
    for line in r.stdout.splitlines():
        parts = line.split("\t")
        if len(parts) == 3:
            seen[parts[1]] = parts[2].strip("()")
    out = []
    for path, (name, build_type) in sorted(declared.items()):
        if not build_type:
            continue
        got = seen.get(path)
        if got is None:
            out.append(f"{path}: {name} declares {build_type}, and colcon does not list it")
        elif got != f"ros.{build_type}":
            out.append(f"{path}: {name} declares {build_type}, colcon takes it for {got} (a "
                       f"package.xml colcon could not read; it leaves the ament index)")
    return out


def main() -> int:
    bad, n, declared = [], 0, {}
    for p in sorted((ROOT / "src").rglob("package.xml")):
        if ignored(p):
            continue
        n += 1
        try:
            root = ET.parse(p).getroot()
        except ET.ParseError as e:
            bad.append(f"{p.relative_to(ROOT)}: {e}")
            continue
        if root.tag != "package" or root.findtext("name") is None:
            bad.append(f"{p.relative_to(ROOT)}: no <package> with a <name>")
            continue
        declared[p.parent.relative_to(ROOT).as_posix()] = (
            root.findtext("name"), root.findtext("export/build_type"))
    bad += colcon_types(declared)
    for b in bad:
        print(b)
    print(f"package.xml: {n} checked, {len(bad)} not readable", file=sys.stderr)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
