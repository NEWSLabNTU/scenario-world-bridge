#!/usr/bin/env python3
"""Stage a merged release install into the simulation tarball (roadmap 082 step 2).

    stage_sim.py --root <checkout> --install install --vendor build/release-vendor \
        --repos sim.repos --version V --distro humble --out dist

The tarball unpacks to a prefix root (no top directory):

- colcon's merged install, ELF files stripped;
- build-only parts left out (embree's tutorials and models, static libraries, headers
  and CMake exports), except what a support package builds against: the message packages
  and the world protocol's packages keep theirs;
- ``setup.bash`` / ``local_setup.bash`` with a guard that refuses another ROS release or
  architecture, naming both values;
- ``bin/scensim``: the scensim package's own entry when it installs one, else a plain
  exec into ``lib/scensim/<command>`` (the caller sets up the environment);
- ``scensim-release.yaml``: version, side, ros_distro, arch, autoware, the simulators it
  serves, every repository's commit, each CARLA binary's support tier;
- each package's ``package.xml`` (already under ``share/<pkg>/``, for ``rosdep install
  --from-paths``), and ``LICENSES/<repository>/``.

Prints what it left out and the sizes; writes ``<name>.tar.zst`` and appends its line to
``SHA256SUMS`` in --out.
"""
import argparse
import datetime
import hashlib
import os
import platform
import shutil
import subprocess
import sys
from pathlib import Path

# The world protocol's packages: 081's world servers and pilots build against them.
PROTOCOL_DIR = "src/scensim_world_protocol"
# CARLA versions the CARLA world server ships a binary for, with their support tier.
CARLA_TIERS = [("0.9.16", "tested")]
LICENSE_NAMES = ("LICENSE", "LICENSE.md", "LICENSE.txt", "COPYING", "COPYING.md", "NOTICE")

GUARD = """\
# scensim release guard: this install was built for one ROS release and architecture.
if [ "${{ROS_DISTRO:-}}" != "{distro}" ]; then
  echo "scensim: this install is for ROS {distro} ({arch}); the sourced ROS is '${{ROS_DISTRO:-none}}'" >&2
  echo "scensim: source /opt/ros/{distro}/setup.bash first, or install the release for your ROS (pip install scensim-ros-<distro>)" >&2
  return 1 2>/dev/null || exit 1
fi
if [ "$(uname -m)" != "{arch}" ]; then
  echo "scensim: this install is for {arch}; this machine is $(uname -m)" >&2
  return 1 2>/dev/null || exit 1
fi
"""

# Until the scensim package installs its own bin/scensim (081), a plain exec into
# lib/scensim/<command>. The environment is the caller's: the pip front end sources ROS and
# this prefix's setup.bash before it execs, as any ROS tool needs.
WRAPPER = """\
#!/bin/bash
# scensim <command> [args]: runs this release's lib/scensim/<command>.
prefix=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)
cmd=${1:-}
usage="usage: scensim <command> [args]; commands: $(ls "$prefix/lib/scensim" | tr '\\n' ' ')"
case "$cmd" in
  -h|--help) echo "$usage"; exit 0 ;;
esac
if [ -z "$cmd" ] || [ ! -x "$prefix/lib/scensim/$cmd" ]; then
  echo "$usage" >&2
  exit 2
fi
shift
exec "$prefix/lib/scensim/$cmd" "$@"
"""


def run(cmd, **kw):
    return subprocess.run(cmd, check=True, capture_output=True, text=True, **kw).stdout


def tree_size(path: Path) -> int:
    return sum(f.stat().st_size for f in path.rglob("*") if f.is_file() and not f.is_symlink())


def is_elf(path: Path) -> bool:
    try:
        with open(path, "rb") as f:
            return f.read(4) == b"\x7fELF"
    except OSError:
        return False


def colcon_packages(root: Path, base_paths):
    """{name: source path} of the packages colcon sees."""
    out = run([sys.executable, "-m", "colcon", "list", "--base-paths", *base_paths], cwd=root)
    pkgs = {}
    for line in out.splitlines():
        name, path, _ = line.split("\t")
        pkgs[name] = (root / path).resolve()
    return pkgs


def kept_for_building(prefix: Path, protocol_pkgs) -> set:
    """Packages whose headers and CMake exports stay: messages and the world protocol."""
    index = prefix / "share" / "ament_index" / "resource_index" / "rosidl_interfaces"
    msgs = {p.name for p in index.iterdir()} if index.is_dir() else set()
    return msgs | set(protocol_pkgs)


def prune(prefix: Path, built, keep) -> list:
    """Remove build-only parts; returns [(what, bytes)]."""
    gone = []

    def drop(path: Path, what: str):
        if not path.exists():
            return
        size = tree_size(path) if path.is_dir() else path.stat().st_size
        shutil.rmtree(path) if path.is_dir() else path.unlink()
        gone.append((what, size))

    bin_dir = prefix / "bin"
    if bin_dir.is_dir():
        for p in sorted(bin_dir.iterdir()):
            if p.name.startswith("embree_") or p.name == "models":
                drop(p, f"bin/{p.name} (embree tutorial)")
    # embree_vendor also installs its example sources at the prefix root
    src = prefix / "src"
    if src.is_dir() and (src / "minimal" / "minimal.cpp").is_file():
        drop(src, "src/ (embree example sources)")
    for p in sorted((prefix / "lib").glob("*.a")):
        drop(p, f"lib/{p.name} (static library)")
    include = prefix / "include"
    if include.is_dir():
        for p in sorted(include.iterdir()):
            if p.name not in keep:
                drop(p, f"include/{p.name} (headers)")
    for pkg in sorted(built):
        if pkg not in keep:
            drop(prefix / "share" / pkg / "cmake", f"share/{pkg}/cmake (CMake exports)")
    return gone


def foreign_shebangs(prefix: Path) -> list:
    """Scripts whose interpreter is outside the system's (/usr/bin, /bin).

    setuptools writes the building interpreter into each entry script, so a build run from
    a virtual environment (or with any non-system python3 first on PATH) leaves its path
    there, and every such script fails on another host with "bad interpreter". The release
    refuses them rather than rewriting them: build with the system's Python.
    """
    found = []
    for f in sorted(prefix.rglob("*")):
        if not f.is_file() or f.is_symlink():
            continue
        try:
            with open(f, "rb") as fh:
                head = fh.readline(512)
        except OSError:
            continue
        # a shebang names a path; Rust's inner attributes (#![...]) also start with #!
        if not head.startswith((b"#!/", b"#! /")):
            continue
        interp = head[2:].strip().split(b" ")[0].decode(errors="replace")
        if not interp.startswith(("/usr/bin/", "/bin/")):
            found.append(f"{f.relative_to(prefix)}: {interp}")
    return found


def build_root_refs(prefix: Path, roots) -> dict:
    """{kind: count} of text files that still name a build-host path."""
    needles = [str(r).encode() for r in roots]
    counts = {}
    for f in prefix.rglob("*"):
        # compiled Python records its source path; Python ignores it when the file moves
        if not f.is_file() or f.is_symlink() or is_elf(f) or f.suffix == ".pyc":
            continue
        try:
            data = f.read_bytes()
        except OSError:
            continue
        if any(n in data for n in needles):
            rel = f.relative_to(prefix)
            kind = ("ament index" if "ament_index" in rel.parts else
                    "package setup scripts" if f.suffix in (".sh", ".bash", ".zsh", ".dsv", ".ps1") else
                    f"{rel.parts[0]}/")
            counts[kind] = counts.get(kind, 0) + 1
    return counts


def strip_all(prefix: Path) -> int:
    n = 0
    for f in prefix.rglob("*"):
        if f.is_file() and not f.is_symlink() and is_elf(f) and os.access(f, os.R_OK):
            kind = run(["file", "-b", str(f)])
            if "relocatable" in kind:
                continue
            subprocess.run(["strip", "--strip-unneeded", str(f)], check=False, capture_output=True)
            n += 1
    return n


def commits(root: Path, vendor: Path) -> dict:
    out = {".": run(["git", "-C", str(root), "rev-parse", "HEAD"]).strip()}
    if run(["git", "-C", str(root), "status", "--porcelain", "--untracked-files=no"]).strip():
        out["."] += "+dirty"
    for line in run(["git", "-C", str(root), "submodule", "status", "--recursive"]).splitlines():
        sha, path = line[1:].split()[:2]
        out[path] = sha
    for repo in sorted(p for p in vendor.iterdir() if (p / ".git").exists()):
        out[f"vendor/{repo.name}"] = run(["git", "-C", str(repo), "rev-parse", "HEAD"]).strip()
    return out


def licenses(prefix: Path, pkgs: dict, built) -> int:
    """Copy each built package's repository license into LICENSES/<repository>/."""
    dest = prefix / "LICENSES"
    seen = {}
    for name in sorted(built):
        src = pkgs.get(name)
        if src is None:
            continue
        top = Path(run(["git", "-C", str(src), "rev-parse", "--show-toplevel"]).strip())
        for d in (src, top):
            files = [d / n for n in LICENSE_NAMES if (d / n).is_file()]
            if files:
                break
        key = top.name if d == top else f"{top.name}/{name}"
        if files and key not in seen:
            (dest / key).mkdir(parents=True, exist_ok=True)
            for f in files:
                shutil.copy2(f, dest / key / f.name)
            seen[key] = name
    return len(seen)


def guard(prefix: Path, distro: str, arch: str):
    text = GUARD.format(distro=distro, arch=arch)
    for name in ("setup.bash", "local_setup.bash"):
        f = prefix / name
        body = f.read_text()
        lines = body.splitlines(keepends=True)
        head = lines[0] if lines and lines[0].startswith("#!") else ""
        f.write_text(head + text + "".join(lines[1 if head else 0:]))


def release_yaml(prefix: Path, a, arch: str, commit_map: dict, build_s: int):
    lines = [
        f'version: "{a.version}"',
        "side: sim",
        f"ros_distro: {a.distro}",
        f"arch: {arch}",
        "autoware: null",
        # Isaac's world server is not yet packaged into the prefix (its Python and aib run
        # from the source tree; roadmap 082): this tarball serves CARLA only.
        "simulators: [carla]",
        f'built: "{datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")}"',
        f"build_seconds: {build_s}",
        "carla:",
    ]
    lines += [f'  - {{version: "{v}", tier: {t}}}' for v, t in CARLA_TIERS]
    lines.append("commits:")
    lines += [f'  "{k}": "{v}"' for k, v in commit_map.items()]
    (prefix / "scensim-release.yaml").write_text("\n".join(lines) + "\n")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--root", type=Path, required=True)
    ap.add_argument("--install", default="install")
    ap.add_argument("--vendor", required=True)
    ap.add_argument("--repos", type=Path, required=True)
    ap.add_argument("--version", required=True)
    ap.add_argument("--distro", required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--build-seconds", type=int, default=0)
    a = ap.parse_args(argv)

    root = a.root.resolve()
    arch = platform.machine()
    name = f"scensim-sim-{a.version}-{a.distro}-{arch}"
    stage = root / "build" / "release-stage" / name
    if stage.exists():
        shutil.rmtree(stage)
    stage.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(root / a.install, stage, symlinks=True)
    built_size = tree_size(stage)

    pkgs = colcon_packages(root, ["src", a.vendor])
    index = stage / "share" / "colcon-core" / "packages"
    built = sorted(p.name for p in index.iterdir())
    protocol = [n for n, p in pkgs.items() if str(p).startswith(str(root / PROTOCOL_DIR))]
    keep = kept_for_building(stage, protocol)

    foreign = foreign_shebangs(stage)
    if foreign:
        print(f"stage_sim.py: refused: {len(foreign)} script(s) name an interpreter outside "
              f"/usr/bin (built with a non-system python3 first on PATH?); build with the "
              f"system's Python:", file=sys.stderr)
        for line in foreign[:10]:
            print(f"  {line}", file=sys.stderr)
        return 3
    n_strip = strip_all(stage)
    stripped_size = tree_size(stage)
    gone = prune(stage, built, keep)
    guard(stage, a.distro, arch)
    entry = stage / "bin" / "scensim"
    if not entry.exists():  # the scensim package's own entry wins once it installs one (081)
        entry.parent.mkdir(exist_ok=True)
        entry.write_text(WRAPPER)
        entry.chmod(0o755)
    n_lic = licenses(stage, pkgs, built)
    release_yaml(stage, a, arch, commits(root, root / a.vendor), a.build_seconds)
    missing_xml = [p for p in built if not (stage / "share" / p / "package.xml").is_file()]
    final_size = tree_size(stage)

    a.out.mkdir(parents=True, exist_ok=True)
    tarball = a.out / f"{name}.tar.zst"
    tar = subprocess.Popen(["tar", "-C", str(stage), "-cf", "-", "."], stdout=subprocess.PIPE)
    with open(tarball, "wb") as f:
        subprocess.run(["zstd", "-19", "-T0", "-q", "-c"], stdin=tar.stdout, stdout=f, check=True)
    if tar.wait() != 0:
        print("stage_sim.py: tar failed", file=sys.stderr)
        return 1
    digest = hashlib.sha256(tarball.read_bytes()).hexdigest()
    with open(a.out / "SHA256SUMS", "a") as f:
        f.write(f"{digest}  {tarball.name}\n")

    mb = 1e6
    print(f"packages built: {len(built)}; kept for building against: {len(keep & set(built))}")
    print(f"install as built: {built_size / mb:.1f} MB; stripped ({n_strip} ELF): {stripped_size / mb:.1f} MB")
    print(f"left out: {sum(s for _, s in gone) / mb:.1f} MB in {len(gone)} entries")
    for what, size in sorted(gone, key=lambda g: -g[1])[:8]:
        print(f"  {size / mb:7.1f} MB  {what}")
    print(f"staged: {final_size / mb:.1f} MB; {tarball.name}: {tarball.stat().st_size / mb:.1f} MB")
    print(f"LICENSES/: {n_lic} entries; package.xml missing for: {missing_xml or 'none'}")
    venv = Path(sys.executable).resolve().parent.parent
    refs = build_root_refs(stage, [root] + ([venv] if venv != Path("/usr") else []))
    print(f"text files still naming the build host: {sum(refs.values())} "
          f"({', '.join(f'{k}: {v}' for k, v in sorted(refs.items())) or 'none'})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
