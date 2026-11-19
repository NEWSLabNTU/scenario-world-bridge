#!/usr/bin/env python3
"""Move one of our repositories to a landed commit, and every pin that follows from it, in the order that
keeps scripts/check_pins.py green on main (scripts/land_pin.sh runs this):

    scripts/land_pin.sh [--dry-run] <repo> <sha> [note]

1. <sha> must already be on <repo>'s origin/main (push and land it there first).
2. The cascade: every component whose deps/ pins a moving repository moves too (a new commit on its main
   with those deps bumped), transitively -- sim_vehicle_spec moves autoware_sim_vehicle, which moves
   autoware_isaac_bridge, which moves scensim_isaac_world. Each component is bumped once, after its own
   moving deps, with all of them in one commit.
3. Every commit is made locally first: the components' (from their origin/main), then this repository's
   pins, one commit for the whole cascade. check_pins runs on that. Only then is anything pushed: the
   components in dependency order, then this repository. A failed check pushes nothing; a component
   whose main moved meanwhile stops the push before this repository's pins, so main never names a
   partial cascade.

Plain fast-forward pushes of rebased mains, never forced. --dry-run prints the cascade and touches
nothing. LAND_PIN_TRAILER, when set, is appended to each commit message (the session's attribution).
"""

import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def git(*a, cwd=ROOT, check=True, capture=True):
    r = subprocess.run(["git", "-C", str(cwd), *a], text=True,
                       stdout=subprocess.PIPE if capture else None, stderr=subprocess.PIPE if capture else None)
    if check and r.returncode:
        raise SystemExit(f"git -C {cwd} {' '.join(a)}: {(r.stderr or '').strip()}")
    return (r.stdout or "").strip()


def ok(*a, cwd=ROOT):
    return subprocess.run(["git", "-C", str(cwd), *a], stdout=subprocess.DEVNULL,
                          stderr=subprocess.DEVNULL).returncode == 0


def pin(repo_dir, rev, path):
    """The gitlink at `path` in `rev` of the repository at repo_dir, or None."""
    out = git("ls-tree", rev, path, cwd=repo_dir, check=False)
    return out.split()[2] if out else None


def main(argv):
    dry = bool(argv) and argv[0] == "--dry-run"
    if dry:
        argv = argv[1:]
    if len(argv) < 2:
        print(__doc__, file=sys.stderr)
        return 2
    repo, sha, note = argv[0], argv[1], argv[2] if len(argv) > 2 else ""
    trailer = "\n\n" + os.environ["LAND_PIN_TRAILER"] if os.environ.get("LAND_PIN_TRAILER") else ""
    src = ROOT / "src"
    comps = sorted(p.name for p in src.iterdir() if (p / ".git").exists())
    if repo not in comps:
        print(f"no submodule src/{repo}", file=sys.stderr)
        return 2

    git("fetch", "-q", "origin")
    if git("status", "--porcelain", "--untracked-files=no", "--ignore-submodules=all"):
        print(f"{ROOT} has local changes", file=sys.stderr)
        return 2
    base = "origin/main"  # this repository's pins are read from, and committed onto, its origin/main

    top = src / repo
    git("fetch", "-q", "origin", cwd=top)
    full = git("rev-parse", "-q", "--verify", f"{sha}^{{commit}}", cwd=top, check=False)
    if not full:
        print(f"{repo} has no commit {sha} (pushed?)", file=sys.stderr)
        return 2
    if not ok("merge-base", "--is-ancestor", full, "origin/main", cwd=top):
        print(f"{repo} {sha} is not on its origin/main: land it there first", file=sys.stderr)
        return 2

    # each component's deps (repos it pins under deps/), read from the component's origin/main
    deps = {}
    for c in comps:
        d = src / c / "deps"
        if not d.is_dir():
            continue
        git("fetch", "-q", "origin", cwd=src / c)
        deps[c] = {n for n in comps if pin(src / c, "origin/main", f"deps/{n}")}

    target = {repo: full}
    # the cascade: a component moves when it pins a moving repo at anything but that repo's target
    moving = [repo]
    order = []
    changed = True
    while changed:
        changed = False
        for c in sorted(deps):
            if c in moving or c in order:
                continue
            need = [d for d in deps[c] if d in moving and (d != repo or pin(src / c, "origin/main", f"deps/{d}") != full)]
            if need:
                order.append(c)
                moving.append(c)
                changed = True
    # dependency order among the moving components
    ordered = []
    while len(ordered) < len(order):
        for c in order:
            if c not in ordered and all(d not in order or d in ordered for d in deps[c]):
                ordered.append(c)
    short = lambda s: s[:7] if s else "(new)"  # noqa: E731
    print(f"{repo} -> {short(full)} ({git('log', '-1', '--format=%s', full, cwd=top)})")
    for c in ordered:
        mv = sorted(d for d in deps[c] if d in moving)
        print(f"== {c}: " + ", ".join(f"deps/{d} {short(pin(src / c, 'origin/main', f'deps/{d}'))} -> "
                                      f"{short(target.get(d))}" for d in mv))
        target[c] = None if dry else ""
        if dry:
            continue
        cd = src / c
        git("checkout", "-q", "-B", "main", "origin/main", cwd=cd)
        for d in mv:
            git("submodule", "update", "-q", "--init", f"deps/{d}", cwd=cd)
            if d == repo:
                git("fetch", "-q", "origin", cwd=cd / "deps" / d)
            else:  # a component bumped above: its commit is still only local, so fetch it from there
                git("fetch", "-q", str(src / d), target[d], cwd=cd / "deps" / d)
            git("checkout", "-q", target[d], cwd=cd / "deps" / d)
            git("add", f"deps/{d}", cwd=cd)
        names = ", ".join(f"{d} {short(target[d])}" for d in mv)
        git("commit", "-q", "-m", f"deps: {names}{trailer}", cwd=cd)
        target[c] = git("rev-parse", "HEAD", cwd=cd)
    for c in [repo] + ordered:
        cur = pin(ROOT, base, f"src/{c}")
        if cur != target[c]:
            print(f"== src/{c} {short(cur)} -> {short(target[c])}")
    if dry:
        print("dry run: nothing changed")
        return 0

    git("checkout", "-q", "-B", "land-pin", base)
    for c in [repo] + ordered:
        git("checkout", "-q", target[c], cwd=src / c)
        git("add", f"src/{c}")
    if not git("diff", "--cached", "--name-only"):
        subprocess.run([sys.executable, "scripts/check_pins.py"], cwd=ROOT, check=True)
        print(f"pins already at {short(full)}")
        return 0
    names = ", ".join(f"{c} {short(target[c])}" for c in [repo] + ordered)
    git("commit", "-q", "-m", f"pins: {names}{' (' + note + ')' if note else ''}{trailer}")
    if subprocess.run([sys.executable, "scripts/check_pins.py"], cwd=ROOT).returncode:
        print("check_pins failed: nothing pushed (the local commits stay for a look)", file=sys.stderr)
        return 1
    for c in ordered:  # dependency order: a component's deps are on their remotes before it is
        r = subprocess.run(["git", "-C", str(src / c), "push", "-q", "origin", "main"])
        if r.returncode:
            print(f"push of {c} failed (its main moved?): stopped before this repository's pins; pushed so "
                  f"far: {ordered[:ordered.index(c)] or 'none'}. Rerun to cascade again.", file=sys.stderr)
            return 3
    git("push", "-q", "origin", "HEAD:main", capture=False)
    print(git("log", "--oneline", "-1"))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
