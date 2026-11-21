#!/usr/bin/env python3
"""Move one of our repositories to a landed commit, and every pin that follows from it, in the order that
keeps scripts/check_pins.py green on main (scripts/land_pin.sh runs this):

    scripts/land_pin.sh [--dry-run] [--onto <branch> [--gated <sha>]] <repo> <sha> [<repo> <sha> ...] [-- note]
    scripts/land_pin.sh [--dry-run] <repo> <sha> [note]

1. Each <sha> must already be on its <repo>'s origin/main (push and land it there first). Several targets land
   as one cascade and one commit here (issue 083: check_pins wants every deps/ pin equal to this repository's,
   so a component whose main moved two repositories needs both moved here together).
2. The cascade: every component whose deps/ pins a moving repository moves too, transitively --
   sim_vehicle_spec moves autoware_sim_vehicle, which moves autoware_isaac_bridge, which moves
   scensim_isaac_world. In dependency order, each gets a new commit on its main with its moving deps bumped,
   all in one commit, when its main pins any of them elsewhere than its target; otherwise (its main already
   pins every one at its target, issue 083) it moves here to its main as it is, with no commit of its own.
   A target is treated the same, from its given commit: when that commit pins a repository the cascade
   moves (aib pinning autoware_sim_vehicle while sim_vehicle_spec lands), it gets a new commit on top
   bumping it. That commit goes on the target's main, so the given commit must be its main's tip; one
   behind it is refused by name rather than landing the commits after it (issue 085).
3. This repository's checkout must hold nothing that is not on its origin/main: the pins are committed onto
   origin/main, so a local commit would be left out. It is refused, in a dry run too, naming the commits.
4. Every commit is made locally first: the components' (from their origin/main), then this repository's
   pins, one commit for the whole cascade. check_pins runs on that. Only then is anything pushed: the
   components in dependency order, then this repository. A failed check pushes nothing; a component
   whose main moved meanwhile stops the push before this repository's pins, so main never names a
   partial cascade.

5. --onto <branch>: this repository's pins go on origin/<branch> instead of origin/main, so a change here that
   needs them (code using a component's new interface) lands with them, as one tip. The branch must be pushed
   and fast-forward origin/main; the checkout may hold its commits. Since that lands code, not only pins, the
   tip must have passed the full gate: --gated <sha> names a commit the gate passed on whose tree is the
   tip's (the pin commit is made here, so the tip's own sha is new; its content is what was gated; refused
   when the cascade makes new component commits, whose shas are new each run), or else
   the command in LAND_PIN_GATE (a shell line, run in this checkout at the tip, with the components' new
   commits in place) runs, and must exit 0. Without either it is refused. Nothing is pushed before that.
6. Guards (issue 096): the checkout's HEAD, the landing components' commits and a clean tree are
   recorded before LAND_PIN_GATE runs and compared after it; any change (the checkout switched or
   edited while the gate ran) refuses the push, since the gate then tested something else. Just
   before pushing, origin/main here and each new component's origin/main are fetched again; if
   any moved past what the landing built on, nothing is pushed (exit 3), so a moved main never
   leaves components pushed and this repository's pins refused. The commits pushed are the
   ones built, by SHA: if HEAD here or a component's main is anything else at push time,
   nothing is pushed.

Plain fast-forward pushes of rebased mains, never forced. --dry-run prints the cascade and touches
nothing. LAND_PIN_TRAILER, when set, is appended to each commit message (the session's attribution).
"""

import os
import re
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


# What a build adds to a Cargo.lock on this workspace's hosts and nothing else: colcon-cargo
# patches the ROS crates in, and cargo records the ones a crate does not use (issue 096)
_BUILD_ONLY_LOCK = re.compile(r'(\[\[patch\.unused\]\]|name = "[^"]*"|version = "[^"]*"|)')


def _build_only(cwd, line) -> bool:
    """Whether a dirty `git status` line is a build's own: a Cargo.lock whose only change is
    added `[[patch.unused]]` entries."""
    path = line[3:]
    if line[:2] != " M" or Path(path).name != "Cargo.lock":
        return False
    diff = git("diff", "-U0", "--no-color", "--", path, cwd=cwd, check=False).splitlines()
    body = [d for d in diff if d[:1] in "+-" and not d.startswith(("+++", "---"))]
    return bool(body) and all(d.startswith("+") and _BUILD_ONLY_LOCK.fullmatch(d[1:].strip())
                              for d in body)


def _dirty(cwd, label="") -> list:
    # raw, not git(): its output is stripped, and a porcelain line's first column is a space
    out = subprocess.run(["git", "status", "--porcelain", "--untracked-files=no",
                          "--ignore-submodules=all"], cwd=cwd, capture_output=True,
                         text=True).stdout
    return [f"{label}{line}" for line in out.splitlines() if line and not _build_only(cwd, line)]


def checkout_state(src, components) -> dict:
    """This checkout's HEAD, each landing component's checked-out commit, and what is uncommitted
    in tracked files (here or in those components), less what a build itself writes (a
    Cargo.lock's unused patches): compared before and after a gate."""
    heads = {".": git("rev-parse", "HEAD")}
    for c in components:
        heads[f"src/{c}"] = git("rev-parse", "HEAD", cwd=src / c, check=False)
    dirty = [d for c in components for d in _dirty(src / c, f"src/{c}: ")]
    dirty += _dirty(ROOT)
    return {"heads": heads, "dirty": dirty}


def options(argv):
    """(argv without --onto/--gated, onto, gated); None when an option lacks its value or --gated comes
    without --onto."""
    argv, onto, gated = list(argv), None, None
    for flag in ("--onto", "--gated"):
        if flag in argv:
            i = argv.index(flag)
            if i + 1 >= len(argv) or argv[i + 1].startswith("-"):
                return None
            value = argv[i + 1]
            del argv[i:i + 2]
            if flag == "--onto":
                onto = value
            else:
                gated = value
    if gated and not onto:
        return None
    return argv, onto, gated


def parse(argv):
    """(dry, [(repo, sha), ...], note) of the arguments; None when they make no landing."""
    dry = bool(argv) and argv[0] == "--dry-run"
    if dry:
        argv = argv[1:]
    note = ""
    if "--" in argv:
        i = argv.index("--")
        argv, note = argv[:i], " ".join(argv[i + 1:])
    elif len(argv) == 3:  # the one-target form: <repo> <sha> [note]
        argv, note = argv[:2], argv[2]
    if len(argv) < 2 or len(argv) % 2:
        return None
    pairs = list(zip(argv[0::2], argv[1::2]))
    if len({r for r, _ in pairs}) != len(pairs):
        return None
    return dry, pairs, note


def main(argv):
    opts = options(argv)
    parsed = opts and parse(opts[0])
    if not parsed:
        print(__doc__, file=sys.stderr)
        return 2
    _, onto, gated = opts
    dry, pairs, note = parsed
    trailer = "\n\n" + os.environ["LAND_PIN_TRAILER"] if os.environ.get("LAND_PIN_TRAILER") else ""
    src = ROOT / "src"
    comps = sorted(p.name for p in src.iterdir() if (p / ".git").exists())
    for repo, _ in pairs:
        if repo not in comps:
            print(f"no submodule src/{repo}", file=sys.stderr)
            return 2

    git("fetch", "-q", "origin")
    if _dirty(ROOT):  # a build's own Cargo.lock patches aside
        print(f"{ROOT} has local changes", file=sys.stderr)
        return 2
    base = "origin/main"  # this repository's pins are read from, and committed onto, its origin/main
    if onto:  # or onto a pushed branch carrying code here that needs them (5)
        base = f"origin/{onto}"
        if not ok("rev-parse", "-q", "--verify", f"{base}^{{commit}}"):
            print(f"no branch {onto} on origin (push it first)", file=sys.stderr)
            return 2
        if not ok("merge-base", "--is-ancestor", "origin/main", base):
            print(f"{onto} does not fast-forward origin/main: rebase it onto origin/main and push it under a "
                  "new name", file=sys.stderr)
            return 2
        if not gated and not os.environ.get("LAND_PIN_GATE"):
            print(f"--onto lands code: give --gated <sha> (a commit the full gate passed on, with the tip's "
                  "tree) or set LAND_PIN_GATE to the gate's command", file=sys.stderr)
            return 2
    # A commit here that is not on origin/main would be left out of the landing, silently (the pins go on
    # origin/main): refuse it, in a dry run too, naming it
    ahead = git("log", "--format=%h %s", f"{base}..HEAD")
    if ahead:
        print(f"{ROOT}'s checkout has commits that are not on {base}, which this landing would leave out:\n"
              + "\n".join(f"  {line}" for line in ahead.splitlines())
              + f"\npush or land these first, or run from a checkout at {base}", file=sys.stderr)
        return 2

    targets = {}
    for repo, sha in pairs:
        top = src / repo
        git("fetch", "-q", "origin", cwd=top)
        full = git("rev-parse", "-q", "--verify", f"{sha}^{{commit}}", cwd=top, check=False)
        if not full:
            print(f"{repo} has no commit {sha} (pushed?)", file=sys.stderr)
            return 2
        if not ok("merge-base", "--is-ancestor", full, "origin/main", cwd=top):
            print(f"{repo} {sha} is not on its origin/main: land it there first", file=sys.stderr)
            return 2
        targets[repo] = full

    # each component's deps (repos it pins under deps/), read from the component's origin/main
    deps = {}
    for c in comps:
        d = src / c / "deps"
        if not d.is_dir():
            continue
        git("fetch", "-q", "origin", cwd=src / c)
        deps[c] = {n for n in comps if pin(src / c, "origin/main", f"deps/{n}")}

    # the moving set: the targets and, transitively, every component that pins a moving repo
    moving = set(targets)
    changed = True
    while changed:
        changed = False
        for c in sorted(deps):
            if c not in moving and deps[c] & moving:
                moving.add(c)
                changed = True
    order = []  # the moving components, the targets among them, in dependency order
    rest = sorted(moving)
    while rest:
        ready = [c for c in rest if all(d not in rest for d in deps.get(c, ()))]
        if not ready:
            print(f"a dependency cycle among {rest}", file=sys.stderr)
            return 2
        order += ready
        rest = [c for c in rest if c not in ready]
    # each: a new commit bumping its moving deps when its main (a target: the given commit) pins any of them
    # elsewhere than its target (a dep that gets a new commit itself counts as elsewhere), else that commit
    # as it is (issue 083). A target bumped so must be its main's tip: the new commit goes on its main, and
    # anything after the given commit would land with it (issue 085).
    target, new = dict(targets), []
    for c in order:
        mv = sorted(deps.get(c, set()) & moving - {c})
        at = targets.get(c) or "origin/main"
        if any(d in new or pin(src / c, at, f"deps/{d}") != target[d] for d in mv):
            if c in targets and targets[c] != git("rev-parse", "origin/main", cwd=src / c):
                print(f"{c} {targets[c][:7]} pins " + ", ".join(
                    d for d in mv if d in new or pin(src / c, at, f"deps/{d}") != target[d])
                      + " elsewhere than this landing moves it, and is not its main's tip: land its main, "
                      "or move those pins in it first", file=sys.stderr)
                return 2
            new.append(c)
            target[c] = None
        elif c not in targets:
            target[c] = git("rev-parse", "origin/main", cwd=src / c)
    if gated and new:  # a new component commit gets a new sha each run: no earlier gate saw this tree
        print("--gated cannot name this landing's content: the cascade makes new commits in "
              + ", ".join(new) + ", whose shas no earlier gate saw; run the gate in this landing "
              "(LAND_PIN_GATE), or land those components first", file=sys.stderr)
        return 2
    short = lambda s: s[:7] if s else "(new)"  # noqa: E731
    for repo, full in targets.items():
        print(f"{repo} -> {short(full)} ({git('log', '-1', '--format=%s', full, cwd=src / repo)})")
    for c in order:
        if c not in new:
            if c not in targets:
                print(f"== {c}: its main {short(target[c])} already pins its moving deps")
            continue
        mv = sorted(deps[c] & moving - {c})
        print(f"== {c}: " + ", ".join(f"deps/{d} {short(pin(src / c, 'origin/main', f'deps/{d}'))} -> "
                                      f"{short(target.get(d))}" for d in mv))
        if dry:
            continue
        cd = src / c
        git("checkout", "-q", "-B", "main", "origin/main", cwd=cd)
        for d in mv:
            git("submodule", "update", "-q", "--init", f"deps/{d}", cwd=cd)
            if d in new:  # a component bumped above: its commit is still only local, so fetch it from there
                # without recursing: its own deps/ may name commits that are only local too, in other checkouts
                git("fetch", "-q", "--no-recurse-submodules", str(src / d), target[d], cwd=cd / "deps" / d)
            else:  # a target, or a component at its main: on its remote
                git("fetch", "-q", "--no-recurse-submodules", "origin", cwd=cd / "deps" / d)
            git("checkout", "-q", target[d], cwd=cd / "deps" / d)
            git("add", f"deps/{d}", cwd=cd)
        names = ", ".join(f"{d} {short(target[d])}" for d in mv)
        git("commit", "-q", "-m", f"deps: {names}{trailer}", cwd=cd)
        target[c] = git("rev-parse", "HEAD", cwd=cd)
    landing = order
    for c in landing:
        cur = pin(ROOT, base, f"src/{c}")
        if cur != target[c]:
            print(f"== src/{c} {short(cur)} -> {short(target[c])}")
    if onto:
        print(f"== onto {onto} ({git('log', '-1', '--format=%h %s', base)}); gate: "
              + (f"--gated {gated}" if gated else f"LAND_PIN_GATE ({os.environ['LAND_PIN_GATE']})"))
    if dry:
        print("dry run: nothing changed")
        return 0

    git("checkout", "-q", "-B", "land-pin", base)
    for c in landing:
        git("checkout", "-q", target[c], cwd=src / c)
        git("add", f"src/{c}")
    if not git("diff", "--cached", "--name-only"):
        if not onto:
            subprocess.run([sys.executable, "scripts/check_pins.py"], cwd=ROOT, check=True)
            print("pins already at " + ", ".join(f"{r} {short(f)}" for r, f in targets.items()))
            return 0
        print(f"pins already at their targets in {onto}: it lands as it is")  # its code still lands
    else:
        names = ", ".join(f"{c} {short(target[c])}" for c in landing
                          if pin(ROOT, base, f"src/{c}") != target[c])
        git("commit", "-q", "-m", f"pins: {names}{' (' + note + ')' if note else ''}{trailer}")
    if subprocess.run([sys.executable, "scripts/check_pins.py"], cwd=ROOT).returncode:
        print("check_pins failed: nothing pushed (the local commits stay for a look)", file=sys.stderr)
        return 1
    # the landing as built: these exact commits, and only they, are pushed (issue 096)
    built = {".": git("rev-parse", "HEAD")}
    built.update({c: git("rev-parse", "main", cwd=src / c) for c in new})
    if onto and gated:  # the gate passed on this content (5)
        full = git("rev-parse", "-q", "--verify", f"{gated}^{{commit}}", check=False)
        tree = git("rev-parse", "HEAD^{tree}")
        if not full or git("rev-parse", f"{full}^{{tree}}") != tree:
            print(f"--gated {gated} is not this tip's content (tree {tree[:12]}): gate the tip, or name the "
                  "commit the gate passed on; nothing pushed", file=sys.stderr)
            return 1
    elif onto:
        print(f"== gate: {os.environ['LAND_PIN_GATE']}")
        # what the gate is about to test: anything that moves under it (a checkout switched, a
        # file edited mid-run) means it tested something else, and nothing is pushed (issue 096)
        before = checkout_state(src, landing)
        if before["dirty"]:
            print("the checkout has uncommitted changes before the gate: nothing pushed\n  "
                  + "\n  ".join(before["dirty"]), file=sys.stderr)
            return 1
        if subprocess.run(os.environ["LAND_PIN_GATE"], shell=True, cwd=ROOT).returncode:
            print("the gate failed: nothing pushed (the local commits stay for a look)", file=sys.stderr)
            return 1
        after = checkout_state(src, landing)
        if after != before:
            moved = [k for k in sorted(set(before["heads"]) | set(after["heads"]))
                     if before["heads"].get(k) != after["heads"].get(k)]
            print("the checkout changed while the gate ran, so the gate tested something else: "
                  "nothing pushed\n  " + "\n  ".join(
                      [f"{k}: {short(before['heads'].get(k))} -> {short(after['heads'].get(k))}"
                       for k in moved] + after["dirty"]), file=sys.stderr)
            return 1
    # a main that moved since the landing began would leave the components pushed and this
    # repository's pins refused, a partial landing: look again just before pushing (issue 096)
    git("fetch", "-q", "origin")
    if not ok("merge-base", "--is-ancestor", "origin/main", "HEAD"):
        print("origin/main moved during the landing (" + git("log", "-1", "--format=%h %s", "origin/main")
              + "): nothing pushed; rebase and land again", file=sys.stderr)
        return 3
    for c in new:  # and each component's main, so no component is pushed when a later one would fail
        git("fetch", "-q", "origin", cwd=src / c)
        if not ok("merge-base", "--is-ancestor", "origin/main", "main", cwd=src / c):
            print(f"{c}'s origin/main moved during the landing: nothing pushed; land again",
                  file=sys.stderr)
            return 3
    now = {".": git("rev-parse", "HEAD")}
    now.update({c: git("rev-parse", "main", cwd=src / c) for c in new})
    if now != built:
        print("what would be pushed is not the landing as built: nothing pushed\n  " + "\n  ".join(
            f"{k}: built {short(built[k])}, now {short(now[k])}" for k in built if now[k] != built[k]),
            file=sys.stderr)
        return 1
    for c in new:  # dependency order: a component's deps are on their remotes before it is
        r = subprocess.run(["git", "-C", str(src / c), "push", "-q", "origin", f"{built[c]}:main"])
        if r.returncode:
            print(f"push of {c} failed (its main moved?): stopped before this repository's pins; pushed so "
                  f"far: {new[:new.index(c)] or 'none'}. Rerun to cascade again.", file=sys.stderr)
            return 3
    git("push", "-q", "origin", f"{built['.']}:main", capture=False)
    print(git("log", "--oneline", "-1"))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
