"""scripts/land_pin.py on scratch repositories (issue 083): pure git on the local disk, no network.

The scratch world mirrors this workspace's shape: bare "origins" for a superproject and five components, A and B
pinned by C under deps/ (as sim_vehicle_spec and sim_vehicle_ecu are by aib), C pinned by D, D by E (a chain of
three new commits, whose deepest is only local while the next is made: autoware_sim_vehicle, aib,
scensim_isaac_world), and the superproject pinning all five under src/ with copies of check_pins.py and
land_pin.py.
"""
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent


def run(*a, cwd=None, check=True):
    r = subprocess.run(list(a), cwd=cwd, capture_output=True, text=True)
    if check and r.returncode:
        raise AssertionError(f"{a} in {cwd}: {r.returncode}\n{r.stdout}\n{r.stderr}")
    return r


def git(cwd, *a):
    return run("git", *a, cwd=cwd).stdout.strip()


def commit(cwd, msg):
    (Path(cwd) / "f.txt").write_text(msg + "\n")
    git(cwd, "add", "-A")
    git(cwd, "commit", "-q", "-m", msg)
    return git(cwd, "rev-parse", "HEAD")


class World:
    """origin/<name>.git for A, B, C, D and super; a work clone of each component in work/<name>."""

    def __init__(self, tmp):
        self.tmp = tmp
        self.origin = tmp / "origin"
        self.work = tmp / "work"
        for n in ("A", "B", "C", "D", "E", "super"):
            run("git", "init", "-q", "--bare", "-b", "main", str(self.origin / f"{n}.git"))
        self.sha = {}
        for n in ("A", "B"):
            d = self.clone(n)
            self.sha[n] = commit(d, f"{n} 1")
            git(d, "push", "-q", "origin", "main")
        self.sha["C"] = self.component("C", ("A", "B"))
        self.sha["D"] = self.component("D", ("C",))
        self.sha["E"] = self.component("E", ("D",))
        s = self.clone("super")
        (s / "scripts").mkdir()
        for f in ("check_pins.py", "land_pin.py"):
            shutil.copy(HERE / f, s / "scripts" / f)
        for n in ("A", "B", "C", "D", "E"):
            git(s, "submodule", "add", "-q", self.url(n), f"src/{n}")
        git(s, "commit", "-q", "-m", "super 1")
        git(s, "push", "-q", "origin", "main")
        git(s, "submodule", "update", "-q", "--init", "--recursive")  # as a workspace's checkouts are
        self.super = s

    def url(self, n):
        return str(self.origin / f"{n}.git")

    def clone(self, n):
        d = self.work / n
        run("git", "clone", "-q", self.url(n), str(d))
        git(d, "checkout", "-q", "-B", "main")
        return d

    def component(self, n, deps):
        d = self.clone(n)
        commit(d, f"{n} 1")
        for x in deps:
            git(d, "submodule", "add", "-q", self.url(x), f"deps/{x}")
        git(d, "commit", "-q", "-m", f"{n} deps")
        git(d, "push", "-q", "origin", "main")
        return git(d, "rev-parse", "HEAD")

    def advance(self, n):
        """A new commit on n's origin/main; its sha."""
        d = self.work / n
        git(d, "pull", "-q", "--rebase", "origin", "main")
        sha = commit(d, f"{n} {len(git(d, 'log', '--oneline').splitlines()) + 1}")
        git(d, "push", "-q", "origin", "main")
        return sha

    def bump_deps(self, n, moves):
        """A commit on n's origin/main moving deps/<x> to sha, for x, sha in moves; its sha."""
        d = self.work / n
        git(d, "pull", "-q", "--rebase", "origin", "main")
        for x, sha in moves.items():
            sub = d / "deps" / x
            git(sub, "fetch", "-q", "origin")
            git(sub, "checkout", "-q", sha)
            git(d, "add", f"deps/{x}")
        git(d, "commit", "-q", "-m", f"{n} deps " + " ".join(moves))
        git(d, "push", "-q", "origin", "main")
        return git(d, "rev-parse", "HEAD")

    def land(self, *args):
        return run(sys.executable, "scripts/land_pin.py", *args, cwd=self.super, check=False)

    def pin(self, rev, path, cwd=None):
        out = git(cwd or self.super, "ls-tree", rev, path).split()
        return out[2] if out else None

    def remote_pin(self, n):
        git(self.super, "fetch", "-q", "origin")
        return self.pin("origin/main", f"src/{n}")

    def check_pins(self):
        return run(sys.executable, "scripts/check_pins.py", cwd=self.super, check=False)


@pytest.fixture
def world(tmp_path, monkeypatch):
    home = tmp_path / "home"
    home.mkdir()
    (home / ".gitconfig").write_text("[user]\n\tname = t\n\temail = t@example.com\n"
                                     "[protocol \"file\"]\n\tallow = always\n[init]\n\tdefaultBranch = main\n")
    monkeypatch.setenv("HOME", str(home))
    for k in ("GIT_DIR", "GIT_WORK_TREE", "GIT_INDEX_FILE", "LAND_PIN_TRAILER"):
        monkeypatch.delenv(k, raising=False)
    w = World(tmp_path)
    assert w.check_pins().returncode == 0
    return w


def test_one_target_cascades_as_before(world):
    a2 = world.advance("A")
    r = world.land("A", a2[:7])
    assert r.returncode == 0, r.stdout + r.stderr
    assert world.remote_pin("A") == a2
    c = world.remote_pin("C")
    assert c != world.sha["C"] and world.pin(c, "deps/A", world.super / "src" / "C") == a2
    d = world.remote_pin("D")
    assert world.pin(d, "deps/C", world.super / "src" / "D") == c
    e = world.remote_pin("E")                             # three new commits in a chain (083's second gap)
    assert world.pin(e, "deps/D", world.super / "src" / "E") == d
    assert world.check_pins().returncode == 0


def test_the_dry_run_changes_nothing(world):
    a2 = world.advance("A")
    before = world.remote_pin("A")
    r = world.land("--dry-run", "A", a2)
    assert r.returncode == 0 and "dry run: nothing changed" in r.stdout
    assert world.remote_pin("A") == before == world.sha["A"]


def test_two_targets_land_in_one_commit(world):
    a2, b2 = world.advance("A"), world.advance("B")
    r = world.land("A", a2, "B", b2, "--", "both")
    assert r.returncode == 0, r.stdout + r.stderr
    log = git(world.super, "log", "--oneline", "origin/main")
    assert len(log.splitlines()) == 2 and "(both)" in log.splitlines()[0]
    c = world.remote_pin("C")
    cdir = world.super / "src" / "C"
    assert (world.pin(c, "deps/A", cdir), world.pin(c, "deps/B", cdir)) == (a2, b2)
    assert world.check_pins().returncode == 0


def test_step_2s_case_a_component_already_at_both_targets(world):
    """091 step 2: C's main already pins the new A and B (aib already pinned the new spec and ecu): C moves to its
    main with no commit of its own, D is bumped to it, and the pins check green. The old script moved only the
    target and failed check_pins."""
    a2, b2 = world.advance("A"), world.advance("B")
    c2 = world.bump_deps("C", {"A": a2, "B": b2})
    r = world.land("--dry-run", "A", a2, "B", b2)
    assert r.returncode == 0 and "C: its main" in r.stdout and "already pins" in r.stdout, r.stdout
    r = world.land("A", a2, "B", b2)
    assert r.returncode == 0, r.stdout + r.stderr
    assert world.remote_pin("C") == c2                    # its main, not a new commit
    git(world.work / "C", "fetch", "-q", "origin")
    assert git(world.work / "C", "rev-parse", "origin/main") == c2
    d = world.remote_pin("D")
    assert world.pin(d, "deps/C", world.super / "src" / "D") == c2
    assert world.check_pins().returncode == 0


def test_one_target_with_its_partner_already_landed_in_a_dependent(world):
    """The failure 083 records: landing B alone while C's main pins the new A and B. C then has to move here, and with
    it A: check_pins refuses the landing, naming A, and nothing is pushed."""
    a2, b2 = world.advance("A"), world.advance("B")
    world.bump_deps("C", {"A": a2, "B": b2})
    r = world.land("B", b2)
    assert r.returncode == 1 and "deps/A" in r.stdout, r.stdout + r.stderr
    assert world.remote_pin("B") == world.sha["B"]        # nothing pushed


def test_bad_arguments(world):
    assert world.land("A").returncode == 2
    assert world.land("A", "x", "B", "y", "C").returncode == 2  # pairs and a stray argument
    assert world.land("A", "deadbeef", "A", "deadbeef").returncode == 2  # a repository twice
    assert world.land("Z", "deadbeef").returncode == 2
