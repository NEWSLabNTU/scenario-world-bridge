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


def test_a_target_pinning_a_repo_its_own_cascade_moves_is_bumped(world):
    """Issue 085 (091 W19's shape): A and D land together; D's given commit pins C, which the cascade moves
    (C pins A). D gets a new commit on top of the given one, pinning C's new commit; E follows. Before the fix
    D landed as given and check_pins refused it."""
    a2 = world.advance("A")
    d2 = world.advance("D")                               # D's main tip, still pinning C's old commit
    r = world.land("--dry-run", "A", a2, "D", d2)
    assert r.returncode == 0, r.stdout + r.stderr
    r = world.land("A", a2, "D", d2)
    assert r.returncode == 0, r.stdout + r.stderr
    c = world.remote_pin("C")
    assert c != world.sha["C"] and world.pin(c, "deps/A", world.super / "src" / "C") == a2
    d = world.remote_pin("D")
    ddir = world.super / "src" / "D"
    assert d != d2 and git(ddir, "rev-parse", f"{d}~1") == d2   # on top of the given commit
    assert world.pin(d, "deps/C", ddir) == c
    e = world.remote_pin("E")
    assert world.pin(e, "deps/D", world.super / "src" / "E") == d
    assert world.check_pins().returncode == 0


def test_a_target_behind_its_main_that_would_need_a_bump_is_refused(world):
    """Issue 085: the bump goes on the target's main, so a given commit behind its main's tip is refused by
    name rather than landing the commits after it unasked."""
    a2 = world.advance("A")
    d2 = world.advance("D")
    world.advance("D")                                    # D's main moves past d2
    r = world.land("A", a2, "D", d2)
    assert r.returncode == 2 and "not its main's tip" in r.stderr and "C" in r.stderr, r.stdout + r.stderr
    assert world.remote_pin("A") == world.sha["A"]        # nothing pushed


def test_a_local_commit_here_is_refused_by_name_in_a_dry_run_too(world):
    """A commit in this repository's checkout that is not on origin/main would be left out of the landing
    (the pins go on origin/main): refused, naming it, dry run or not; a detached HEAD at origin/main passes."""
    a2 = world.advance("A")
    (world.super / "notes.txt").write_text("local\n")
    git(world.super, "add", "notes.txt")
    git(world.super, "commit", "-q", "-m", "a local note")
    for args in (("--dry-run", "A", a2), ("A", a2)):
        r = world.land(*args)
        assert r.returncode == 2 and "a local note" in r.stderr and "push or land these first" in r.stderr, \
            r.stdout + r.stderr
    assert world.remote_pin("A") == world.sha["A"]        # nothing pushed
    git(world.super, "checkout", "-q", "--detach", "origin/main")
    r = world.land("--dry-run", "A", a2)
    assert r.returncode == 0, r.stdout + r.stderr


# --- --onto: code here that needs the pins lands with them, as one tip (5) ---------------------------------

def code_branch(world, name="feat"):
    """A pushed branch of the superproject with a code commit on origin/main; checked out; its sha."""
    s = world.super
    git(s, "fetch", "-q", "origin")
    git(s, "checkout", "-q", "-B", name, "origin/main")
    (s / "code.txt").write_text("uses A's new interface\n")
    git(s, "add", "code.txt")
    git(s, "commit", "-q", "-m", "code that needs A 2")
    git(s, "push", "-q", "origin", name)
    return git(s, "rev-parse", "HEAD")


def test_onto_lands_the_code_and_the_pins_as_one_tip(world, monkeypatch):
    a2 = world.advance("A")
    code = code_branch(world)
    monkeypatch.setenv("LAND_PIN_GATE", "test -f code.txt")   # runs in the checkout, at the tip
    r = world.land("--onto", "feat", "A", a2)
    assert r.returncode == 0, r.stdout + r.stderr
    assert world.remote_pin("A") == a2
    tip = git(world.super, "rev-parse", "origin/main")
    assert git(world.super, "rev-parse", "origin/main~1") == code   # the pins on top of the code
    assert git(world.super, "show", "origin/main:code.txt") == "uses A's new interface"
    assert git(world.super, "log", "--merges", "--oneline", "origin/main") == ""
    assert tip != code and world.check_pins().returncode == 0


def test_onto_without_a_gate_is_refused(world):
    a2 = world.advance("A")
    code_branch(world)
    for args in (("--dry-run", "--onto", "feat", "A", a2), ("--onto", "feat", "A", a2)):
        r = world.land(*args)
        assert r.returncode == 2 and "--gated" in r.stderr and "LAND_PIN_GATE" in r.stderr, r.stdout + r.stderr
    assert world.remote_pin("A") == world.sha["A"]
    r = world.land("--gated", "abc", "A", a2)                 # --gated alone means nothing
    assert r.returncode == 2


def test_onto_a_branch_behind_main_is_refused(world, monkeypatch):
    a2 = world.advance("A")
    code_branch(world)
    other = world.work / "super2"
    run("git", "clone", "-q", world.url("super"), str(other))
    commit(other, "someone else lands first")
    git(other, "push", "-q", "origin", "main")
    monkeypatch.setenv("LAND_PIN_GATE", "true")
    r = world.land("--onto", "feat", "A", a2)
    assert r.returncode == 2 and "does not fast-forward origin/main" in r.stderr, r.stdout + r.stderr
    assert world.remote_pin("A") == world.sha["A"]


def test_onto_a_failed_gate_pushes_nothing_and_gated_takes_the_tips_content(world, monkeypatch):
    # E has no dependents: no cascade, so the tip's content is the same on a second run
    e2 = world.advance("E")
    code_branch(world)
    monkeypatch.setenv("LAND_PIN_GATE", "false")
    r = world.land("--onto", "feat", "E", e2)
    assert r.returncode == 1 and "the gate failed: nothing pushed" in r.stderr, r.stdout + r.stderr
    assert world.remote_pin("E") == world.sha["E"]
    gated = git(world.super, "rev-parse", "HEAD")             # the tip the gate ran on, left for a look
    monkeypatch.delenv("LAND_PIN_GATE")
    git(world.super, "checkout", "-q", "feat")
    # a commit with other content is not the tip's: refused, nothing pushed
    r = world.land("--onto", "feat", "--gated", git(world.super, "rev-parse", "feat"), "E", e2)
    assert r.returncode == 1 and "is not this tip's content" in r.stderr, r.stdout + r.stderr
    assert world.remote_pin("E") == world.sha["E"]
    # the gated commit's tree is the new tip's (the pin commit is made anew: its sha may differ, or not,
    # when made within the same second; the content is what counts)
    git(world.super, "checkout", "-q", "feat")
    r = world.land("--onto", "feat", "--gated", gated, "E", e2)
    assert r.returncode == 0, r.stdout + r.stderr
    tip = git(world.super, "rev-parse", "origin/main")
    assert git(world.super, "rev-parse", f"{tip}^{{tree}}") == \
        git(world.super, "rev-parse", f"{gated}^{{tree}}")


def test_onto_gated_is_refused_when_the_cascade_makes_new_commits(world):
    a2 = world.advance("A")                                   # A moves C, D and E: new commits each run
    code_branch(world)
    r = world.land("--onto", "feat", "--gated", "HEAD", "A", a2)
    assert r.returncode == 2 and "the cascade makes new commits in C, D, E" in r.stderr, r.stdout + r.stderr
    assert world.remote_pin("A") == world.sha["A"]


def test_onto_dry_run_names_the_branch_and_the_gate(world, monkeypatch):
    a2 = world.advance("A")
    code_branch(world)
    monkeypatch.setenv("LAND_PIN_GATE", "just test")
    r = world.land("--dry-run", "--onto", "feat", "A", a2)
    assert r.returncode == 0 and "== onto feat" in r.stdout and "LAND_PIN_GATE (just test)" in r.stdout, \
        r.stdout + r.stderr
    assert "dry run: nothing changed" in r.stdout and world.remote_pin("A") == world.sha["A"]


# --- guards (issue 096): the gate tests what lands, and a moved main pushes nothing ----------------------

def test_a_checkout_changed_while_the_gate_ran_pushes_nothing(world, monkeypatch):
    e2 = world.advance("E")
    code_branch(world)
    # the gate's own run switches the checkout (139's case: a survey command in the same tree)
    monkeypatch.setenv("LAND_PIN_GATE", "git checkout -q --detach origin/main")
    r = world.land("--onto", "feat", "E", e2)
    assert r.returncode == 1 and "changed while the gate ran" in r.stderr and ".: " in r.stderr, \
        r.stdout + r.stderr
    assert world.remote_pin("E") == world.sha["E"]
    # or edits a tracked file in it
    git(world.super, "checkout", "-q", "feat")
    monkeypatch.setenv("LAND_PIN_GATE", "echo x >> code.txt")
    r = world.land("--onto", "feat", "E", e2)
    assert r.returncode == 1 and "changed while the gate ran" in r.stderr and "code.txt" in r.stderr, \
        r.stdout + r.stderr
    assert world.remote_pin("E") == world.sha["E"]


def test_an_uncommitted_change_before_the_gate_is_refused(world, monkeypatch):
    e2 = world.advance("E")
    code_branch(world)
    (world.super / "code.txt").write_text("edited, not committed\n")
    monkeypatch.setenv("LAND_PIN_GATE", "true")
    r = world.land("--onto", "feat", "E", e2)
    assert r.returncode in (1, 2) and world.remote_pin("E") == world.sha["E"], r.stdout + r.stderr


def other_lands_on(world, name):
    """A shell line that lands a commit on `name`'s origin/main from another clone."""
    d = world.work / f"{name}-other"
    if not d.exists():
        run("git", "clone", "-q", world.url(name), str(d))
    return f"cd {d} && git pull -q --rebase origin main && git commit -q --allow-empty -m mid && git push -q origin main"


def test_a_main_that_moves_during_the_landing_pushes_nothing(world, monkeypatch):
    """The gate passes, but someone landed meanwhile: no component is pushed, so there is no
    partial landing (components moved, this repository's pins refused)."""
    a2 = world.advance("A")
    code_branch(world)
    c_main = git(world.origin / "C.git", "rev-parse", "main")
    monkeypatch.setenv("LAND_PIN_GATE", other_lands_on(world, "super"))
    r = world.land("--onto", "feat", "A", a2)
    assert r.returncode == 3 and "origin/main moved during the landing" in r.stderr, r.stdout + r.stderr
    assert git(world.origin / "C.git", "rev-parse", "main") == c_main   # C's bump not pushed
    assert world.remote_pin("A") == world.sha["A"]


def test_a_component_main_that_moves_pushes_nothing(world, monkeypatch):
    a2 = world.advance("A")
    code_branch(world)
    d_main = git(world.origin / "D.git", "rev-parse", "main")
    c_main = git(world.origin / "C.git", "rev-parse", "main")
    monkeypatch.setenv("LAND_PIN_GATE", other_lands_on(world, "E"))   # the last in the cascade
    r = world.land("--onto", "feat", "A", a2)
    assert r.returncode == 3 and "E's origin/main moved" in r.stderr, r.stdout + r.stderr
    assert git(world.origin / "C.git", "rev-parse", "main") == c_main  # nothing before E pushed either
    assert git(world.origin / "D.git", "rev-parse", "main") == d_main
