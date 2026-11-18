"""<prefix>/bin/scensim runs the console script colcon generated, whatever python is first on
PATH (issue 062: a uv python first on PATH had no ROS modules, `No module named 'yaml'`)."""
import os
import subprocess
from pathlib import Path

SRC = Path(__file__).resolve().parents[1] / "bin" / "scensim"


def prefix(tmp: Path, link: bool) -> Path:
    """An install prefix: bin/scensim (a copy, or as --symlink-install makes it, a link to the
    source) and lib/scensim/scensim (a stand-in for the generated console script)."""
    p = tmp / "install"
    (p / "bin").mkdir(parents=True)
    (p / "lib" / "scensim").mkdir(parents=True)
    entry = p / "lib" / "scensim" / "scensim"
    entry.write_text('#!/bin/sh\necho "entry $*"\n')
    entry.chmod(0o755)
    if link:
        (p / "bin" / "scensim").symlink_to(SRC)
    else:
        (p / "bin" / "scensim").write_text(SRC.read_text())
        (p / "bin" / "scensim").chmod(0o755)
    return p


def run(cmd, tmp: Path):
    fake = tmp / "uv"
    fake.mkdir(exist_ok=True)
    py = fake / "python3"
    py.write_text("#!/bin/sh\necho \"ModuleNotFoundError: No module named 'yaml'\" >&2\nexit 1\n")
    py.chmod(0o755)
    env = {**os.environ, "PATH": f"{fake}:{os.environ['PATH']}"}
    return subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=30)


def test_the_source_script_names_no_python():
    assert SRC.read_text().startswith("#!/bin/sh\n")
    assert "python" not in SRC.read_text().splitlines()[0]


def test_a_python_first_on_path_is_not_used(tmp_path):
    for link in (False, True):
        p = prefix(tmp_path / str(link), link)
        r = run([str(p / "bin" / "scensim"), "run", "a b"], tmp_path)
        assert r.returncode == 0 and r.stdout == "entry run a b\n", (link, r.stderr)


def test_a_link_to_it_from_elsewhere_on_path_finds_its_install(tmp_path):
    p = prefix(tmp_path, False)
    (tmp_path / "home_bin").mkdir()
    (tmp_path / "home_bin" / "scensim").symlink_to(p / "bin" / "scensim")
    r = run([str(tmp_path / "home_bin" / "scensim"), "--version"], tmp_path)
    assert r.returncode == 0 and r.stdout == "entry --version\n", r.stderr


def test_without_its_entry_it_says_how_to_fix(tmp_path):
    p = prefix(tmp_path, False)
    (p / "lib" / "scensim" / "scensim").unlink()
    r = run([str(p / "bin" / "scensim")], tmp_path)
    assert r.returncode == 127 and "rebuild scensim" in r.stderr
