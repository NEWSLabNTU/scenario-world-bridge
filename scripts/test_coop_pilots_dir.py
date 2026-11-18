"""081 step 4, group C: the coop study testbenches (c2b, s5, e2) find scripts/coop where the
coop_pilots package installs it, and fail naming the package when it is not installed; never
the checkout."""
import importlib.util
import os
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
STUDIES = ["c2b", "s5", "e2"]


def load(study):
    spec = importlib.util.spec_from_file_location(
        f"coop_pilots_dir_{study}", ROOT / "scenarios" / "projects" / study / "coop_pilots_dir.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def fake_install(tmp_path, with_modules=True):
    prefix = tmp_path / "install"
    index = prefix / "share" / "ament_index" / "resource_index" / "packages"
    index.mkdir(parents=True)
    (index / "coop_pilots").write_text("")
    coop = prefix / "share" / "coop_pilots" / "coop"
    coop.mkdir(parents=True)
    if with_modules:
        (coop / "measures.py").write_text("")
    return prefix, coop


@pytest.mark.parametrize("study", STUDIES)
def test_the_installed_directory(tmp_path, monkeypatch, study):
    pytest.importorskip("ament_index_python")
    prefix, coop = fake_install(tmp_path)
    monkeypatch.setenv("AMENT_PREFIX_PATH", str(prefix))
    assert load(study).coop_dir() == coop


@pytest.mark.parametrize("study", STUDIES)
def test_no_package_fails_naming_it(tmp_path, monkeypatch, study):
    pytest.importorskip("ament_index_python")
    monkeypatch.setenv("AMENT_PREFIX_PATH", str(tmp_path / "nothing"))
    with pytest.raises(SystemExit, match="coop_pilots is not installed"):
        load(study).coop_dir()
    prefix, _ = fake_install(tmp_path, with_modules=False)
    monkeypatch.setenv("AMENT_PREFIX_PATH", str(prefix))
    with pytest.raises(SystemExit, match="coop_pilots is installed without its coop modules"):
        load(study).coop_dir()


def test_no_study_reaches_into_the_checkout():
    for study in STUDIES:
        for f in (ROOT / "scenarios" / "projects" / study).rglob("*.py"):
            text = f.read_text()
            assert '"scripts" / "coop"' not in text and "parents[" not in text, f
    assert os.path.isfile(ROOT / "src" / "coop_pilots" / "package.xml")
