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


# --- CARLA 1.0's wheels (083 step 6) ---------------------------------------------------------
# CARLA 1.0 (Chaos) has no WheelPhysicsControl.position (world cm); its `location` is in m. T3 on
# 1.0 failed every coop agent's set_goal on it, twice (s5's agents, then c2b's cutin_agent the
# first fix missed). Wheel positions go through arena.wheel_xy, or a script's own getattr guard.
import re  # noqa: E402

WHEEL_POSITION = re.compile(r"(?:wheels?\[\d+\]|\bw\[\d+\]|\bwh?|\bwheel)\.position\b")


def test_no_script_reads_a_wheels_position_unguarded():
    bad = []
    for f in sorted((ROOT / "scripts").rglob("*.py")):
        if f.name == "test_coop_pilots_dir.py":
            continue
        for n, line in enumerate(f.read_text(errors="replace").splitlines(), 1):
            if WHEEL_POSITION.search(line) and "getattr" not in line:
                bad.append(f"{f.relative_to(ROOT)}:{n}: {line.strip()}")
    assert not bad, "a wheel's position read directly (fails on CARLA 1.0):\n" + "\n".join(bad)


def test_wheel_xy_reads_both_carlas():
    import sys
    import types
    sys.path.insert(0, str(ROOT / "scripts" / "coop"))
    try:
        AR = pytest.importorskip("arena")
    finally:
        sys.path.pop(0)
    V = lambda x, y: types.SimpleNamespace(x=x, y=y)  # noqa: E731
    at = [(1.4, -0.8), (1.4, 0.8), (-1.5, -0.8), (-1.5, 0.8)]
    old = [types.SimpleNamespace(position=V(x * 100, y * 100)) for x, y in at]   # 0.9.16, cm
    new = [types.SimpleNamespace(location=V(x, y)) for x, y in at]               # 1.0, m
    for ws in (old, new):
        assert AR.wheel_xy(ws[0]) == pytest.approx((1.4, -0.8))
        assert AR.wheel_geometry(ws) == pytest.approx((2.9, 1.6))


def test_the_coop_car_is_the_servers_body():
    """083 step 6: on CARLA 1.0 the coop agents drive 1.0's car (physics and pedal maps)."""
    import sys
    sys.path.insert(0, str(ROOT / "scripts" / "coop"))
    try:
        AR = pytest.importorskip("arena")
    finally:
        sys.path.pop(0)
    old = AR.body_files("0.9.16")
    assert old == AR.body_files(None)
    assert old[0].name == "carla_physics.yaml" and old[1].parent.name == "tesla_model3_description"
    new = AR.body_files("1.0")
    assert new == AR.body_files("1.0.0") == AR.body_files("1.0-82c52a7c2")
    assert new[0].name == "carla_physics_1.0.yaml"
    assert new[1].parent.name == "tesla_model3_carla_1_0_description"
    for f in (*old, *new):
        assert f.exists(), f
    # the maps differ where it matters: 1.0 needs far more throttle for the same acceleration
    a_old, a_new = AR.PedalMap(old[1] / "accel_map.csv"), AR.PedalMap(new[1] / "accel_map.csv")
    assert a_new.pedal_for(1.0, 5.0, True) > 1.5 * a_old.pedal_for(1.0, 5.0, True)
