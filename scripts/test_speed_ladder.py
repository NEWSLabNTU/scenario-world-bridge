"""speed_ladder's project (077 item 26): the frame against Isaac's physics step (issue 051) and
rung_check's speed band, from 077's measured tops (063 recorded none)."""
import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PROJECT = ROOT / "scenarios" / "projects" / "speed_ladder"
sys.path.insert(0, str(ROOT / "src" / "scensim"))


def rung_check():
    spec = importlib.util.spec_from_file_location("rung_check", PROJECT / "judges" / "rung_check.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def test_the_band_takes_077s_measured_tops_and_refuses_a_runaway_or_a_shortfall():
    ok = rung_check().speed_ok
    # 077: rung 10 at 20 Hz, rung 15 at 20 Hz, rung 10 at 100 Hz (12 % over)
    for top, rung in ((10.36, 10), (16.2, 15), (11.22, 10)):
        assert ok(top, rung, 0.10, 1.5), (top, rung)
    assert not ok(11.6, 10, 0.10, 1.5)     # 1.6 m/s past rung 10
    assert not ok(8.9, 10, 0.10, 1.5)      # never reached it
    assert ok(32.9, 30, 0.10, 1.5) and not ok(33.1, 30, 0.10, 1.5)   # 10 % above 15 m/s


def test_the_ladders_frame_is_whole_physics_steps():
    from scensim import config, world
    settings = config.project_settings(PROJECT)
    assert settings["ssv2"]["frame_rate"] == 100
    assert world.isaac_frame_problem(settings) is None
