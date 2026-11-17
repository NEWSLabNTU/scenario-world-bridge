import os

from scensim import slices
from scensim.slices import LogMark


def mk(proc, rel, text):
    p = proc / rel
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, "a") as f:
        f.write(text)
    return p


def test_cut_byte_range(tmp_path):
    proc = tmp_path / "proc"
    mk(proc, "bridge/scenario_world_bridge.log", "before\n")
    mk(proc, "pilot-ego/acb_bridge.log", "old\n")
    start = LogMark.take(proc)
    mk(proc, "bridge/scenario_world_bridge.log", "during\n")
    end = LogMark.take(proc)
    mk(proc, "bridge/scenario_world_bridge.log", "after\n")  # past the end mark
    out = slices.cut(start, end, tmp_path / "logs")
    assert (tmp_path / "logs/bridge/scenario_world_bridge.log").read_text() == "during\n"
    # an unchanged file has an empty slice and is not written
    assert not (tmp_path / "logs/pilot-ego").exists()
    assert len(out) == 1


def test_new_file_taken_whole(tmp_path):
    proc = tmp_path / "proc"
    proc.mkdir()
    start = LogMark.take(proc)
    mk(proc, "relay/scenario_agent_relay.log", "hello\n")
    mk(proc, "top.log", "x\n")
    end = LogMark.take(proc)
    slices.cut(start, end, tmp_path / "logs")
    assert (tmp_path / "logs/relay/scenario_agent_relay.log").read_text() == "hello\n"
    assert (tmp_path / "logs/stack/top.log").read_text() == "x\n"


def test_truncated_file_from_zero(tmp_path):
    proc = tmp_path / "proc"
    p = mk(proc, "simulator/carla.log", "a" * 100)
    start = LogMark.take(proc)
    p.write_text("short\n")  # truncated in place (same inode), smaller than the start mark
    end = LogMark.take(proc)
    slices.cut(start, end, tmp_path / "logs")
    assert (tmp_path / "logs/simulator/carla.log").read_text() == "short\n"


def test_rotation(tmp_path):
    proc = tmp_path / "proc"
    p = mk(proc, "bridge/b.log", "pre\n")
    start = LogMark.take(proc)
    mk(proc, "bridge/b.log", "run-1\n")
    os.rename(p, proc / "bridge/b.log.1")  # rotated
    mk(proc, "bridge/b.log", "run-2\n")  # recreated
    end = LogMark.take(proc)
    slices.cut(start, end, tmp_path / "logs")
    assert (tmp_path / "logs/bridge/b.log.1").read_text() == "run-1\n"
    assert (tmp_path / "logs/bridge/b.log").read_text() == "run-2\n"


def test_file_gone_and_missing_dir(tmp_path):
    proc = tmp_path / "proc"
    p = mk(proc, "bridge/b.log", "x")
    start = LogMark.take(tmp_path / "none")
    assert start.files == {}
    end = LogMark.take(proc)
    p.unlink()
    assert slices.cut(start, end, tmp_path / "logs") == []


def test_mark_round_trip(tmp_path):
    proc = tmp_path / "proc"
    mk(proc, "g/a.log", "abc")
    m = LogMark.take(proc)
    assert m.size("g/a.log") == 3 and m.size("nope") is None
    m2 = LogMark.read(m.write(tmp_path / "mark.json"))
    assert m2 == m


def test_group_of():
    assert slices.group_of("pilot-ego/x/y.log") == ("pilot-ego", "x/y.log")
    assert slices.group_of("a.log") == ("stack", "a.log")
