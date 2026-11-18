"""077 G77: a stack node that exits again and again before the stack is ready ends the start,
named with how it exited, its last stderr line and Kit's crash dump, from either launcher's
logs (play_launch's play_launch.log and node/<name>/err.<k>; ros2 launch's console.log)."""
import pytest

from scensim import crashloop, run

PL = "2026-11-17_05-10-00"


def pl_exit(name, code=None, sig=None, state=True):
    """play_launch's lines for one exit, as 139's logs hold them (2026-11-16/17): the WARN or
    ERROR line always; the DEBUG state event (the signal) only at that log level."""
    what = f"with code {code}" if code is not None else "without code"
    line = ("2026-11-17T05:10:01.000000Z  WARN play_launch::member_actor::regular_node_actor: "
            f"[node:/{name}] Exited {what} (crash 2 since last healthy run)\n")
    if not state:
        return line
    c = f"Some({code})" if code is not None else "None"
    s = f"Some({sig})" if sig is not None else "None"
    return line + ("2026-11-17T05:10:01.000000Z DEBUG play_launch::member_actor::coordinator::"
                   f'runner: State event: Exited {{ name: "node:/{name}", exit_code: {c}, '
                   f"signal: {s}, at_ms: 1, uptime_ms: Some(1500), oom_killed: None }}\n")


def play_launch_side(proc, name, exits, err_lines):
    d = proc / "simulator" / PL
    (d / "node" / name).mkdir(parents=True)
    (proc / "simulator" / "latest").symlink_to(PL)   # as play_launch links it: not counted
    (d / "play_launch.log").write_text("".join(pl_exit(name, *e) for e in exits))
    for k, line in enumerate(err_lines, 1):
        (d / "node" / name / f"err.{k}").write_text(f"starting\n{line}\n\n")
    return d


def test_how_names_signals_from_either_launcher():
    assert crashloop.how(1) == "exited 1"
    assert crashloop.how(139) == "exited 139 (SIGSEGV)"     # a shell wrapper's 128 + 11
    assert crashloop.how(-11) == "exited -11 (SIGSEGV)"     # ros2 launch's -signal
    assert crashloop.how(None, 11) == "killed by signal 11 (SIGSEGV)"   # play_launch's signal
    assert crashloop.how(None, None) == "exited without a code"


def test_play_launch_five_exits_end_the_start(tmp_path):
    play_launch_side(tmp_path, "scensim_isaac_world", [(1,)] * 4,
                     ["ImportError: no module named isaacsim"] * 4)
    assert crashloop.looping(tmp_path) is None          # 4: still a start
    d = tmp_path / "simulator" / PL
    with open(d / "play_launch.log", "a") as f:
        f.write(pl_exit("scensim_isaac_world", 1))
    # the 5th instance's err is not renamed to err.5 until its respawn (live, 2026-11-17)
    (d / "node" / "scensim_isaac_world" / "err").write_text("ImportError: no module named isaacsim\n")
    msg = crashloop.looping(tmp_path)
    assert msg.startswith("stack node scensim_isaac_world exited 1 5 times before the stack was "
                          "ready")
    assert "its last stderr line: ImportError: no module named isaacsim" in msg
    assert f"({d / 'node' / 'scensim_isaac_world' / 'err'})" in msg


def test_signals_count_and_the_crash_dump_is_named(tmp_path):
    """241's log sort: 118 segfaults (exit 139) at Kit's renderer init, respawned every ~14 s."""
    play_launch_side(tmp_path, "scensim_isaac_world", [(None, 11)] * 2 + [(139,)] * 3,
                     ["Segmentation fault"] * 4 +
                     ["[crashreporter] crash dump written to /tmp/kit/dumps/abc-123.dmp"])
    msg = crashloop.looping(tmp_path)
    assert "stack node scensim_isaac_world exited 139 (SIGSEGV) 5 times" in msg
    assert "crash dump in /tmp/kit/dumps" in msg


def test_ros2_launch_console(tmp_path):
    side = tmp_path / "simulator"
    side.mkdir()
    lines = ["[INFO] [launch]: All log files can be found below /x"]
    for i in range(5):
        lines += [f"[scensim_isaac_world_server-1] fake isaac: this venv is broken {i}",
                  "[ERROR] [scensim_isaac_world_server-1]: process has died [pid 10, exit code "
                  f"{-11 if i == 4 else 1}, cmd 'scensim_isaac_world_server --port 5590'].",
                  "[INFO] [scensim_isaac_world_server-1]: process started with pid [11]"]
    (side / "console.log").write_text("\n".join(lines) + "\n")
    msg = crashloop.looping(tmp_path)
    assert msg.startswith("stack node scensim_isaac_world_server exited -11 (SIGSEGV) 5 times")
    assert "its last stderr line: fake isaac: this venv is broken 4" in msg


def test_ros2_launch_stop_deaths_do_not_count(tmp_path):
    side = tmp_path / "bridge"
    side.mkdir()
    died = ("[ERROR] [scenario_world_bridge-1]: process has died [pid 10, exit code -2, "
            "cmd 'x'].")
    (side / "console.log").write_text(
        "\n".join([died] * 2 + ["[INFO] [launch]: user interrupted with ctrl-c (SIGINT)"]
                  + [died] * 5) + "\n")
    assert crashloop.looping(tmp_path) is None


def test_the_stacks_own_launch_and_a_nested_launch_that_cannot_parse(tmp_path):
    """The live check on 139 (2026-11-17): the stack's nested `simulator` launch itself
    exited 1 at once, 14 times in 70 s (a parser error), logged only in the stack's own
    launch directory, without state events."""
    d = tmp_path / "launch" / "2026-11-17_05-29-03"
    (d / "node" / "simulator").mkdir(parents=True)
    (d / "play_launch.log").write_text("".join(pl_exit("simulator", 1, state=False)
                                               for _ in range(5)))
    err = ("Error: Rust parser error while parsing .../scensim_isaac_world.launch.xml: Python "
           "error: Failed to read Python file: No such file or directory (os error 2)\n\n"
           "Hint: if this is a parser limitation rather than a bad launch file, re-run the "
           "same command with `--parser python` (slower, maximum compatibility).\n")
    for k in range(1, 6):
        (d / "node" / "simulator" / f"err.{k}").write_text(err)
    assert crashloop.looping(tmp_path / "proc") is None
    msg = crashloop.looping(None, launch_dir=tmp_path / "launch")
    assert msg.startswith("stack node simulator exited 1 5 times")
    assert "its last stderr line: Error: Rust parser error" in msg


def test_exits_of_different_nodes_are_counted_apart(tmp_path):
    d = tmp_path / "simulator" / PL
    d.mkdir(parents=True)
    (d / "play_launch.log").write_text(
        "".join(pl_exit(n, 1) for n in ("a", "b") * 4))
    assert crashloop.looping(tmp_path) is None


def test_wait_started_ends_on_a_crash_loop(tmp_path, monkeypatch):
    """`scensim run`'s wait for a stack it started: a crash loop ends it at once, not at the
    timeout ("not ready within 1800 s (starting)", 118)."""
    proc = tmp_path / "stack" / "proc"
    play_launch_side(proc, "scensim_carla_world", [(75,)] * 5, ["CARLA hung"] * 5)

    class P:
        def poll(self):
            return None

    started = run.StartedStack(proc=P(), argv=[], log=tmp_path / "console.log", seat=0)
    monkeypatch.setattr(run, "find_stack", lambda project, h: {"state": "starting",
                                                                "proc": str(proc)})
    with pytest.raises(run.StackFailed, match="scensim_carla_world exited 75 5 times"):
        run.wait_started(started, tmp_path, {"logs": str(tmp_path)}, timeout=30)


# --- issue 052: any exit during the start ends it at once, exit 0 too -------------------------

KIT_CLEAN = ["Traceback (most recent call last):",
             "ImportError: cannot import name 'step_rate' from 'scensim_isaac_world'",
             "SimulationApp.close() was not called explicitly. Shutting down automatically"]


def test_ros2_launch_a_clean_exit_ends_the_start_with_its_last_lines(tmp_path):
    """run A (2026-11-18): Kit exits 0 after the world server's ImportError."""
    side = tmp_path / "simulator"
    side.mkdir()
    lines = ["[INFO] [launch]: All log files can be found below /x",
             "[INFO] [scensim_isaac_world_server-1]: process started with pid [10]"]
    lines += [f"[scensim_isaac_world_server-1] {x}" for x in KIT_CLEAN]
    lines += ["[INFO] [scensim_isaac_world_server-1]: process has finished cleanly [pid 10]"]
    (side / "console.log").write_text("\n".join(lines) + "\n")
    assert crashloop.looping(tmp_path) is None     # one exit is no loop
    msg = crashloop.ended(tmp_path)
    assert msg.startswith("stack node scensim_isaac_world_server exited 0 during the stack's "
                          "start")
    assert "ImportError: cannot import name 'step_rate'" in msg
    assert "Shutting down automatically" in msg
    assert str(side / "console.log") in msg


def test_ros2_launch_exits_after_the_stop_do_not_end_the_start(tmp_path):
    side = tmp_path / "bridge"
    side.mkdir()
    (side / "console.log").write_text(
        "[INFO] [launch]: user interrupted with ctrl-c (SIGINT)\n"
        "[INFO] [scenario_world_bridge-1]: process has finished cleanly [pid 10]\n")
    assert crashloop.ended(tmp_path) is None


def test_play_launch_a_successful_exit_ends_the_start(tmp_path):
    d = tmp_path / "simulator" / PL
    (d / "node" / "scensim_isaac_world").mkdir(parents=True)
    (d / "play_launch.log").write_text(
        "2026-11-18T05:10:01.000000Z  INFO play_launch::member_actor::regular_node_actor: "
        "[node:/scensim_isaac_world] Exited successfully\n")
    (d / "node" / "scensim_isaac_world" / "err").write_text("\n".join(KIT_CLEAN) + "\n")
    msg = crashloop.ended(tmp_path)
    assert msg.startswith("stack node scensim_isaac_world exited 0 during the stack's start")
    assert "ImportError" in msg


def test_a_restart_exit_is_left_to_the_limit(tmp_path):
    """The CARLA world server's 75 (restart after a hung CARLA) may happen while CARLA starts."""
    play_launch_side(tmp_path, "scensim_carla_world", [(75,)] * 4, ["CARLA hung"] * 4)
    assert crashloop.ended(tmp_path) is None
    assert crashloop.looping(tmp_path) is None
    with open(tmp_path / "simulator" / PL / "play_launch.log", "a") as f:
        f.write(pl_exit("scensim_carla_world", 1))
    assert crashloop.ended(tmp_path).startswith("stack node scensim_carla_world exited 1 during")


@pytest.mark.parametrize("server", ["scensim_isaac_world_server", "scensim_carla_world"])
def test_wait_started_ends_at_once_on_a_world_server_that_exits_0(tmp_path, monkeypatch, server):
    """A stub world server that ends cleanly while the stack starts: `wait_started` fails in
    seconds, naming it, not at the timeout (issue 052)."""
    proc = tmp_path / "stack" / "proc"
    side = proc / "simulator"
    side.mkdir(parents=True)
    import subprocess
    import sys
    # the stub: a world server that prints its error and ends with 0, as Kit does
    stub = subprocess.run([sys.executable, "-c",
                           "print(\"ImportError: stub world server cannot start\"); "
                           "raise SystemExit(0)"], capture_output=True, text=True)
    assert stub.returncode == 0
    lines = [f"[{server}-1] {x}" for x in stub.stdout.splitlines()]
    lines += [f"[INFO] [{server}-1]: process has finished cleanly [pid 10]"]
    (side / "console.log").write_text("\n".join(lines) + "\n")

    class P:
        def poll(self):
            return None

    started = run.StartedStack(proc=P(), argv=[], log=tmp_path / "console.log", seat=0)
    monkeypatch.setattr(run, "find_stack", lambda project, h: {"state": "starting",
                                                                "proc": str(proc)})
    import time
    t0 = time.time()
    with pytest.raises(run.StackFailed, match=f"{server} exited 0 during the stack's start") as e:
        run.wait_started(started, tmp_path, {"logs": str(tmp_path)}, timeout=60)
    assert time.time() - t0 < 5
    assert "ImportError: stub world server cannot start" in str(e.value)
