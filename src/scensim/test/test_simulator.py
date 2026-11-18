"""scensim.simulator and the keeper's CARLA: the probe, the decision, a started CARLA's life.

No CARLA runs here: a fake CarlaUE4.sh (a shell launcher with a Python child, as the real one
has) answers msgpack-rpc ``version`` on its -carla-rpc-port."""
import json
import os
import socket
import stat
import sys
import textwrap
import threading
import time
from pathlib import Path

import pytest
import yaml

from scensim import assets, project, simulator as sim, stack

# what CARLA answers (Response<std::string>: [[1, "0.9.16"]]), measured 2026-11-16
REPLY = b"\x94\x01\x01\xc0\x91\x92\x01\xa60.9.16"

FAKE_CARLA = textwrap.dedent('''\
    import json, os, signal, socket, sys, time
    from pathlib import Path
    here = Path(__file__).resolve().parent
    port = int(next(a for a in sys.argv[1:] if a.startswith("-carla-rpc-port="))
               .split("=", 1)[1])
    (here / "started.json").write_text(json.dumps({
        "argv": sys.argv[1:], "vk": os.environ.get("VK_ICD_FILENAMES"), "pid": os.getpid(),
        "cwd": os.getcwd()}))
    mode = (here / "mode").read_text().strip() if (here / "mode").exists() else "serve"
    signal.signal(signal.SIGINT, lambda *a: sys.exit(0))
    signal.signal(signal.SIGTERM, lambda *a: sys.exit(0))
    if mode == "exit":
        sys.exit(7)
    if mode == "silent":
        time.sleep(600)
    time.sleep(float((here / "delay").read_text()) if (here / "delay").exists() else 0.3)
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("127.0.0.1", port))
    s.listen(8)
    print("serving", port, flush=True)
    while True:
        c, _ = s.accept()
        try:
            req = c.recv(256)
            maps = here / "maps"
            if b"get_available_maps" in req and maps.exists():
                # Response<std::vector<std::string>>: [1, 2, nil, [[1, [names]]]]
                names = maps.read_text().split()
                body = bytes([0xdc]) + len(names).to_bytes(2, "big") + b"".join(
                    bytes([0xd9, len(n)]) + n.encode() for n in names)
                c.sendall(b"\\x94\\x01\\x02\\xc0\\x91\\x92\\x01" + body)
            else:
                c.sendall(b"\\x94\\x01\\x01\\xc0\\x91\\x92\\x01\\xa60.9.16")
        finally:
            c.close()
''')


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


class Server:
    """A TCP server on a free port answering every connection with ``reply``."""

    def __init__(self, reply):
        self.sock = socket.socket()
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.listen(8)
        self.port = self.sock.getsockname()[1]
        self.reply = reply
        self.stop = False
        self.thread = threading.Thread(target=self._serve, daemon=True)
        self.thread.start()

    def _serve(self):
        self.sock.settimeout(0.2)
        while not self.stop:
            try:
                c, _ = self.sock.accept()
            except OSError:
                continue
            try:
                c.settimeout(1.0)
                try:
                    c.recv(256)
                except OSError:
                    pass
                c.sendall(self.reply)
            except OSError:
                pass
            finally:
                c.close()

    def close(self):
        self.stop = True
        self.thread.join(timeout=2)
        self.sock.close()


@pytest.fixture
def carla_rpc():
    s = Server(REPLY)
    yield s
    s.close()


def fake_root(tmp: Path) -> Path:
    root = tmp / "CARLA_0.9.16"
    root.mkdir()
    (root / "fake_carla.py").write_text(FAKE_CARLA)
    # the real launcher runs the Shipping binary as its child, not with exec
    (root / "CarlaUE4.sh").write_text(
        f'#!/bin/sh\n"{sys.executable}" "$(dirname "$0")/fake_carla.py" "$@"\n')
    (root / "CarlaUE4.sh").chmod(0o755 | stat.S_IXUSR)
    (root / "VERSION").write_text("0.9.16\n")   # what the package ships
    return root


# --- the probe and the decision -------------------------------------------------------------

def test_probe(carla_rpc):
    assert sim.probe("127.0.0.1", carla_rpc.port) == (sim.CARLA, "0.9.16")
    assert sim.probe("127.0.0.1", free_port()) == (sim.FREE, None)
    other = Server(b"HTTP/1.0 400 Bad Request\r\n\r\n")
    try:
        assert sim.probe("127.0.0.1", other.port) == (sim.OTHER, None)
    finally:
        other.close()


def test_decide(tmp_path, carla_rpc):
    root = fake_root(tmp_path)
    never = lambda *a: pytest.fail("probed")  # noqa: E731
    # external: nothing probed, nothing started
    assert sim.decide({"root": str(root), "port": 1}, "external", never)["owner"] == sim.EXTERNAL
    # auto without a root: a CARLA serving is used; nothing serving is refused at once (G39)
    d = sim.decide({"root": "", "host": "127.0.0.1", "port": carla_rpc.port}, "auto")
    assert d["owner"] == sim.EXTERNAL and d["version"] == "0.9.16"
    with pytest.raises(sim.SimulatorRefused, match="carla.root is not set, so the stack"):
        sim.decide({"root": "", "port": free_port()}, "auto")
    # auto with a remote CARLA: its business
    assert sim.decide({"root": str(root), "host": "carla.example.invalid", "port": 1},
                      None, never)["owner"] == sim.EXTERNAL
    # auto, nothing on the port: started, with the launcher's arguments
    port = free_port()
    d = sim.decide({"root": str(root), "host": "localhost", "port": port, "ready_timeout": 42})
    assert d["owner"] == sim.STARTED and d["mode"] == "auto" and d["ready_timeout"] == 42
    # the version the stack starts: its install's VERSION (roadmap 083: the body follows it)
    assert d["version"] == "0.9.16" and d["version_from"] == str(root / "VERSION")
    assert d["argv"] == [str(root / "CarlaUE4.sh"), f"-carla-rpc-port={port}", "-nosound",
                         "-RenderOffScreen", "-quality-level=Epic"]
    # auto, CARLA answering: that one, external
    d = sim.decide({"root": str(root), "host": "127.0.0.1", "port": carla_rpc.port})
    assert d["owner"] == sim.EXTERNAL and d["version"] == "0.9.16"
    # the carla.start host setting is the default mode
    with pytest.raises(sim.SimulatorRefused, match="already serves"):
        sim.decide({"root": str(root), "host": "127.0.0.1", "port": carla_rpc.port,
                    "start": "start"})


# CARLA 1.0's packaged VERSION file, as the release ships it: git hashes, no version (issue:
# the stack took the whole text for one, and the cars fell back to the 0.9.16 body)
CARLA_1_0_VERSION = (
    "Carla git hash:         3ea166118d2c3c094b13b15c5b7a66a9a279998d\n"
    "Content git hash:       3ea166118d2c3c094b13b15c5b7a66a9a279998d\n"
    "UnrealEngine git hash:  \n")


def carla_1_0_root(tmp: Path) -> Path:
    root = tmp / "Carla-1.0-Linux-Shipping"
    (root / "CarlaUnreal" / "Binaries" / "Linux").mkdir(parents=True)
    (root / "CarlaUnreal.sh").write_text("#!/bin/sh\n")
    (root / "VERSION").write_text(CARLA_1_0_VERSION)
    return root


def test_a_carla_1_0_package_is_1_0_by_its_layout(tmp_path):
    """083: 1.0's VERSION holds git hashes, so its version comes from its layout
    (CarlaUnreal.sh), never the text, and never nothing (the 0.9.16 body)."""
    root = carla_1_0_root(tmp_path)
    assert sim.root_version_from(root) == ("1.0", str(root / "CarlaUnreal.sh"))
    from scensim import vehicles
    assert vehicles.carla_body(sim.root_version(root)) == "carla-1.0"
    d = sim.decide({"root": str(root), "port": free_port()}, "start",
                   lambda *a: (sim.FREE, None))
    assert d["owner"] == sim.STARTED and d["version"] == "1.0"
    engine = root / "CarlaUnreal" / "Binaries" / "Linux" / "CarlaUnreal-Linux-Shipping"
    assert d["argv"][0] == str(root / "CarlaUnreal.sh")   # no engine: the script
    engine.write_text("")
    assert sim.carla_argv(root, 2400)[:2] == [str(engine), "CarlaUnreal"]
    # a VERSION that reads as one wins over the layout (a source build's 1.0-<commit> too)
    (root / "VERSION").write_text("1.0-3ea1661\n")
    assert sim.root_version(root) == "1.0-3ea1661"
    # 0.9.x's layout says no version: without its VERSION the root says nothing
    old = fake_root(tmp_path)
    (old / "VERSION").unlink()
    assert sim.root_version_from(old) == (None, None)


def test_a_started_carla_whose_version_differs_from_the_plan_is_refused(tmp_path):
    """083: the probe after start checks the root's version; a disagreement, or a root that
    said none, ends the start naming both (the cars' body was chosen from the plan)."""
    rec = {"root": "/opt/carla", "version": "1.0", "version_from": "/opt/carla/CarlaUnreal.sh"}
    assert sim.version_problem(rec, "1.0") is None
    assert sim.version_problem(rec, "1.0.0") is None
    assert sim.version_problem(rec, "1.0-3ea1661") is None
    err = sim.version_problem(rec, "0.9.16")
    assert "answers version 0.9.16" in err and "planned for 1.0" in err
    assert "/opt/carla/CarlaUnreal.sh" in err
    assert sim.version_problem({**rec, "version": "0.9.16"}, "0.9.15") is not None
    err = sim.version_problem({"root": "/opt/carla", "version": None}, "0.9.16")
    assert "says no version" in err and "/opt/carla/VERSION" in err
    assert sim.version_problem(rec, None) is None   # an answer without a version: not judged


def pack_maps(names, index=1):
    """CARLA's reply to get_available_maps: [1, 2, nil, [[index, [names]]]] (index 0: an
    error, whose ResponseError is an array of its text)."""
    body = bytes([0x90 | len(names)]) + b"".join(bytes([0xa0 | len(n)]) + n.encode()
                                                   for n in names)
    return b"\x94\x01\x02\xc0\x91\x92" + bytes([index]) + body


def test_available_maps_reads_carlas_reply():
    maps = ["/Game/Carla/Maps/Town01_Opt", "/Game/Carla/Maps/Town10HD_Opt"]
    srv = Server(pack_maps(maps))
    try:
        assert sim.available_maps("127.0.0.1", srv.port) == maps
    finally:
        srv.close()
    for reply in (pack_maps(["no such function"], index=0), REPLY,
                  b"\x94\x01\x02\xa5oops!\xc0"):
        srv = Server(reply)
        try:
            assert sim.available_maps("127.0.0.1", srv.port, timeout=2) is None, reply
        finally:
            srv.close()
    assert sim.available_maps("127.0.0.1", free_port()) is None


def test_a_town_a_serving_carla_lacks_is_named():
    """083: 1.0's towns are in content packs and named only _Opt; 0.9.16's by name."""
    one = ["/Game/Carla/Maps/Town01_Opt", "/Game/Carla/Maps/Town10HD_Opt"]
    alias = {"Town10": "Town10HD"}
    assert sim.served_town_problems(["Town01", "Town10"], one, alias) == []
    assert sim.served_town_problems(["Town01", "Town03"], one, alias) == [("Town03", "Town03")]
    old = ["/Game/Carla/Maps/Town01", "/Game/Carla/Maps/Town03_Opt"]
    assert sim.served_town_problems(["Town01", "Town03", "NTU"], old, alias) == [("NTU", "NTU")]


def test_keeper_ends_the_start_when_carla_has_no_town_for_a_map(where, maps_hub):
    """083: once a started CARLA serves, a map whose town it cannot load ends the start,
    named, before any pilot; one it can is served as before."""
    root = fake_root(where["tmp"])
    (root / "maps").write_text("/Game/Carla/Maps/Town02_Opt /Game/Carla/Maps/Town10HD_Opt\n")
    d = make(where, {"root": str(root), "port": free_port(), "ready_timeout": 60})
    p = stack.plan(project_arg=str(d), map_name="Town01")
    stack.write_plan(p)
    assert stack.keep(p.stack_dir, poll=0.01, once=True, pilot_command=fake_pilot) == 2
    rec = json.loads(p.stack_json.read_text())
    assert "maps: Town01" in rec["error"] and "no town Town01 (nor Town01_Opt)" in rec["error"]
    assert "AdditionalMaps" in rec["error"]
    assert "pilot.start" not in [e["event"] for e in events(p)]
    assert gone(json.loads((root / "started.json").read_text())["pid"])


def test_decide_refusals(tmp_path):
    root = fake_root(tmp_path)
    other = Server(b"nope")
    try:
        for mode in ("auto", "start"):
            with pytest.raises(sim.SimulatorRefused, match="not CARLA's RPC server"):
                sim.decide({"root": str(root), "host": "127.0.0.1", "port": other.port}, mode)
    finally:
        other.close()
    port = free_port()
    with pytest.raises(sim.SimulatorRefused, match="carla.root is not set"):
        sim.decide({"root": "", "port": port}, "start")
    with pytest.raises(sim.SimulatorRefused, match="no CarlaUnreal.sh or CarlaUE4.sh"):
        sim.decide({"root": str(tmp_path), "port": port}, "start")
    with pytest.raises(sim.SimulatorRefused, match="not this host"):
        sim.decide({"root": str(root), "host": "carla.example.invalid", "port": port}, "start",
                   lambda *a: (sim.FREE, None))
    with pytest.raises(sim.SimulatorRefused, match="one of auto, start, external"):
        sim.decide({}, "sometimes")


def test_vulkan_pinned_only_when_unset(tmp_path, monkeypatch):
    icd = tmp_path / "nvidia_icd.json"
    icd.write_text("{}")
    monkeypatch.setattr(sim, "NVIDIA_ICD", str(icd))
    assert sim.carla_env({})["VK_ICD_FILENAMES"] == str(icd)
    assert sim.carla_env({"VK_ICD_FILENAMES": "/mine.json"})["VK_ICD_FILENAMES"] == "/mine.json"
    monkeypatch.setattr(sim, "NVIDIA_ICD", str(tmp_path / "none.json"))
    assert "VK_ICD_FILENAMES" not in sim.carla_env({})


# --- the stack's CARLA ----------------------------------------------------------------------

@pytest.fixture
def where(host):
    tmp = host["tmp"]
    (tmp / "system" / "host.yaml").write_text(yaml.safe_dump(
        {"seat_locks": str(tmp / "locks"), "logs": str(tmp / "logs"), "seat": 2}))
    return host


def make(where, carla):
    d = where["tmp"] / "proj"
    project.new_project(d, "multi-pilot")
    assets.fetch(d, out=lambda s: None)
    (d / ".scensim").mkdir(exist_ok=True)
    (d / ".scensim" / "local.yaml").write_text(yaml.safe_dump({"carla": carla}))
    return d


def fake_pilot(rec, p, entry, log_dir):
    return [sys.executable, "-c", "import time; time.sleep(60)"]


def events(p):
    return [json.loads(x) for x in (p.stack_dir / "events.jsonl").read_text().splitlines()]


def gone(pid, within=10.0):
    deadline = time.time() + within
    while time.time() < deadline:
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return True
        try:   # a zombie of ours is gone too
            with open(f"/proc/{pid}/stat") as f:
                if f.read().split()[2] == "Z":
                    return True
        except OSError:
            return True
        time.sleep(0.05)
    return False


def test_keeper_starts_waits_for_and_stops_carla(where, maps_hub, monkeypatch):
    monkeypatch.delenv("VK_ICD_FILENAMES", raising=False)
    icd = where["tmp"] / "nvidia_icd.json"
    icd.write_text("{}")
    monkeypatch.setattr(sim, "NVIDIA_ICD", str(icd))
    root = fake_root(where["tmp"])
    port = free_port()
    d = make(where, {"root": str(root), "port": port, "ready_timeout": 20})
    p = stack.plan(project_arg=str(d), map_name="Town01")
    assert p.record["simulator_start"] == "started"
    assert p.record["carla"]["owner"] == "started" and p.record["carla"]["port"] == port
    assert p.launch_args["carla_port"] == str(port)
    for x in p.record["pilots"]:
        x["ready"] = ["true"]
    stack.write_plan(p)
    assert stack.keep(p.stack_dir, poll=0.01, once=True, pilot_command=fake_pilot) == 0
    started = json.loads((root / "started.json").read_text())
    assert started["argv"] == [f"-carla-rpc-port={port}", "-nosound", "-RenderOffScreen",
                               "-quality-level=Epic"]
    assert started["vk"] == str(icd) and started["cwd"] == str(root)
    rec = json.loads(p.stack_json.read_text())
    c = rec["carla"]
    assert c["state"] == "stopped" and c["ready_at"] and c["version"] == "0.9.16"
    log = p.stack_dir / "proc" / "simulator" / "carla.log"
    assert c["log"] == str(log) and "serving" in log.read_text()
    # the pilots start once CARLA serves, and the stack is ready after both
    names = [e["event"] for e in events(p)]
    assert names.index("carla.ready") < names.index("pilot.start") < names.index("stack.ready")
    assert names[-1] == "carla.stop"
    # the launcher and its child are gone; the port is free again
    assert gone(started["pid"])
    assert sim.probe("127.0.0.1", port)[0] == sim.FREE


def test_keeper_fails_when_carla_never_serves(where, maps_hub):
    root = fake_root(where["tmp"])
    (root / "mode").write_text("silent")
    d = make(where, {"root": str(root), "port": free_port(), "ready_timeout": 1.5})
    p = stack.plan(project_arg=str(d), map_name="Town01")
    stack.write_plan(p)
    t0 = time.time()
    assert stack.keep(p.stack_dir, poll=0.01, once=True, pilot_command=fake_pilot) == 2
    assert time.time() - t0 < 30
    rec = json.loads(p.stack_json.read_text())
    assert rec["state"] == "stopped" and "did not serve RPC" in rec["error"]
    assert "carla.ready_timeout" in rec["error"]
    assert "pilot.start" not in [e["event"] for e in events(p)]
    assert gone(json.loads((root / "started.json").read_text())["pid"])


def test_keeper_ends_the_start_when_carla_answers_another_version(where, maps_hub):
    """083: a root that says 1.0 serving 0.9.16 is refused once it serves, naming both, and
    no pilot starts on the wrong body."""
    root = fake_root(where["tmp"])
    (root / "VERSION").write_text("1.0\n")
    d = make(where, {"root": str(root), "port": free_port(), "ready_timeout": 60})
    p = stack.plan(project_arg=str(d), map_name="Town01")
    stack.write_plan(p)
    assert stack.keep(p.stack_dir, poll=0.01, once=True, pilot_command=fake_pilot) == 2
    rec = json.loads(p.stack_json.read_text())
    assert "answers version 0.9.16" in rec["error"] and "planned for 1.0" in rec["error"]
    assert "pilot.start" not in [e["event"] for e in events(p)]
    assert gone(json.loads((root / "started.json").read_text())["pid"])


def test_keeper_fails_when_carla_exits_at_start(where, maps_hub):
    root = fake_root(where["tmp"])
    (root / "mode").write_text("exit")
    d = make(where, {"root": str(root), "port": free_port(), "ready_timeout": 20})
    p = stack.plan(project_arg=str(d), map_name="Town01")
    stack.write_plan(p)
    assert stack.keep(p.stack_dir, poll=0.01, once=True, pilot_command=fake_pilot) == 2
    rec = json.loads(p.stack_json.read_text())
    assert "CARLA exited (7) before it served RPC" in rec["error"]


def test_external_carla_is_used_and_never_stopped(where, maps_hub, carla_rpc):
    root = fake_root(where["tmp"])
    d = make(where, {"root": str(root), "host": "127.0.0.1", "port": carla_rpc.port})
    p = stack.plan(project_arg=str(d), map_name="Town01")
    assert p.record["simulator_start"] == "external"
    assert p.record["carla"]["version"] == "0.9.16"
    for x in p.record["pilots"]:
        x["ready"] = ["true"]
    stack.write_plan(p)
    assert stack.keep(p.stack_dir, poll=0.01, once=True, pilot_command=fake_pilot) == 0
    assert not (root / "started.json").exists()
    assert not any(e["event"].startswith("carla.") for e in events(p))
    assert sim.probe("127.0.0.1", carla_rpc.port)[0] == sim.CARLA
    # simulator_start:=start against it is refused at the plan, before anything starts
    with pytest.raises(stack.StackRefused, match="already serves"):
        stack.plan(project_arg=str(d), map_name="Town01", simulator_start="start")
    with pytest.raises(stack.StackRefused, match="simulator_start 'always'"):
        stack.plan(project_arg=str(d), simulator_start="always")


def test_a_started_carla_that_dies_is_started_again(tmp_path):
    root = fake_root(tmp_path)
    port = free_port()
    rec = sim.decide({"root": str(root), "port": port, "ready_timeout": 20}, "start")
    seen = []
    c = sim.Carla(rec, tmp_path / "proc", respawn_delay=0.1,
                  events=lambda n, **kw: seen.append(n))

    def until(cond, within=15.0):
        deadline = time.time() + within
        while time.time() < deadline and not cond():
            c.poll()
            time.sleep(0.02)
        assert cond()

    c.start()
    try:
        until(lambda: c.ready)
        first = json.loads((root / "started.json").read_text())["pid"]
        os.kill(first, 9)                      # CARLA crashes; its launcher ends with it
        until(lambda: rec["state"] == sim.STARTING and rec["restarts"] == 1)
        until(lambda: c.ready)
        assert json.loads((root / "started.json").read_text())["pid"] != first
        assert seen == ["carla.start", "carla.ready", "carla.exit", "carla.start", "carla.ready"]
    finally:
        c.stop()
    assert rec["state"] == sim.STOPPED and sim.probe("127.0.0.1", port)[0] == sim.FREE


def test_isaac_records_its_world_server(where, maps_hub):
    d = where["tmp"] / "proj"
    project.new_project(d, "isaac")
    assets.fetch(d, out=lambda s: None)
    p = stack.plan(project_arg=str(d))
    assert p.record["simulator_start"] == "world_server" and p.record["carla"] is None


def test_carla_argv_runs_the_engine_itself(tmp_path):
    root = tmp_path / "CARLA_0.9.16"
    (root / "CarlaUE4" / "Binaries" / "Linux").mkdir(parents=True)
    (root / "CarlaUE4.sh").write_text("#!/bin/sh\n")
    assert sim.carla_argv(root, 2000)[0].endswith("CarlaUE4.sh")   # no engine: the script
    (root / sim.ENGINE).write_text("")
    argv = sim.carla_argv(root, 2010)
    assert argv[:2] == [str(root / sim.ENGINE), "CarlaUE4"]
    assert "-carla-rpc-port=2010" in argv


def test_a_started_carla_dies_with_a_killed_keeper(tmp_path):
    """The keeper SIGKILLed: its CARLA still gets SIGTERM (the parent-death signal)."""
    import subprocess as sp
    pidfile = tmp_path / "carla.pid"
    script = tmp_path / "keeper.py"
    script.write_text(textwrap.dedent(f"""
        import subprocess, sys, time
        from scensim import simulator
        subprocess.Popen([sys.executable, "-c",
                          "import os, time; open({str(pidfile)!r}, 'w').write(str(os.getpid())); time.sleep(120)"],
                         start_new_session=True, preexec_fn=simulator._die_with_parent)
        time.sleep(120)
    """))
    root = str(Path(sim.__file__).resolve().parents[1])
    keeper = sp.Popen([sys.executable, str(script)],
                      env=dict(os.environ, PYTHONPATH=root + os.pathsep + os.environ.get("PYTHONPATH", "")))
    deadline = time.time() + 15
    while not (pidfile.exists() and pidfile.read_text()) and time.time() < deadline:
        time.sleep(0.05)
    assert pidfile.exists(), "the stand-in CARLA never started"
    child = int(pidfile.read_text())
    keeper.kill()
    keeper.wait()
    deadline = time.time() + 10
    while time.time() < deadline:
        try:
            os.kill(child, 0)
        except ProcessLookupError:
            return
        time.sleep(0.05)
    os.kill(child, 9)
    raise AssertionError("the started CARLA outlived its SIGKILLed keeper")


def test_a_carla_deaf_to_sigterm_is_killed_after_its_keeper(tmp_path):
    """UE's graceful exit can hang: the reaper SIGKILLs CARLA after the grace."""
    import subprocess as sp
    pidfile = tmp_path / "carla.pid"
    script = tmp_path / "keeper.py"
    script.write_text(textwrap.dedent(f"""
        import subprocess, sys, time
        from scensim import simulator
        p = subprocess.Popen([sys.executable, "-c",
                              "import os, signal, time; signal.signal(signal.SIGTERM, signal.SIG_IGN); "
                              "open({str(pidfile)!r}, 'w').write(str(os.getpid())); time.sleep(120)"],
                             start_new_session=True, preexec_fn=simulator._die_with_parent)
        simulator.start_reaper(p.pid, grace=1.0)
        time.sleep(120)
    """))
    root = str(Path(sim.__file__).resolve().parents[1])
    keeper = sp.Popen([sys.executable, str(script)],
                      env=dict(os.environ, PYTHONPATH=root + os.pathsep + os.environ.get("PYTHONPATH", "")))
    deadline = time.time() + 15
    while not (pidfile.exists() and pidfile.read_text()) and time.time() < deadline:
        time.sleep(0.05)
    assert pidfile.exists(), "the stand-in CARLA never started"
    child = int(pidfile.read_text())
    time.sleep(1.0)   # the reaper is up
    keeper.kill()
    keeper.wait()
    deadline = time.time() + 15
    while time.time() < deadline:
        try:
            os.kill(child, 0)
        except ProcessLookupError:
            return
        time.sleep(0.05)
    os.kill(child, 9)
    raise AssertionError("a CARLA that ignores SIGTERM outlived its SIGKILLed keeper")


def test_the_reaper_leaves_with_a_carla_its_keeper_stopped(tmp_path):
    import subprocess as sp
    carla = sp.Popen([sys.executable, "-c", "import time; time.sleep(60)"])
    reaper = sim.start_reaper(carla.pid, grace=1.0)
    time.sleep(0.5)
    carla.terminate()
    carla.wait()
    assert reaper.wait(timeout=10) == 0


def test_a_map_this_hosts_carla_has_no_town_for_is_named(tmp_path):
    """G13: a town installed into the CARLA binary is a host fact."""
    content = tmp_path / "CARLA_0.9.16" / "CarlaUE4" / "Content"
    (content / "Carla" / "Maps").mkdir(parents=True)
    (content / "Carla" / "Maps" / "Town01.umap").write_text("")
    (content / "Carla" / "Maps" / "Town10HD.umap").write_text("")
    carla = {"root": str(tmp_path / "CARLA_0.9.16")}
    alias = {"Town10": "Town10HD"}
    assert sim.town_problems(["Town01", "Town10"], carla, alias) == []
    [(m, town, dirs)] = sim.town_problems(["Town01", "NTU"], carla, alias)
    assert (m, town) == ("NTU", "NTU") and dirs == [content]
    pkgs = tmp_path / "packages"
    (pkgs / "NTU" / "Maps" / "NTU").mkdir(parents=True)
    (pkgs / "NTU" / "Maps" / "NTU" / "NTU.umap").write_text("")
    assert sim.town_problems(["NTU"], {"map_packages": [str(pkgs)]}, alias) == []
    # a CARLA elsewhere (no root, no packages): not checked
    assert sim.town_problems(["NTU"], {}, alias) == []
