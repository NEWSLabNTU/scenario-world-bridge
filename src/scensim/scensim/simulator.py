"""The simulator under a stack: CARLA started by the stack, or one running already.

docs/design/projects-and-pilots.md, "The stack". On CARLA ``simulator_start`` (the launch's
``simulator_start:=``, else the host settings' ``carla.start``, else ``auto``) decides:

- ``external``: never start CARLA, never stop it; the stack uses whatever serves
  ``carla.host:carla.port`` (the world server waits for it, as before).
- ``start``: start CARLA from ``carla.root``; refused when something already holds the port
  (a second CARLA on a held port crashes), when ``carla.root`` has no ``CarlaUE4.sh``, or
  when ``carla.host`` is not this host.
- ``auto``: ``start`` when ``carla.root`` is set, ``carla.host`` is this host and nothing
  answers on the port; else ``external``. A port that answers, but not as CARLA's RPC
  server, is refused either way, and so is a local port where nothing answers when there is
  no ``carla.root`` to start CARLA from (roadmap 077, G39: the stack used to wait for one
  until its readiness timeout, 30 min, without a word); ``external`` still waits for one.

"Answers as CARLA" is one msgpack-rpc call, ``version`` (what ``carla.Client.
get_server_version`` sends), over a plain socket: no carla module, nothing changed on the
server, no tick.

A started CARLA is the stack's: the keeper (scensim.stack) starts it in its own session
(``CarlaUE4.sh -carla-rpc-port=<port> -nosound -RenderOffScreen -quality-level=Epic``; 1.0's
``CarlaUnreal.sh``; the engine itself where the install has it,
``VK_ICD_FILENAMES`` pinned to the NVIDIA ICD when unset), logs it to
``<stack>/proc/simulator/carla.log``, waits for its RPC (``carla.ready_timeout``, 300 s)
before it starts pilots or calls the stack ready, starts it again when it dies after it was
ready, and stops its process group on the way out (SIGINT, SIGTERM, SIGKILL; by pid, never by
name). On Isaac the world server starts Isaac itself; the stack only records that.

The version a started CARLA is, which picks the body every car drives (roadmap 083), is its
install's: ``VERSION`` when it reads as a version, else the layout (``CarlaUnreal.sh``: 1.0;
1.0's package writes git hashes in VERSION); the probe checks it once CARLA serves, and a
disagreement, or an install that said none, ends the start naming both.
"""
import os
import re
import shlex
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

MODES = ("auto", "start", "external")
STARTED, EXTERNAL, WORLD_SERVER = "started", "external", "world_server"
STARTING, READY, FAILED, STOPPED = "starting", "ready", "failed", "stopped"
FREE, CARLA, OTHER = "free", "carla", "other"
LAUNCHER = "CarlaUE4.sh"
# each install layout: its launcher script, its engine and the engine's project argument, and
# the version the layout alone says (CARLA 1.0 ships CarlaUnreal.sh; 0.9.x CarlaUE4.sh, which
# does not say which 0.9.x)
LAYOUTS = (
    ("CarlaUnreal.sh", Path("CarlaUnreal") / "Binaries" / "Linux" / "CarlaUnreal-Linux-Shipping",
     "CarlaUnreal", "1.0"),
    (LAUNCHER, Path("CarlaUE4") / "Binaries" / "Linux" / "CarlaUE4-Linux-Shipping",
     "CarlaUE4", None),
)
VERSION_LINE = re.compile(r"^\d+\.\d+(\.\d+)*(-[\w.]+)?$")
NVIDIA_ICD = "/usr/share/vulkan/icd.d/nvidia_icd.json"
DEFAULT_READY_TIMEOUT = 300.0
QUALITY = "Epic"   # Low segfaults on a town change (CLAUDE.md, carla-simulator/carla#4940)
RESPAWN_DELAY = 5.0
PROBE_EVERY = 1.0
LOG = Path("simulator") / "carla.log"

# msgpack-rpc request [0, 1, "version", []]
# [0, 1, "version", [[false]]]: LibCarla's client sends a Metadata ([asynchronous: false]) as
# a call's first argument; without it CARLA's server refuses the call ("invalid number of
# arguments"), whose error reply still says the server is CARLA's, but not its version
# (roadmap 077, G45: "CARLA serves RPC (None)"). Measured on CARLA 0.9.15 and 0.9.16.
_VERSION_REQUEST = (bytes([0x94, 0x00, 0x01, 0xa7]) + b"version"
                    + bytes([0x91, 0x91, 0xc2]))


# [0, 2, "get_available_maps", [[false]]]: what carla.Client.get_available_maps sends
_MAPS_REQUEST = (bytes([0x94, 0x00, 0x02, 0xa0 | len("get_available_maps")])
                 + b"get_available_maps" + bytes([0x91, 0x91, 0xc2]))


class _Short(Exception):
    """The reply is not all here yet."""


def _unpack(b: bytes, i: int):
    """``(value, next index)`` of the msgpack object at ``b[i]``: nil, booleans, integers,
    strings, arrays and maps, what a CARLA reply holds; _Short when it runs past the end."""
    def need(n):
        if i + n > len(b):
            raise _Short()
    need(1)
    t = b[i]
    i += 1
    if t <= 0x7f:
        return t, i
    if t >= 0xe0:
        return t - 0x100, i
    if t == 0xc0:
        return None, i
    if t in (0xc2, 0xc3):
        return t == 0xc3, i
    sizes = {0xcc: 1, 0xcd: 2, 0xce: 4, 0xcf: 8, 0xd0: 1, 0xd1: 2, 0xd2: 4, 0xd3: 8}
    if t in sizes:
        n = sizes[t]
        need(n)
        return int.from_bytes(b[i:i + n], "big", signed=t >= 0xd0), i + n
    if 0xa0 <= t <= 0xbf or t in (0xd9, 0xda, 0xdb):
        if t <= 0xbf:
            n = t & 0x1f
        else:
            w = {0xd9: 1, 0xda: 2, 0xdb: 4}[t]
            need(w)
            n, i = int.from_bytes(b[i:i + w], "big"), i + w
        need(n)
        return b[i:i + n].decode("utf-8", "replace"), i + n
    if 0x90 <= t <= 0x9f or t in (0xdc, 0xdd) or 0x80 <= t <= 0x8f or t in (0xde, 0xdf):
        is_map = 0x80 <= t <= 0x8f or t in (0xde, 0xdf)
        if t in (0xdc, 0xdd, 0xde, 0xdf):
            w = 2 if t in (0xdc, 0xde) else 4
            need(w)
            n, i = int.from_bytes(b[i:i + w], "big"), i + w
        else:
            n = t & 0x0f
        items = []
        for _ in range(n * (2 if is_map else 1)):
            v, i = _unpack(b, i)
            items.append(v)
        return (dict(zip(items[::2], items[1::2])) if is_map else items), i
    raise ValueError(f"msgpack type 0x{t:02x}")


def available_maps(host, port, timeout=10.0):
    """The maps the CARLA on ``host:port`` can load (``get_available_maps``: ``/Game/Carla/
    Maps/Town01``, 1.0's ``.../Town01_Opt``), or None when it cannot tell (not serving, an
    error reply, a reply it cannot read)."""
    try:
        s = socket.create_connection((host, int(port)), timeout=timeout)
    except OSError:
        return None
    try:
        s.settimeout(timeout)
        s.sendall(_MAPS_REQUEST)
        data = b""
        while True:
            chunk = s.recv(65536)
            if not chunk:
                return None
            data += chunk
            try:
                reply, _ = _unpack(data, 0)
                break
            except _Short:
                continue
        if not (isinstance(reply, list) and len(reply) == 4 and reply[0] == 1
                and reply[2] is None):
            return None
        result = reply[3]
        # CARLA's Response<T> is a variant [index, value], packed in a one-item array:
        # index 0 is its ResponseError (whose text would read as a list of strings)
        if (isinstance(result, list) and len(result) == 1 and isinstance(result[0], list)
                and len(result[0]) == 2 and isinstance(result[0][0], int)):
            if result[0][0] == 0:
                return None
            result = result[0][1]
        return result if isinstance(result, list) and all(
            isinstance(x, str) for x in result) else None
    except (OSError, ValueError):
        return None
    finally:
        s.close()


def served_town_problems(maps, available, alias=None) -> list:
    """``(map, town)`` for each project map whose town is not among the maps a serving CARLA
    can load (``available``, available_maps): by its last path segment, the town or its
    ``_Opt`` name (CARLA 1.0 has only those)."""
    alias = town_alias() if alias is None else alias
    have = {str(a).rstrip("/").rsplit("/", 1)[-1] for a in available or []}
    return [(m, alias.get(m, m)) for m in maps or []
            if isinstance(m, str) and m and not ({alias.get(m, m), f"{alias.get(m, m)}_Opt"}
                                                 & have)]


class SimulatorRefused(RuntimeError):
    """The simulator cannot be started or used as asked; the message says why."""


def probe(host, port, timeout=3.0):
    """``(FREE | CARLA | OTHER, version or None)`` for ``host:port``."""
    try:
        s = socket.create_connection((host, int(port)), timeout=timeout)
    except socket.timeout:
        return OTHER, None       # something swallows the connection: not ours to start over
    except OSError:
        return FREE, None        # refused, unreachable: nothing answers
    try:
        s.settimeout(timeout)
        s.sendall(_VERSION_REQUEST)
        data = b""
        while len(data) < 4:
            chunk = s.recv(256)
            if not chunk:
                break
            data += chunk
        # a reply [1, 1, error, result]: fixarray 4, type 1, msgid 1
        if len(data) >= 3 and data[0] == 0x94 and data[1] == 0x01 and data[2] == 0x01:
            if len(data) < 16:
                try:
                    data += s.recv(256)
                except OSError:
                    pass
            return CARLA, _version(data)
        return OTHER, None
    except OSError:
        return OTHER, None
    finally:
        s.close()


def _version(data: bytes):
    """The result string of a msgpack-rpc reply whose error is nil, else None: a string, or
    CARLA's Response wrapping it (``[[<index>, "0.9.16"]]``)."""
    if len(data) < 5 or data[3] != 0xc0:
        return None
    i = 4
    while i < len(data) and (0x90 <= data[i] <= 0x9f or data[i] <= 0x7f):
        i += 1   # array headers and the variant's index (positive fixints)
    if i >= len(data):
        return None
    b, rest = data[i], data[i + 1:]
    if 0xa0 <= b <= 0xbf:
        n = b & 0x1f
    elif b == 0xd9 and rest:
        n, rest = rest[0], rest[1:]
    else:
        return None
    return rest[:n].decode("utf-8", "replace") if len(rest) >= n else None


def is_local(host) -> bool:
    host = str(host or "localhost")
    if host in ("localhost", "::1", socket.gethostname(), socket.getfqdn()) \
            or host.startswith("127."):
        return True
    try:
        addr = socket.gethostbyname(host)
    except OSError:
        return False
    return addr.startswith("127.")


def layout(root):
    """The install layout at ``root`` (a LAYOUTS row, by its launcher script), else None."""
    root = Path(str(root)).expanduser()
    for row in LAYOUTS:
        if (root / row[0]).is_file():
            return row
    return None


def launcher(root) -> Path:
    """The install's launcher script (CarlaUnreal.sh or CarlaUE4.sh); CarlaUE4.sh's path when
    it has neither."""
    row = layout(root) or LAYOUTS[-1]
    return Path(str(root)).expanduser() / row[0]


ENGINE = LAYOUTS[-1][1]


def carla_argv(root, port, quality=QUALITY) -> list:
    """CARLA's engine itself where the install has it (what CarlaUE4.sh runs, but as the
    keeper's own child, so the parent-death signal reaches it), else the launcher script."""
    root = Path(str(root)).expanduser()
    flags = [f"-carla-rpc-port={int(port)}", "-nosound", "-RenderOffScreen",
             f"-quality-level={quality}"]
    script, engine, project, _v = layout(root) or LAYOUTS[-1]
    if (root / engine).is_file():
        return [str(root / engine), project, *flags]
    return [str(root / script), *flags]


def _die_with_parent():
    """SIGTERM to this process when its parent (the keeper) dies, even by SIGKILL. The
    signal follows the thread that forked, so start CARLA from the keeper's main thread."""
    try:
        import ctypes
        ctypes.CDLL("libc.so.6", use_errno=True).prctl(1, signal.SIGTERM)  # PR_SET_PDEATHSIG
    except OSError:
        pass


def carla_env(base=None) -> dict:
    env = dict(os.environ if base is None else base)
    if not env.get("VK_ICD_FILENAMES") and os.path.isfile(NVIDIA_ICD):
        env["VK_ICD_FILENAMES"] = NVIDIA_ICD
    return env


def decide(carla: dict, mode=None, probe_fn=probe) -> dict:
    """The stack's CARLA as stack.json records it (``carla``), or SimulatorRefused.

    ``owner`` is ``started`` (the stack starts it) or ``external``."""
    carla = dict(carla or {})
    mode = (mode or carla.get("start") or "auto")
    if mode not in MODES:
        raise SimulatorRefused(f"simulator_start {mode!r}: one of {', '.join(MODES)}")
    host = str(carla.get("host") or "localhost")
    port = int(carla.get("port") or 2000)
    root = str(carla.get("root") or "")
    try:
        ready_timeout = float(carla.get("ready_timeout") or DEFAULT_READY_TIMEOUT)
    except (TypeError, ValueError):
        raise SimulatorRefused(f"carla.ready_timeout {carla.get('ready_timeout')!r}: "
                               "seconds") from None
    out = {"mode": mode, "owner": EXTERNAL, "host": host, "port": port, "root": root or None,
           "ready_timeout": ready_timeout, "version": None, "version_from": None,
           "state": None, "pid": None,
           "log": None, "argv": None, "started_at": None, "ready_at": None, "restarts": 0,
           "error": None}
    if mode == "external":
        return out
    if mode == "auto" and not is_local(host):
        return out       # a remote CARLA: whatever serves there
    state, version = probe_fn(host, port)
    where = f"{host}:{port}"
    if mode == "auto" and not root and state == FREE:
        raise SimulatorRefused(
            f"nothing serves CARLA on {where} and carla.root is not set, so the stack cannot "
            "start one: start CARLA there, set carla.root (~/.config/scensim/host.yaml, or "
            "<project>/.scensim/local.yaml: carla: {root: <CARLA_0.9.16 directory>}) and the "
            "stack starts it, or simulator_start:=external to wait for one")
    if state == OTHER:
        raise SimulatorRefused(
            f"{where} is taken by something that is not CARLA's RPC server; free it, or set "
            "carla.port (host settings) to CARLA's")
    if state == CARLA:
        if mode == "start":
            raise SimulatorRefused(
                f"CARLA {version or ''} already serves {where}; simulator_start:=external (or "
                "auto) uses it, a second CARLA on a held port crashes".replace("  ", " "))
        out["version"] = version
        return out
    if not is_local(host):
        raise SimulatorRefused(f"simulator_start:=start but carla.host {host} is not this "
                               "host: start CARLA there, or set carla.host: localhost")
    if not root:
        raise SimulatorRefused("simulator_start:=start but carla.root is not set (host "
                               "settings: carla: {root: <CARLA_0.9.16 directory>})")
    if layout(root) is None:
        raise SimulatorRefused(f"carla.root {root}: no {' or '.join(r[0] for r in LAYOUTS)} "
                               "there")
    version, source = root_version_from(root)
    out.update(owner=STARTED, argv=carla_argv(root, port), version=version,
               version_from=source)
    return out


def root_version_from(root):
    """``(version, source)`` of the CARLA install at ``root``: its ``VERSION`` file when that
    reads as a version (``0.9.16``; CARLA 1.0's package writes git hashes there instead), else
    what its layout says (``CarlaUnreal.sh``: ``1.0``), else ``(None, None)``: the probe tells
    once it serves, and the keeper refuses it then (``version_problem``)."""
    root = Path(str(root)).expanduser()
    try:
        lines = [x.strip() for x in (root / "VERSION").read_text().splitlines() if x.strip()]
    except OSError:
        lines = []
    if lines and VERSION_LINE.match(lines[0]):
        return lines[0], f"{root / 'VERSION'}"
    row = layout(root)
    if row is not None and row[3]:
        return row[3], f"{root / row[0]}"
    return None, None


def root_version(root):
    """The version of the CARLA install at ``root`` (``root_version_from``), or None."""
    return root_version_from(root)[0]


def same_version(a, b) -> bool:
    """``a`` and ``b`` name the same release: equal once a source build's ``-<commit>`` is
    dropped, or one a prefix of the other at a dot (the layout's ``1.0`` and ``1.0.0``)."""
    pa, pb = (str(v).strip().split("-")[0].split(".") for v in (a, b))
    n = min(len(pa), len(pb))
    return pa[:n] == pb[:n]


def version_problem(rec: dict, probed):
    """Why a started CARLA that answers ``probed`` is not the one the stack planned for, or
    None. The stack chose its cars' bodies from the version it knew before CARLA started."""
    planned = rec.get("version")
    if not probed:
        return None
    if planned and not same_version(planned, probed):
        return (f"CARLA at {rec.get('root')} answers version {probed}, but the stack planned "
                f"for {planned} (from {rec.get('version_from')}): the cars would drive the "
                "wrong body; fix that file or carla.root, then restart the stack")
    if not planned:
        return (f"CARLA at {rec.get('root')} answers version {probed}, but its install says "
                "no version (no VERSION file that reads as one, no known layout), so the stack "
                "chose its cars before it knew the body; write "
                f"{probed.split('-')[0]} into {Path(str(rec.get('root'))).expanduser() / 'VERSION'}, "
                "then restart the stack")
    return None


class Carla:
    """A CARLA the stack started: its process, readiness, respawn and stop.

    ``rec`` is stack.json's ``carla`` block; it is updated in place."""

    def __init__(self, rec: dict, proc_dir, probe_fn=probe, start=None, events=None,
                 respawn_delay=RESPAWN_DELAY):
        self.rec = rec
        self.proc_dir = Path(proc_dir)
        self.probe = probe_fn
        self._start_process = start
        self.events = events or (lambda name, **kw: None)
        self.respawn_delay = respawn_delay
        self.proc = None
        self.reaper = None
        self.t_start = None
        self.next_probe = 0.0
        self.due = None

    @property
    def ready(self) -> bool:
        return self.rec.get("state") == READY

    @property
    def failed(self) -> bool:
        return self.rec.get("state") == FAILED

    def start(self):
        log = self.proc_dir / LOG
        log.parent.mkdir(parents=True, exist_ok=True)
        argv = list(self.rec.get("argv") or carla_argv(self.rec["root"], self.rec["port"]))
        root = launcher(self.rec["root"]).parent if self.rec.get("root") else None
        with open(log, "ab") as fh:
            fh.write(f"# scensim stack: {shlex.join(argv)}\n".encode())
        if self._start_process is not None:
            self.proc = self._start_process(argv, carla_env(), log, root)
        else:
            fh = open(log, "ab")
            try:
                self.proc = subprocess.Popen(argv, env=carla_env(), stdout=fh,
                                             stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                             cwd=str(root) if root else None,
                                             start_new_session=True,
                                             preexec_fn=_carla_child)
            finally:
                fh.close()
            self.reaper = start_reaper(self.proc.pid)
        self.t_start = time.time()
        self.next_probe = 0.0
        self.due = None
        self.rec.update(state=STARTING, pid=self.proc.pid, log=str(log),
                        started_at=_iso(), ready_at=None, error=None)
        self.events("carla.start", pid=self.proc.pid, port=self.rec["port"])

    def poll(self) -> bool:
        """Readiness, death and respawn; True when ``rec`` changed."""
        now = time.time()
        if self.due is not None:
            if now < self.due:
                return False
            self.start()
            return True
        if self.proc is None or self.failed:
            return False
        code = self.proc.poll()
        if code is not None:
            _kill_group(self.proc.pid, signal.SIGKILL)  # what the launcher left of it
            if self.rec.get("state") == READY:
                self.rec.update(state=STARTING, pid=None, ready_at=None,
                                restarts=int(self.rec.get("restarts") or 0) + 1)
                self.events("carla.exit", code=code)
                self.due = now + self.respawn_delay
                self.proc = None
                return True
            self.rec.update(state=FAILED, pid=None,
                            error=f"CARLA exited ({code}) before it served RPC on "
                                  f"{self.rec['host']}:{self.rec['port']}; see {self.rec['log']}")
            self.events("carla.failed", code=code)
            self.proc = None
            return True
        if self.rec.get("state") == STARTING and now >= self.next_probe:
            self.next_probe = now + PROBE_EVERY
            state, version = self.probe(self.rec["host"], self.rec["port"])
            if state == CARLA:
                problem = version_problem(self.rec, version)
                if problem:
                    self.stop()
                    self.rec.update(state=FAILED, error=problem)
                    self.events("carla.failed", error="version")
                    return True
                self.rec.update(state=READY, ready_at=_iso(), version=version,
                                ready_after_s=round(now - self.t_start, 1))
                self.events("carla.ready", after_s=round(now - self.t_start, 1))
                return True
            if now - self.t_start >= float(self.rec.get("ready_timeout")
                                           or DEFAULT_READY_TIMEOUT):
                self.stop()
                self.rec.update(state=FAILED,
                                error=f"CARLA did not serve RPC on {self.rec['host']}:"
                                      f"{self.rec['port']} within "
                                      f"{float(self.rec['ready_timeout']):.0f} s "
                                      "(carla.ready_timeout); see " + str(self.rec['log']))
                self.events("carla.failed", error="ready timeout")
                return True
        return False

    def stop(self, grace=20.0):
        """Its process group: SIGINT, SIGTERM, SIGKILL; the pid it started, nothing else."""
        self.due = None
        proc, self.proc = self.proc, None
        if proc is None:
            return
        live = proc.poll() is None
        for sig, wait in ((signal.SIGINT, grace), (signal.SIGTERM, 10.0), (signal.SIGKILL, 5.0)):
            if not live:
                break
            _kill_group(proc.pid, sig)
            deadline = time.time() + wait
            while time.time() < deadline and proc.poll() is None:
                time.sleep(0.05)
            live = proc.poll() is None
        _kill_group(proc.pid, signal.SIGKILL)  # the Shipping binary under the launcher's shell
        self.rec.update(state=STOPPED, pid=None)
        self.events("carla.stop", pid=proc.pid)


def _carla_child():
    """CARLA's launcher before exec: the stop's SIGINT at its default (see
    scensim.stack.default_signals), and SIGTERM when the keeper dies."""
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGQUIT):
        signal.signal(sig, signal.SIG_DFL)
    _die_with_parent()


REAP_GRACE = 20.0   # a CARLA whose keeper died: SIGTERM, this long, then SIGKILL


def start_reaper(pid, grace=REAP_GRACE):
    """A watcher for the CARLA at ``pid`` that ends it when the keeper dies. The parent-death
    signal gives CARLA SIGTERM, but UE's graceful exit can hang, and play_launch SIGKILLs the
    keeper seconds after SIGTERM, before its own SIGKILL to CARLA: the watcher, in its own
    session (play_launch's stop does not reach it), escalates instead. None if it cannot
    start (CARLA then has only its parent-death signal)."""
    try:
        return subprocess.Popen([sys.executable, "-m", "scensim.simulator", "reap", str(pid),
                                 "--grace", str(grace), "--parent", str(os.getpid())],
                                stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                                start_new_session=True, preexec_fn=_die_with_parent)
    except OSError:
        return None


def reap(pid: int, parent: int, grace: float = REAP_GRACE, poll: float = 1.0) -> int:
    """Wait while ``parent`` lives; then SIGTERM CARLA (its process group too), wait
    ``grace`` seconds, SIGKILL. Through a pidfd: a pid used again is never signalled. Returns
    at once when CARLA ends first (its keeper stopped it)."""
    import select
    try:
        fd = os.pidfd_open(pid)
    except (ProcessLookupError, OSError):
        return 0
    gone = {"now": False}
    signal.signal(signal.SIGTERM, lambda *_: gone.update(now=True))
    signal.signal(signal.SIGINT, signal.SIG_IGN)

    def ended(timeout):
        return bool(select.select([fd], [], [], timeout)[0])

    def send(sig):
        try:
            signal.pidfd_send_signal(fd, sig)
        except ProcessLookupError:
            return
        if not ended(0):
            _kill_group(pid, sig)   # the engine under a launcher script, in CARLA's session
    while not gone["now"] and os.getppid() == parent:
        if ended(poll):
            return 0
    send(signal.SIGTERM)
    if ended(grace):
        return 0
    send(signal.SIGKILL)
    return 0


def _kill_group(pgid, sig):
    try:
        os.killpg(pgid, sig)
    except (ProcessLookupError, PermissionError):
        pass


def _iso():
    import datetime
    return datetime.datetime.now().astimezone().isoformat(timespec="seconds")


def main(argv=None) -> int:
    import argparse
    ap = argparse.ArgumentParser(prog="python3 -m scensim.simulator")
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("reap", help="end a stack's CARLA when its keeper dies (internal)")
    r.add_argument("pid", type=int)
    r.add_argument("--parent", type=int, required=True)
    r.add_argument("--grace", type=float, default=REAP_GRACE)
    a = ap.parse_args(argv)
    return reap(a.pid, a.parent, a.grace)


if __name__ == "__main__":
    sys.exit(main())


# --- the towns this host's CARLA has (roadmap 077, G13) ------------------------------------

# CARLA matches a town by its exact name; a map whose directory name differs from CARLA's
# town has an alias in the CARLA world server's config (map_alias). These are its defaults.
TOWN_ALIAS = {"Town10": "Town10HD"}


def town_alias() -> dict:
    """The CARLA world server's map_alias (its installed config), else TOWN_ALIAS."""
    try:
        import yaml
        from ament_index_python.packages import get_package_share_directory
        p = Path(get_package_share_directory("scensim_carla_world")) / "config" / \
            "scensim_carla_world.yaml"
        return dict((yaml.safe_load(p.read_text()) or {}).get("map_alias") or TOWN_ALIAS)
    except Exception:  # noqa: BLE001 (not installed: the defaults)
        return dict(TOWN_ALIAS)


def map_package_dirs(carla: dict) -> list:
    """Where this host's CARLA towns are: ``carla.map_packages``, else the install's own
    Content directory; [] when neither is known (a CARLA elsewhere: not checked)."""
    dirs = carla.get("map_packages") or []
    if isinstance(dirs, str):
        dirs = [dirs]
    if not dirs and carla.get("root"):
        dirs = [Path(os.path.expanduser(str(carla["root"]))) / "CarlaUE4" / "Content"]
    return [Path(os.path.expanduser(str(d))) for d in dirs]


def has_town(dirs, town: str) -> bool:
    """A ``<town>.umap`` in a package's Maps (``<dir>/<package>/Maps/[<map>/]<town>.umap``)."""
    return any(any(Path(d).glob(pattern)) for d in dirs
               for pattern in (f"*/Maps/{town}.umap", f"*/Maps/*/{town}.umap",
                               f"Maps/{town}.umap", f"Maps/*/{town}.umap"))


def town_problems(maps, carla: dict, alias=None) -> list:
    """``(map, town, dirs)`` for each project map this host's CARLA has no town for."""
    dirs = map_package_dirs(carla)
    if not dirs or not any(Path(d).is_dir() for d in dirs):
        return []
    alias = town_alias() if alias is None else alias
    out = []
    for m in maps or []:
        town = alias.get(m, m)
        if isinstance(m, str) and m and not has_town(dirs, town):
            out.append((m, town, dirs))
    return out
