"""Seats: one stack's share of a host, derived from one number by the host's table.

docs/design/projects-and-pilots.md, "Seats". Seat ``n`` of a table:

- scenario domain ``domains.base + domains.stride * n``, pilot domains the next
  ``domains.stride - 1``;
- SSv2 port ``ports.base + ports.stride * n``, its sensor-release sockets +1, +2, the relay
  +5, the world server +36;
- web UI ``web.base + web.stride * n`` (+0 the stack, +1 the run's scenario);
- GPU ``gpus[n]``, or the host settings' ``gpu:``.

A seat is held by an exclusive ``flock`` on ``<seat_locks>/seat-<n>.lock``, whose content
names the holder (JSON: user, pid, host, project, since). The kernel drops the lock when its
process dies, so a dead holder's file is simply reclaimed by the next one. ``seat take``
only records the choice (``seat:`` in the user's host.yaml); the stack holds the lock while
it is up.

Exit codes of the command: 0 ok, 3 refused (a taken seat, an invalid table or argument).
"""
import argparse
import datetime
import errno
import fcntl
import getpass
import json
import os
import socket
import sys
from dataclasses import dataclass, field
from pathlib import Path

from swb import config
from swb.config import ConfigError

MAX_DOMAIN = 57          # the localhost Cyclone profile's DomainGain 1000 (ports <= 65535)
RELEASE_OFFSETS = (1, 2)
RELAY_OFFSET = 5
WORLD_OFFSET = 36
PORT_BLOCK = WORLD_OFFSET + 1   # ports a seat uses: SSv2 +0 .. world server +36
WEB_BLOCK = 2                   # web UI: +0 stack, +1 the run's scenario
MAX_PORT = 65535
CYCLONE_PROFILE = "cyclonedds-localhost.xml"

EXIT_OK = 0
EXIT_REFUSED = 3


class SeatTaken(RuntimeError):
    """A seat whose lock another live process holds; ``holder`` is its lock content."""

    def __init__(self, index: int, holder: dict):
        self.index = index
        self.holder = holder or {}
        super().__init__(f"seat {index} is taken: {describe_holder(self.holder)}")


@dataclass
class Seat:
    index: int
    scenario_domain: int
    pilot_domains: list
    ssv2_port: int
    release_ports: tuple
    relay_port: int
    world_port: int
    web_port: int
    web_port_run: int
    gpu: int
    lock_dir: str = field(default=config.DEFAULTS["seat_locks"], repr=False)

    def numbers(self) -> dict:
        return {
            "seat": self.index,
            "scenario_domain": self.scenario_domain,
            "pilot_domains": list(self.pilot_domains),
            "ssv2_port": self.ssv2_port,
            "release_ports": list(self.release_ports),
            "relay_port": self.relay_port,
            "world_port": self.world_port,
            "web_port": self.web_port,
            "web_port_run": self.web_port_run,
            "gpu": self.gpu,
        }


# ---------------------------------------------------------------------------- the table

def seats_source(project: Path = None) -> str:
    """The file whose ``seats:`` wins (for messages), or ``defaults``."""
    candidates = []
    if project is not None:
        candidates.append(Path(project) / ".swb" / "local.yaml")
    candidates += [config.user_dir() / "host.yaml", config.system_dir() / "host.yaml"]
    for path in candidates:
        try:
            if "seats" in config.read_yaml(path):
                return str(path)
        except ConfigError:
            continue
    return "defaults"


def _int(value, where: str, key: str, minimum: int = None) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ConfigError(f"{where}: {key} must be an integer, not {value!r}")
    if minimum is not None and value < minimum:
        raise ConfigError(f"{where}: {key} must be at least {minimum}, not {value}")
    return value


def table(settings: dict, source: str = None) -> list:
    """Every seat of ``settings["seats"]``; a table that breaks a rule raises ConfigError."""
    where = source or "host settings"
    t = settings.get("seats")
    if not isinstance(t, dict):
        raise ConfigError(f"{where}: seats must be a mapping")
    for k in ("domains", "ports", "web"):
        if not isinstance(t.get(k), dict):
            raise ConfigError(f"{where}: seats.{k} must be a mapping with base and stride")
    count = _int(t.get("count"), where, "seats.count", 1)
    d_base = _int(t["domains"].get("base"), where, "seats.domains.base", 0)
    d_stride = _int(t["domains"].get("stride"), where, "seats.domains.stride", 2)
    p_base = _int(t["ports"].get("base"), where, "seats.ports.base", 1)
    p_stride = _int(t["ports"].get("stride"), where, "seats.ports.stride", PORT_BLOCK)
    w_base = _int(t["web"].get("base"), where, "seats.web.base", 1)
    w_stride = _int(t["web"].get("stride"), where, "seats.web.stride", WEB_BLOCK)

    last_domain = d_base + d_stride * count - 1
    if last_domain > MAX_DOMAIN:
        raise ConfigError(
            f"{where}: seats.domains: {count} seats of stride {d_stride} from {d_base} reach "
            f"domain {last_domain}; every domain must be 0-{MAX_DOMAIN} (lower count or stride)")
    last_port = p_base + p_stride * (count - 1) + PORT_BLOCK - 1
    if last_port > MAX_PORT:
        raise ConfigError(f"{where}: seats.ports: seat {count - 1} reaches port {last_port}, "
                          f"past {MAX_PORT}")
    last_web = w_base + w_stride * (count - 1) + WEB_BLOCK - 1
    if last_web > MAX_PORT:
        raise ConfigError(f"{where}: seats.web: seat {count - 1} reaches port {last_web}, "
                          f"past {MAX_PORT}")
    # SSv2/relay/world ports and web ports are one TCP space: they must not meet
    seat_ports = {p_base + p_stride * n + o for n in range(count) for o in range(PORT_BLOCK)}
    web_ports = {w_base + w_stride * n + o for n in range(count) for o in range(WEB_BLOCK)}
    clash = sorted(seat_ports & web_ports)
    if clash:
        raise ConfigError(f"{where}: seats.web overlaps seats.ports at port {clash[0]}")

    gpus = t.get("gpus")
    if gpus is not None:
        if not isinstance(gpus, list) or len(gpus) != count:
            raise ConfigError(f"{where}: seats.gpus must list one GPU per seat ({count}), "
                              f"not {gpus!r}")
        for i, g in enumerate(gpus):
            _int(g, where, f"seats.gpus[{i}]", 0)
    gpu_override = settings.get("gpu")
    if gpu_override is not None:
        _int(gpu_override, where, "gpu", 0)

    lock_dir = str(settings.get("seat_locks") or config.DEFAULTS["seat_locks"])
    seats = []
    for n in range(count):
        dom = d_base + d_stride * n
        port = p_base + p_stride * n
        web = w_base + w_stride * n
        gpu = gpu_override if gpu_override is not None else (gpus[n] if gpus else 0)
        seats.append(Seat(
            index=n, scenario_domain=dom, pilot_domains=list(range(dom + 1, dom + d_stride)),
            ssv2_port=port, release_ports=tuple(port + o for o in RELEASE_OFFSETS),
            relay_port=port + RELAY_OFFSET, world_port=port + WORLD_OFFSET,
            web_port=web, web_port_run=web + 1, gpu=gpu, lock_dir=lock_dir))
    return seats


def parse_choice(value, where: str = "host settings"):
    """``seat:`` as an int or ``"auto"``."""
    if value is None or value == "auto":
        return "auto"
    if isinstance(value, bool):
        raise ConfigError(f"{where}: seat must be a number or auto, not {value!r}")
    if isinstance(value, int):
        return value
    if isinstance(value, str) and value.strip().isdigit():
        return int(value.strip())
    raise ConfigError(f"{where}: seat must be a number or auto, not {value!r}")


def resolve(settings: dict, source: str = None) -> Seat:
    """The seat ``settings["seat"]`` names; ``auto``: the first whose lock is free."""
    seats = table(settings, source)
    choice = parse_choice(settings.get("seat"), source or "host settings")
    if choice == "auto":
        for s in seats:
            if holder(s.index, s.lock_dir) is None:
                return s
        raise SeatTaken(-1, {"note": f"all {len(seats)} seats are taken"})
    if not 0 <= choice < len(seats):
        raise ConfigError(f"{source or 'host settings'}: seat {choice} is not in the table "
                          f"(0-{len(seats) - 1})")
    return seats[choice]


# ---------------------------------------------------------------------------- locks

def _lock_dir(lock_dir) -> Path:
    return Path(lock_dir or config.host_settings()["seat_locks"]).expanduser()


def ensure_lock_dir(lock_dir) -> Path:
    """The lock directory, created world-writable and sticky (1777) when absent."""
    d = _lock_dir(lock_dir)
    if not d.is_dir():
        d.mkdir(parents=True, exist_ok=True)
        try:
            os.chmod(d, 0o1777)
        except PermissionError:
            pass
    return d


def lock_path(index: int, lock_dir=None) -> Path:
    return _lock_dir(lock_dir) / f"seat-{index}.lock"


def _open_lock(path: Path, create: bool):
    # Open an existing file without O_CREAT: in a sticky world-writable directory
    # (fs.protected_regular) O_CREAT on another user's file is refused.
    try:
        return os.open(path, os.O_RDWR)
    except FileNotFoundError:
        if not create:
            return None
    except PermissionError:
        return os.open(path, os.O_RDONLY)   # flock works on a read-only descriptor
    try:
        fd = os.open(path, os.O_RDWR | os.O_CREAT | os.O_EXCL, 0o666)
        try:
            os.fchmod(fd, 0o666)    # other users must be able to reclaim it
        except PermissionError:
            pass
        return fd
    except FileExistsError:
        return os.open(path, os.O_RDWR)


def _read(fd) -> dict:
    os.lseek(fd, 0, os.SEEK_SET)
    raw = b""
    while True:
        chunk = os.read(fd, 65536)
        if not chunk:
            break
        raw += chunk
    try:
        data = json.loads(raw.decode() or "{}")
    except (ValueError, UnicodeDecodeError):
        return {}
    return data if isinstance(data, dict) else {}


def pid_alive(pid) -> bool:
    if not isinstance(pid, int) or pid <= 0:
        return False
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def holder(index: int, lock_dir=None) -> dict:
    """Who holds seat ``index`` (its lock content), or None when it is free. A file whose
    process died is free: the kernel released its flock."""
    path = lock_path(index, lock_dir)
    fd = _open_lock(path, create=False) if path.exists() else None
    if fd is None:
        return None
    try:
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as e:
            if e.errno not in (errno.EWOULDBLOCK, errno.EAGAIN):
                raise
            info = _read(fd)
            return info or {"note": "held (holder not yet written)"}
        fcntl.flock(fd, fcntl.LOCK_UN)
        return None
    finally:
        os.close(fd)


def describe_holder(info: dict) -> str:
    if not info:
        return "unknown holder"
    if "user" not in info:
        return info.get("note", "unknown holder")
    s = f"{info.get('user')} (pid {info.get('pid')} on {info.get('host')}"
    if info.get("project"):
        s += f", project {info['project']}"
    if info.get("since"):
        s += f", since {info['since']}"
    return s + ")"


class Lock:
    """An exclusive hold on one seat; released by ``release()``, the context's end, or the
    process's death."""

    def __init__(self, seat: Seat, fd: int, path: Path, info: dict):
        self.seat, self.fd, self.path, self.info = seat, fd, path, info

    @property
    def held(self) -> bool:
        return self.fd is not None

    def release(self):
        if self.fd is None:
            return
        try:
            os.ftruncate(self.fd, 0)
        except OSError:
            pass
        fcntl.flock(self.fd, fcntl.LOCK_UN)
        os.close(self.fd)
        self.fd = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.release()
        return False

    def __del__(self):
        try:
            self.release()
        except Exception:
            pass


def acquire(seat: Seat, project=None, lock_dir=None) -> Lock:
    """Hold ``seat`` for this process; SeatTaken when a live process holds it."""
    d = ensure_lock_dir(lock_dir or seat.lock_dir)
    path = d / f"seat-{seat.index}.lock"
    fd = _open_lock(path, create=True)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError as e:
        info = _read(fd)
        os.close(fd)
        if e.errno in (errno.EWOULDBLOCK, errno.EAGAIN):
            raise SeatTaken(seat.index, info) from None
        raise
    info = {
        "user": getpass.getuser(),
        "pid": os.getpid(),
        "host": socket.gethostname(),
        "project": str(project) if project is not None else "",
        "since": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
    }
    try:
        os.ftruncate(fd, 0)
        os.lseek(fd, 0, os.SEEK_SET)
        os.write(fd, json.dumps(info).encode())
    except OSError:
        pass
    return Lock(seat, fd, path, info)


# ---------------------------------------------------------------------------- env

def cyclone_profile() -> Path:
    """swb_launch's localhost Cyclone profile: the installed share, else the workspace's."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory("swb_launch"))
        return share / "config" / CYCLONE_PROFILE
    except Exception:
        pass
    here = Path(__file__).resolve()
    for d in here.parents:
        cand = d / "install" / "swb_launch" / "share" / "swb_launch" / "config" / CYCLONE_PROFILE
        if cand.is_file():
            return cand
        cand = d / "src" / "swb_launch" / "config" / CYCLONE_PROFILE
        if cand.is_file():
            return cand
    # src/swb/swb/seat.py -> src/swb_launch/config/
    return here.parents[2] / "swb_launch" / "config" / CYCLONE_PROFILE


def group_domain(seat: Seat, group: str) -> int:
    """``scenario``, ``pilot:<k>`` or ``<k>``: the domain."""
    if group == "scenario":
        return seat.scenario_domain
    k = group[len("pilot:"):] if group.startswith("pilot:") else group
    if not k.isdigit():
        raise ConfigError(f"group {group!r}: give scenario, pilot:<k> or <k> (a pilot index; "
                          "entity names need a running stack's stack.json)")
    k = int(k)
    if k >= len(seat.pilot_domains):
        raise ConfigError(f"group {group!r}: seat {seat.index} has pilot indices "
                          f"0-{len(seat.pilot_domains) - 1}")
    return seat.pilot_domains[k]


def env_line(seat: Seat, group: str) -> str:
    return (f"export ROS_DOMAIN_ID={group_domain(seat, group)} "
            f"RMW_IMPLEMENTATION=rmw_cyclonedds_cpp "
            f"CYCLONEDDS_URI=file://{cyclone_profile()}")


# ---------------------------------------------------------------------------- command

def _format_numbers(s: Seat) -> str:
    pilots = f"{s.pilot_domains[0]}-{s.pilot_domains[-1]}"
    return (f"seat {s.index}: scenario domain {s.scenario_domain}, pilot domains {pilots}, "
            f"SSv2 {s.ssv2_port} (+{s.release_ports[0]}, {s.release_ports[1]}), "
            f"relay {s.relay_port}, world server {s.world_port}, "
            f"web {s.web_port} (run {s.web_port_run}), GPU {s.gpu}")


def _project(arg):
    try:
        return config.find_project(arg) if arg else config.find_project()
    except ConfigError:
        if arg:
            raise
        return None


def _err(msg: str) -> int:
    print(f"swb seat: {msg}", file=sys.stderr)
    return EXIT_REFUSED


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="ros2 run swb seat", description="Seats on this host.")
    ap.add_argument("--project", help="the project (default: the working directory's)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("take", help="record a seat as yours (seat: in your host.yaml)")
    p.add_argument("seat", help="a seat number or auto")
    sub.add_parser("show", help="every seat, its numbers and its holder")
    sub.add_parser("release", help="set your seat back to auto")
    p = sub.add_parser("env", help="the ROS exports for a group of your seat")
    p.add_argument("group", help="scenario, pilot:<k> or <k>")
    try:
        args = ap.parse_args(argv)
    except SystemExit as e:
        return EXIT_OK if e.code == 0 else EXIT_REFUSED

    try:
        project = _project(args.project)
        settings = config.host_settings(project)
        source = seats_source(project)
        seats = table(settings, source)
        me = getpass.getuser()

        if args.cmd == "take":
            choice = parse_choice(args.seat, "argument")
            if choice == "auto":
                s = resolve(dict(settings, seat="auto"), source)
            else:
                if not 0 <= choice < len(seats):
                    return _err(f"seat {choice} is not in the table (0-{len(seats) - 1}, "
                                f"{source})")
                s = seats[choice]
                h = holder(s.index, s.lock_dir)
                if h is not None and h.get("user") != me:
                    return _err(f"seat {s.index} is taken: {describe_holder(h)}")
            path = config.write_user_setting("seat", s.index)
            print(f"{_format_numbers(s)}\n(recorded in {path}; the stack holds the seat "
                  "while it is up)")
            return EXIT_OK

        if args.cmd == "show":
            mine = parse_choice(settings.get("seat"))
            print(f"seats ({source}); locks in {settings['seat_locks']}")
            for s in seats:
                h = holder(s.index, s.lock_dir)
                state = f"held by {describe_holder(h)}" if h is not None else "free"
                tag = "  <- yours" if (mine == s.index or
                                       (h is not None and h.get("user") == me)) else ""
                print(f"{_format_numbers(s)}; {state}{tag}")
            if mine == "auto":
                print("your seat: auto (the first free one when the stack starts)")
            return EXIT_OK

        if args.cmd == "release":
            path = config.write_user_setting("seat", "auto")
            print(f"seat set back to auto in {path}")
            for s in seats:
                h = holder(s.index, s.lock_dir)
                if h is not None and h.get("user") == me:
                    print(f"seat {s.index} is still held by your stack "
                          f"({describe_holder(h)}); stop the stack to free it")
            return EXIT_OK

        if args.cmd == "env":
            mine = parse_choice(settings.get("seat"))
            if mine == "auto":
                held = [s for s in seats
                        if (holder(s.index, s.lock_dir) or {}).get("user") == me]
                if len(held) != 1:
                    return _err("your seat is auto and no single stack of yours holds one: "
                                "run `ros2 run swb seat take <n>`")
                s = held[0]
            else:
                s = resolve(settings, source)
            print(env_line(s, args.group))
            return EXIT_OK
    except SeatTaken as e:
        return _err(str(e))
    except ConfigError as e:
        return _err(str(e))
    return EXIT_REFUSED


if __name__ == "__main__":
    sys.exit(main())
