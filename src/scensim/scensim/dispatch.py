"""``scensim run <suite> --seats <list|auto>``: one suite on several seats at once (roadmap 088).

The dispatcher (this module's ``Dispatcher``, in the ``scensim run`` process) holds the
suite's runs in a queue and starts one worker process per seat. A worker
(``python -m scensim.dispatch worker``) is the ordinary runner on one seat: it starts the
seat's stack (its own CARLA, from the seat table's ``seats.carla``), then asks for one run
at a time and runs it with ``run.run_spec``. The protocol is JSON lines: the worker writes
``ready``, ``done`` and ``retired`` events on stdout and reads ``run`` and ``stop`` orders
on stdin; whatever else it prints (the run's own lines) goes to stderr, which the
dispatcher shows prefixed with the seat.

The dispatcher admits a seat's start only when the host has room (free VRAM on its GPU,
the 1-minute load), starts seats ``dispatch.start_gap`` s apart, lowers the number of seats
handed work when a run's acb_bridge skipped more than ``dispatch.skip_limit`` CARLA frames
(overload), retires a seat whose stack does not become ready or whose pilots are broken, and
merges every run into the suite's one directory (``<logs>/<testbench>/suites/<suite>/``), so
``summary.csv`` and ``junit.xml`` are written as for one seat and T3's compare reads them
unchanged.
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import os
import re
import selectors
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from pathlib import Path

from scensim import config, results, runs as runsmod, seat as seatmod

WORKER_MODULE = "scensim.dispatch"
STOP_GRACE = 60.0      # s a worker has to stop its stack after "stop" or SIGTERM
POLL = 1.0             # s between the dispatcher's looks at admission and its workers
ADMIT_TIMEOUT = 1800.0  # s the suite waits for room when no seat of it runs, then gives up
SKIPPED = re.compile(r"Skipped (\d+) CARLA frame")
ANSI = re.compile(r"\x1b\[[0-9;]*m")


class DispatchRefused(Exception):
    """A --seats request that cannot run: printed as given, exit 3."""


# ----------------------------------------------------------------------- seats and admission

def settings(host: dict) -> dict:
    d = dict(config.DEFAULTS["dispatch"])
    d.update(host.get("dispatch") or {})
    if d.get("overload_load") is None:
        d["overload_load"] = float(os.cpu_count() or 1)
    for k in ("vram_per_seat_mb", "load_per_seat", "skip_limit", "start_gap", "overload_load"):
        v = d.get(k)
        if isinstance(v, bool) or not isinstance(v, (int, float)) or v < 0:
            raise DispatchRefused(f"host settings: dispatch.{k} must be a number >= 0, not {v!r}")
    return d


def parse_seats(value: str, host: dict, source: str = None, holder=seatmod.holder) -> list:
    """The seats ``--seats`` names: ``auto`` (every seat of the table whose lock is free) or
    a comma list of seat numbers. Each seat must have its own CARLA (``seats.carla``)."""
    try:
        table = seatmod.table(host, source)
    except config.ConfigError as e:
        raise DispatchRefused(str(e)) from None
    if any(s.carla_port is None for s in table):
        raise DispatchRefused(
            "--seats needs a CARLA per seat: give the seat table seats.carla: {base: <port>, "
            "stride: 10} in your host settings, or two seats would share one CARLA")
    by_index = {s.index: s for s in table}
    lock_dir = host.get("seat_locks")
    if str(value).strip() == "auto":
        free = [s for s in table if not holder(s.index, lock_dir)]
        if not free:
            raise DispatchRefused(f"--seats auto: every seat of the table ({len(table)}) is held")
        return free
    out = []
    for part in str(value).split(","):
        part = part.strip()
        if not part.isdigit():
            raise DispatchRefused(f"--seats {value!r}: give seat numbers (0,1,2) or auto")
        n = int(part)
        if n not in by_index:
            raise DispatchRefused(f"--seats: seat {n} is not in the seat table (0-{len(table) - 1})")
        if any(s.index == n for s in out):
            raise DispatchRefused(f"--seats: seat {n} is named twice")
        out.append(by_index[n])
    if not out:
        raise DispatchRefused("--seats: name at least one seat")
    return out


def gpu_free_mb(gpu) -> int:
    """Free memory on GPU ``gpu`` (MiB), or None when nvidia-smi cannot tell."""
    try:
        r = subprocess.run(["nvidia-smi", "--query-gpu=memory.free", "--format=csv,noheader,"
                            "nounits", "-i", str(0 if gpu is None else gpu)],
                           capture_output=True, text=True, timeout=10)
        return int(r.stdout.split()[0]) if r.returncode == 0 and r.stdout.strip() else None
    except (OSError, ValueError, IndexError, subprocess.TimeoutExpired):
        return None


def admission(seat, cfg: dict, free_mb=gpu_free_mb, load=os.getloadavg,
              cores=os.cpu_count) -> str:
    """Why seat ``seat`` may not start its stack now, or "" when the host has room."""
    free = free_mb(seat.gpu)
    if free is not None and free < cfg["vram_per_seat_mb"]:
        return (f"GPU {seat.gpu if seat.gpu is not None else 0} has {free} MiB free, a seat "
                f"needs {cfg['vram_per_seat_mb']:.0f}")
    l1 = load()[0]
    n = cores() or 1
    if l1 + cfg["load_per_seat"] > n:
        return f"the load is {l1:.1f} on {n} cores, a seat adds about {cfg['load_per_seat']:.0f}"
    return ""


def acb_skips(proc_dir, offsets: dict) -> int:
    """CARLA frames acb_bridge skipped since ``offsets`` (file -> bytes read, updated in
    place), over the stack's logs under ``proc_dir``: play_launch's per-node
    ``.../node/acb_bridge/out`` and ``err``, and the console lines ``ros2 launch`` prefixes
    with ``[acb_bridge``."""
    root = Path(proc_dir) if proc_dir else None
    if root is None or not root.is_dir():
        return 0
    total = 0
    for f in sorted(root.rglob("*")):
        if not f.is_file():
            continue
        own = "acb_bridge" in f.parts
        try:
            size = f.stat().st_size
            start = offsets.get(str(f), 0)
            offsets[str(f)] = size
            if size <= start:
                continue
            with open(f, "rb") as fh:
                fh.seek(start)
                text = fh.read(size - start).decode(errors="replace")
        except OSError:
            continue
        for ln in ANSI.sub("", text).splitlines():
            if own or "[acb_bridge" in ln:
                m = SKIPPED.search(ln)
                if m:
                    total += int(m.group(1))
    return total


# ----------------------------------------------------------------------------- the protocol

def spec_to_json(spec) -> dict:
    d = dataclasses.asdict(spec)
    d["scenario"] = str(spec.scenario)
    return d


def spec_from_json(d: dict):
    from scensim.run import RunSpec
    return RunSpec(**{**d, "scenario": Path(d["scenario"])})


def emit(out, **event):
    out.write(json.dumps(event) + "\n")
    out.flush()


# ------------------------------------------------------------------------------- the worker

def stack_pilot_broken(ctx) -> str:
    """A stack pilot of the seat whose launch did not load a composable (issue 064's check,
    timeline.missing_composables on its console), named with why; "" when none. Such a seat
    will not recover by running again."""
    from scensim import run as runmod, timeline
    rec = ctx.stack or {}
    if not rec.get("proc"):
        return ""
    for p in rec.get("pilots") or []:
        if p.get("lifetime", "stack") != "stack":
            continue
        missing = timeline.missing_composables(runmod.pilot_console(rec, p, None))
        if missing:
            return f"pilot {p['entity']}'s launch did not load {missing}"
    return ""


def worker_loop(ctx, inp, out, run_one=None, line=None, outcome=None, broken=None) -> int:
    """Serve ``run`` orders on ``inp`` with ``ctx`` until ``stop`` or EOF; answer on ``out``.
    After a run that ended ``error``, a seat whose pilots are broken (they did not come back
    after a crash, or a composable did not load) says so in its ``done`` and retires."""
    from scensim import run as runmod
    run_one = run_one or runmod.run_spec
    line = line or runmod.line
    outcome = outcome or runmod.outcome
    broken = broken or stack_pilot_broken
    emit(out, event="ready", stack=ctx.stack.get("stack_id"), proc=str(ctx.stack.get("proc")))
    for raw in inp:
        try:
            order = json.loads(raw)
        except ValueError:
            continue
        if order.get("stop"):
            break
        spec = spec_from_json(order["spec"])
        res = run_one(ctx, spec, int(order.get("attempt") or 0))
        d = ctx.runs_root / res.run_id
        why = ""
        if getattr(ctx, "pilots_broken", None):
            why = f"its pilots did not come back: {ctx.pilots_broken}"
        elif res.verdict == results.ERROR:
            found = broken(ctx)
            why = f"its {found}" if found else ""
        emit(out, event="done", key=spec.key, run_dir=str(d), verdict=res.verdict,
             line=line(res, d, spec.expect), outcome=outcome(res, spec.expect),
             seat_broken=why)
        if why:
            emit(out, event="retired", why=why)
            break
    return 0


def worker_main(argv=None) -> int:
    """``python -m scensim.dispatch worker --project P --seat N [--logs L] ...``: one seat's
    runner. Its stack is started (a seat of its own), served, and stopped at the end unless
    --keep-stack; SIGTERM stops it through the runner's own path."""
    from scensim import run as runmod
    ap = argparse.ArgumentParser(prog="scensim.dispatch worker")
    ap.add_argument("--project", required=True)
    ap.add_argument("--seat", required=True)
    ap.add_argument("--logs")
    ap.add_argument("--ready-timeout", type=float, default=runmod.READY_TIMEOUT)
    ap.add_argument("--stack-timeout", type=float)
    ap.add_argument("--keep-stack", action="store_true")
    ap.add_argument("--testbench", help="JSON {name, workspace} of the testbench run")
    a = ap.parse_args(argv)
    proto = sys.stdout
    sys.stdout = sys.stderr   # everything but the protocol is the seat's output
    ctx = None
    previous = signal.signal(signal.SIGTERM, runmod._on_term)
    try:
        try:
            ctx = runmod.context(a.project, a.seat, a.logs, 0.0, start=True,
                                 stack_timeout=a.stack_timeout, ready_timeout=a.ready_timeout,
                                 out=sys.stderr)
        except (runmod.Refused, runmod.StackFailed, config.ConfigError) as e:
            emit(proto, event="retired", why=f"its stack did not become ready: {e}")
            return 0
        ctx.testbench = json.loads(a.testbench) if a.testbench else None
        return worker_loop(ctx, sys.stdin, proto)
    except KeyboardInterrupt:
        return 130
    finally:
        signal.signal(signal.SIGTERM, previous)
        if ctx is not None:
            runmod.release_turn(ctx)
            if ctx.started is not None and not a.keep_stack:
                sys.stderr.write(f"stack    stopping seat {a.seat}'s stack\n")
                ctx.started.stop(out=sys.stderr)


# --------------------------------------------------------------------------- the dispatcher

@dataclasses.dataclass
class Worker:
    seat: object
    proc: subprocess.Popen = None
    state: str = "waiting"     # waiting, starting, ready, busy, parked, retired, ended
    job: tuple = None          # (spec, attempt) in hand
    proc_dir: str = ""
    offsets: dict = dataclasses.field(default_factory=dict)
    why: str = ""
    buf: str = ""
    loads: list = dataclasses.field(default_factory=list)   # 1-min load, sampled in the run


class Dispatcher:
    def __init__(self, specs, seats, cfg: dict, suite_dir: Path, retry: dict, out=None,
                 worker_argv=None, admit=None, skips=None, clock=time.monotonic,
                 load=lambda: os.getloadavg()[0]):
        self.queue = deque((s, 0) for s in specs)
        self.workers = [Worker(seat=s) for s in seats]
        self.cfg = cfg
        self.suite_dir = suite_dir
        r = retry or {}
        self.retries = int(r.get("times") or 0) if (r.get("on") or "error") == "error" else 0
        self.out = out or sys.stdout
        self.worker_argv = worker_argv
        self.admit = admit or (lambda seat: admission(seat, cfg))
        self.skips = skips or (lambda w: acb_skips(w.proc_dir, w.offsets))
        self.clock = clock
        self.load = load
        self.overload_requeued = set()   # keys run again once after an overloaded run
        self.limit = len(self.workers)   # seats handed work; lowered on overload
        self.worst = 0
        self.requeued = set()
        self.sel = selectors.DefaultSelector()
        self.lock = threading.Lock()
        self.last_start = None
        self.failed = []    # keys that never ran (every seat retired)
        self.blocked_since = None   # no seat live and the next one not admitted, since

    # ---- output
    def say(self, text: str):
        with self.lock:
            self.out.write(text.rstrip("\n") + "\n")
            self.out.flush()

    def _relay(self, w: Worker):
        for raw in w.proc.stderr:
            self.say(f"[seat {w.seat.index}] {raw.decode(errors='replace').rstrip()}")

    # ---- workers
    def _start(self, w: Worker):
        w.proc = subprocess.Popen(self.worker_argv(w.seat), stdin=subprocess.PIPE,
                                  stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                  start_new_session=True)
        w.state = "starting"
        self.last_start = self.clock()
        self.sel.register(w.proc.stdout, selectors.EVENT_READ, w)
        threading.Thread(target=self._relay, args=(w,), daemon=True).start()
        self.say(f"dispatch seat {w.seat.index}: starting its stack (CARLA {w.seat.carla_port})")

    def _send(self, w: Worker, **order):
        try:
            w.proc.stdin.write((json.dumps(order) + "\n").encode())
            w.proc.stdin.flush()
        except (BrokenPipeError, OSError):
            pass

    def _hand(self, w: Worker):
        spec, attempt = self.queue.popleft()
        w.job = (spec, attempt)
        w.state = "busy"
        w.loads = []
        self.skips(w)   # the run's skips are counted from here
        self._send(w, run=True, spec=spec_to_json(spec), attempt=attempt)

    def _retire(self, w: Worker, why: str):
        w.why = why
        if w.job is not None:
            spec, attempt = w.job
            if spec.key not in self.requeued:
                self.requeued.add(spec.key)
                self.queue.appendleft((spec, attempt))
                self.say(f"dispatch {spec.key}: back in the queue (seat {w.seat.index} retired)")
            w.job = None
        if w.state not in ("retired", "ended"):
            self.say(f"dispatch seat {w.seat.index}: retired: {why}")
        w.state = "retired"
        self._send(w, stop=True)

    def overload(self, skipped: int, loads: list) -> str:
        """Why a run was overloaded (its acb_bridge skipped frames, or the host's mean load
        while it ran), or ""."""
        why = []
        if skipped > self.cfg["skip_limit"]:
            why.append(f"acb_bridge skipped {skipped} CARLA frames (limit "
                       f"{self.cfg['skip_limit']:.0f})")
        mean = sum(loads) / len(loads) if loads else 0.0
        if mean > self.cfg["overload_load"]:
            why.append(f"mean load {mean:.1f} (limit {self.cfg['overload_load']:.0f})")
        return "; ".join(why)

    def _done(self, w: Worker, ev: dict):
        spec, attempt = w.job
        w.job = None
        run_dir = Path(ev["run_dir"])
        skipped = self.skips(w)
        loads = w.loads
        why = self.overload(skipped, loads)
        self._record(run_dir, w, skipped, loads, why)
        self.say((ev.get("line") or f"{ev.get('verdict')}  {spec.key}")
                 + (f"  overloaded: {why}" if why else ""))
        parked = False
        if why and self.limit > 1:
            self.limit -= 1
            parked = True
            self.say(f"dispatch seat {w.seat.index}: the host is overloaded ({why}); seat "
                     f"parked, {self.limit} seat(s) take work from now")
        if ev.get("seat_broken") and spec.key not in self.requeued:
            # the seat broke, not the scenario: the run goes to another seat, once
            self.requeued.add(spec.key)
            self.queue.appendleft((spec, attempt))
            self.say(f"dispatch {spec.key}: back in the queue (seat {w.seat.index} is broken)")
            self._retire(w, ev["seat_broken"])
            return
        elif why and spec.key not in self.overload_requeued:
            # an overloaded run's verdict is the host's as much as the scenario's: run it
            # again (once), on fewer seats; the first try stays on disk, unlinked
            self.overload_requeued.add(spec.key)
            self.queue.appendleft((spec, attempt))
            self.say(f"dispatch {spec.key}: run again (overloaded)")
        elif ev.get("verdict") == results.ERROR and attempt < self.retries:
            self.queue.appendleft((spec, attempt + 1))
        else:
            self._link(spec.key, run_dir)
            self.worst = max(self.worst, int(ev.get("outcome") or 0))
        w.state = "parked" if parked else "ready"

    def _record(self, run_dir: Path, w: Worker, skipped: int, loads: list, why: str):
        """dispatch.json in the run's directory: the seat, its acb_bridge skips and the
        host's load while it ran, and whether it was overloaded (runs.summarize reads it)."""
        try:
            (run_dir / "dispatch.json").write_text(json.dumps({
                "seat": w.seat.index, "acb_skipped_frames": skipped,
                "load_mean": round(sum(loads) / len(loads), 1) if loads else None,
                "load_max": round(max(loads), 1) if loads else None,
                "overloaded": why}, indent=2) + "\n")
        except OSError:
            pass

    def _link(self, key: str, run_dir: Path):
        if self.suite_dir is None:
            return
        link = self.suite_dir / key
        if link.is_symlink():
            link.unlink()
        os.symlink(run_dir, link)

    def _event(self, w: Worker, ev: dict):
        kind = ev.get("event")
        if kind == "ready":
            w.state = "ready"
            w.proc_dir = ev.get("proc") or ""
            self.say(f"dispatch seat {w.seat.index}: ready (stack {ev.get('stack')})")
        elif kind == "done" and w.job is not None:
            self._done(w, ev)
        elif kind == "retired":
            self._retire(w, ev.get("why") or "")

    def _read(self, w: Worker):
        data = os.read(w.proc.stdout.fileno(), 65536)
        if not data:
            self.sel.unregister(w.proc.stdout)
            if w.state not in ("retired", "ended"):
                self._retire(w, f"its worker ended (exit {w.proc.wait()})")
            w.state = "ended" if w.state == "retired" and w.proc.poll() is not None else w.state
            return
        w.buf += data.decode(errors="replace")
        while "\n" in w.buf:
            raw, w.buf = w.buf.split("\n", 1)
            try:
                ev = json.loads(raw)
            except ValueError:
                continue
            self._event(w, ev)

    def _admit_next(self):
        """Start the next waiting seat when the gap has passed and the host has room."""
        if not self.queue:
            return
        waiting = [w for w in self.workers if w.state == "waiting"]
        if not waiting:
            return
        # one seat comes up at a time: its start load shows before the next is admitted
        if any(w.state == "starting" for w in self.workers):
            return
        if self.last_start is not None and self.clock() - self.last_start < self.cfg["start_gap"]:
            return
        w = waiting[0]
        why = self.admit(w.seat)
        if why:
            if w.why != why:
                self.say(f"dispatch seat {w.seat.index}: waits: {why}")
                w.why = why
            if self._live():
                self.blocked_since = None
            elif self.blocked_since is None:
                self.blocked_since = self.clock()
            elif self.clock() - self.blocked_since > ADMIT_TIMEOUT:
                for x in waiting:
                    x.state = "retired"
                    x.why = f"never admitted: {why}"
                self.say(f"dispatch no seat could start in {ADMIT_TIMEOUT:.0f} s: {why}")
            return
        w.why = ""
        self.blocked_since = None
        self._start(w)

    def _assign(self):
        # a parked seat takes work again when no other seat can
        if self.queue and not any(w.state in ("busy", "ready", "starting", "waiting")
                                  for w in self.workers):
            for w in self.workers:
                if w.state == "parked":
                    w.state = "ready"
                    self.limit = max(self.limit, 1)
                    self.say(f"dispatch seat {w.seat.index}: unparked (no other seat is left)")
                    break
        active = sum(1 for w in self.workers if w.state == "busy")
        for w in self.workers:
            if not self.queue or active >= self.limit:
                return
            if w.state == "ready":
                self._hand(w)
                active += 1

    def _live(self):
        return [w for w in self.workers if w.state in ("starting", "ready", "busy", "parked")]

    def run(self) -> int:
        try:
            while True:
                self._admit_next()
                self._assign()
                if not self.queue and not any(w.state == "busy" for w in self.workers):
                    break
                if self.queue and not self._live() and not any(
                        w.state == "waiting" for w in self.workers):
                    self.failed = [s.key for s, _ in self.queue]
                    self.say(f"dispatch every seat is retired; {len(self.queue)} run(s) not "
                             f"run: {', '.join(self.failed)}")
                    break
                for key, _ in self.sel.select(timeout=POLL):
                    self._read(key.data)
                if any(w.state == "busy" for w in self.workers):
                    now = self.load()
                    for w in self.workers:
                        if w.state == "busy":
                            w.loads.append(now)
        except KeyboardInterrupt:
            self.say("dispatch interrupted: stopping every seat's run and stack")
            self.stop(now=True)
            raise
        finally:
            self.stop()
        if self.suite_dir is not None:
            csv_path, _ = runsmod.summarize(self.suite_dir)
            self.say(f"summary: {csv_path}")
        if self.failed:
            from scensim.run import EXIT_INFRA
            self.worst = max(self.worst, EXIT_INFRA)
        return self.worst

    def stop(self, grace: float = STOP_GRACE, now: bool = False):
        """Every worker told to stop (its stack with it), then SIGTERM, then SIGKILL, by
        process group (each worker is its own session). ``now``: SIGTERM at once (an
        interrupt: a worker in a run reads no order until the run ends)."""
        procs = [w.proc for w in self.workers if w.proc is not None and w.proc.poll() is None]
        if now:
            for p in procs:
                try:
                    os.killpg(p.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
        for w in self.workers:
            if w.proc is not None and w.proc.poll() is None:
                self._send(w, stop=True)
                try:
                    w.proc.stdin.close()
                except OSError:
                    pass
        deadline = time.monotonic() + grace
        for p in procs:
            try:
                p.wait(timeout=max(0.1, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(p.pid, signal.SIGTERM)
                    p.wait(timeout=grace)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    try:
                        os.killpg(p.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass


def worker_argv_for(project: Path, logs, ready_timeout: float, stack_timeout, keep: bool,
                    testbench: dict = None):
    def argv(seat) -> list:
        a = [sys.executable, "-m", WORKER_MODULE, "worker", "--project", str(project),
             "--seat", str(seat.index), "--ready-timeout", str(ready_timeout)]
        if logs:
            a += ["--logs", str(logs)]
        if stack_timeout:
            a += ["--stack-timeout", str(stack_timeout)]
        if keep:
            a.append("--keep-stack")
        if testbench:
            a += ["--testbench", json.dumps(testbench)]
        return a
    return argv


def main(argv=None) -> int:
    args = sys.argv[1:] if argv is None else argv
    if args[:1] == ["worker"]:
        return worker_main(args[1:])
    sys.stderr.write("usage: python -m scensim.dispatch worker --project P --seat N ...\n")
    return 3


if __name__ == "__main__":
    sys.exit(main())
