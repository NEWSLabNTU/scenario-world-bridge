"""Attachments: processes tied to a run, which read the truth stream (roadmap 092 K4).

docs/design/ml-training-interface.md section 6.4. A testbench lists them in ``testbench.yaml``
(only there: the stack publishes the truth topic and runs the tick gate for them)::

    attachments:
      - name: netsim                       # attachments/netsim/ in each run directory
        run: [python3, -m, my_v2x.mobility]
        lockstep: false                    # true: it follows the world's ticks (CARLA's tick gate)
        required: false                    # true: its failure makes the run an error run
        stop_timeout: 10                   # s it has to end after SIGINT, before SIGTERM
        lifetime: run                      # the only lifetime for now

Each one starts for a run once its pilots are ready, before SSv2, so it sees the first frame,
and is stopped after SSv2 ends, before the judges: SIGINT, then SIGTERM, to its whole process
group. It runs in the project directory with the scenario domain's ROS environment and:

- ``SCENSIM_RUN_DIR``: the run directory; ``SCENSIM_ATTACHMENT_DIR``: its own directory in it,
  ``attachments/<name>/``, for what it writes (kept with the run's logs, ``standard`` and up);
- ``SCENSIM_RUN_SEED``: the run's seed, for the attachment's own randomness;
- ``SCENSIM_ROLES``: JSON, each entity of the scenario to its role (``ego``, or the pilot's kind,
  or ``scenario`` for an entity SSv2 drives);
- ``SCENSIM_TRUTH_TOPIC``: ``/scensim/truth`` (and ``/scensim/truth/entities``, latched);
- with ``lockstep: true``, ``SCENSIM_TICK_GATE``: the world server's tick gate (``host:port``),
  which it joins as a follower (``HELLO``, ``DONE <frame>`` after each frame, ``BYE``;
  docs/design/time-and-ticking.md). CARLA only: Isaac's world server has no gate yet.

What became of each is recorded in ``result.json`` under ``attachments``: when it started and
stopped, its exit code, and whether it ended before the run did. It never changes the run's
verdict, as a judge never does, unless ``required: true``: then an attachment that ends early,
or exits other than 0 or by the SIGINT it was sent, makes the run an error run, which rates
leave out (a V2X study whose network simulator died mid-run must not count that run).
"""
import json
import os
import signal
import subprocess
import time
from pathlib import Path

KEYS = ("name", "run", "lockstep", "required", "stop_timeout", "lifetime")
LIFETIMES = ("run",)
DEFAULT_STOP_TIMEOUT = 10.0
DIR = "attachments"
TRUTH_TOPIC = "/scensim/truth"


def problems(attachments, simulator=None) -> list:
    """Every way a testbench's ``attachments:`` list is wrong, as ``(where, message)``."""
    if attachments is None:
        return []
    if not isinstance(attachments, list):
        return [("attachments", "is not a list")]
    out, names = [], set()
    for i, a in enumerate(attachments):
        w = f"attachments[{i}]"
        if not isinstance(a, dict):
            out.append((w, "is not a mapping"))
            continue
        for k in a:
            if k not in KEYS:
                out.append((w, f"unknown key {k!r} (known: {', '.join(KEYS)})"))
        name = a.get("name")
        if not isinstance(name, str) or not name or "/" in name or name.startswith("."):
            out.append((w, "name: a plain name is required (it names attachments/<name>/)"))
        elif name in names:
            out.append((w, f"name {name!r} is given twice"))
        else:
            names.add(name)
        run = a.get("run")
        if not (isinstance(run, list) and run and all(isinstance(x, str) and x for x in run)):
            out.append((w, "run: a command as a list of strings is required"))
        for k in ("lockstep", "required"):
            if k in a and not isinstance(a[k], bool):
                out.append((w, f"{k}: {a[k]!r} is not true or false"))
        st = a.get("stop_timeout")
        if st is not None and (isinstance(st, bool) or not isinstance(st, (int, float)) or st <= 0):
            out.append((w, f"stop_timeout: {st!r} is not a positive number of seconds"))
        lt = a.get("lifetime", "run")
        if lt not in LIFETIMES:
            out.append((w, f"lifetime: {lt!r}: only 'run' is built (an attachment that lives for "
                           "a lab, as a network simulator would, is an open question: "
                           "ml-training-interface.md section 9, question 6)"))
        if a.get("lockstep") and simulator not in (None, "carla"):
            out.append((w, f"lockstep: the {simulator} world server has no tick gate yet "
                           "(CARLA's has); remove lockstep or run on CARLA"))
    return out


def lockstep(attachments) -> bool:
    """Whether any attachment follows the world's ticks."""
    return any(isinstance(a, dict) and a.get("lockstep") for a in attachments or [])


def env_for(a: dict, base: dict, run_dir: Path, seed, roles: dict, tick_gate=None) -> dict:
    """The environment attachment `a` runs with: `base` (the scenario domain's ROS environment)
    plus scensim's variables."""
    env = dict(base)
    env.update({
        "SCENSIM_RUN_DIR": str(run_dir),
        "SCENSIM_ATTACHMENT_DIR": str(run_dir / DIR / a["name"]),
        "SCENSIM_RUN_SEED": "" if seed is None else str(seed),
        "SCENSIM_ROLES": json.dumps(roles, sort_keys=True),
        "SCENSIM_TRUTH_TOPIC": TRUTH_TOPIC,
    })
    if a.get("lockstep") and tick_gate:
        env["SCENSIM_TICK_GATE"] = tick_gate
    return env


class Running:
    """One attachment's process for one run."""

    def __init__(self, a: dict, proc, log: Path, started: float):
        self.a, self.proc, self.log, self.started = a, proc, log, started
        self.stopped = None
        self.code = None
        self.early = False     # it had ended before scensim stopped it
        self.error = ""

    @property
    def name(self):
        return self.a["name"]

    def failed(self) -> str:
        """Why a required attachment fails the run, or ''."""
        if self.error:
            return self.error
        if self.early:
            return f"ended before the run did (exit {self.code}; see {self.log})"
        if self.code not in (0, -signal.SIGINT, 128 + signal.SIGINT, 130):
            return f"exited {self.code} when stopped (see {self.log})"
        return ""

    def record(self) -> dict:
        return {"started": self.started, "stopped": self.stopped, "exit": self.code,
                "ended_early": self.early, "required": bool(self.a.get("required")),
                **({"error": self.error} if self.error else {})}


def start(a: dict, env: dict, run_dir: Path, project: Path) -> Running:
    """Start attachment `a` for a run, in its own process group; its console goes to
    ``attachments/<name>/console.log``."""
    d = run_dir / DIR / a["name"]
    d.mkdir(parents=True, exist_ok=True)
    log = d / "console.log"
    t = time.time()
    try:
        with open(log, "ab") as f:
            proc = subprocess.Popen(a["run"], cwd=str(project), env=env, stdout=f,
                                    stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                    start_new_session=True)
    except OSError as e:
        r = Running(a, None, log, t)
        r.error = f"did not start: {e}"
        r.stopped = t
        return r
    return Running(a, proc, log, t)


def stop(r: Running) -> Running:
    """Stop `r` (SIGINT, then SIGTERM to its process group after its stop timeout, then
    SIGKILL), recording whether it had already ended."""
    if r.proc is None:
        return r
    code = r.proc.poll()
    if code is not None:
        r.early, r.code, r.stopped = True, code, time.time()
        return r
    grace = float(r.a.get("stop_timeout") or DEFAULT_STOP_TIMEOUT)
    for sig, wait in ((signal.SIGINT, grace), (signal.SIGTERM, 5.0), (signal.SIGKILL, 5.0)):
        try:
            os.killpg(r.proc.pid, sig)
        except ProcessLookupError:
            break
        try:
            r.proc.wait(timeout=wait)
            break
        except subprocess.TimeoutExpired:
            continue
    r.code, r.stopped = r.proc.poll(), time.time()
    return r


def roles(entities, pilots) -> dict:
    """Each scenario entity to its role: ``ego``, the pilot's kind, or ``scenario``."""
    by_entity = {p["entity"]: p for p in pilots}
    out = {}
    for e in entities:
        if getattr(e, "ego", False):
            out[e.name] = "ego"
        elif e.name in by_entity:
            out[e.name] = by_entity[e.name].get("kind") or "pilot"
        else:
            out[e.name] = "scenario"
    return out
