"""scensim from Python: one run against the project's live stack, for ask-and-tell searches.

    from scensim.api import run_one
    r = run_one("~/my_project", "scenarios/cut_in.xosc", {"parameters": {"gap_m": 6}, "seed": 3})
    r.verdict, r.reason, r.run_id

The same as one run of ``ros2 run scensim run`` (scensim.run; docs/design/projects-and-pilots.md
"Runs"): it finds the stack of ``project`` on its seat, refuses (``scensim.run.Refused``) a missing,
stopped, unready or different stack, and returns the run's ``RunResult`` (also written to
``runs/<id>/result.json``).

``overrides``: ``parameters`` (name -> value), ``repetition``, ``seed``, ``pilot_args``
(entity -> launch arguments for its ``lifetime: run`` pilot), ``logging`` (``capture``,
``keep``), ``timeout`` (SSv2's global_timeout, s), ``ssv2`` (run settings, scensim.ssv2: over
testbench.yaml's), ``seat``, ``logs``, ``wait`` (s to wait for the stack to be ready),
``ready_timeout``, ``expect``, ``start_stack`` (true: start the project's stack when none is up, as ``--start-stack``, and leave it up), ``stack_timeout``;
and for tests ``scenario_command`` / ``pilot_command`` / ``stack_command`` (callables
returning the argv to start instead of play_launch).

A search that wants a stack only for its own lifetime holds one with ``stack()``:

    with stack("~/my_project") as s:      # started when none is up
        for x in candidates:
            run_one(s.project, "scenarios/cut_in.xosc", {"parameters": x})
    # stopped here if this block started it (as --stop-stack), else left as it was
"""
import contextlib
from pathlib import Path

from scensim import run as runmod
from scensim.results import RunResult  # noqa: F401  (the return type)
from scensim.run import Refused, StackFailed  # noqa: F401

SPEC_KEYS = ("parameters", "repetition", "seed", "pilot_args", "logging", "timeout", "expect",
             "ssv2")
CONTEXT_KEYS = ("ready_timeout", "scenario_command", "pilot_command", "workspace", "out",
                "lost_probe", "end_episodes")
STACK_KEYS = ("start_stack", "stack_command", "stack_timeout")


def _context(project, o):
    return runmod.context(str(Path(project).expanduser()), o.get("seat"), o.get("logs"),
                          o.get("wait") or 0.0, start=bool(o.get("start_stack")),
                          stack_command=o.get("stack_command"),
                          stack_timeout=o.get("stack_timeout"),
                          **{k: o[k] for k in CONTEXT_KEYS if k in o})


def run_one(project, scenario, overrides=None) -> RunResult:
    o = dict(overrides or {})
    unknown = [k for k in o
               if k not in SPEC_KEYS + CONTEXT_KEYS + STACK_KEYS + ("seat", "logs", "wait")]
    if unknown:
        raise ValueError(f"run_one: unknown override {unknown[0]!r}")
    ctx = _context(project, o)
    path = Path(scenario).expanduser()
    if not path.is_absolute():
        path = (ctx.project / path).resolve()
    if not path.is_file():
        raise Refused(f"{path}: no such scenario")
    spec = runmod.RunSpec(path, dict(o.get("parameters") or {}), int(o.get("repetition") or 0),
                          o.get("seed"), o.get("expect"), o.get("pilot_args") or {},
                          o.get("logging") or {}, timeout=o.get("timeout"),
                          ssv2=dict(o.get("ssv2") or {}))
    try:
        return runmod.run_spec(ctx, spec)
    finally:
        runmod.release_turn(ctx)


@contextlib.contextmanager
def stack(project, seat=None, logs=None, timeout=None, stack_command=None, out=None):
    """The project's stack for the block: the live one, else one started for it and stopped
    when the block ends. Yields the run Context (``.project``, ``.stack``, ``.started``)."""
    ctx = runmod.context(str(Path(project).expanduser()), seat, logs, 0.0, start=True,
                         stack_command=stack_command, stack_timeout=timeout, out=out)
    try:
        yield ctx
    finally:
        if ctx.started is not None:
            ctx.started.stop()
