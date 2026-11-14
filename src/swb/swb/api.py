"""swb from Python: one run against the project's live stack, for ask-and-tell searches.

    from swb.api import run_one
    r = run_one("~/my_project", "scenarios/cut_in.xosc", {"parameters": {"gap_m": 6}, "seed": 3})
    r.verdict, r.reason, r.run_id

The same as one run of ``ros2 run swb run`` (swb.run; docs/design/projects-and-pilots.md
"Runs"): it finds the stack of ``project`` on its seat, refuses (``swb.run.Refused``) a missing,
stopped, unready or different stack, and returns the run's ``RunResult`` (also written to
``runs/<id>/result.json``).

``overrides``: ``parameters`` (name -> value), ``repetition``, ``seed``, ``pilot_args``
(entity -> launch arguments for its ``lifetime: run`` pilot), ``logging`` (``capture``,
``keep``), ``timeout`` (SSv2's global_timeout, s), ``seat``, ``logs``, ``wait`` (s to wait for
the stack to be ready), ``ready_timeout``, ``expect``; and for tests ``scenario_command`` /
``pilot_command`` (callables returning the argv to start instead of play_launch).
"""
from pathlib import Path

from swb import run as runmod
from swb.results import RunResult  # noqa: F401  (the return type)
from swb.run import Refused  # noqa: F401

SPEC_KEYS = ("parameters", "repetition", "seed", "pilot_args", "logging", "timeout", "expect")
CONTEXT_KEYS = ("ready_timeout", "scenario_command", "pilot_command", "workspace", "out")


def run_one(project, scenario, overrides=None) -> RunResult:
    o = dict(overrides or {})
    unknown = [k for k in o if k not in SPEC_KEYS + CONTEXT_KEYS + ("seat", "logs", "wait")]
    if unknown:
        raise ValueError(f"run_one: unknown override {unknown[0]!r}")
    ctx = runmod.context(str(Path(project).expanduser()), o.get("seat"), o.get("logs"),
                         o.get("wait") or 0.0, **{k: o[k] for k in CONTEXT_KEYS if k in o})
    path = Path(scenario).expanduser()
    if not path.is_absolute():
        path = (ctx.project / path).resolve()
    if not path.is_file():
        raise Refused(f"{path}: no such scenario")
    spec = runmod.RunSpec(path, dict(o.get("parameters") or {}), int(o.get("repetition") or 0),
                          o.get("seed"), o.get("expect"), o.get("pilot_args") or {},
                          o.get("logging") or {}, timeout=o.get("timeout"))
    return runmod.run_spec(ctx, spec)
