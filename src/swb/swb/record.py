"""A run's recording: rosbags of its pilots' key topics, at capture ``full`` and ``debug``.

docs/design/projects-and-pilots.md, "Logs". One ``ros2 bag record`` per pilot domain, started
when every pilot is ready and stopped (SIGINT, so the bag closes) when SSv2 ends, into
``runs/<id>/record/<entity>/``. ``full`` records the key topics (``KEY_TOPICS``, or the
project's ``logging.topics``); ``debug`` records every topic in each pilot's domain and in the
scenario domain (``record/scenario/``). A topic that never appears is simply absent.
"""
import sys
from pathlib import Path

KEY_TOPICS = (
    "/localization/kinematic_state",            # the ego's state, as the autopilot sees it
    "/control/command/control_cmd",              # what it commands
    "/vehicle/status/velocity_status",
    "/vehicle/status/steering_status",
    "/perception/object_recognition/objects",    # what it detects
    "/planning/scenario_planning/trajectory",
    "/swb/wheels",                               # Isaac's wheel telemetry
)
LEVELS = ("full", "debug")


def topics_problems(topics) -> list:
    if topics is None:
        return []
    if not (isinstance(topics, list) and topics
            and all(isinstance(t, str) and t.startswith("/") for t in topics)):
        return ["topics: a non-empty list of topic names (each starting with /)"]
    return []


def argv(out_dir: Path, level: str, topics=None) -> list:
    """The recorder's command line for one domain."""
    base = ["ros2", "bag", "record", "-o", str(out_dir)]
    if level == "debug":
        return base + ["-a"]
    return base + list(topics or KEY_TOPICS)


CONTEXT_LOG = "context.jsonl"


def context_argv(run_dir: Path) -> list:
    """The storyboard log's recorder (swb.context), into ``<run>/context.jsonl``."""
    return [sys.executable, "-m", "swb.context", str(Path(run_dir) / CONTEXT_LOG)]


def plan(run_dir: Path, level: str, pilots, scenario_domain, topics=None,
         context=False) -> list:
    """``[(name, domain, argv)]`` for a run: one per pilot domain (pilots sharing a domain
    share a recorder, named after the first), plus the scenario domain at ``debug``; and, at
    any level, the storyboard log in the scenario domain when ``context`` (a judge
    ``needs: [context]``)."""
    out = []
    if context and scenario_domain is not None:
        out.append(("context", scenario_domain, context_argv(run_dir)))
    if level not in LEVELS:
        return out
    rec_dir = Path(run_dir) / "record"
    seen = set()
    for p in pilots:
        d = p.get("domain")
        if d is None or d in seen:
            continue
        seen.add(d)
        out.append((p["entity"], d, argv(rec_dir / p["entity"], level, topics)))
    if level == "debug" and scenario_domain is not None and scenario_domain not in seen:
        out.append(("scenario", scenario_domain, argv(rec_dir / "scenario", level)))
    return out
