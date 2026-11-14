"""The bridge's frame budget in a run's slice, as ``result.json`` metrics.

The bridge logs, every 200 SSv2 frames (10 s at 20 Hz), one INFO line per window
(docs/design/failure-and-frame-budget.md, "Frame budget")::

    Frame budget, last 200 frames: 200 frames, p50/p95/max ms: processing 0.23/0.28/1.59,
      tick 1.26/1.57/2.03, UpdateEntityStatus 0.13/0.17/0.23; entities 1/1/1;
      processing > 25 ms: 0, tick > 100 ms: 0

*processing* is the bridge's own time per frame, *tick* the world server's step. The whole
run's line ("Frame budget, whole run: ...") is logged at the next ``Initialize``, so it lands
in the next run's slice and is not this run's; it is left out. A run's numbers are therefore
its full windows': the last window, shorter than 200 frames, is not logged, and a run shorter
than one window has none (the metrics are then absent, not zero).

Metrics: ``frame_windows``, ``frame_count`` (the frames the windows cover), ``frame_p50_ms``
(the frame-weighted median of the windows' p50s), ``frame_p95_ms`` and ``frame_max_ms`` (the
largest window p95 and max: a bound, never below the run's own), ``tick_p50_ms``,
``tick_p95_ms``, ``tick_max_ms`` (the same for the tick), ``frames_over_budget`` and
``ticks_over_budget`` (frames over 25 ms of processing, ticks over 100 ms).
"""
import re
from pathlib import Path

from swb import slices

GROUP = "bridge"
_ANSI = re.compile(r"\x1b\[[0-9;]*m")
_STAMP = re.compile(r"\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d(\.\d+)?Z?")
_TRIPLE = r"([0-9.]+)/([0-9.]+)/([0-9.]+)"
_WINDOW = re.compile(
    r"Frame budget, last \d+ frames: (\d+) frames, p50/p95/max ms: processing " + _TRIPLE
    + r", tick " + _TRIPLE + r".*?processing > \d+ ms: (\d+), tick > \d+ ms: (\d+)")


def windows(text: str) -> list:
    """One dict per window line in ``text``."""
    out, seen = [], set()
    for line in _ANSI.sub("", text).splitlines():
        m = _WINDOW.search(line)
        # the same line in two of the group's files (a node's output and a launch's copy of
        # it) is one window: each line carries its own timestamp
        if not m:
            continue
        ts = _STAMP.search(line)
        key = (ts.group(0) if ts else line.strip(), m.group(0))
        if key in seen:
            continue
        seen.add(key)
        g = m.groups()
        out.append({"frames": int(g[0]),
                    "processing": tuple(float(x) for x in g[1:4]),
                    "tick": tuple(float(x) for x in g[4:7]),
                    "over_processing": int(g[7]), "over_tick": int(g[8])})
    return out


def _weighted_median(pairs):
    pairs = sorted(pairs)
    total = sum(w for _, w in pairs)
    acc = 0
    for v, w in pairs:
        acc += w
        if acc * 2 >= total:
            return v
    return None


def metrics(ws: list) -> dict:
    """The run's metrics from its windows; {} when there are none."""
    if not ws:
        return {}
    out = {"frame_windows": len(ws), "frame_count": sum(w["frames"] for w in ws)}
    for name, key in (("frame", "processing"), ("tick", "tick")):
        out[f"{name}_p50_ms"] = _weighted_median([(w[key][0], w["frames"]) for w in ws])
        out[f"{name}_p95_ms"] = max(w[key][1] for w in ws)
        out[f"{name}_max_ms"] = max(w[key][2] for w in ws)
    out["frames_over_budget"] = sum(w["over_processing"] for w in ws)
    out["ticks_over_budget"] = sum(w["over_tick"] for w in ws)
    return out


def slice_text(mark0: slices.LogMark, mark1: slices.LogMark, group: str = GROUP) -> str:
    """The text every log file of ``group`` gained between the marks."""
    root = Path(mark1.root)
    parts = []
    for rel, entry in sorted(mark1.files.items()):
        if slices.group_of(rel)[0] != group:
            continue
        begin, stop = slices._start_offset(rel, mark0, entry), entry[0]
        if stop <= begin:
            continue
        try:
            with open(root / rel, "rb") as f:
                f.seek(begin)
                parts.append(f.read(stop - begin).decode("utf-8", "replace"))
        except OSError:
            continue
    return "\n".join(parts)


def from_marks(mark0, mark1, group: str = GROUP) -> dict:
    return metrics(windows(slice_text(mark0, mark1, group)))
