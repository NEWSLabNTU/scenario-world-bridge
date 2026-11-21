"""The run's truth file for judges (roadmap 092 K4, docs/design/truth-stream.md).

Each run's truth is the bridge's gap trace in ``runs/<id>/gap_trace/``, one file per episode,
recorded by default. A judge reads it through this module::

    from scensim import truth
    for trace in truth.load(run_dir):          # gap_trace.Trace, schema 2's names for either schema
        for frame in trace.frames:
            ego = frame.entities["ego"].sim_pose

and ``truth.boxes(trace, frame)`` gives each entity's declared box where the simulator had it.

A reference judge, the ego's closest approach to any other entity, box to box::

    judges:
      - name: closest
        run: [python3, -m, scensim.truth, closest, --min-gap, "0.5"]

It measures between the scenario's declared boxes, placed at the simulator's poses (the body
pose with the entity's origin offset taken back off), so the gap is the clearance between the
boxes SSv2 judges collisions with, where the simulator put them. Its verdict is ``failed`` when
that gap goes below ``--min-gap`` (metres; negative is penetration), naming the entity, frame and
time. It prints one JSON object (scensim.judges).

``truth.clock(traces, mode, max_lag_s, rtf)`` is the run's clock record (time-and-ticking.md,
lockstep and paced): each frame's lag behind wall time, from the bridge's ``wall_ns``.
"""
import argparse
import importlib.util
import json
import math
import sys
from pathlib import Path

from scensim import episodes

_MODULES = {}


def _module(name: str):
    """A library module of scensim_launch's scripts (gap_trace, gap_geometry): installed, else
    this workspace's source tree."""
    if name in _MODULES:
        return _MODULES[name]
    from scensim import gap
    try:
        here = Path(gap.tool("gap_oracle")[1]).parent
    except FileNotFoundError:
        raise FileNotFoundError(f"{name}: scensim_launch is neither installed nor beside scensim")
    spec = importlib.util.spec_from_file_location(name, here / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules.setdefault(name, mod)
    spec.loader.exec_module(mod)
    _MODULES[name] = mod
    return mod


def files(run_dir) -> list:
    """The run's truth files, one per episode, in order."""
    return sorted((Path(run_dir) / episodes.GAP_TRACE).glob("*.jsonl"))


def load(run_dir, require_end=False) -> list:
    """Every episode's trace of the run (a trace cut short reads, without its end line)."""
    gt = _module("gap_trace")
    return [gt.load_trace(f, require_end=require_end) for f in files(run_dir)]


def boxes(trace, frame) -> dict:
    """Each entity of `frame` the simulator had: its declared box (gap_geometry.OrientedBox)
    at the simulator's pose, the entity's origin offset taken back off."""
    gg = _module("gap_geometry")
    out = {}
    for name, s in frame.entities.items():
        h = trace.entities.get(name)
        if h is None or s.sim_pose is None:
            continue
        body = gg.Pose2D(s.sim_pose.x, s.sim_pose.y, s.sim_pose.yaw)
        o = h.origin_offset
        entity = gg.entity_pose_from_actor(body, o.x, o.y, o.yaw)
        out[name] = gg.ssv2_box(entity, h.declared.center, h.declared.dimensions)
    return out


def closest(traces) -> dict:
    """The ego's smallest box-to-box gap to any other entity over the traces: ``{gap, entity,
    frame, sim_time, episode}``, or None without an ego or another entity."""
    gg = _module("gap_geometry")
    best = None
    for t in traces:
        egos = [h.name for h in t.headers if h.kind == "ego"]
        if not egos:
            continue
        for f in t.frames:
            b = boxes(t, f)
            for ego in egos:
                if ego not in b:
                    continue
                for name, box in b.items():
                    if name == ego:
                        continue
                    d = gg.signed_distance(b[ego], box)
                    if best is None or d < best["gap"]:
                        best = {"gap": d, "ego": ego, "entity": name, "frame": f.frame,
                                "sim_time": f.sim_time, "episode": t.episode.episode}
    return best


CLOCK_MODES = ("lockstep", "paced")


def lags(trace, rtf=1.0) -> list:
    """Each frame's lag behind wall time, seconds, from the trace's first frame that has a wall
    stamp: ``(wall_k - wall_0) - (sim_k - sim_0) / rtf``, positive when the world is behind.
    Frames without ``wall_ns`` (a trace written before it existed) are skipped."""
    out, base = [], None
    for f in trace.frames:
        if f.wall_ns is None:
            continue
        sim = f.sim_time_ns if f.sim_time_ns is not None else round(f.sim_time * 1e9)
        if base is None:
            base = (f.wall_ns, sim)
        out.append(((f.wall_ns - base[0]) - (sim - base[1]) / rtf) / 1e9)
    return out


def settings(project_settings) -> dict:
    """testbench.yaml's ``clock`` with its defaults: {mode, max_lag_s, rtf}."""
    c = (project_settings or {}).get("clock") or {}
    ssv2 = (project_settings or {}).get("ssv2") or {}
    return {"mode": c.get("mode", "lockstep"), "max_lag_s": c.get("max_lag_s"),
            "rtf": float(ssv2.get("global_real_time_factor", 1.0) or 1.0)}


def problems(project_settings) -> list:
    """testbench.yaml's ``clock``, checked: (where, message) pairs."""
    ps = project_settings or {}
    c = ps.get("clock")
    if c is None:
        return []
    if not isinstance(c, dict):
        return [("clock", "is {mode: lockstep|paced, max_lag_s: seconds}")]
    out = [(f"clock.{k}", "unknown key (mode, max_lag_s)") for k in c if k not in ("mode", "max_lag_s")]
    mode = c.get("mode", "lockstep")
    if mode not in CLOCK_MODES:
        out.append(("clock.mode", f"is lockstep or paced, not {mode!r}"))
    lag = c.get("max_lag_s")
    if lag is not None and not (isinstance(lag, (int, float)) and not isinstance(lag, bool)
                                and lag > 0 and math.isfinite(lag)):
        out.append(("clock.max_lag_s", "is a positive number of seconds"))
    if mode == "paced":
        if lag is None:
            out.append(("clock.max_lag_s", "a paced clock needs its budget, seconds of lag"))
        rtf = ((ps.get("ssv2") or {}).get("global_real_time_factor", 1.0))
        if rtf != 1 and rtf != 1.0:
            out.append(("clock.mode", f"paced holds simulated time to wall time, so "
                                      f"ssv2.global_real_time_factor must be 1, not {rtf}"))
        if ((ps.get("logging") or {}).get("gap_trace")) is False:
            out.append(("clock.mode", "paced measures its lag from the truth file: "
                                      "logging.gap_trace: false cannot be paced"))
    return out


def _rank(sorted_xs, q):
    return sorted_xs[min(len(sorted_xs) - 1, max(0, math.ceil(q * len(sorted_xs)) - 1))]


def clock(traces, mode="lockstep", max_lag_s=None, rtf=1.0) -> dict:
    """The result's ``clock``, or None when no frame has a wall stamp. ``ok`` is false only in
    paced mode, when a frame's lag exceeds ``max_lag_s``; a lockstep run's lag is a record."""
    xs = [x for t in traces for x in lags(t, rtf)]
    if not xs:
        return None
    srt = sorted(xs)
    over = sum(x > max_lag_s for x in xs) if max_lag_s is not None else 0
    out = {"mode": mode, "frames": len(xs),
           "lag_s": {"max": round(srt[-1], 6), "p50": round(_rank(srt, 0.5), 6),
                     "p99": round(_rank(srt, 0.99), 6), "final": round(xs[-1], 6)},
           "frames_over": over, "ok": not (mode == "paced" and over > 0)}
    if max_lag_s is not None:
        out["max_lag_s"] = max_lag_s
    if rtf != 1.0:
        out["global_real_time_factor"] = rtf
    return out


def _closest(args) -> int:
    traces = load(args.run_dir)
    if not traces:
        print(json.dumps({"verdict": "error", "message": f"no truth file in {args.run_dir}/"
                          f"{episodes.GAP_TRACE}/ (logging.gap_trace: false?)"}))
        return 2
    c = closest(traces)
    if c is None:
        print(json.dumps({"verdict": "passed", "message": "no ego and other entity to measure",
                          "measure": "box to box"}))
        return 0
    gap = round(c["gap"], 3)
    failed = c["gap"] < args.min_gap
    print(json.dumps({
        "verdict": "failed" if failed else "passed",
        "message": (f"{c['ego']} came within {gap} m of {c['entity']} (frame {c['frame']}, "
                    f"{c['sim_time']:.2f} s), below {args.min_gap} m" if failed else
                    f"closest {gap} m, to {c['entity']} at {c['sim_time']:.2f} s"),
        "measure": "box to box: the declared boxes at the simulator's poses",
        "min_gap": args.min_gap, "closest": dict(c, gap=gap),
    }))
    return 1 if failed else 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python3 -m scensim.truth", description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("closest", help="the ego's closest approach to any entity, box to box")
    c.add_argument("--min-gap", type=float, default=0.0,
                   help="fail below this gap, m (default 0: fail on contact)")
    c.add_argument("run_dir", type=Path)
    args = ap.parse_args(argv)
    if math.isnan(args.min_gap):
        ap.error("--min-gap: not a number")
    return _closest(args)


if __name__ == "__main__":
    sys.exit(main())
