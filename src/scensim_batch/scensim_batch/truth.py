"""The batch's truth per slot (roadmap 092 K2.3; truth-stream.md section 5): every step's entity
states as tensors, exported per slot and episode as a schema-2 truth file (the gap trace's
JSON Lines, which scensim_launch's gap_trace reader reads) with the slot's sample record and
outcome beside it, so any slot's episode replays on seats from its record.

    rec = TruthRecorder(run, map_name="Town01")
    obs = run.reset(); rec.begin(obs)
    for ...: obs = run.step(...); rec.record(obs)
    rec.write(out_dir)                # out_dir/slot_<k>_ep<e>/{truth.jsonl, sample.json, outcome.json}

An episode closes at its slot's exit (success or failure), at the slot's next `begin`, or at
`write`, as running: each file holds every frame recorded (`end.complete` true), and
outcome.json says how the episode ended and whether its storyboard had completed or stopped.

Values: the ego's pose and speed are the engine's (`velocity_source` measured); SSv2's default
vehicles and props are the batch's kinematic ones (`derived`, flag `kinematic`). A frame's
`ssv2` pose is the batch's own: in the batch SSv2's intent and the world are the same. Contact
lines are the ego's box overlaps, at their onset, impulse unknown (0). The episode line's
`npc_motion` is `kinematic`.
"""
import json
import math
import time
from pathlib import Path

import torch

from . import storyboard as sbmod

KIND = {"vehicle": "vehicle", "pedestrian": "pedestrian", "misc": "misc_object"}


class _Frame:
    """storyboard.overlap's context over recorded rows."""

    def __init__(self, run, row):
        self.run, self.n, self.row = run, run.c.n, row

    def col(self, name, i):
        return self.row[name][:, i]


class TruthRecorder:
    def __init__(self, run, map_name="", step_time=None, start_unix=None):
        self.run, self.map, self.step_time = run, map_name, step_time
        self.start_unix = time.time() if start_unix is None else start_unix
        n = run.c.n
        self.rows = []                       # per step: {x, y, yaw, v [N, E], t [N], ...}
        self.start = [0] * n                 # each slot's episode's first row
        self.episode = [0] * n
        self.done = []                       # (slot, episode, first row, last row, outcome)
        self.open = [False] * n

    def _row(self, obs):
        ego = obs["ego"]
        return {"x": obs["x"].clone(), "y": obs["y"].clone(), "yaw": obs["yaw"].clone(),
                "v": obs["v"].clone(), "t": self.run.ctx.t.clone(),
                "yaw_rate": ego["yaw_rate"].clone(),
                "outcome": (obs["outcome"].clone() if "outcome" in obs
                            else torch.zeros(self.run.c.n, dtype=torch.int8)),
                "complete": obs.get("storyboard_complete",
                                    torch.zeros(self.run.c.n, dtype=torch.bool)).clone(),
                "stopped": obs.get("storyboard_stopped",
                                   torch.zeros(self.run.c.n, dtype=torch.bool)).clone()}

    def begin(self, obs, mask=None):
        """Start an episode for the slots in `mask` (all) at the reset's observation."""
        n = self.run.c.n
        mask = torch.ones(n, dtype=torch.bool) if mask is None else torch.as_tensor(mask)
        self.rows.append(self._row(obs))
        for k in torch.nonzero(mask).flatten().tolist():
            if self.open[k]:
                self._close(k, len(self.rows) - 2, "running")
                self.episode[k] += 1
            self.start[k] = len(self.rows) - 1
            self.open[k] = True

    def record(self, obs):
        """One step's observation; a slot whose outcome is no longer running closes its episode."""
        self.rows.append(self._row(obs))
        last = len(self.rows) - 1
        out = self.rows[-1]["outcome"]
        for k in range(self.run.c.n):
            if self.open[k] and int(out[k]) != 0:
                self._close(k, last, sbmod.OUTCOMES[int(out[k])])

    def _close(self, k, last, outcome):
        self.done.append((k, self.episode[k], self.start[k], last, outcome))
        self.open[k] = False

    def write(self, out_dir):
        """Every closed episode, and every open one (`complete` false), as files. Returns their
        directories."""
        out = Path(out_dir)
        eps = list(self.done) + [(k, self.episode[k], self.start[k], len(self.rows) - 1,
                                  "running") for k in range(self.run.c.n) if self.open[k]]
        dirs = []
        for k, e, first, last, outcome in eps:
            d = out / f"slot_{k}_ep{e}"
            d.mkdir(parents=True, exist_ok=True)
            (d / "truth.jsonl").write_text("".join(json.dumps(x) + "\n" for x in
                                                   self.lines(k, e, first, last, outcome)))
            c = self.run.c
            rec = c.records[k] if k < len(c.records) else {}
            (d / "sample.json").write_text(json.dumps(rec, indent=1) + "\n")
            row = self.rows[last]
            (d / "outcome.json").write_text(json.dumps({
                "outcome": outcome, "slot": k, "episode": e, "scenario": c.scenario,
                "storyboard_complete": bool(row["complete"][k]),
                "storyboard_stopped": bool(row["stopped"][k]),
                "seed": c.seeds[k] if k < len(c.seeds) else None,
                "frames": last - first + 1}, indent=1) + "\n")
            dirs.append(d)
        return dirs

    def lines(self, k, e, first, last, outcome):
        run, c = self.run, self.run.c
        dt = self.step_time
        if dt is None:
            dt = (float(self.rows[first + 1]["t"][k] - self.rows[first]["t"][k])
                  if last > first else 0.0)
        yield {"type": "episode", "schema": 2, "episode": e, "start_unix": self.start_unix,
               "npc_motion": "kinematic", "map": self.map, "step_time": dt}
        for i, ent in enumerate(c.entities):
            yield {"type": "entity", "frame": 0, "name": ent.name, "index": i,
                   "role": "ego" if i == run.ego else "scenario",
                   "kind": "ego" if i == run.ego else KIND[ent.kind],
                   "subtype": {"vehicle": "car", "pedestrian": "pedestrian",
                               "misc": "misc"}[ent.kind],
                   "declared": {"center": [ent.cx, ent.cy, ent.height / 2],
                                "dimensions": [ent.length, ent.width, ent.height]},
                   "origin_offset": [0.0, 0.0, 0.0], "sim_id": i, "asset": "scensim_batch",
                   "asset_box": None}
        touching = set()
        for f, r in enumerate(range(first, last + 1)):
            row = self.rows[r]
            t = float(row["t"][k])
            ents = {}
            for i, ent in enumerate(c.entities):
                x, y, h, v = (float(row[q][k, i]) for q in ("x", "y", "yaw", "v"))
                pose = [x, y, 0.0, 0.0, 0.0, h]
                if i == run.ego:
                    w, src, flags = float(row["yaw_rate"][k]), "measured", []
                else:
                    prev = self.rows[r - 1] if r > first else row
                    dh = math.remainder(h - float(prev["yaw"][k, i]), 2 * math.pi)
                    w, src, flags = (dh / dt if dt else 0.0), "derived", ["kinematic"]
                if f == 0:
                    flags = ["spawn"] + flags
                ents[ent.name] = {"ssv2": pose, "sim_pose": pose,
                                  "velocity": [v * math.cos(h), v * math.sin(h), 0.0],
                                  "angular_velocity": [0.0, 0.0, w], "velocity_source": src,
                                  "flags": flags}
            yield {"type": "frame", "frame": f, "sim_time": t,
                   "sim_time_ns": int(round(t * 1e9)), "sim_frame": f, "entities": ents}
            ctx = _Frame(run, row)
            for i, ent in enumerate(c.entities):
                if i == run.ego:
                    continue
                hit = bool(sbmod.overlap(ctx, run.ego, i)[k])
                if hit and i not in touching:
                    yield {"type": "contact", "frame": f, "sim_frame": f, "sim_time": t,
                           "ego_id": run.ego, "subject": c.entities[run.ego].name,
                           "other_id": i, "name": ent.name, "other_type": "scensim_batch",
                           "other_kind": KIND[ent.kind], "impulse": 0.0}
                (touching.add if hit else touching.discard)(i)
        yield {"type": "end", "frame": last - first,
               "sim_time": float(self.rows[last]["t"][k]), "frames": last - first + 1,
               "dropped_frames": 0, "complete": True}
