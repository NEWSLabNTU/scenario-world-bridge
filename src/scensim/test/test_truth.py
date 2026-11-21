"""scensim.truth (roadmap 092 K4): the run's truth file for judges, and the closest-approach judge."""
import json
import math

import pytest

from scensim import truth


def entity(name, kind, index, offset=(0.0, 0.0, 0.0), center=(0.0, 0.0, 0.75), dims=(4.0, 2.0, 1.5)):
    return {"type": "entity", "frame": 0, "name": name, "index": index,
            "role": "ego" if kind == "ego" else "scenario", "kind": kind, "subtype": "car",
            "declared": {"center": list(center), "dimensions": list(dims)},
            "origin_offset": list(offset), "sim_id": 10 + index, "asset": "m", "asset_box": None}


def frame(n, poses):
    return {"type": "frame", "frame": n, "sim_time": 0.05 * n, "sim_time_ns": 10**12 + 5 * 10**7 * n,
            "sim_frame": 100 + n, "entities": {
                k: {"ssv2": [x, y, 0, 0, 0, yaw], "sim_pose": [x, y, 0, 0, 0, yaw],
                    "velocity": [0, 0, 0], "angular_velocity": [0, 0, 0], "velocity_source": "measured"}
                for k, (x, y, yaw) in poses.items()}}


def write(run_dir, lines, name="20261121T000000.000Z_1.jsonl"):
    d = run_dir / "gap_trace"
    d.mkdir(parents=True, exist_ok=True)
    (d / name).write_text("".join(json.dumps(x) + "\n" for x in lines))


EP = {"type": "episode", "schema": 2, "episode": 1, "start_unix": 0.0, "npc_motion": "tracked",
      "map": "Town01", "step_time": 0.05}
END = {"type": "end", "frame": 3, "sim_time": 0.15, "frames": 3, "dropped_frames": 0, "complete": True}


def test_closest_is_box_to_box_with_the_offset_taken_off(tmp_path):
    # the npc's body sits 1.5 m ahead of its entity origin (offset), so its box is where the
    # entity origin is; ego at 0, npc's body at 7.5 => entity at 6.0; boxes 4 m long: gap 2.0
    write(tmp_path, [EP, entity("ego", "ego", 0), entity("npc", "vehicle", 1, offset=(1.5, 0, 0)),
                     frame(1, {"ego": (0, 0, 0), "npc": (9.5, 0, 0)}),
                     frame(2, {"ego": (0, 0, 0), "npc": (7.5, 0, 0)}),
                     frame(3, {"ego": (0, 0, 0), "npc": (8.5, 0, 0)}), END])
    c = truth.closest(truth.load(tmp_path))
    assert c["entity"] == "npc" and c["frame"] == 2 and math.isclose(c["gap"], 2.0, abs_tol=1e-9)


def test_the_judge_passes_and_fails_by_min_gap(tmp_path, capsys):
    write(tmp_path, [EP, entity("ego", "ego", 0), entity("npc", "vehicle", 1),
                     frame(1, {"ego": (0, 0, 0), "npc": (4.3, 0, 0)}), END])
    assert truth.main(["closest", "--min-gap", "0.2", str(tmp_path)]) == 0
    out = json.loads(capsys.readouterr().out)
    assert out["verdict"] == "passed" and out["measure"].startswith("box to box")
    assert truth.main(["closest", "--min-gap", "0.5", str(tmp_path)]) == 1
    out = json.loads(capsys.readouterr().out)
    assert out["verdict"] == "failed" and "within 0.3 m of npc" in out["message"]
    # overlapping: negative
    write(tmp_path, [EP, entity("ego", "ego", 0), entity("npc", "vehicle", 1),
                     frame(1, {"ego": (0, 0, 0), "npc": (3.0, 0, 0)}), END])
    assert truth.main(["closest", str(tmp_path)]) == 1
    assert json.loads(capsys.readouterr().out)["closest"]["gap"] < 0


def test_a_run_without_a_truth_file_is_an_error(tmp_path, capsys):
    assert truth.main(["closest", str(tmp_path)]) == 2
    assert json.loads(capsys.readouterr().out)["verdict"] == "error"


def test_a_schema_1_trace_reads_the_same(tmp_path):
    ep1 = {k: v for k, v in EP.items() if k not in ("map", "step_time")} | {"schema": 1}
    ent = []
    for e in (entity("ego", "ego", 0), entity("npc", "vehicle", 1)):
        e = {k: v for k, v in e.items() if k not in ("index", "role", "subtype")}
        e["actor_id"], e["blueprint"], e["blueprint_box"] = e.pop("sim_id"), e.pop("asset"), e.pop("asset_box")
        ent.append(e)
    f = frame(1, {"ego": (0, 0, 0), "npc": (5.0, 0, 0)})
    f.pop("sim_time_ns")
    f["carla_frame"] = f.pop("sim_frame")
    for s in f["entities"].values():
        s["carla"] = s.pop("sim_pose")
        s.pop("angular_velocity"), s.pop("velocity_source")
    write(tmp_path, [ep1, *ent, f, END])
    c = truth.closest(truth.load(tmp_path))
    assert math.isclose(c["gap"], 1.0, abs_tol=1e-9)
