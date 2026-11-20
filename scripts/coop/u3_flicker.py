#!/usr/bin/env python3
"""074 U3: czp's flicker per czp run, from its bag (133's two counts):

  (a) drops: a decision change that removes a partner from the conflict set while that partner is still within
      DROP_M (30 m) of the ego -- by its own track (same object id on /perception/object_recognition/objects:
      "drop_seen") or, its track gone, by any car-labelled object within DROP_M ("drop_lost": perception lost it);
      "drops_back" counts the drops whose partner is back in the conflict set within BACK_S (2 s): flicker, where
      the other drops include the partner's own release once it has crossed;
  (b) flips: a change to "stock: no conflict" followed by a change back to a non-stock decision within FLIP_S (1 s).

    scripts/coop/u3_flicker.py <stage dir> [--out flicker.jsonl]

One JSON line per czp run (tag, drops_seen, drops_lost, drops_far, flips, decisions) and a total. Needs Autoware's
setup sourced (rosbag2_py and the message types).
"""

import argparse
import json
import math
import sys
from bisect import bisect_right
from pathlib import Path

DROP_M = 30.0
FLIP_S = 1.0
BACK_S = 2.0  # a dropped partner back in the zone's conflict set within this: the drop was flicker, not a release
CAR_LABELS = {1, 2, 3, 4, 5}  # CAR, TRUCK, BUS, TRAILER, MOTORCYCLE (autoware_perception_msgs ObjectClassification)
OBJ = "/perception/object_recognition/objects"
ODOM = "/localization/kinematic_state"
DEC = "/interaction_strategy/decision"


def uuid_hex(u):
    return bytes(u.uuid).hex()


def read(bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    types = {OBJ: "autoware_perception_msgs/msg/PredictedObjects", ODOM: "nav_msgs/msg/Odometry", DEC: "std_msgs/msg/String"}
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    r.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    cls = {k: get_message(v) for k, v in types.items()}
    objs, ego, decs = [], [], []
    while r.has_next():
        topic, data, ts = r.read_next()
        m = deserialize_message(data, cls[topic])
        t = ts / 1e9
        if topic == ODOM:
            ego.append((t, m.pose.pose.position.x, m.pose.pose.position.y))
        elif topic == OBJ:
            objs.append((t, [(uuid_hex(o.object_id), o.kinematics.initial_pose_with_covariance.pose.position.x,
                              o.kinematics.initial_pose_with_covariance.pose.position.y,
                              max(o.classification, key=lambda c: c.probability).label if o.classification else 0)
                             for o in m.objects]))
        else:
            decs.append((t, json.loads(m.data)))
    return objs, ego, decs


def at(series, t):
    """The last sample at or before t (else the first)."""
    i = bisect_right([s[0] for s in series], t) - 1
    return series[max(i, 0)] if series else None


def count(rd):
    objs, ego, decs = read(rd / "bag")
    drops = {"seen": 0, "lost": 0, "far": 0, "back": 0}
    flips = 0
    by_zone = {}
    for k, (t, d) in enumerate(decs):
        prev = by_zone.get(d["zone"])
        by_zone[d["zone"]] = d
        if prev is not None:
            gone = set(prev.get("conflict_set") or []) - set(d.get("conflict_set") or [])
            e = at(ego, t)
            o = at(objs, t)
            for pid in gone:
                if any(d2["zone"] == d["zone"] and pid in (d2.get("conflict_set") or [])
                       for t2, d2 in decs[k + 1:] if t2 - t <= BACK_S):
                    drops["back"] += 1
                if e is None or o is None:
                    continue
                mine = [x for x in o[1] if x[0] == pid]
                if mine:
                    near = math.hypot(mine[0][1] - e[1], mine[0][2] - e[2]) <= DROP_M
                    drops["seen" if near else "far"] += 1
                elif any(x[3] in CAR_LABELS and math.hypot(x[1] - e[1], x[2] - e[2]) <= DROP_M for x in o[1]):
                    drops["lost"] += 1
                else:
                    drops["far"] += 1
        if d["decision"] == "stock" and d["reason"] == "no conflict" and prev is not None and prev["decision"] != "stock":
            nxt = next(((t2, d2) for t2, d2 in decs[k + 1:] if d2["zone"] == d["zone"]), None)
            if nxt is not None and nxt[0] - t <= FLIP_S and nxt[1]["decision"] != "stock":
                flips += 1
    return {"tag": rd.name, "drops_seen": drops["seen"], "drops_lost": drops["lost"], "drops_far": drops["far"],
            "drops_back": drops["back"], "flips": flips, "decisions": len(decs)}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stage")
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    rows = []
    for rd in sorted(Path(a.stage).glob("on_*")):
        if (rd / "bag" / "metadata.yaml").exists():
            try:
                rows.append(count(rd))
            except Exception as e:  # noqa: BLE001 -- a broken bag is reported, not fatal
                rows.append({"tag": rd.name, "error": repr(e)[:200]})
    tot: dict = {k: sum(r.get(k, 0) for r in rows) for k in ("drops_seen", "drops_lost", "drops_far", "drops_back", "flips", "decisions")}
    tot.update(tag="TOTAL", runs=len(rows), runs_with_flicker=sum(1 for r in rows if r.get("flips") or r.get("drops_back")))
    out = open(a.out, "w") if a.out else sys.stdout
    for r in rows + [tot]:
        print(json.dumps(r), file=out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
