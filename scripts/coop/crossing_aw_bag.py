#!/usr/bin/env python3
"""Roadmap 078 Phase 2: what crossing_aw.py's bags say -- RTC on a real conflict, and EV4's tracks.

Per run (a crossing_aw.py run directory: bag/, coop_a.jsonl with both cars' truth, decisions.jsonl
when the node was on), in the run's time (0 at A's lock, S2's run start), aligned to the bag by the
ego's own pose (CARLA elapsed <-> ROS time):

- **the node**: its decision changes (decision, reason, band, role), its heartbeat (gaps; the modules
  it held manual; its limit), and the watchdog's interventions (from the ego's play_log);
- **RTC**: the intersection module's cooperate status per uuid -- safe (the module's own judgement),
  command_status (ACTIVATE 1 / DEACTIVATE 0), auto_mode, start_distance -- its changes, and whether
  each node decision reached it (the command, then the mode, within 0.5 s) and took effect: the
  intersection planning factor (stop / none, its distance) and B's speed (stopped before Z or not);
- **mode back to auto** at the end of the conflict;
- **EV4**: every tracked object (/perception/object_recognition/tracking/objects) matched to A's
  truth; per message A's distance to Z, the tracked and true speed, the nominal arrival
  (distance / speed, czp's nominal_in) from the track and from the truth at the same stamp, its error,
  and A's stop instant (speed < rest_speed) as tracked and as true. First tracked distance.

  scripts/coop/crossing_aw_bag.py --out <runs dir> [--only tag,...]   -> <dir>/bag/<tag>.json, tracks.csv
"""

import argparse
import bisect
import csv
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import crossing_game as G  # noqa: E402

SIM_BASE = 1000.0  # s: ROS stamp = CARLA elapsed + this (scensim_core sim_time BASE_NS)
MATCH_R = 3.0  # m: a track within this of A's true centre is A
REST_V = 0.1  # m/s, czp's rest_speed
CMD = {0: "DEACTIVATE", 1: "ACTIVATE"}  # tier4_rtc_msgs/Command


def bag_rows(bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    ty = {t.name: t.type for t in r.get_all_topics_and_types()}
    while r.has_next():
        tp, d, t_rec = r.read_next()
        yield tp, deserialize_message(d, get_message(ty[tp])), t_rec * 1e-9


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def tstamp(t):
    return t.sec + t.nanosec * 1e-9


def load_log(rd):
    rows, ev = [], {}
    for line in open(Path(rd) / "coop_a.jsonl"):
        if not line.strip():
            continue
        r = json.loads(line)
        if "event" in r:
            ev.setdefault(r["event"], r)
        else:
            rows.append(r)
    return rows, ev


class Truth:
    """A's and B's truth by CARLA elapsed time, interpolated."""

    def __init__(self, rows):
        self.rows = rows
        self.el = [r["el"] for r in rows]

    def at(self, el):
        i = bisect.bisect_left(self.el, el)
        if i <= 0 or i >= len(self.el):
            return None
        r0, r1 = self.rows[i - 1], self.rows[i]
        if r1["el"] - r0["el"] > 0.2:
            return None
        f = (el - r0["el"]) / (r1["el"] - r0["el"])
        lerp = lambda a, b: a + f * (b - a)  # noqa: E731
        out = {}
        for k in ("A", "B"):
            out[k] = [lerp(a, b) for a, b in zip(r0[k], r1[k])]
        for k in ("ua", "ub", "va", "vb"):
            out[k] = lerp(r0[k], r1[k])
        return out


def clock_offset(msgs, rows):
    """ROS stamp - CARLA elapsed, from the ego's pose: B moving along CARLA +y at constant x."""
    by_y = sorted((r["B"][1], r["el"]) for r in rows if math.hypot(r["B"][3], r["B"][4]) > 1.0)
    ys = [b[0] for b in by_y]
    offs = []
    for tp, m, _ in msgs:
        if tp != "/localization/kinematic_state" or m.twist.twist.linear.x < 1.0:
            continue
        y = -m.pose.pose.position.y  # CARLA y
        i = bisect.bisect_left(ys, y)
        if 0 < i < len(by_y):
            (y0, e0), (y1, e1) = by_y[i - 1], by_y[i]
            if 1e-6 < y1 - y0 < 1.0:
                offs.append(stamp(m.header) - (e0 + (e1 - e0) * (y - y0) / (y1 - y0)))
    if not offs:
        return None, None
    offs.sort()
    return offs[len(offs) // 2], offs[int(0.9 * (len(offs) - 1))] - offs[int(0.1 * (len(offs) - 1))]


def analyse(rd):
    rd = Path(rd)
    rows, ev = load_log(rd)
    if "lock" not in ev or not (rd / "bag").exists():
        return {"error": "no lock or no bag"}, []
    setup = ev["setup"]
    el_lock = ev["lock"]["el"] if "el" in ev["lock"] else setup["t0"] + ev["lock"]["t"]
    msgs = list(bag_rows(rd / "bag"))
    # ROS time is CARLA's elapsed + BASE (1000 s) exactly (scensim_core sim_time); the offset read from the ego's
    # localization against its truth is kept as a diagnostic: it is that much late (EKF/NDT lag)
    loc_off, spread = clock_offset(msgs, rows)
    off = SIM_BASE
    tr = lambda s: round(s - off - el_lock, 3)  # noqa: E731  ROS stamp -> run time
    truth = Truth(rows)
    cx, cy = setup["centre"]
    la = setup["lane_a"]
    yaw_a = math.radians(la[4])
    da = (math.cos(yaw_a), math.sin(yaw_a))
    L_A = setup["ext"]["A"][0]
    u_of = lambda x, y: (x - cx) * da[0] + (y - cy) * da[1] + L_A / 2 + G.W_BOX / 2  # noqa: E731  CARLA frame
    out = {"clock_offset": off, "loc_lag": None if loc_off is None else round(loc_off - off, 3),
           "loc_lag_spread": None if spread is None else round(spread, 3)}
    # --- tracks (EV4) ----------------------------------------------------------------------------------
    tracks = []
    first = {}
    for tp, m, _ in msgs:
        if tp not in ("/perception/object_recognition/tracking/objects", "/perception/object_recognition/objects"):
            continue
        s = stamp(m.header)
        tru = truth.at(s - off)
        if tru is None:
            continue
        ax, ay = tru["A"][0], -tru["A"][1]  # ROS
        best = None
        for ob in m.objects:
            k = ob.kinematics
            pose = (k.pose_with_covariance if hasattr(k, "pose_with_covariance") else k.initial_pose_with_covariance).pose
            tw = (k.twist_with_covariance if hasattr(k, "twist_with_covariance") else k.initial_twist_with_covariance).twist
            d = math.hypot(pose.position.x - ax, pose.position.y - ay)
            if d < MATCH_R and (best is None or d < best[0]):
                label = max(ob.classification, key=lambda c: c.probability).label if ob.classification else 0
                best = (d, pose, tw, label, bytes(bytearray(ob.object_id.uuid)).hex()[:8])
        kind = "track" if "tracking" in tp else "pred"
        bx, by = tru["B"][0], -tru["B"][1]
        if best is None:
            continue
        d, pose, tw, label, uid = best
        px, py = pose.position.x, -pose.position.y  # CARLA
        u_tr = u_of(px, py)
        q = pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        v_tr = math.hypot(tw.linear.x, tw.linear.y)
        dist_ego = math.hypot(ax - bx, ay - by)
        first.setdefault(kind, {"tr": tr(s), "dist_z": round(-tru["ua"], 1), "dist_ego": round(dist_ego, 1), "label": label})
        tracks.append({"kind": kind, "tr": tr(s), "stamp": round(s, 3), "uid": uid, "label": label, "pos_err": round(d, 3),
                       "u_true": round(tru["ua"], 3), "v_true": round(tru["va"], 3), "u_trk": round(u_tr, 3),
                       "v_trk": round(v_tr, 3), "yaw_err": round(math.degrees(math.atan2(math.sin(yaw + yaw_a), math.cos(yaw + yaw_a))), 2),
                       "dist_ego": round(dist_ego, 2), "ub": round(tru["ub"], 3), "vb": round(tru["vb"], 3)})
    out["first_tracked"] = first
    # A's stop instant: true and as tracked (first after which the speed stays < REST_V for 0.5 s, before Z)
    # (after the lock: before it A stood at its start)
    out["stop_true"] = rest_instant([(r["t"] - (el_lock - setup["t0"]), r["va"], r["ua"]) for r in rows
                                     if r["t"] - (el_lock - setup["t0"]) >= 0])
    out["stop_trk"] = rest_instant([(x["tr"], x["v_trk"], x["u_true"]) for x in tracks if x["kind"] == "track" and x["tr"] >= 0])
    # --- the node, RTC, factors, B ------------------------------------------------------------------
    rtc, hb, dec, fac, lim, ego = [], [], [], [], [], []
    for tp, m, trec in msgs:
        if tp == "/planning/cooperate_status/intersection":
            for s in m.statuses:
                rtc.append((tr(tstamp(m.stamp)), bytes(bytearray(s.uuid.uuid)).hex()[:8], bool(s.safe),
                            CMD.get(s.command_status.type, s.command_status.type), bool(s.auto_mode),
                            round(s.start_distance, 1), int(s.state.type), bool(s.requested)))
        elif tp == "/interaction_strategy/heartbeat":
            hb.append((tr(trec), json.loads(m.data)))
        elif tp == "/interaction_strategy/decision":
            d = json.loads(m.data)
            dec.append(d)
        elif tp == "/planning/planning_factors/intersection":
            for f in m.factors:
                dist = f.control_points[0].distance if f.control_points else None
                fac.append((tr(stamp(m.header)), f.behavior, None if dist is None else round(dist, 1)))
            if not m.factors:
                fac.append((tr(stamp(m.header)), None, None))
        elif tp == "/planning/scenario_planning/max_velocity_candidates":
            lim.append((tr(tstamp(m.stamp)) if m.stamp.sec else tr(trec), m.sender, round(m.max_velocity, 2)))
        elif tp == "/localization/kinematic_state":
            ego.append((tr(stamp(m.header)), m.twist.twist.linear.x))
    # the node's decisions from its own log when the topic missed some (first line wins per t)
    if (rd / "decisions.jsonl").exists():
        dec = [json.loads(line) for line in open(rd / "decisions.jsonl") if line.strip()]
    zone_dec = [d for d in dec if d.get("conflict_set")]
    out["decisions"] = [{"tr": tr(d["t"]), "zone": d["zone"], "decision": d["decision"], "reason": d["reason"], "band": d["band"],
                         "role": d.get("role"), "probe": d.get("probe"),
                         "slack": next(({"own": a["band"]["slack_own"], "other": a["band"]["slack_other"]}
                                        for a in d["agents"].values() if a.get("band")), None)}
                        for d in zone_dec]
    # heartbeat: gaps, manual modules over time
    gaps = [b[0] - a[0] for a, b in zip(hb, hb[1:])]
    out["heartbeat"] = {"n": len(hb), "max_gap": round(max(gaps), 3) if gaps else None,
                        "manual_spans": spans([(t, bool(h.get("manual"))) for t, h in hb]),
                        "limited_spans": spans([(t, bool(h.get("limited"))) for t, h in hb])}
    # RTC: changes per uuid
    ch, last = [], {}
    for t, u, safe, cmd, auto, sd, state, req in rtc:
        k = (safe, cmd, auto)
        if last.get(u) != k:
            last[u] = k
            ch.append({"tr": t, "uuid": u, "safe": safe, "cmd": cmd, "auto": auto, "start_d": sd, "state": state})
    out["rtc_changes"] = ch
    out["rtc_final_auto"] = None if not rtc else all(r[4] for r in rtc if r[0] >= rtc[-1][0] - 0.5)
    fch, lastf = [], object()
    for t, b, d in fac:
        if b != lastf:
            lastf = b
            fch.append({"tr": t, "behavior": b, "dist": d})
    out["factor_changes"] = fch
    out["limits"] = [{"tr": t, "sender": s, "v": v} for t, s, v in lim if s == "interaction_strategy"]
    # B: when it came to rest before Z, and when it started again (from the truth, run time)
    rt = [(r["t"] - (el_lock - setup["t0"]), r["vb"], r["ub"]) for r in rows]
    out["b_rest"] = rest_spans(rt)
    out["a_rest"] = rest_spans([(r["t"] - (el_lock - setup["t0"]), r["va"], r["ua"]) for r in rows])
    out["effects"] = effects(out["decisions"], rtc, rt)
    return out, tracks


def spans(series):
    out, on = [], None
    for t, x in series:
        if x and on is None:
            on = t
        elif not x and on is not None:
            out.append([on, t])
            on = None
    if on is not None:
        out.append([on, series[-1][0]])
    return out


def rest_instant(series, hold=0.5):
    """First t (u < 0) after which v stays < REST_V for `hold` s."""
    cand = None
    for t, v, u in series:
        if u >= 0:
            break
        if v < REST_V:
            if cand is None:
                cand = t
            elif t - cand >= hold:
                return round(cand, 3)
        else:
            cand = None
    return None


def rest_spans(series, v_rest=0.3):
    """B's (or A's) spans below v_rest before Z, run time."""
    return spans([(t, v < v_rest and u < 0) for t, v, u in series])


def effects(decs, rtc, b_series):
    """Per node decision that is not stock: did the RTC status show the command (and manual mode) within
    0.5 s, and what did B do next (stopped before Z within the next 5 s, entered Z, when)."""
    out = []
    for d in decs:
        if d["decision"] in ("stock",):
            continue
        want = "ACTIVATE" if d["decision"] == "go" or (d["decision"] == "creep" and str(d.get("probe")).find("creep") >= 0) else "DEACTIVATE"
        # the first status sample within 1 s that shows the command, in manual mode (rtc: (tr, uuid, safe, cmd, auto, ...))
        seen = next((c for c in rtc if d["tr"] - 0.05 <= c[0] <= d["tr"] + 1.0 and c[3] == want and not c[4]), None)
        nxt = [x for x in b_series if d["tr"] <= x[0] <= d["tr"] + 5.0]
        b_in = next((x[0] for x in nxt if x[2] >= 0), None)
        b_stop = next((x[0] for x in nxt if x[1] < 0.3 and x[2] < 0), None)
        out.append({"tr": d["tr"], "decision": d["decision"], "reason": d["reason"], "want": want,
                    "rtc_seen": None if seen is None else round(seen[0] - d["tr"], 2),
                    "b_in_within5": b_in, "b_stopped_within5": b_stop})
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--only")
    a = ap.parse_args()
    base = Path(a.out)
    dst = base / "bagfacts"
    dst.mkdir(exist_ok=True)
    rows = [json.loads(line) for line in open(base / "runs.jsonl") if line.strip()]
    last = {}
    for r in rows:
        last[r["tag"]] = r
    tags = a.only.split(",") if a.only else [t for t, r in last.items() if "error" not in r]
    allt = []
    for tag in tags:
        try:
            res, tracks = analyse(base / tag)
        except Exception as e:  # noqa: BLE001
            res, tracks = {"error": repr(e)[:300]}, []
        (dst / f"{tag}.json").write_text(json.dumps(res, indent=1))
        for x in tracks:
            allt.append({"tag": tag, **{k: last[tag][k] for k in ("arm", "dt", "rule", "seed")}, **x})
        print(tag, json.dumps({k: res.get(k) for k in ("clock_offset", "loc_lag", "first_tracked", "stop_true", "stop_trk", "error")}))
    if allt:
        with open(dst / "tracks.csv", "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(allt[0].keys()))
            w.writeheader()
            w.writerows(allt)
    return 0


if __name__ == "__main__":
    sys.exit(main())
