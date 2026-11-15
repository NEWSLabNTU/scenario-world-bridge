#!/usr/bin/env python3
"""Roadmap 074 N5's swb check: Town03 junction 1901's two lanes as ymerge3's paths.

The diagonal (road 44 lane -4, connector 1930, then road 22) and the main road (road 21 lane -1,
connector 1909, road 22), each from CARLA's waypoints every STEP m: BACK m behind the junction's
entry (through the junctions before it, the straightest branch at each) and AHEAD m past it along
22 -> 1679 -> 23. M, the merge point, is the diagonal's arc end: its centre within MERGE_TOL of the main road's
and its heading within MERGE_DEG of the main road's; Z is the last Y.Z_LEN m of each path before M (on Town03 the diagonal's centre is
3.5 m from the main road's 12.0 m before M, as ymerge_game's 12.6 m). A car's u is its front past Z's
near edge along its path, ymerge_game's coordinate.

Built from the town's OpenDRIVE (carla.Map offline; no CARLA needed).

  scripts/coop/ymerge3_town03.py --out <lanes.json> [--xodr <Town03.xodr>]
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ymerge_game as Y  # noqa: E402

STEP = 0.25
BACK, AHEAD = 260.0, 160.0
ENTRY = {"A": (-126.9, 48.0), "B": (-149.0, 38.4)}  # CARLA: the diagonal's and the main road's junction entries
FORWARD_ROADS = {1930, 1909, 22, 1679, 23}
MERGE_TOL = 0.1
MERGE_DEG = 0.5


def _angd(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def build(xodr_path):
    import carla

    m = carla.Map("Town03", open(xodr_path).read())
    yaw = lambda w: math.radians(w.transform.rotation.yaw)  # noqa: E731
    lines = {}
    for k, (xc, yc) in ENTRY.items():
        w0 = m.get_waypoint(carla.Location(x=xc, y=yc, z=0.0))
        back, w = [], w0
        for _ in range(int(BACK / STEP)):
            w = min(w.previous(STEP), key=lambda c: _angd(yaw(c), yaw(w)))
            back.append(w)
        fwd, w = [], w0
        for _ in range(int(AHEAD / STEP)):
            cs = w.next(STEP)
            w = min([c for c in cs if c.road_id in FORWARD_ROADS] or cs, key=lambda c: _angd(yaw(c), yaw(w)))
            fwd.append(w)
        ws = back[::-1] + [w0] + fwd
        pts = [(round(p.transform.location.x, 3), round(p.transform.location.y, 3), round(p.transform.location.z, 3)) for p in ws]
        lines[k] = {"pts": pts, "entry": len(back), "roads": sorted({p.road_id for p in ws})}
    main = lines["B"]["pts"]

    def d_main(p):
        return min(math.hypot(p[0] - q[0], p[1] - q[1]) for q in main)

    dg = lines["A"]["pts"]
    hd = lambda pts, i: math.atan2(pts[i + 1][1] - pts[i][1], pts[i + 1][0] - pts[i][0])  # noqa: E731
    h_main = hd(main, lines["B"]["entry"] + 4)  # the main road is straight through the junction
    # M: the arc's end, where the diagonal runs on the main road's centre with its heading
    k_m = next(i for i in range(lines["A"]["entry"], len(dg) - 1)
               if d_main(dg[i]) < MERGE_TOL and _angd(hd(dg, i), h_main) < math.radians(MERGE_DEG))
    xm, ym = dg[k_m][:2]
    j_m = min(range(len(main)), key=lambda j: math.hypot(main[j][0] - xm, main[j][1] - ym))
    lines["A"]["s_m"], lines["B"]["s_m"] = k_m * STEP, j_m * STEP
    # the diagonal's distance from the main road Z_LEN before M, as a check on the zone
    lines["zone_check_m"] = round(d_main(dg[k_m - int(Y.Z_LEN / STEP)]), 3)
    lines["M"] = [xm, ym]
    lines["step"] = STEP
    return lines


class Lane:
    """One path in CARLA's frame: project a point to its arc length (searched near the last one per
    key), the pose at s, and u for a car's centre."""

    def __init__(self, d):
        self.pts, self.s_m, self.step = d["pts"], d["s_m"], d.get("step", STEP)
        self.last = {}

    def project(self, key, x, y):
        k0 = self.last.get(key)
        lo, hi = (0, len(self.pts)) if k0 is None else (max(0, k0 - 80), min(len(self.pts), k0 + 80))
        k = min(range(lo, hi), key=lambda j: (self.pts[j][0] - x) ** 2 + (self.pts[j][1] - y) ** 2)
        self.last[key] = k
        j = k + 1 if k + 1 < len(self.pts) else k - 1
        ax, ay = self.pts[k][:2]
        bx, by = self.pts[j][:2]
        dx, dy = bx - ax, by - ay
        f = ((x - ax) * dx + (y - ay) * dy) / max(dx * dx + dy * dy, 1e-9)
        return (k + f * (j - k)) * self.step

    def at(self, s):
        k = min(max(int(round(s / self.step)), 0), len(self.pts) - 2)
        x, y, z = self.pts[k]
        x2, y2, _ = self.pts[k + 1]
        return x, y, z, math.atan2(y2 - y, x2 - x)

    def u_of(self, key, x, y, length=Y.L):
        return self.project(key, x, y) - (self.s_m - Y.Z_LEN) + length / 2

    def s_of_u(self, u, length=Y.L):
        """The centre's arc length for a car whose front is at u."""
        return u + self.s_m - Y.Z_LEN - length / 2


def load(path):
    d = json.load(open(path))
    return {"A": Lane(d["A"]), "B": Lane(d["B"]), "C": Lane(d["B"])}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--xodr", default=os.path.join(os.environ.get("CARLA_DIR", os.path.expanduser("~/Downloads/CARLA_0.9.16")),
                                                   "CarlaUE4/Content/Carla/Maps/OpenDrive/Town03.xodr"))
    a = ap.parse_args()
    d = build(a.xodr)
    json.dump(d, open(a.out, "w"))
    print(f"M at CARLA {d['M']}; diagonal M {d['A']['s_m']:.2f} m along ({len(d['A']['pts'])} points, roads {d['A']['roads']}), "
          f"main {d['B']['s_m']:.2f} m ({d['B']['roads']}); the diagonal {d['zone_check_m']} m from the main road Z_LEN before M")
    return 0


if __name__ == "__main__":
    sys.exit(main())
