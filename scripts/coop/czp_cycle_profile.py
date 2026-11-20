#!/usr/bin/env python3
"""Issue 075: the cost of czp's decision cycle (Strategy.cycle) on a conflict tick, grid074 s17's scene.

The ego at rest 0.6 m before the Town05 crossing j108210_108811_109526 (heading south in the map frame, a left turn
ahead), the other car at rest at its line east of the zone (heading west), a 60-point trajectory as the node
passes it (every 5th point of Autoware's), one RTC status. Prints the CPU and wall time per cycle (p50/p90/p99/max)
over --cycles conflict ticks and the same with no object; --profile adds cProfile's top functions.

    python3 scripts/coop/czp_cycle_profile.py --zones <Town05 zones.yaml> [--czp src/czp] [--cycles 600] [--profile]
"""

from __future__ import annotations

import argparse
import cProfile
import json
import os
import pstats
import sys


def quantiles(xs):
    xs = sorted(xs)
    return {k: round(1e3 * xs[int(q * (len(xs) - 1))], 3) for k, q in (("p50", 0.5), ("p90", 0.9), ("p99", 0.99),
                                                                       ("max", 1.0))}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--zones", required=True)
    ap.add_argument("--czp", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "src", "czp"))
    ap.add_argument("--cycles", type=int, default=600)
    ap.add_argument("--profile", action="store_true")
    a = ap.parse_args(argv)
    sys.path.insert(0, a.czp)
    from czp.strategy import Obj, Status, Strategy
    from czp.zones import load_zones
    _, zones = load_zones(a.zones)
    st = Strategy(zones)
    # grid074 on_dt0.3_t1.5p5_s17 at 1894.9 (coop_a.jsonl, CARLA frame -> map frame: y and yaw negated), the centres
    # 1.4 m ahead of base_link as the node shifts them
    ego = (-131.956, 9.648 - 1.4, -1.5619, 0.0, 0.0)
    other = Obj("a", 1, -126.989 - 1.4, 4.142, -3.1392, 0.0, 0.0, 4.79, 2.16)
    traj = [(-131.95, 8.2 - 0.5 * i) for i in range(12)] + [(-131.9 + 1.0 * i, 2.2) for i in range(48)]
    stat = [Status("intersection", "ab" * 16)]

    def run(n, t0, objs):
        cpu, wall, acts = [], [], None
        for k in range(n):
            acts = st.cycle(t0 + 0.1 * k, ego, objs, traj, stat, 0.05, 0.0)
            cpu.append(st.last_cycle_cpu)
            wall.append(st.last_cycle)
        return cpu[1:], wall[1:], acts
    cpu, wall, acts = run(a.cycles, 0.0, [other])
    res = {"decision": None if acts.decision is None else [acts.decision.kind, acts.decision.reason, acts.zone],
           "conflict_cpu_ms": quantiles(cpu), "conflict_wall_ms": quantiles(wall)}
    cpu, wall, _ = run(min(a.cycles, 200), 1000.0, [])
    res["no_object_cpu_ms"] = quantiles(cpu)
    res["loadavg"] = open("/proc/loadavg").read().split()[:3]
    print(json.dumps(res))
    if a.profile:
        pr = cProfile.Profile()
        pr.enable()
        run(300, 2000.0, [other])
        pr.disable()
        pstats.Stats(pr).sort_stats("cumulative").print_stats(15)
    return 0


if __name__ == "__main__":
    sys.exit(main())
