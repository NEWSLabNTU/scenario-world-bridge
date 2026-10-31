# 028: Autoware's behavior planning container runs away on SVL's Shalun map

**Severity**: high for roadmap 064 (Autoware cannot drive Shalun), and dangerous to the host:
the container grows until the kernel kills it. **Status**: open. **Found**: 2026-10-31 on
newslab-241, Autoware 1.5.0's planning simulator on `svl_to_autoware.py`'s Shalun map dir
(roadmap 064).

## What happens

`planning_simulator.launch.xml` with `map_path:=` the Shalun map dir, `sample_vehicle`,
`sample_sensor_kit`, ROS domain 42. Localization initialised and the route was accepted
(`/api/routing/set_route_points` true) every time. Autonomous mode was never available:
`/autoware/planning/topic_rate_check/trajectory` ERROR.

**Run 1.** Start in lanelet 1712, a junction lanelet (`turn_direction: straight`); goal on
lanelet 103 (61 m). behavior_velocity_planner registered its intersection module for 1712.
The `behavior_planning_container` then died, and play_launch logged
`Container exited with code None` (killed by a signal). Nothing downstream of
behavior_path_planner ever published.

**Run 2.** Start and goal on plain lanelets: 527 to 56, 252 m through 7 junction lanelets.
- behavior_path_planner published `path_with_lane_id` (124 points, 10 Hz).
- behavior_velocity_planner registered 7 intersection modules
  (`[1449, 1064, 913, 2326, 2354, 1198, 1166]`), then never published `path`.
- One thread of the container sat at 100 % CPU.
- Its RSS grew by about 75 MB/s (metrics.csv: 65.5 GB to 65.8 GB in 4 s), reaching 60.8 GB
  when I killed it by hand at 125 GB total host memory. Run 1's death was most likely the OOM
  killer at the same stage.

The last line it logged:

```
[WARN] [Interpolator]: Input value 216.237402 is outside the range of the interpolator [0.000000, 216.235252].
```

Every input topic of behavior_velocity_planner had a publisher except
`/perception/traffic_light_recognition/traffic_signals`, `/awapi/tmp/virtual_traffic_light_states`
and `max_velocity_default`, which is normal for the planning simulator. Its traffic-light
module warned once that regulatory element 2552's signal was not received.

## What the map is

SVL's Lanelet2 export, with lat/lon re-projected and two Autoware conventions applied
(roadmap 064):
- 263 lanelets;
- 28 traffic-light and 23 stop-sign regulatory elements;
- 152 lanelets with `turn_direction`;
- no `intersection_area` polygons and no `right_of_way` elements.

Lanelet2's loader reads it with 0 errors, and Lanelet2's routing graph finds the routes.

## Suspects (not yet tested)

1. **The intersection module on SVL's junction lanelets:** a geometry it does not expect
   (overlapping or reversed bounds, zero-length segments, conflicting lanelets that share all
   their points) making an attention-area or interpolation loop never end. It registered just
   before the runaway both times.
2. **The interpolator overshoot** (216.2374 against a range ending at 216.2353) feeding a
   loop that steps by a tiny or zero increment.
3. **The crosswalk or traffic-light modules** on SVL's regulatory elements.

## Next

- Run the planning simulator with one behavior_velocity module at a time (autoware_launch's
  `behavior_velocity_planner.param.yaml` `launch_*` flags), under a memory cap
  (`systemd-run --user -p MemoryMax=16G` or `ulimit -v`) so a runaway dies early instead of
  taking the host.
- Check the junction lanelets' geometry: bounds' direction, duplicate points, and
  self-intersecting or zero-area lanelets.
- A core dump or a gdb run of the container, under `ptrace_scope=1` started by gdb itself, to
  see the looping frame.
