# sim_compare: one scene, both simulators

The setup for comparing a sensor on Isaac against the same sensor on CARLA (roadmap 084b). A
comparison is only like-for-like when both simulators place the ego and every other actor at the
same map pose. In 084 the two scenes were 39 m apart, which put a tree row inside one lidar's
40-60 m band and outside the other's, and put the side car at 13.6 m on one and 16.4 m on the
other.

| file | what |
|---|---|
| `lidar_compare.yaml` | The scene: the map, the ego's pose in CARLA's frame (Town01's first spawn point with 160 m of straight lane, acb `vehicle_check_host.py`'s rule), and the parked cars by box centre in the ego's base_link (rear axle on the ground; x forward, y left). |
| `scene.py` | `load()`: the file, plus every pose in the map frame (ROS; map y = -CARLA y). |
| `carla_scene.py` | CARLA: spawns the hero and the cars, holds them, writes their box centres in the hero's base_link. |
| `carla_record.py` | CARLA, without acb: the spec's top lidar recorded straight to a bag of base_link clouds as acb_bridge publishes them, at a drifting column phase (below). |
| `isaac_scene.py` | Isaac: the same over the world protocol, against an Isaac world server (`scensim_isaac_world`). |

Both scripts write the same truth json (`{car: [x, y]}` in base_link) and stop with an error if
a car lands more than 5 cm from the scene's centre. On CARLA 0.9.16, a Model 3's box centre is
3 cm ahead of its actor origin.

```bash
# CARLA (a server of your own on PORT, Town01 or any town: it loads the scene's)
python3 carla_scene.py PORT truth.json [scene.yaml]
# Isaac (Isaac's venv python, scensim_world_server on PYTHONPATH)
python3 isaac_scene.py ISAAC_PORT <world.yaml> <map dir> HOLD_S truth.json [scene.yaml]
```

Record the sensor on each side while the scene holds (`SCENE-READY` on stdout): `ros2 bag record
/sensing/lidar/top/pointcloud_before_sync /tf_static /tf` on the ego's ROS domain (acb_bridge's
on CARLA, the Isaac world server's `EGO_ROS_DOMAIN_ID`), with Cyclone as the stack's RMW.

The bodies are CARLA 0.9.16's (`body:` in the scene); on CARLA 1.0 the Model 3 differs (wheel
base 2.876 m), so a 1.0 comparison needs a scene file with that body's values.

## Column phase: let the lidar drift

Both simulators fire a phase-locked lidar by default: CARLA's turns at 20 Hz against a 20 Hz
tick, and Isaac's rotary lidar starts every scan at the same azimuth. Every scan then fires the
same columns, and a thin object (a pole, a car's edge at 35 m) is hit in every scan or in none.
A sub-column change of the ego's pose flips it: in 084e a 35 m car's TransFusion recall spread
0.49 (CARLA) and 0.39 (Isaac) over five ego positions within +-1.4 m. With the phase drifting,
as a real lidar's does, the spreads were 0.21 and 0.17 and the means agreed (0.36, 0.36).
So run comparisons drifting:

- **CARLA**: spin the lidar at 20.00268 Hz, not 20 (`carla_record.py`'s default, `--rotation`):
  each scan starts ~0.048 deg on, an eighth of a 0.35 deg column, and still covers 360 deg.
- **Isaac**: `phase_jitter: {seed: N}` in the lidar's profile (aib's rig or ego override,
  `sensors.top.isaac.profile`): each frame the lidar prim turns by a seeded angle within one
  column and its points are turned back. It needs one revolution a frame (20 Hz lidar, 20 Hz
  frames).

```bash
# CARLA: a bag of base_link clouds, the scene held; MOUNT_BAG is any bag acb_bridge recorded
# with /tf_static (it gives the top lidar's mount)
python3 carla_record.py PORT out_dir 7 --mount-bag MOUNT_BAG
```

Neither is the default yet: the spec's lidar is phase-locked, and changing that changes
`spec_hash` (roadmap 084, a candidate for the next spec change).
