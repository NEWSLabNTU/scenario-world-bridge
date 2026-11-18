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

Record the sensor on each side while the scene holds (`SCENE-READY` on stdout).
