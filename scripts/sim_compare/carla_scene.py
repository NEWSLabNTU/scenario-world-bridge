"""The comparison scene on CARLA: the hero and the parked cars from lidar_compare.yaml, held
for hold_s, their box centres in the hero's base_link written to OUT (json) as the Isaac
script writes them. carla_scene.py <port> <out json> [<scene yaml>]"""
import json, math, sys, time

import carla
import numpy as np

import scene as scenemod

port, out = int(sys.argv[1]), sys.argv[2]
s = scenemod.load(*sys.argv[3:4])
c = carla.Client("localhost", port); c.set_timeout(60)
w = c.get_world()
if not w.get_map().name.endswith(s["map"]):
    w = c.load_world(s["map"])
bl = w.get_blueprint_library()
e = s["ego"]["carla"]
hero_bp = bl.find(s["ego"]["blueprint"]); hero_bp.set_attribute("role_name", "hero")
actors = []
try:
    hero = w.try_spawn_actor(hero_bp, carla.Transform(carla.Location(e["x"], e["y"], e["z"]),
                                                      carla.Rotation(yaw=e["yaw"])))
    if hero is None:
        hero = next(v for v in w.get_actors().filter("vehicle.*")
                    if v.attributes.get("role_name") == "hero")
        hero.set_transform(carla.Transform(carla.Location(e["x"], e["y"], e["z"]),
                                           carla.Rotation(yaw=e["yaw"])))
    else:
        actors.append(hero)
    for _ in range(20): w.wait_for_tick(10)
    ht = hero.get_transform(); yaw0 = math.radians(ht.rotation.yaw)
    rear = s["ego"]["rear_axle_m"]
    cars = {}
    for name, car in s["cars"].items():
        dx, dy = car["centre"]
        # base_link (x fwd, y left) -> CARLA's actor frame (x fwd, y right), then the world
        ax, ay = dx + rear, -dy
        x = ht.location.x + math.cos(yaw0) * ax - math.sin(yaw0) * ay
        y = ht.location.y + math.sin(yaw0) * ax + math.cos(yaw0) * ay
        bp = bl.find(s["car_blueprint"])
        t = carla.Transform(carla.Location(x, y, ht.location.z + 0.5),
                            carla.Rotation(yaw=ht.rotation.yaw - car["yaw"]))
        a = w.spawn_actor(bp, t); actors.append(a); cars[name] = a
    for _ in range(40): w.wait_for_tick(10)
    inv = np.array(hero.get_transform().get_inverse_matrix()); truth = {}
    for k, a in cars.items():
        l = a.get_transform().transform(a.bounding_box.location)
        p = inv @ np.array([l.x, l.y, l.z, 1.0])
        truth[k] = [round(p[0] - rear, 2), round(-p[1], 2)]
    json.dump(truth, open(out, "w"), indent=1); print("truth", truth, flush=True)
    err = max(math.hypot(truth[k][0] - car["centre"][0], truth[k][1] - car["centre"][1])
              for k, car in s["cars"].items())
    if err > 0.05:  # a Model 3's box centre is 3 cm off its actor origin
        raise SystemExit(f"cars placed up to {err:.2f} m from the scene's centres")
    print("SCENE-READY", flush=True)
    time.sleep(s["hold_s"])
finally:
    for a in actors[::-1]:
        a.destroy()
    print("SCENE-DONE", flush=True)
