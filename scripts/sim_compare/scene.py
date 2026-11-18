"""The comparison scene, read by both simulators' scripts: poses in the map frame (ROS)."""
import math, os, yaml
HERE = os.path.dirname(os.path.abspath(__file__))

def load(path=os.path.join(HERE, "lidar_compare.yaml")):
    s = yaml.safe_load(open(path))
    e = s["ego"]["carla"]
    # CARLA spawns the actor's origin there; base_link (the rear axle, the Isaac entity's
    # origin) is rear_axle_m along the actor's heading from it (-1.386 m on a Model 3)
    eyaw = math.radians(-e["yaw"])                                  # CARLA -> map
    c, sn = math.cos(eyaw), math.sin(eyaw)
    ra = s["ego"]["rear_axle_m"]
    ex, ey = e["x"] + c * ra, -e["y"] + sn * ra
    s["ego_map"] = (ex, ey, eyaw)
    bx = s["box_centre_m"][0]
    s["cars_map"] = {}
    for name, car in s["cars"].items():
        dx, dy = car["centre"]
        # base_link is the ego's rear axle: the ego's entity origin in map terms
        cx, cy = ex + c * dx - sn * dy, ey + sn * dx + c * dy
        yaw = eyaw + math.radians(car["yaw"])
        # an entity's origin is its rear axle, bx behind its box centre
        s["cars_map"][name] = (cx - math.cos(yaw) * bx, cy - math.sin(yaw) * bx, yaw)
    return s
