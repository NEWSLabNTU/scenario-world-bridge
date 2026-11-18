"""sim_compare's scene: the map-frame poses both simulators are placed by."""
import math
import os
import sys

import pytest
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import scene  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))


def test_the_scenes_ego_is_carlas_rear_axle():
    s = scene.load()
    x, y, yaw = s["ego_map"]
    # CARLA spawns the actor's origin at (299.4, 129.75), yaw 180; its rear axle (base_link,
    # the Isaac entity's origin) is 1.386 m behind it
    assert (round(x, 3), round(y, 3)) == (300.786, -129.75)
    assert math.isclose(abs(yaw), math.pi)


def _with(tmp_path, x, y, yaw, cars=None):
    s = yaml.safe_load(open(os.path.join(HERE, "lidar_compare.yaml")))
    s["ego"]["carla"] = {"x": x, "y": y, "z": 0.3, "yaw": yaw}
    s["cars"] = cars or {}
    p = tmp_path / "scene.yaml"
    yaml.safe_dump(s, open(p, "w"))
    return scene.load(str(p))


@pytest.mark.parametrize("yaw", [0.0, 90.0, -90.0, 180.0, 37.0])
def test_base_link_is_the_actor_origin_plus_the_rear_axle_along_the_heading(tmp_path, yaw):
    s = _with(tmp_path, 10.0, 20.0, yaw)
    ra = s["ego"]["rear_axle_m"]
    # in CARLA's frame (left-handed: y right) the rear axle is at origin + ra (cos, sin) of yaw
    cx = 10.0 + ra * math.cos(math.radians(yaw))
    cy = 20.0 + ra * math.sin(math.radians(yaw))
    x, y, myaw = s["ego_map"]
    assert math.isclose(x, cx, abs_tol=1e-9) and math.isclose(y, -cy, abs_tol=1e-9)
    assert math.isclose(math.cos(myaw), math.cos(math.radians(-yaw)), abs_tol=1e-12)


def test_a_car_lands_at_its_box_centre_in_base_link(tmp_path):
    s = _with(tmp_path, 0.0, 0.0, 90.0, {"a": {"centre": [10.0, 2.0], "yaw": 0.0}})
    ex, ey, eyaw = s["ego_map"]
    x, y, yaw = s["cars_map"]["a"]
    bx = s["box_centre_m"][0]
    # back from the car's origin to its box centre, then into the ego's base_link
    cx, cy = x + math.cos(yaw) * bx - ex, y + math.sin(yaw) * bx - ey
    fx = math.cos(eyaw) * cx + math.sin(eyaw) * cy
    fy = -math.sin(eyaw) * cx + math.cos(eyaw) * cy
    assert math.isclose(fx, 10.0, abs_tol=1e-9) and math.isclose(fy, 2.0, abs_tol=1e-9)
