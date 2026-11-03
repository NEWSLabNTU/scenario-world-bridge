#!/usr/bin/env python3
"""Roadmap 072 C10: the NTU map's picture in CARLA from fixed viewpoints, the same as 059's
`render_probe.py` uses in Isaac, to compare the two.

    carla_probe.py OUT_DIR --view name,x,y,yaw_deg[,height,back,pitch_deg] ... [--port 2000]
        [--map NTU] [--fixed-exposure] [--sun 55,225]

A view is a chase camera `back` m behind and `height` m above the ground at (x, y) of the map
frame less MAP_ORIGIN, looking along `yaw` (deg, counter-clockwise from +x) and `pitch` deg down
(3.4, 7.0, 15 by default), as render_probe.py's. CARLA's frame is that one with y negated, so y
and the yaw change sign. The ground is found by a ray cast. 1280 x 720, 80 deg, CARLA's RGB
camera with its histogram auto-exposure (`--fixed-exposure`: manual), exposure_min_bright 7 and
exposure_compensation -1.5 (072's sweep: CARLA's default floor of 10 pins every view of this
map, so the canopy stays dark whatever the compensation). Sun at elevation 55 deg,
azimuth 225: 059 N10's world sun at 135, in CARLA's convention (the map's azimuth + 90). Writes
OUT_DIR/<name>.png.

`--cam key=value` sets any other attribute of the RGB camera (exposure_min_bright, ...).
`--semantic` also writes OUT_DIR/<name>_sem.png, CARLA's semantic camera in its CityScapes
palette, to tell which class a surface was tagged with. The world's settings and weather are
put back afterwards, since the CARLA world server owns them while it runs.
"""
import argparse
import math
import os
import queue
import sys

import numpy as np


def main(argv=None):
    import carla

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("out")
    ap.add_argument("--view", action="append", required=True)
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--map", default="NTU")
    ap.add_argument("--fixed-exposure", action="store_true")
    ap.add_argument("--sun", default="55,225", help="altitude,azimuth deg (CARLA's azimuth: the map's + 90)")
    ap.add_argument("--ev", type=float, default=-1.5, help="the camera's exposure_compensation")
    ap.add_argument("--cam", action="append", default=[], help="key=value, another camera attribute")
    ap.add_argument("--semantic", action="store_true")
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    client = carla.Client("localhost", a.port)
    client.set_timeout(300.0)
    world = client.get_world()
    if not world.get_map().name.endswith(a.map):
        world = client.load_world(a.map)
    before, weather_before = world.get_settings(), world.get_weather()
    s = world.get_settings()
    s.synchronous_mode, s.fixed_delta_seconds = True, 0.05
    world.apply_settings(s)
    alt, az = (float(v) for v in a.sun.split(","))
    w = carla.WeatherParameters(cloudiness=10.0, precipitation=0.0, sun_altitude_angle=alt,
                                sun_azimuth_angle=az, fog_density=0.0)
    world.set_weather(w)
    bp = world.get_blueprint_library().find("sensor.camera.rgb")
    for k, v in (("image_size_x", "1280"), ("image_size_y", "720"), ("fov", "80"),
                 ("exposure_mode", "manual" if a.fixed_exposure else "histogram"),
                 ("exposure_compensation", str(a.ev)), ("exposure_min_bright", "7")) + tuple(kv.split("=", 1) for kv in a.cam):
        bp.set_attribute(k, v)
    cam = world.spawn_actor(bp, carla.Transform(carla.Location(0, 0, 100)))
    q = queue.Queue()
    cam.listen(q.put)
    sem, qs = None, queue.Queue()
    if a.semantic:
        sbp = world.get_blueprint_library().find("sensor.camera.semantic_segmentation")
        for k, v in (("image_size_x", "1280"), ("image_size_y", "720"), ("fov", "80")):
            sbp.set_attribute(k, v)
        sem = world.spawn_actor(sbp, carla.Transform(carla.Location(0, 0, 100)))
        sem.listen(qs.put)
    try:
        for v in a.view:
            f = v.split(",")
            name, x, y, yaw = f[0], float(f[1]), float(f[2]), float(f[3])
            height, back, pitch = (float(f[4]), float(f[5]), float(f[6])) if len(f) > 4 else (3.4, 7.0, 15.0)
            cy = -y
            hits = world.cast_ray(carla.Location(x, cy, 200.0), carla.Location(x, cy, -100.0))
            z = hits[0].location.z if hits else 0.0
            r = math.radians(yaw)
            loc = carla.Location(x - back * math.cos(r), -(y - back * math.sin(r)), z + height)
            cam.set_transform(carla.Transform(loc, carla.Rotation(pitch=-pitch, yaw=-yaw)))
            if sem is not None:
                sem.set_transform(carla.Transform(loc, carla.Rotation(pitch=-pitch, yaw=-yaw)))
            img = simg = None
            for _ in range(40):  # let auto-exposure settle
                world.tick()
                img = q.get(timeout=60)
                simg = qs.get(timeout=60) if sem is not None else None
            arr = np.frombuffer(img.raw_data, np.uint8).reshape(img.height, img.width, 4)[..., :3][..., ::-1]
            from PIL import Image

            Image.fromarray(arr).save(os.path.join(a.out, f"{name}.png"))
            if simg is not None:
                simg.convert(carla.ColorConverter.CityScapesPalette)
                sarr = np.frombuffer(simg.raw_data, np.uint8).reshape(simg.height, simg.width, 4)[..., :3][..., ::-1]
                Image.fromarray(sarr).save(os.path.join(a.out, f"{name}_sem.png"))
            print(f"view {name}: ground {z:.2f} ({len(hits)} hits), mean {arr.mean():.0f}", flush=True)
    finally:
        for c in (cam, sem):
            if c is not None:
                c.stop()
                c.destroy()
        world.apply_settings(before)
        world.set_weather(weather_before)
    return 0


if __name__ == "__main__":
    sys.exit(main())
