#!/usr/bin/env python3
"""Roadmap 072: a proof video of a car driving the NTU map in CARLA 0.9.16. CARLA's own
BasicAgent drives route r01 from its start to its goal, planning on the map's OpenDRIVE (not
Autoware: that is 072's C11). Recorded in synchronous mode at 20 Hz:
- a chase camera, large;
- the front camera;
- the site's orthophoto with the driven track and the agent's route;
- a status bar: sim time, speed, steering, distance to goal.

    carla_drive_video.py OUT.mp4 [--port 2400] [--start x,y,yaw] [--goal x,y] [--speed 20]

Poses are in the map frame less MAP_ORIGIN (x, y, yaw deg counter-clockwise from +x), as
059's scenario and 072's OpenDRIVE; CARLA's frame negates y and yaw. Run with the NTU tool
venv, with the CARLA release's PythonAPI/carla on sys.path for the agents.
"""
import argparse
import json
import math
import os
import queue
import subprocess
import sys

import cv2
import numpy as np

CARLA_DIR = os.environ.get("CARLA_DIR", "/mnt/p5plus/aeon/carla/CARLA_0.9.16")
sys.path.insert(0, os.path.join(CARLA_DIR, "PythonAPI", "carla"))
GROUND = "/mnt/p5plus/aeon/ntu-twin/ground"
ORIGIN = (52500.0, 67700.0)
W, H = 1920, 1080


class TopDown:
    """The site orthophoto around the route, the track and the plan drawn on it."""

    def __init__(self, pts, size=(640, 600), margin=30.0):
        meta = json.load(open(f"{GROUND}/dtm.json"))
        ortho = cv2.imread(f"{GROUND}/site_ortho_25cm.jpg")
        shift = (-0.54, 0.20)  # vendor = map + shift (059)
        self.size = size
        P = np.asarray(pts) + ORIGIN  # map frame
        lo, hi = P.min(0) - margin, P.max(0) + margin
        span = max((hi[0] - lo[0]) / size[0], (hi[1] - lo[1]) / size[1])
        c = (lo + hi) / 2
        self.mpp, self.ox, self.oy = span, c[0] - size[0] / 2 * span, c[1] + size[1] / 2 * span
        x0, ytop, cell = meta["x0"] - shift[0], meta["y0"] + meta["height"] * meta["cell"] - shift[1], meta["cell"]
        c0, r0 = int((self.ox - x0) / cell), int((ytop - self.oy) / cell)
        c1, r1 = int((self.ox + size[0] * span - x0) / cell), int((ytop - self.oy + size[1] * span) / cell)
        # pad with black where the view runs past the site ortho (a route at the scan's edge, as
        # tour leg 03 is: a negative row index made the crop empty and the recorder died)
        pad = max(0, -r0, -c0, r1 - ortho.shape[0], c1 - ortho.shape[1])
        if pad:
            ortho = cv2.copyMakeBorder(ortho, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=(0, 0, 0))
            r0, r1, c0, c1 = r0 + pad, r1 + pad, c0 + pad, c1 + pad
        self.base = cv2.addWeighted(cv2.resize(ortho[r0:r1, c0:c1], size), 0.8, np.zeros((size[1], size[0], 3), np.uint8), 0.2, 0)

    def px(self, x, y):  # x, y in the map frame less ORIGIN
        return int((x + ORIGIN[0] - self.ox) / self.mpp), int((self.oy - y - ORIGIN[1]) / self.mpp)

    def draw(self, track, plan, pose, goal):
        img = self.base.copy()
        if len(plan) > 1:
            cv2.polylines(img, [np.array([self.px(*p) for p in plan], np.int32)], False, (0, 255, 0), 2, cv2.LINE_AA)
        if len(track) > 1:
            cv2.polylines(img, [np.array([self.px(*p) for p in track], np.int32)], False, (255, 200, 0), 3, cv2.LINE_AA)
        cv2.circle(img, self.px(*goal), 9, (0, 0, 255), 2)
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        poly = np.array([self.px(x + c * a - s * b, y + s * a + c * b)
                         for a, b in [(-1.0, -1.08), (3.8, -1.08), (3.8, 1.08), (-1.0, 1.08)]], np.int32)
        cv2.fillPoly(img, [poly], (0, 0, 255))
        cv2.putText(img, "track", (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)
        cv2.putText(img, "agent route", (90, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        bar = int(50 / self.mpp)
        cv2.line(img, (12, self.size[1] - 15), (12 + bar, self.size[1] - 15), (255, 255, 255), 3)
        cv2.putText(img, "50 m", (16 + bar, self.size[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        return img


def main(argv=None):
    import carla
    from agents.navigation.basic_agent import BasicAgent

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("out")
    ap.add_argument("--port", type=int, default=2400)
    ap.add_argument("--start", default="314.25,115.19,90.37")  # r01's start (059 N8)
    ap.add_argument("--goal", default="78.0,135.0")  # r01's goal, mid lanelet 1004936
    ap.add_argument("--speed", type=float, default=20.0, help="km/h")
    ap.add_argument("--timeout", type=float, default=240.0, help="sim s")
    ap.add_argument("--title", default="NTU campus in CARLA 0.9.16 (built from the scan) - route r01, CARLA's BasicAgent")
    a = ap.parse_args(argv)
    sx, sy, syaw = (float(v) for v in a.start.split(","))
    gx, gy = (float(v) for v in a.goal.split(","))

    client = carla.Client("localhost", a.port)
    client.set_timeout(300.0)
    world = client.get_world()
    if not world.get_map().name.endswith("NTU"):
        world = client.load_world("NTU")
    s = world.get_settings()
    s.synchronous_mode, s.fixed_delta_seconds = True, 0.05
    world.apply_settings(s)
    world.set_weather(carla.WeatherParameters(cloudiness=10.0, sun_altitude_angle=55.0, sun_azimuth_angle=135.0))
    bl = world.get_blueprint_library()
    hit = world.cast_ray(carla.Location(sx, -sy, 200.0), carla.Location(sx, -sy, -100.0))
    z = hit[0].location.z if hit else 12.0
    car = world.spawn_actor(bl.find("vehicle.tesla.model3"),
                            carla.Transform(carla.Location(sx, -sy, z + 0.5), carla.Rotation(yaw=-syaw)))
    actors = [car]
    try:
        def camera(tf, fov="80", w="1280", h="720"):
            bp = bl.find("sensor.camera.rgb")
            for k, v in (("image_size_x", w), ("image_size_y", h), ("fov", fov), ("exposure_mode", "histogram")):
                bp.set_attribute(k, v)
            c = world.spawn_actor(bp, tf, attach_to=car)
            q = queue.Queue()
            c.listen(q.put)
            actors.append(c)
            return q

        qchase = camera(carla.Transform(carla.Location(x=-7.0, z=3.4), carla.Rotation(pitch=-15)))
        qfront = camera(carla.Transform(carla.Location(x=0.9, z=2.0)), fov="74.5", w="640", h="360")
        for _ in range(20):
            world.tick()
            qchase.get()
            qfront.get()
        agent = BasicAgent(car, target_speed=a.speed)
        agent.set_destination(carla.Location(gx, -gy, z))
        plan = [(wp.transform.location.x, -wp.transform.location.y)
                for wp, _ in agent.get_local_planner().get_plan()]
        if not plan:
            raise SystemExit("the agent found no route")
        topdown = TopDown(plan + [(sx, sy), (gx, gy)])
        ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgr24",
                               "-s", f"{W}x{H}", "-r", "20", "-i", "-", "-c:v", "libx264", "-preset", "medium",
                               "-crf", "22", "-pix_fmt", "yuv420p", "-movflags", "+faststart", a.out],
                              stdin=subprocess.PIPE)
        assert ff.stdin is not None
        track, t, arrived, stop_t = [], 0.0, False, None
        while t < a.timeout:
            car.apply_control(agent.run_step())
            world.tick()
            t += 0.05
            ch, fr = qchase.get(), qfront.get()
            tf, v = car.get_transform(), car.get_velocity()
            x, y, yaw = tf.location.x, -tf.location.y, -math.radians(tf.rotation.yaw)
            if not track or math.hypot(x - track[-1][0], y - track[-1][1]) > 0.5:
                track.append((x, y))
            speed = math.hypot(v.x, v.y) * 3.6
            frame = np.full((H, W, 3), 18, np.uint8)
            frame[60:780, 0:1280] = np.frombuffer(ch.raw_data, np.uint8).reshape(720, 1280, 4)[..., :3]
            frame[60:420, 1280:1920] = np.frombuffer(fr.raw_data, np.uint8).reshape(360, 640, 4)[..., :3]
            frame[420:1020, 1280:1920] = topdown.draw(track, plan, (x, y, yaw), (gx, gy))
            cv2.putText(frame, a.title, (14, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
            cv2.putText(frame, "chase camera (CARLA RGB, auto-exposure)", (14, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(frame, "front camera", (1292, 88), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
            d = math.hypot(x - gx, y - gy)
            steer = car.get_control().steer
            for txt, pos in ((f"t {t:5.1f} s", (20, 840)), (f"{speed:4.1f} km/h", (250, 840)),
                             (f"to goal {d:5.1f} m", (540, 840)), (f"steer {steer:+.2f}", (900, 840))):
                cv2.putText(frame, txt, pos, cv2.FONT_HERSHEY_SIMPLEX, 1.1, (255, 255, 255), 2)
            state = "ARRIVED" if arrived else "driving"
            cv2.putText(frame, state, (20, 900), cv2.FONT_HERSHEY_SIMPLEX, 1.1,
                        (0, 255, 0) if arrived else (0, 200, 255), 2)
            ff.stdin.write(frame.tobytes())
            if not arrived and (agent.done() or d < 3.0) and speed < 1.0:
                arrived, stop_t = True, t
            if arrived and t - stop_t > 3.0:
                break
        ff.stdin.close()
        ff.wait()
        driven = sum(math.hypot(b[0] - a_[0], b[1] - a_[1]) for a_, b in zip(track, track[1:]))
        print(f"{'ARRIVED' if arrived else 'NOT ARRIVED'} after {stop_t or t:.1f} s sim, {driven:.0f} m driven, "
              f"end {math.hypot(track[-1][0] - gx, track[-1][1] - gy):.1f} m from the goal -> {a.out}")
    finally:
        for x in actors[::-1]:
            x.destroy()
        s.synchronous_mode = False
        world.apply_settings(s)
    return 0


if __name__ == "__main__":
    sys.exit(main())
