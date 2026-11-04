#!/usr/bin/env python3
"""Roadmap 072: a proof video of a run someone else drives in CARLA. It waits for the ego (role
`hero`, as the CARLA world server spawns it), attaches a chase camera and a front camera to it
and records until the ego has stood still near the goal or is gone. It never ticks the world:
whoever drives it (the bridge, in synchronous mode) does, and each tick gives a frame.

    carla_watch_video.py OUT.mp4 [--port 2400] [--goal x,y] [--title ...] [--wait 600]

The frame is `carla_drive_video.py`'s: the chase camera large, the front camera, the site's
orthophoto with the track, and sim time, speed, steering and distance to the goal. Poses are in
the map frame less MAP_ORIGIN; CARLA's frame negates y.
"""
import argparse
import json
import math
import os
import queue
import subprocess
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from carla_drive_video import TopDown, W, H  # noqa: E402


def find_ego(client, role, deadline):
    while time.time() < deadline:
        try:
            world = client.get_world()
            ego = next((v for v in world.get_actors().filter("vehicle.*")
                        if v.attributes.get("role_name") == role), None)
        except RuntimeError:
            ego = None
        if ego is not None:
            return ego, world
        time.sleep(1.0)
    return None, None


def camera(world, ego, sensors, ev, min_bright, tf, fov="80", w="1280", h="720"):
    bp = world.get_blueprint_library().find("sensor.camera.rgb")
    for k, v in (("image_size_x", w), ("image_size_y", h), ("fov", fov), ("exposure_mode", "histogram"),
                 ("exposure_compensation", str(ev)), ("exposure_min_bright", str(min_bright))):
        bp.set_attribute(k, v)
    c = world.spawn_actor(bp, tf, attach_to=ego)
    q = queue.Queue()
    c.listen(q.put)
    sensors.append(c)
    return q


def compose(ch, fr, topdown, track, pose, goal, title, t, speed, d, steer, arrived):
    frame = np.full((H, W, 3), 18, np.uint8)
    frame[60:780, 0:1280] = np.frombuffer(ch.raw_data, np.uint8).reshape(720, 1280, 4)[..., :3]
    frame[60:420, 1280:1920] = np.frombuffer(fr.raw_data, np.uint8).reshape(360, 640, 4)[..., :3]
    frame[420:1020, 1280:1920] = topdown.draw(track, [], pose, goal)
    cv2.putText(frame, title, (14, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255, 255, 255), 2)
    cv2.putText(frame, "chase camera (CARLA RGB, auto-exposure)", (14, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    cv2.putText(frame, "front camera", (1292, 88), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
    for txt, pos in ((f"t {t:5.1f} s", (20, 840)), (f"{speed:4.1f} km/h", (250, 840)),
                     (f"to goal {d:5.1f} m", (540, 840)), (f"steer {steer:+.2f}", (900, 840))):
        cv2.putText(frame, txt, pos, cv2.FONT_HERSHEY_SIMPLEX, 1.1, (255, 255, 255), 2)
    cv2.putText(frame, "Autoware 1.5 (ego stack via acb, scenario by SSv2)", (20, 900),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
    if arrived:
        cv2.putText(frame, "ARRIVED", (20, 960), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 255, 0), 2)
    return frame


def main(argv=None):
    import carla

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("out")
    ap.add_argument("--port", type=int, default=2400)
    ap.add_argument("--goal", default="78.0,135.0")
    ap.add_argument("--start", default="314.25,115.19")
    ap.add_argument("--role", default="hero")
    ap.add_argument("--wait", type=float, default=900.0, help="s to wait for the ego")
    ap.add_argument("--ev", type=float, default=-1.5, help="exposure_compensation (072's sweep)")
    ap.add_argument("--min-bright", type=float, default=7.0, help="exposure_min_bright (072's sweep)")
    ap.add_argument("--state", help="also write state.jsonl here: per frame sim t, ego x, y, z (map minus "
                    "origin) and speed (m/s), as 059's recorder (parity P6)")
    ap.add_argument("--title", default="Autoware drives route r01 on the NTU campus in CARLA 0.9.16 (map built from the scan)")
    a = ap.parse_args(argv)
    gx, gy = (float(v) for v in a.goal.split(","))
    sx, sy = (float(v) for v in a.start.split(","))
    client = carla.Client("localhost", a.port)
    client.set_timeout(60.0)
    topdown = TopDown([(sx, sy), (gx, gy)], margin=60.0)
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgr24",
                           "-s", f"{W}x{H}", "-r", "20", "-i", "-", "-c:v", "libx264", "-preset", "medium",
                           "-crf", "22", "-pix_fmt", "yuv420p", "-movflags", "+faststart", a.out], stdin=subprocess.PIPE)
    assert ff.stdin is not None
    track, n, t_first, still_since, arrived = [], 0, None, None, False
    state = open(a.state, "w", buffering=1) if a.state else None
    ours = []  # every camera this client spawned, destroyed again at the end whatever happened
    deadline = time.time() + a.wait
    try:
        # The ego found may be gone before a frame comes (a world load, or the end of the last
        # scenario): find it again until one carries frames. Once recorded, the run's end
        # (the ego despawned, no more ticks) ends the video.
        while n == 0 and time.time() < deadline:
            ego, world = find_ego(client, a.role, deadline)
            if ego is None:
                break
            print(f"ego {ego.id} {ego.type_id}, frame {world.get_snapshot().frame}", flush=True)
            sensors = []
            try:
                qchase = camera(world, ego, sensors, a.ev, a.min_bright, carla.Transform(carla.Location(x=-7.0, z=3.4), carla.Rotation(pitch=-15)))
                qfront = camera(world, ego, sensors, a.ev, a.min_bright, carla.Transform(carla.Location(x=0.9, z=2.0)), fov="74.5", w="640", h="360")
                wait = max(1.0, deadline - time.time()) if n == 0 else 30.0
                while True:
                    try:
                        ch = qchase.get(timeout=wait)
                        fr = qfront.get(timeout=wait)
                    except queue.Empty:
                        print(f"no frames for {wait:.0f} s", flush=True)
                        break
                    wait = 30.0
                    if not ego.is_alive:
                        break
                    t_first = ch.timestamp if t_first is None else t_first
                    t = ch.timestamp - t_first
                    tf, v = ego.get_transform(), ego.get_velocity()
                    x, y, yaw = tf.location.x, -tf.location.y, -math.radians(tf.rotation.yaw)
                    if not track or math.hypot(x - track[-1][0], y - track[-1][1]) > 0.5:
                        track.append((x, y))
                    speed = math.hypot(v.x, v.y) * 3.6
                    d = math.hypot(x - gx, y - gy)
                    if d < 5.0 and speed < 0.5:
                        still_since = t if still_since is None else still_since
                        arrived = True
                    else:
                        still_since = None
                    frame = compose(ch, fr, topdown, track, (x, y, yaw), (gx, gy), a.title, t, speed, d,
                                    ego.get_control().steer, arrived)
                    ff.stdin.write(frame.tobytes())
                    if state:
                        state.write(json.dumps({"t": round(t, 3), "x": round(x, 3), "y": round(y, 3),
                                                "z": round(tf.location.z, 3), "speed": round(speed / 3.6, 3)}) + "\n")
                    n += 1
                    if still_since is not None and t - still_since > 3.0:
                        break
            finally:
                # stop and destroy each on its own: a stop that raised used to skip the destroy,
                # leaving a listening camera behind ("sensor object went out of the scope but the
                # sensor is still alive")
                for s in sensors:
                    ours.append(s.id)
                    for f in (s.stop, s.destroy):
                        try:
                            f()
                        except RuntimeError:
                            pass  # gone with its world
    finally:
        ff.stdin.close()
        ff.wait()
        if state:
            state.close()
        try:
            left = [x.id for x in client.get_world().get_actors(ours)] if ours else []
            if left:
                client.apply_batch_sync([carla.command.DestroyActor(i) for i in left])
                print(f"destroyed {len(left)} cameras left alive", flush=True)
        except RuntimeError:
            pass
    driven = sum(math.hypot(b[0] - p[0], b[1] - p[1]) for p, b in zip(track, track[1:]))
    print(f"{n} frames, {driven:.0f} m driven, {'ARRIVED' if arrived else 'not arrived'} -> {a.out}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
