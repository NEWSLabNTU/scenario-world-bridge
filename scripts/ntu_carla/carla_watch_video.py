#!/usr/bin/env python3
"""Roadmap 072: a proof video of a run someone else drives in CARLA. It waits for the ego (role
`hero`, as the CARLA world server spawns it), attaches a chase camera and a front camera to it
and records until the ego has stood still near the goal or is gone. It never ticks the world:
whoever drives it (the bridge, in synchronous mode) does, and each tick gives a frame.

    carla_watch_video.py OUT.mp4 [--port 2400] [--goal x,y] [--title ...] [--wait 600]

The frame is `carla_drive_video.py`'s: the chase camera large, the front camera, the site's
orthophoto with the track, and sim time, speed, steering and distance to the goal. Poses are in
the map frame less MAP_ORIGIN; CARLA's frame negates y.

`--mounts rig` (the default) puts the cameras where 059's Isaac recorder has them (parity P6):
the rig's poses in Autoware's base_link (the rear axle, on the ground), moved to the CARLA
actor's origin, which is REAR_AXLE ahead of the axle and at ground level on the model3. So
CARLA's frames compare with Isaac's one to one. `--mounts legacy` keeps the first videos'.
`--raw PREFIX` also writes the two cameras alone at 1280x720, PREFIX_chase.mp4 and
PREFIX_front.mp4, for side-by-side clips.
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


REAR_AXLE = 1.386  # m, the model3's rear axle behind the CARLA actor's origin (swb_isaac_world)
# (x, y, z) in base_link, (pitch, yaw) in CARLA's convention (degrees; pitch up, yaw right), fov
MOUNTS = {
    "rig": {"chase": ((-7.0, 0.0, 3.4), (-15.0, 0.0), 80.0),
            "front": ((0.9, 0.0, 2.0), (-0.86, 2.09), 74.5)},   # aib's camera6
    "legacy": {"chase": ((-7.0 + REAR_AXLE, 0.0, 3.4), (-15.0, 0.0), 80.0),
               "front": ((0.9 + REAR_AXLE, 0.0, 2.0), (0.0, 0.0), 74.5)},
}


def mount(carla, name, which):
    (x, y, z), (pitch, yaw), fov = MOUNTS[name][which]
    return carla.Transform(carla.Location(x=x - REAR_AXLE, y=y, z=z), carla.Rotation(pitch=pitch, yaw=yaw)), fov


def rgb(img):
    return np.frombuffer(img.raw_data, np.uint8).reshape(img.height, img.width, 4)[..., :3]


def compose(ch, fr, topdown, track, pose, goal, title, t, speed, d, steer, arrived):
    frame = np.full((H, W, 3), 18, np.uint8)
    frame[60:780, 0:1280] = rgb(ch)
    frame[60:420, 1280:1920] = cv2.resize(rgb(fr), (640, 360), interpolation=cv2.INTER_AREA)
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
    ap.add_argument("--mounts", choices=sorted(MOUNTS), default="rig",
                    help="rig: 059's Isaac camera poses (parity P6); legacy: the first videos'")
    ap.add_argument("--raw", default="", help="also write PREFIX_chase.mp4 and PREFIX_front.mp4, 1280x720 each")
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

    def encoder(path):
        return subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgr24",
                                 "-s", "1280x720", "-r", "20", "-i", "-", "-c:v", "libx264", "-preset", "medium",
                                 "-crf", "20", "-pix_fmt", "yuv420p", "-movflags", "+faststart", path],
                                stdin=subprocess.PIPE)
    raw = [encoder(f"{a.raw}_chase.mp4"), encoder(f"{a.raw}_front.mp4")] if a.raw else []
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
                tf_ch, fov_ch = mount(carla, a.mounts, "chase")
                tf_fr, fov_fr = mount(carla, a.mounts, "front")
                qchase = camera(world, ego, sensors, a.ev, a.min_bright, tf_ch, fov=str(fov_ch))
                qfront = camera(world, ego, sensors, a.ev, a.min_bright, tf_fr, fov=str(fov_fr))
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
                    for enc, img in zip(raw, (ch, fr)):
                        enc.stdin.write(np.ascontiguousarray(rgb(img)).tobytes())
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
        for enc in [ff] + raw:
            enc.stdin.close()
            enc.wait()
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
