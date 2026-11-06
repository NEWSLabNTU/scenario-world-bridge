#!/usr/bin/env python3
"""Roadmaps 072 and 059: the two proof runs side by side, CARLA on the left, Isaac Sim on the right.

    python3 scripts/ntu_carla/proof_side_by_side.py CARLA_RUN ISAAC_RUN OUT.mp4 --legs 00,...,r02
        [--speed 5] [--card-s 4]

Each run directory holds, per leg, NN.mp4 (the recorder's 1920x1080 composite, one frame per
NN_state.jsonl line, 20 fps of sim time) and table.json (tour_table.py's rows). CARLA's composite
is carla_watch_video.py's: chase camera at (0, 60, 1280x720), front camera at (1280, 60,
640x360), the track map at (1280, 420, 640x600). Isaac's is the chase camera, full frame, with
the front camera inset at the top right; CARLA's view is put together the same way here.

The two sides start each leg together at the ego's first motion (speed over START_SPEED, less
LEAD frames) and run at --speed; the one that arrives first holds its last frame. Under them:
each side's time, speed and the table's result, and CARLA's track map in the middle.
"""
import argparse
import json
import pathlib
import subprocess
import sys

import cv2
import numpy as np

W, H, FPS = 1920, 1080, 20
VW, VH = 960, 540  # one side's view
TOP = 70  # the title band
FONT = cv2.FONT_HERSHEY_SIMPLEX
START_SPEED = 0.3  # m/s
LEAD = 20  # frames kept before the first motion
BG = (32, 28, 24)


def text(img, s, xy, scale=1.0, color=(255, 255, 255), thick=2):
    cv2.putText(img, s, xy, FONT, scale, (0, 0, 0), thick + 3, cv2.LINE_AA)
    cv2.putText(img, s, xy, FONT, scale, color, thick, cv2.LINE_AA)


def card(lines, sizes=None):
    img = np.full((H, W, 3), BG, np.uint8)
    y = 160
    for i, s in enumerate(lines):
        sc = (sizes or {}).get(i, 0.9)
        text(img, s, (110, y), sc, thick=2 if sc < 1.2 else 3)
        y += int(60 * sc) + 14
    return img


def carla_view(img):
    """CARLA's composite as Isaac's: the chase camera with the front camera inset at the top right."""
    v = cv2.resize(img[84:780, 0:1280], (VW, VH), interpolation=cv2.INTER_AREA)
    f = cv2.resize(img[60:420, 1280:1920], (288, 162), interpolation=cv2.INTER_AREA)
    v[12:12 + 162, VW - 12 - 288:VW - 12] = f
    return v


def carla_map(img):
    return img[420:1020, 1280:1920]


def isaac_view(img):
    return cv2.resize(img, (VW, VH), interpolation=cv2.INTER_AREA)


class Side:
    def __init__(self, run, leg, row):
        self.cap = cv2.VideoCapture(str(run / f"{leg}.mp4"))
        self.state = [json.loads(s) for s in open(run / f"{leg}_state.jsonl")]
        self.row = row
        moving = [i for i, s in enumerate(self.state) if abs(s.get("speed", 0)) > START_SPEED]
        self.start = max(0, (moving[0] if moving else 0) - LEAD)
        self.i = 0
        self.img = None
        self.done = not self.cap.isOpened()
        while not self.done and self.i < self.start:
            self.done = not self.cap.grab()
            self.i += 1

    def step(self, n):
        """Advance n frames; keep the last one at the end."""
        for k in range(n):
            if self.done:
                return
            ok = self.cap.grab() if k < n - 1 else False
            if k == n - 1:
                ok, img = self.cap.read()
                if ok:
                    self.img = img
            if not ok:
                self.done = True
                return
            self.i += 1

    def status(self):
        s = self.state[min(max(self.i - 1, 0), len(self.state) - 1)]
        t = (min(self.i, len(self.state)) - self.start) / FPS
        return t, abs(s.get("speed", 0.0)) * 3.6


def result(row):
    ok = row.get("junit") == 'failures="0" errors="0"'
    s = f"{'PASS' if ok else 'FAIL'}  {float(row.get('driven') or 0):.0f} m  MRM {int(row.get('mrm') or 0)}"
    if "aeb" in row:
        s += f"  AEB {int(row.get('aeb') or 0)}"
    nv = row.get("nvtl_min")
    if nv is not None:
        s += f"  NVTL min {nv:.2f}"
    return s, ok


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("carla")
    ap.add_argument("isaac")
    ap.add_argument("out")
    ap.add_argument("--legs", required=True)
    ap.add_argument("--speed", type=int, default=5)
    ap.add_argument("--card-s", type=float, default=4.0)
    ap.add_argument("--title", default="Autoware drives the NTU campus twin: CARLA 0.9.16 and Isaac Sim")
    a = ap.parse_args(argv)
    runs = {k: pathlib.Path(getattr(a, k)) for k in ("carla", "isaac")}
    # table.json in the run directory, or beside it (the NAS layout: <side>/table.json, <side>/runs/)
    tables = {k: {str(r["leg"]): r for r in json.load(open(p / "table.json" if (p / "table.json").exists()
                                                         else p.parent / "table.json"))}
              for k, p in runs.items()}
    legs = a.legs.split(",")

    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgr24",
                           "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-preset", "medium",
                           "-crf", "23", "-pix_fmt", "yuv420p", "-movflags", "+faststart", a.out],
                          stdin=subprocess.PIPE)

    def put(img, n=1):
        for _ in range(n):
            ff.stdin.write(img.tobytes())

    ncard = int(a.card_s * FPS)
    summary = {}
    for k in runs:
        rows = [tables[k].get(n, {}) for n in legs]
        summary[k] = (sum(result(r)[1] for r in rows), sum(float(r.get("driven") or 0) for r in rows) / 1000)
    put(card([a.title,
              f"The same {len(legs)} legs, the same Lanelet2 map, the same Autoware 1.5 and vehicle spec",
              f"CARLA 0.9.16: {summary['carla'][0]} of {len(legs)} pass, {summary['carla'][1]:.1f} km",
              f"Isaac Sim: {summary['isaac'][0]} of {len(legs)} pass, {summary['isaac'][1]:.1f} km",
              f"Each leg starts together on both sides at the ego's first motion, {a.speed}x real time"],
             {0: 1.3}), ncard)

    for leg in legs:
        sides = {k: Side(runs[k], leg, tables[k].get(leg, {})) for k in runs}
        for s in sides.values():
            s.step(1)
        while True:
            frame = np.full((H, W, 3), BG, np.uint8)
            text(frame, f"leg {leg}", (24, 48), 1.2, thick=3)
            text(frame, "CARLA 0.9.16", (VW // 2 - 110, 48), 1.0)
            text(frame, "Isaac Sim", (VW + VW // 2 - 80, 48), 1.0)
            c, i = sides["carla"], sides["isaac"]
            if c.img is not None:
                frame[TOP:TOP + VH, 0:VW] = carla_view(c.img)
                m = cv2.resize(carla_map(c.img), (400, 375), interpolation=cv2.INTER_AREA)
                frame[TOP + VH + 12:TOP + VH + 12 + 375, VW - 200:VW + 200] = m
            if i.img is not None:
                frame[TOP:TOP + VH, VW:2 * VW] = isaac_view(i.img)
            for k, x0 in (("carla", 24), ("isaac", VW + 220)):
                s = sides[k]
                t, v = s.status()
                res, ok = result(s.row)
                y = TOP + VH + 50
                text(frame, f"t {t:6.1f} s   {v:5.1f} km/h", (x0, y), 0.9)
                text(frame, "arrived" if s.done else "driving", (x0, y + 50), 0.9,
                     (120, 230, 120) if s.done else (200, 200, 200))
                text(frame, res, (x0, y + 100), 0.75, (255, 255, 255) if ok else (80, 80, 255))
            text(frame, f"{a.speed}x", (W - 90, 48), 1.0)
            put(frame)
            if all(s.done for s in sides.values()):
                put(frame, FPS // 2)
                break
            for s in sides.values():
                s.step(a.speed)
        for s in sides.values():
            s.cap.release()
    put(card([a.title,
              f"CARLA 0.9.16: {summary['carla'][0]} of {len(legs)} legs pass, {summary['carla'][1]:.1f} km",
              f"Isaac Sim: {summary['isaac'][0]} of {len(legs)} legs pass, {summary['isaac'][1]:.1f} km"],
             {0: 1.3}), ncard)
    ff.stdin.close()
    sys.exit(ff.wait())


if __name__ == "__main__":
    main()
