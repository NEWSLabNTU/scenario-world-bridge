#!/usr/bin/env python3
"""Roadmap 072, the proof run: one H.264 video of every leg of a run_proof.sh batch.

A title card, then each leg's recorder video (NN.mp4, carla_watch_video.py's 1920x1080
composite) at SPEED times real time with its label (leg, route, result, metres driven,
MRM, NVTL min), then an end card with the per-leg table from tour_table.py.

    python3 scripts/ntu_carla/proof_video.py RUN_DIR OUT.mp4 --legs 00,01,...,r01,r02
        [--speed 5] [--title TEXT] [--title-line TEXT]... [--table RUN_DIR/table.json]

--title replaces the first title line (default: the CARLA one); roadmap 059's Isaac proof run
uses the same script with its own.

The table is tour_table.py's --json output for RUN_DIR (computed here when not given).
"""
import argparse
import json
import pathlib
import subprocess
import sys

import cv2
import numpy as np

W, H, FPS = 1920, 1080, 20
FONT = cv2.FONT_HERSHEY_SIMPLEX


def text(img, s, xy, scale=1.0, color=(255, 255, 255), thick=2):
    cv2.putText(img, s, xy, FONT, scale, (0, 0, 0), thick + 3, cv2.LINE_AA)
    cv2.putText(img, s, xy, FONT, scale, color, thick, cv2.LINE_AA)


def card(lines, sizes=None):
    img = np.zeros((H, W, 3), np.uint8)
    img[:] = (32, 28, 24)
    y = 140
    for i, s in enumerate(lines):
        sc = (sizes or {}).get(i, 0.9)
        text(img, s, (110, y), sc, thick=2 if sc < 1.2 else 3)
        y += int(60 * sc) + 10
    return img


def table_cards(rows, per=26):
    head = f"{'leg':>4}  {'result':<6} {'driven':>8} {'route':>7} {'sim':>7}  {'MRM':>3}  {'NVTL min':>8}"
    cards = []
    for k in range(0, len(rows), per):
        img = np.zeros((H, W, 3), np.uint8)
        img[:] = (32, 28, 24)
        n_pass = sum(r["pass"] for r in rows)
        text(img, f"Per-leg results ({n_pass} of {len(rows)} pass)  [{k // per + 1}/{(len(rows) + per - 1) // per}]",
             (110, 90), 1.1, thick=3)
        cv2.putText(img, head, (110, 150), cv2.FONT_HERSHEY_PLAIN, 2.0, (200, 200, 200), 2, cv2.LINE_AA)
        for i, r in enumerate(rows[k:k + per]):
            nv = "-" if r.get("nvtl_min") is None else f"{r['nvtl_min']:.2f}"
            s = (f"{r['leg']:>4}  {'pass' if r['pass'] else 'FAIL':<6} {r['driven']:>6.0f} m {r['route']:>5.0f} m "
                 f"{r['moving']:>5.0f} s  {r['mrm']:>3}  {nv:>8}")
            col = (220, 220, 220) if r["pass"] else (80, 80, 255)
            cv2.putText(img, s, (110, 190 + 33 * i), cv2.FONT_HERSHEY_PLAIN, 2.0, col, 2, cv2.LINE_AA)
        cards.append(img)
    return cards


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("run")
    ap.add_argument("out")
    ap.add_argument("--legs", required=True, help="comma-separated: 00,01,...,r01,r02")
    ap.add_argument("--speed", type=int, default=5)
    ap.add_argument("--title", default="Autoware drives the NTU campus twin in CARLA 0.9.16")
    ap.add_argument("--title-line", action="append", default=[])
    ap.add_argument("--table")
    ap.add_argument("--card-s", type=float, default=4.0, help="seconds per card")
    a = ap.parse_args(argv)
    run = pathlib.Path(a.run)
    legs = a.legs.split(",")

    tpath = pathlib.Path(a.table) if a.table else run / "table.json"
    if not tpath.exists():
        subprocess.run([sys.executable, str(pathlib.Path(__file__).with_name("tour_table.py")), str(run),
                        "--json", str(tpath)], check=True, stdout=subprocess.DEVNULL)
    table = {str(r["leg"]): r for r in json.load(open(tpath))}
    rows = []
    for n in legs:
        r = table.get(n, {})
        rows.append({"leg": n, "pass": r.get("junit") == 'failures="0" errors="0"',
                     "driven": float(r.get("driven") or 0),
                     "route": float(r.get("planned") or 0), "moving": float(r.get("sim_s") or 0),
                     "mrm": int(r.get("mrm") or 0), "nvtl_min": r.get("nvtl_min")})

    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgr24",
                           "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-preset", "medium",
                           "-crf", "23", "-pix_fmt", "yuv420p", "-movflags", "+faststart", a.out],
                          stdin=subprocess.PIPE)

    def put(img, n=1):
        for _ in range(n):
            ff.stdin.write(img.tobytes())

    ncard = int(a.card_s * FPS)
    n_pass = sum(r["pass"] for r in rows)
    title = [a.title,
             f"Every leg of the tour: {len(rows)} legs, {n_pass} pass, "
             f"{sum(r['driven'] for r in rows) / 1000:.1f} km driven",
             f"Each leg at {a.speed}x real time"] + a.title_line
    put(card(title, {0: 1.4}), ncard)
    for r in rows:
        src = run / f"{r['leg']}.mp4"
        cap = cv2.VideoCapture(str(src))
        if not cap.isOpened():
            put(card([f"leg {r['leg']}: no video"], {0: 1.4}), FPS)
            continue
        nv = "-" if r["nvtl_min"] is None else f"{r['nvtl_min']:.2f}"
        label = (f"leg {r['leg']}  {'PASS' if r['pass'] else 'FAIL'}  {r['driven']:.0f} m  "
                 f"MRM {r['mrm']}  NVTL min {nv}  {a.speed}x")
        i = 0
        while True:
            ok, img = cap.read()
            if not ok:
                break
            if i % a.speed == 0:
                if img.shape[:2] != (H, W):
                    img = cv2.resize(img, (W, H), interpolation=cv2.INTER_AREA)
                cv2.rectangle(img, (0, H - 56), (W, H), (0, 0, 0), -1)
                text(img, label, (20, H - 16), 1.0, (255, 255, 255) if r["pass"] else (80, 80, 255))
                put(img)
            i += 1
        cap.release()
    for c in table_cards(rows):
        put(c, int(ncard * 1.5))
    ff.stdin.close()
    sys.exit(ff.wait())


if __name__ == "__main__":
    main()
