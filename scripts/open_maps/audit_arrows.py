#!/usr/bin/env python3
"""Sheets for the arrow audit by a person (roadmap 062, O4): for each signalised approach, the
map's movements per lane beside what the paint shows, from above and from the street.

    scripts/open_maps/audit_arrows.py <lanelet2_map.osm> <site.yaml> <raw dir> <out dir>
        [--only <light id>,...] [--images 3] [--back 55]

Per approach (a stop line and its lanes), `<out dir>/<light id>.jpg`:

- **From above:** NLSC's open 0.27 m photo resampled along the approach (0.1 m a pixel, the
  stop line at the top, the last `--back` m below it). Each lane's centre is drawn with its
  number and the map's movements (L, S, R, U) from the left. Bright paint on a lane's centre
  2.5-7 m long is boxed as a candidate arrow (a lane line lies on a bound, a stop line or a
  zebra across, so neither is boxed); text markings (公車專用, 慢) are boxed too.
- **Mapillary's arrows** (its detections, `mapillary_features.json`) on the approach, as a
  dot with their letters, where they fall.
- **From the street:** up to `--images` Mapillary images taken on the approach facing the
  stop line (or panoramas, cropped to the approach's heading), one per sequence: those 10-35 m
  before the stop line first, where the arrows read best, then those since late 2023, then the
  newest. Images from a viaduct over the approach are left out (Mapillary's camera height is
  about 2 m on the road, 5-20 m on a viaduct). Each image is cropped to the road ahead. The
  lanes' centres, numbers and the stop line are projected into it, on a flat road from the
  camera's own pose (Mapillary's `computed_rotation`, `camera_parameters` and height). GPS puts
  them 1-2 lanes and up to 15 m off as often as not, so the lanes are to be counted from the
  left; the overlay says which way they run and roughly where the stop line is. Image metadata and
  thumbnails are cached in `<out dir>/mly/`; it needs MAPILLARY_TOKEN.

`<out dir>/arrows.json` lists per approach the lanes (lanelet, the map's moves and their
source), the candidates (lane, metres before the stop line), Mapillary's arrows and the images
used. The verdicts go in the site's audit file (`sites/<site>_audit.yaml`) as for
audit_junctions.py. The photos and Mapillary's images are evidence for a person, never an
input to the map.
"""
import argparse
import json
import math
import os
import time
from collections import defaultdict
from pathlib import Path

import numpy as np
import requests
from PIL import Image, ImageDraw, ImageFont
from scipy import ndimage

from audit_junctions import LETTER, Nlsc, load

RES = 0.1
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
MLY = "https://graph.mapillary.com"
COLS = [(255, 220, 0), (0, 230, 255), (255, 120, 220), (120, 255, 120), (255, 160, 60), (170, 150, 255)]
ARROW_WORDS = {"left": "L", "straight": "S", "right": "R", "u-turn": "U", "uturn": "U"}


def moves_word(mv):
    return "".join(LETTER[m] for m in ("uturn", "left", "straight", "right") if m in mv) or "-"


def resample(pts, n):
    pts = np.asarray(pts, float)
    seg = np.hypot(*np.diff(pts, axis=0).T)
    s = np.concatenate([[0], np.cumsum(seg)])
    t = np.linspace(0, s[-1], n)
    return np.c_[np.interp(t, s, pts[:, 0]), np.interp(t, s, pts[:, 1])]


def rotation(rv):
    rv = np.asarray(rv, float)
    th = np.linalg.norm(rv)
    if th < 1e-9:
        return np.eye(3)
    k = rv / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


class Camera:
    """A Mapillary image's camera (OpenSfM's models; world east-north-up, R world to camera)."""

    def __init__(self, meta, pos, size):
        self.R = rotation(meta["computed_rotation"])
        self.kind = meta.get("camera_type", "perspective")
        p = meta.get("camera_parameters") or [0.8, 0, 0]
        self.f, self.k1, self.k2 = (list(p) + [0, 0])[:3]
        h = meta.get("computed_altitude")
        self.h = h if h and 0.8 < h < 3.5 else 1.6
        self.pos = pos
        self.w, self.h_px = size

    def project(self, pts):
        """Ground points (map x, y) to pixels; None where behind the camera or off the image."""
        out = []
        for x, y in pts:
            c = self.R @ np.array([x - self.pos[0], y - self.pos[1], -self.h])
            if self.kind == "spherical":
                lon = math.atan2(c[0], c[2])
                lat = -math.atan2(c[1], math.hypot(c[0], c[2]))
                u = (lon / (2 * math.pi) + 0.5) * self.w
                v = (-lat / math.pi + 0.5) * self.h_px
                out.append((u, v))
                continue
            if c[2] < 0.5:
                out.append(None)
                continue
            xn, yn = c[0] / c[2], c[1] / c[2]
            r2 = xn * xn + yn * yn
            if self.kind == "fisheye":
                r = math.sqrt(r2)
                th = math.atan(r)
                d = th * (1 + self.k1 * th ** 2 + self.k2 * th ** 4)
                sc = d / r if r > 1e-9 else 1.0
            else:
                if r2 > 1.5:
                    out.append(None)
                    continue
                sc = 1 + self.k1 * r2 + self.k2 * r2 * r2
            m = max(self.w, self.h_px)
            u, v = self.f * xn * sc * m + self.w / 2, self.f * yn * sc * m + self.h_px / 2
            out.append((u, v) if -self.w < u < 2 * self.w and -self.h_px < v < 2 * self.h_px else None)
        return out


class Mapillary:
    def __init__(self, cache):
        self.token = os.environ.get("MAPILLARY_TOKEN")
        self.cache = cache
        cache.mkdir(parents=True, exist_ok=True)

    def get(self, iid):
        meta_p, img_p = self.cache / f"{iid}.json", self.cache / f"{iid}.jpg"
        if not meta_p.exists():
            if not self.token:
                return None, None
            r = requests.get(f"{MLY}/{iid}", params={"access_token": self.token, "fields":
                             "thumb_2048_url,computed_rotation,camera_parameters,camera_type,computed_altitude,captured_at"},
                             timeout=60)
            if r.status_code != 200:
                return None, None
            meta_p.write_text(r.text)
        meta = json.loads(meta_p.read_text())
        if "computed_rotation" not in meta or "thumb_2048_url" not in meta:
            return None, None
        if not img_p.exists():
            for _ in range(3):
                try:
                    img_p.write_bytes(requests.get(meta["thumb_2048_url"], timeout=60).content)
                    break
                except requests.RequestException:
                    time.sleep(2)
        try:
            return meta, Image.open(img_p).convert("RGB")
        except OSError:
            return None, None


def arrow_letters(value):
    words = value.split("--")[-1].replace("split-", "").split("-or-")
    return "".join(sorted({ARROW_WORDS.get(w, "?") for w in words}, key="ULSR?".index))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("map")
    ap.add_argument("site")
    ap.add_argument("raw", type=Path)
    ap.add_argument("out", type=Path)
    ap.add_argument("--only", default="", help="light ids, comma-separated")
    ap.add_argument("--images", type=int, default=6)
    ap.add_argument("--back", type=float, default=55.0)
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    xy, ways, lanes, regs = load(a.map)
    photo = Nlsc(a.site, a.raw)
    frame = photo.frame
    mly = Mapillary(a.out / "mly")
    feats = [f for f in json.loads((a.raw / "mapillary_features.json").read_text())
             if "arrow" in f.get("object_value", "")]
    for f in feats:
        f["xy"] = frame.xy(f["geometry"]["coordinates"][1], f["geometry"]["coordinates"][0])
    images = json.loads((a.raw / "mapillary_images.json").read_text())
    for im in images:
        g = (im.get("computed_geometry") or im["geometry"])["coordinates"]
        im["xy"] = frame.xy(g[1], g[0])
    ixy = np.array([im["xy"] for im in images])
    font = ImageFont.truetype(FONT, 20)
    small = ImageFont.truetype(FONT, 16)
    big = ImageFont.truetype(FONT, 34)

    start_of = defaultdict(list)
    for k, ln in lanes.items():
        start_of[(ln["l"][0], ln["r"][0])].append(k)
    end_of = defaultdict(list)
    for k, ln in lanes.items():
        end_of[(ln["l"][-1], ln["r"][-1])].append(k)

    def centre(k):
        ln = lanes[k]
        L, R = [xy[i] for i in ln["l"]], [xy[i] for i in ln["r"]]
        n = max(len(L), len(R), 2) * 4
        lr, rr = resample(L, n), resample(R, n)
        return (lr + rr) / 2, float(np.median(np.hypot(*(lr - rr).T)))

    def upstream(k, need):
        """The lane's centre line back from its end, through its predecessors, `need` m long."""
        pts, wid = centre(k)
        seen = {k}
        while True:
            length = float(np.hypot(*np.diff(pts, axis=0).T).sum())
            ln = lanes[k]
            prev = [p for p in end_of.get((ln["l"][0], ln["r"][0]), [])
                    if "turn_direction" not in lanes[p]["t"] and p not in seen]
            if length >= need or not prev:
                return pts, wid
            k = prev[0]
            seen.add(k)
            pp, _ = centre(k)
            pts = np.vstack([pp[:-1], pts])

    approaches = defaultdict(list)
    for k, ln in lanes.items():
        for g in ln["regs"]:
            if g in regs:
                approaches[g].append(k)
    only = set(a.only.split(",")) - {""}
    report = []
    for gid, ks in sorted(approaches.items(), key=lambda kv: int(kv[0])):
        line = regs[gid]
        L0, R0 = np.array(xy[line[0]]), np.array(xy[line[-1]])
        cs = {k: upstream(k, a.back + 5) for k in set(ks)}
        # forward: the lanes' mean direction over their last 10 m
        fw = np.zeros(2)
        for pts, _ in cs.values():
            seg = pts[-1] - pts[max(0, len(pts) - 1 - int(10 / max(1e-3, np.hypot(*(pts[-1] - pts[-2])))))]
            fw += seg / (np.linalg.norm(seg) + 1e-9)
        fw /= np.linalg.norm(fw) + 1e-9
        rt = np.array([fw[1], -fw[0]])
        O = (L0 + R0) / 2
        to_sa = lambda p: (float((np.asarray(p) - O) @ fw), float((np.asarray(p) - O) @ rt))
        ks = sorted(set(ks), key=lambda k: to_sa(cs[k][0][-1])[1])
        heading = math.degrees(math.atan2(fw[0], fw[1])) % 360
        name = lanes[ks[0]]["t"].get("name", "")
        if only and gid not in only:
            continue
        succ_moves = []
        for k in ks:
            ln = lanes[k]
            mv, src = set(), set()
            for sk in start_of.get((ln["l"][-1], ln["r"][-1]), []):
                st = lanes[sk]["t"]
                if "turn_direction" in st:
                    mv.add("uturn" if st.get("swb:uturn") else st["turn_direction"])
                    src.add(st.get("swb:turn_source", ""))
            succ_moves.append((sorted(mv), sorted(src)))

        # the strip: across = a, along = s (0 at the stop line, negative behind it)
        offs = [to_sa(cs[k][0][-1])[1] for k in ks]
        widths = [cs[k][1] for k in ks]
        a0 = min(o - w / 2 for o, w in zip(offs, widths)) - 3
        a1 = max(o + w / 2 for o, w in zip(offs, widths)) + 3
        s0, s1 = -a.back, 8.0
        A, S = np.meshgrid(np.arange(a0, a1, RES), np.arange(s1, s0, -RES))
        X = O[0] + S * fw[0] + A * rt[0]
        Y = O[1] + S * fw[1] + A * rt[1]
        strip = np.asarray(photo.sample(X, Y)).astype(float)
        gray = strip.mean(-1)
        bg = ndimage.median_filter(gray, size=(9, 15))
        d = gray - bg
        d[strip.max(-1) - strip.min(-1) > 45] = 0     # paint is white; a taxi is not
        thr = max(14.0, 3.5 * float(np.median(np.abs(d - np.median(d)))))
        rows_s = np.arange(s1, s0, -RES)
        cands = []
        lane_lines = []
        for i, k in enumerate(ks):
            pts = np.array([to_sa(p) for p in cs[k][0]])
            order = np.argsort(pts[:, 0])
            a_of_s = np.interp(rows_s, pts[order, 0], pts[order, 1])
            lane_lines.append(pts)
            band = np.abs(A - a_of_s[:, None]) < min(0.7, widths[i] / 2 - 0.6)
            v = np.where(band, d, -1e9).max(axis=1)
            v[rows_s > -1.5] = -1e9
            hot = ndimage.binary_closing(v > thr, structure=np.ones(7))
            lab, n = ndimage.label(hot)
            for j in range(1, n + 1):
                idx = np.where(lab == j)[0]
                length = len(idx) * RES
                if 2.5 <= length <= 7.0:
                    cands.append({"lane": i, "from_m": round(-rows_s[idx[0]], 1), "to_m": round(-rows_s[idx[-1]], 1),
                                  "a": float(np.median(a_of_s[idx]))})

        # the sheet's top view, 2x
        scale = 2
        top = Image.fromarray(np.clip(strip, 0, 255).astype(np.uint8)).resize(
            (strip.shape[1] * scale, strip.shape[0] * scale), Image.BICUBIC)
        dr = ImageDraw.Draw(top, "RGBA")
        P = lambda s_, a_: ((a_ - a0) / RES * scale, (s1 - s_) / RES * scale)
        sl = [to_sa(L0), to_sa(R0)]
        dr.line([P(*sl[0]), P(*sl[1])], fill=(255, 40, 40, 255), width=5)
        for i, pts in enumerate(lane_lines):
            col = COLS[i % len(COLS)]
            keep = pts[(pts[:, 0] > s0) & (pts[:, 0] < 1)]
            if len(keep) > 1:
                dr.line([P(s_, a_) for s_, a_ in keep], fill=col + (150,), width=2)
        for c in cands:
            x0, y0 = P(-c["from_m"], c["a"] - 0.8)
            x1, y1 = P(-c["to_m"], c["a"] + 0.8)
            dr.rectangle([min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)],
                         outline=COLS[c["lane"] % len(COLS)] + (255,), width=3)
        mly_arrows = []
        for f in feats:
            s_, a_ = to_sa(f["xy"])
            if not (s0 < s_ < 3 and a0 < a_ < a1):
                continue
            dh = (f.get("aligned_direction", heading) - heading + 180) % 360 - 180
            if abs(dh) > 50:
                continue
            word = arrow_letters(f["object_value"])
            near = min(range(len(ks)), key=lambda i: abs(np.interp(s_, *lane_lines[i][np.argsort(lane_lines[i][:, 0])].T) - a_))
            mly_arrows.append({"lane": near, "m": round(-s_, 1), "arrow": word, "seen": f.get("last_seen_at", "")[:10]})
            x, y = P(s_, a_)
            dr.ellipse([x - 7, y - 7, x + 7, y + 7], fill=(255, 60, 200, 230), outline=(0, 0, 0, 255))
            dr.text((x + 10, y), word, fill=(255, 120, 230, 255), font=small, stroke_width=2,
                    stroke_fill=(0, 0, 0, 255), anchor="lm")
        for i, k in enumerate(ks):
            s_, a_ = to_sa(cs[k][0][-1])
            dr.text(P(3.5, a_), f"{i + 1}", fill=COLS[i % len(COLS)] + (255,), font=big, stroke_width=3,
                    stroke_fill=(0, 0, 0, 255), anchor="mm")
            dr.text(P(6.5, a_), moves_word(succ_moves[i][0]), fill=(255, 255, 255, 255), font=font,
                    stroke_width=3, stroke_fill=(0, 0, 0, 255), anchor="mm")
        for m in range(10, int(a.back) + 1, 10):
            dr.text(P(-m, a0 + 0.3), f"{m} m", fill=(255, 255, 255, 220), font=small, stroke_width=2,
                    stroke_fill=(0, 0, 0, 255), anchor="lm")

        # the street: images on the approach, facing it
        near = np.where(np.hypot(*(ixy - O).T) < a.back + 30)[0]
        pick = []
        for ii in near:
            im = images[ii]
            s_, a_ = to_sa(im["xy"])
            if not (-a.back - 10 < s_ < -6 and a0 - 1 < a_ < a1 + 1):
                continue
            pano = im.get("is_pano") or im.get("camera_type") == "spherical"
            dh = (im.get("computed_compass_angle", im.get("compass_angle", 0)) - heading + 180) % 360 - 180
            if not pano and abs(dh) > 30:
                continue
            pick.append((im, s_, a_, pano))
        # not from a viaduct above the approach: Mapillary's camera height is about 2 m on the
        # road (its median over the site) and 5-20 m on the 辛亥/基隆/建國 viaducts; a sequence's
        # own offset shifts it a little, so the approach's lower images set the bar too
        alts = sorted(t[0].get("computed_altitude") or 2.0 for t in pick)
        ref = min(2.5, alts[len(alts) // 5]) if alts else 2.0
        level = [t for t in pick if (t[0].get("computed_altitude") or 2.0) < min(3.8, ref + 2.5)]
        dropped = len(pick) - len(level)
        pick = level
        # the arrows read best from 10-35 m; then the newest
        pick.sort(key=lambda t: (not -35 < t[1] < -10, t[0]["captured_at"] < 1.7e12, -t[0]["captured_at"]))
        shots, seqs = [], set()
        for im, s_, a_, pano in pick:
            if len(shots) >= a.images:
                break
            if im["sequence"] in seqs:
                continue
            meta, img = mly.get(im["id"])
            if meta is None:
                continue
            seqs.add(im["sequence"])
            cam = Camera(dict(meta, camera_type="spherical" if pano else meta.get("camera_type", "perspective")),
                         im["xy"], img.size)
            sd = ImageDraw.Draw(img, "RGBA")
            lw = max(3, img.size[0] // 400)
            for i, k in enumerate(ks):
                pts = [p for p in cs[k][0] if s_ + 3 < to_sa(p)[0] < 0.5]
                uv = cam.project(pts)
                run = []
                for q in uv + [None]:
                    if q is not None and (not run or math.dist(q, run[-1]) < img.size[0] / 3):
                        run.append(q)
                    else:
                        if len(run) > 1:
                            sd.line(run, fill=COLS[i % len(COLS)] + (200,), width=lw)
                        run = [q] if q is not None else []
                lab = [p for p in cs[k][0] if to_sa(p)[0] > s_ + 9]
                if lab:
                    q = cam.project([lab[0]])[0]
                    if q:
                        sd.text(q, f"{i + 1}", fill=COLS[i % len(COLS)] + (255,), font=ImageFont.truetype(FONT, lw * 12),
                                stroke_width=lw, stroke_fill=(0, 0, 0, 255), anchor="mm")
            stop = cam.project([tuple(L0 + (R0 - L0) * t) for t in np.linspace(0, 1, 12)])
            stop = [q for q in stop if q]
            if len(stop) > 1:
                sd.line(stop, fill=(255, 40, 40, 230), width=lw + 2)
            if pano:
                c = cam.project([tuple(O)])[0]
                cx = c[0] if c else img.size[0] / 2
                half = img.size[0] * 60 / 360
                W = img.size[0]
                strip2 = Image.new("RGB", (int(2 * half), int(img.size[1] * 0.3)))
                for dx in (-W, 0, W):
                    strip2.paste(img, (int(half - cx + dx), int(-img.size[1] * 0.47)))
                img = strip2
            else:   # the road: from just above the horizon down (the arrows), less the bonnet
                img = img.crop((0, int(img.size[1] * 0.38), img.size[0], int(img.size[1] * 0.95)))
            when = time.strftime("%Y-%m-%d", time.gmtime(im["captured_at"] / 1000))
            shots.append((img, f"{when}  ~{-s_:.0f} m before the stop line (GPS)  {'360' if pano else meta.get('camera_type', '')}"
                          f"  mapillary {im['id']}", im["id"], when, round(-s_, 1)))

        # compose
        W_img = 1100
        rows = []
        for img, cap, *_ in shots:
            h = int(img.size[1] * W_img / img.size[0])
            rows.append((img.resize((W_img, h), Image.LANCZOS), cap))
        head = 120
        right_h = sum(r[0].size[1] + 30 for r in rows)
        H = head + max(top.size[1], right_h) + 10
        sheet = Image.new("RGB", (top.size[0] + 20 + (W_img if rows else 0) + 10, H), (24, 28, 26))
        sheet.paste(top, (0, head))
        y = head
        sd = ImageDraw.Draw(sheet)
        for img, cap in rows:
            sd.text((top.size[0] + 20, y + 2), cap, fill=(200, 210, 205), font=small)
            sheet.paste(img, (top.size[0] + 20, y + 26))
            y += img.size[1] + 30
        if not rows:
            sd.text((top.size[0] + 20, head + 10), "no Mapillary image on this approach", fill=(200, 210, 205), font=font)
        lanes_txt = "   ".join(f"{i + 1}: {moves_word(m)}" + (f" ({','.join(s)})" if s and s != [''] else "")
                               for i, (m, s) in enumerate(succ_moves))
        sd.text((10, 8), f"light {gid}   {name or '(unnamed)'}   heading {heading:.0f}°   stop line ({O[0]:.0f}, {O[1]:.0f})",
                fill=(255, 255, 255), font=font)
        sd.text((10, 40), f"map, lanes from the left: {lanes_txt}", fill=(255, 230, 120), font=font)
        sd.text((10, 72), f"boxes: bright paint on a lane's centre ({len(cands)})   pink dots: Mapillary arrows "
                          f"({len(mly_arrows)})   {dropped} images from a viaduct left out", fill=(190, 200, 195), font=small)
        sd.text((10, 92), "street images: lanes drawn from GPS, often 1-2 lanes off; count lanes from the left",
                fill=(190, 200, 195), font=small)
        sheet.save(a.out / f"{gid}.jpg", quality=82)
        report.append({"light": int(gid), "road": name, "heading": round(heading), "stop_line": [round(float(v), 1) for v in O],
                       "lanes": [{"lane": i + 1, "lanelet": int(k), "moves": m, "source": s}
                                 for i, (k, (m, s)) in enumerate(zip(ks, succ_moves))],
                       "candidates": [dict(c, lane=c["lane"] + 1, a=round(c["a"], 2)) for c in cands],
                       "mapillary_arrows": [dict(m, lane=m["lane"] + 1) for m in mly_arrows],
                       "images": [{"id": s[2], "date": s[3], "m": s[4]} for s in shots],
                       "viaduct_images_dropped": dropped})
        print(f"{gid} {name} {heading:.0f}: lanes {' | '.join(moves_word(m) for m, _ in succ_moves)}; "
              f"{len(cands)} candidates, {len(mly_arrows)} mapillary arrows, {len(shots)} images", flush=True)
    (a.out / "arrows.json").write_text(json.dumps(report, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
