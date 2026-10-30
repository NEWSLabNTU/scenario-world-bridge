#!/usr/bin/env python3
"""Audit a generated map's lane lines against the paint in 059's 2 cm orthophotos (roadmap 062,
O4). The orthophotos are ground truth and never an input to the map.

    scripts/open_maps/audit_lines.py <lanelet2_map.osm> <ortho dir> [--json out.json]
        [--shift dx,dy] [--vendor <vendor map>] [--step 1.0] [--half 2.0]

Every lane bound (line_thin: solid or dashed) is sampled every `step` m. At each sample, the
intensity image is read across the line, `half` m each way, where the mask says the ground was
observed (per 20 cm block: the mask is sparse at 2 cm); the cross-sections are averaged over 8 m windows. Paint is where intensity stands out from the road: more than 4 robust deviations over
the profile's median, at least 6 cm wide. The offset is from the line to the nearest paint
stripe's centre. Reported per kind of line: how many samples were observed, how many found paint
within `half` m, and the offsets' median, 90th percentile and signed mean (a systematic shift
shows in the mean).

`--vendor` runs the same on the vendor's lanelets (moved from 059's map frame to the orthos'
by +(-0.54, +0.20)), as a floor: their own drift is up to 0.3 m (019).
"""
import argparse
import json
import math
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

import numpy as np
from PIL import Image

TILE, PX, N = 50.0, 0.02, 2500
VENDOR_SHIFT = (-0.54, 0.20)


class Ortho:
    def __init__(self, d):
        self.d = d
        self.cache = {}

    def tile(self, ix, iy):
        k = (ix, iy)
        if k not in self.cache:
            fi, fm = self.d / f"int_{ix}_{iy}.png", self.d / f"mask_{ix}_{iy}.png"
            if fi.exists() and fm.exists():
                # the mask marks pixels holding a real ground point, 19 % of an open road's
                # pixels at 2 cm: ground counts as observed per 20 cm block
                m = np.asarray(Image.open(fm)) > 127
                m = m.reshape(N // 10, 10, N // 10, 10).any((1, 3)).repeat(10, 0).repeat(10, 1)
                self.cache[k] = (np.asarray(Image.open(fi)).astype(np.float32), m)
            else:
                self.cache[k] = None
            if len(self.cache) > 40:
                self.cache.pop(next(iter(self.cache)))
        return self.cache[k]

    def sample(self, xs, ys):
        """intensity and observed flag at points (V frame)."""
        v = np.full(len(xs), np.nan, np.float32)
        ok = np.zeros(len(xs), bool)
        ix = np.floor(xs / TILE).astype(int)
        iy = np.floor(ys / TILE).astype(int)
        for key in set(zip(ix.tolist(), iy.tolist())):
            t = self.tile(*key)
            if t is None:
                continue
            sel = (ix == key[0]) & (iy == key[1])
            c = ((xs[sel] - key[0] * TILE) / PX).astype(int).clip(0, N - 1)
            r = ((key[1] * TILE + TILE - ys[sel]) / PX).astype(int).clip(0, N - 1)
            v[sel] = t[0][r, c]
            ok[sel] = t[1][r, c]
        return v, ok


def tags(e):
    return {t.get("k"): t.get("v") for t in e.findall("tag")}


def lines(path, shift):
    r = ET.parse(path).getroot()
    xy = {}
    for n in r.findall("node"):
        t = tags(n)
        if "local_x" in t:
            xy[n.get("id")] = (float(t["local_x"]) + shift[0], float(t["local_y"]) + shift[1])
    used = set()
    for rel in r.findall("relation"):
        if tags(rel).get("type") == "lanelet" and tags(rel).get("subtype", "road") == "road":
            for m in rel.findall("member"):
                if m.get("role") in ("left", "right"):
                    used.add(m.get("ref"))
    seen = set()
    for w in r.findall("way"):
        if w.get("id") not in used:
            continue
        t = tags(w)
        if t.get("type") != "line_thin":
            continue
        ids = [nd.get("ref") for nd in w.findall("nd")]
        key = frozenset((ids[0], ids[-1]))     # a two-way road's centre line is there twice
        if key in seen:
            continue
        seen.add(key)
        pts = [xy[i] for i in ids if i in xy]
        if len(pts) >= 2:
            yield pts, t.get("subtype", "solid")


def paint_offset(prof, ok, off):
    """Offset of the paint stripe nearest 0 in a profile, or None."""
    if ok.sum() < 0.6 * len(ok):
        return "unobserved"
    v = prof[ok]
    med = np.median(v)
    mad = np.median(np.abs(v - med)) * 1.4826 + 1e-3
    hot = ok & (prof > med + 4 * mad)
    best = None
    i = 0
    while i < len(hot):
        if hot[i]:
            j = i
            while j < len(hot) and hot[j]:
                j += 1
            if j - i >= 3:                       # >= 6 cm wide
                c = (off[i] + off[j - 1]) / 2
                if best is None or abs(c) < abs(best):
                    best = c
            i = j
        else:
            i += 1
    return best


def audit(path, ortho, shift, step, half, window=8):
    """Profiles are averaged over `window` consecutive samples (8 m at 1 m) before the paint is
    looked for: a stripe parallel to the line adds up, a stray bright spot washes out."""
    res = defaultdict(lambda: {"windows": 0, "observed": 0, "found": [], "lines": 0})
    off = np.arange(-half, half + 1e-9, PX)
    for pts, kind in lines(path, shift):
        R = res[kind]
        R["lines"] += 1
        profs, oks = [], []

        def flush():
            if len(profs) < window // 2:
                profs.clear()
                oks.clear()
                return
            P, O = np.array(profs), np.array(oks)
            R["windows"] += 1
            obs = O.mean(0)
            if (obs > 0.5).mean() >= 0.6:
                R["observed"] += 1
                Pm = np.where(O, P, np.nan)
                with np.errstate(all="ignore"):
                    avg = np.nanmean(Pm, 0)
                good = ~np.isnan(avg)
                o = paint_offset(np.nan_to_num(avg), good, off)
                if o not in (None, "unobserved"):
                    R["found"].append(o)
            profs.clear()
            oks.clear()
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            L = math.hypot(x1 - x0, y1 - y0)
            if L < 1e-6:
                continue
            ux, uy = (x1 - x0) / L, (y1 - y0) / L
            nx, ny = -uy, ux                      # left normal
            for k in range(int(L / step)):
                sx = (k + 0.5) * step
                px, py = x0 + ux * sx, y0 + uy * sx
                v, ok = ortho.sample(px + nx * off, py + ny * off)
                if np.isnan(v).all():
                    flush()
                    continue
                profs.append(np.nan_to_num(v))
                oks.append(ok)
                if len(profs) >= window:
                    flush()
        flush()
    out = {}
    for kind, R in res.items():
        f = np.array(R["found"])
        out[kind] = {
            "lines": R["lines"], "windows_in_survey": R["windows"], "observed": R["observed"],
            "paint_found": len(f),
            "paint_found_share": round(len(f) / max(R["observed"], 1), 3),
            "abs_offset_median_m": round(float(np.median(np.abs(f))), 3) if len(f) else None,
            "abs_offset_p90_m": round(float(np.percentile(np.abs(f), 90)), 3) if len(f) else None,
            "within_0.3m_share": round(float((np.abs(f) < 0.3).mean()), 3) if len(f) else None,
            "signed_mean_m": round(float(f.mean()), 3) if len(f) else None,
        }
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("map")
    ap.add_argument("ortho", type=Path)
    ap.add_argument("--json")
    ap.add_argument("--shift", default="0,0")
    ap.add_argument("--vendor")
    ap.add_argument("--step", type=float, default=1.0)
    ap.add_argument("--half", type=float, default=2.0)
    a = ap.parse_args()
    ortho = Ortho(a.ortho)
    rep = {"generated": audit(a.map, ortho, tuple(map(float, a.shift.split(","))), a.step, a.half)}
    if a.vendor:
        rep["vendor"] = audit(a.vendor, ortho, VENDOR_SHIFT, a.step, a.half)
    print(json.dumps(rep, indent=1))
    if a.json:
        Path(a.json).write_text(json.dumps(rep, indent=1))


if __name__ == "__main__":
    main()
