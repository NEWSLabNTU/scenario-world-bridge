"""Lane widths and positions from a public aerial photo (roadmap 062, route C). Experimental:
off in generate.py unless SWB_IMAGERY=1. Tried 2026-10-31 on the NTU box: 98 of 188 major road
pieces fitted, but neighbouring pieces of one road disagreed (one 新生南路 way: widths 2.7-3.8 m,
shifts -1.3 to +1.4 m), and 059's survey sees the arterials' paint in only 4 windows of 20 m, too
few to tell a right fit from a wrong one. Not trusted until it is validated.

OSM gives lane counts and seldom widths (2 ways of the NTU box carry `width`); its centre lines
need not sit on the road's middle. NLSC's open orthophoto (WMTS PHOTO2, 0.27 m per pixel at
z19) shows painted lane lines on Taipei's arterials where trees do not shade them. For a road
piece, the brightness across the road is averaged in 40 m windows along it; a painted line
parallel to the road makes a peak there. The lane model, lines at

    two-way:  d - k w (forward, k = 0..nf),  d + k w (backward, k = 0..nb)
    one-way:  d + (n/2 - k) w                 (k = 0..n)

is fitted to the peaks: the width w (2.7-3.8 m) and the shift d (+-1.5 m) that put the most
lines on paint. A window counts when its best fit puts at least two lines, one of them between
lanes, within 0.35 m of paint and beats the default width's fit; the road piece takes the
median of its counted windows' w and d. A road piece with no counted window keeps its defaults.
"""
import json
import math

import numpy as np
from PIL import Image

STEP = 0.1          # m between profile samples across the road
WINDOW = 40.0       # m along the road per fit
W_RANGE = np.arange(2.70, 3.801, 0.05)
D_RANGE = np.arange(-1.5, 1.501, 0.05)
TOL = 0.35


class Photo:
    def __init__(self, raw, frame):
        meta = json.loads((raw / "nlsc_PHOTO2_z19.json").read_text())
        self.img = np.asarray(Image.open(raw / "nlsc_PHOTO2_z19.png").convert("L")).astype(np.float32)
        self.z, (self.tx0, self.ty0) = meta["zoom"], meta["tiles"][:2]
        self.frame = frame

    def sample(self, xs, ys):
        """Brightness at map-frame points, bilinear; NaN outside the mosaic."""
        out = np.full(len(xs), np.nan, np.float32)
        N = 2 ** self.z * 256
        for i, (x, y) in enumerate(zip(xs, ys)):
            lat, lon = self.frame.ll(x, y)
            px = (lon + 180) / 360 * N - self.tx0 * 256
            py = (1 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2 * N - self.ty0 * 256
            c, r = int(px), int(py)
            if 0 <= r < self.img.shape[0] - 1 and 0 <= c < self.img.shape[1] - 1:
                fx, fy = px - c, py - r
                a = self.img[r:r + 2, c:c + 2]
                out[i] = (a[0, 0] * (1 - fx) * (1 - fy) + a[0, 1] * fx * (1 - fy) +
                          a[1, 0] * (1 - fx) * fy + a[1, 1] * fx * fy)
        return out


def model(chain, w, d):
    if chain.oneway:
        return np.array([d + (chain.nf / 2 - k) * w for k in range(chain.nf + 1)])
    return np.array([d - k * w for k in range(chain.nf + 1)] + [d + k * w for k in range(1, chain.nb + 1)])


def peaks(prof, off):
    """Narrow bright bands: the profile less its 1.5 m running median (shade and kerbs vary
    slowly), peaks over 3 grey levels and 2.5 robust deviations of that residue. At 0.27 m per
    pixel a 15 cm line covers half a pixel and stands 5-10 levels over the asphalt."""
    p = np.array(prof, dtype=float)
    if np.isnan(p).all():
        return np.array([])
    p = np.where(np.isnan(p), np.nanmedian(p), p)
    k = int(round(0.75 / STEP))
    base = np.array([np.median(p[max(0, i - k):i + k + 1]) for i in range(len(p))])
    r = p - base
    mad = np.median(np.abs(r - np.median(r))) * 1.4826 + 1e-3
    thr = max(3.0, 2.5 * mad)
    out = []
    for i in range(2, len(r) - 2):
        if r[i] > thr and r[i] >= r[i - 1] and r[i] >= r[i + 1]:
            out.append(off[i])
    # one peak per band: drop those within 0.4 m of a stronger one
    out.sort(key=lambda x: -r[int(round((x - off[0]) / STEP))])
    keep = []
    for x in out:
        if all(abs(x - y) > 0.4 for y in keep):
            keep.append(x)
    return np.array(sorted(keep))


def score(lines, pk):
    if pk.size == 0:
        return 0.0, 0, 0
    d = np.abs(lines[:, None] - pk[None, :]).min(1)
    hit = d < TOL
    inner = hit[1:-1].sum() if len(lines) > 2 else 0
    return float(np.exp(-(d / 0.2) ** 2 / 2).sum()), int(hit.sum()), int(inner)


def fit(chain, photo):
    """(w, d, windows counted, windows) for a chain, or None."""
    line = chain.full
    total = chain.nf + chain.nb
    half = total * 3.8 / 2 + 2.0 if chain.oneway else max(chain.nf, chain.nb) * 3.8 + 2.0
    off = np.arange(-half, half + 1e-9, STEP)
    L = line.length
    fits, windows = [], 0
    s = 0.0
    while s + WINDOW / 2 <= L:
        e = min(L, s + WINDOW)
        acc, n = np.zeros(len(off)), np.zeros(len(off))
        for k in np.arange(s + 0.5, e, 1.0):
            a = line.interpolate(max(0.0, k - 0.5))
            b = line.interpolate(min(L, k + 0.5))
            h = math.atan2(b.y - a.y, b.x - a.x)
            p = line.interpolate(k)
            v = photo.sample(p.x - math.sin(h) * off, p.y + math.cos(h) * off)
            good = ~np.isnan(v)
            acc[good] += v[good]
            n[good] += 1
        s += WINDOW
        if n.max() == 0:
            continue
        windows += 1
        pk = peaks(np.where(n > 0, acc / np.maximum(n, 1), np.nan), off)
        if pk.size < 2:
            continue
        default = score(model(chain, chain.w, 0.0), pk)[0]
        best = None
        for w in W_RANGE:
            for d in D_RANGE:
                sc, hits, inner = score(model(chain, w, d), pk)
                if best is None or sc > best[0]:
                    best = (sc, hits, inner, w, d)
        sc, hits, inner, w, d = best
        if hits >= 2 and (inner >= 1 or total == 1) and sc > default + 0.3:
            fits.append((w, d))
    if not fits:
        return None
    ws, ds = zip(*fits)
    return float(np.median(ws)), float(np.median(ds)), len(fits), windows
