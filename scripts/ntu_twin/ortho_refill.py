#!/usr/bin/env python3
"""The NTU twin's ground texture without blocks (roadmap 059, rendering): refill the
orthophotos `ground_ortho.py` pass 3 wrote, from their own covered pixels.

At 2 cm a pixel holds a scan point in 9-57 % of the route's tiles (fewer under trees and away
from the vendor's track). Pass 3 filled the rest from successively coarser block averages, so
a sparse area renders as flat squares 0.1-1 m wide. Here every pixel is a Gaussian-weighted
mean of the covered pixels around it (normalized convolution), at the finest scale whose
kernel holds about `SUPPORT` points: sharp where the scan is dense (road markings stay as
they are), smooth where it is sparse, and blended between neighbouring scales so no edge
shows. The eight neighbouring tiles are padded in, so tile borders match.

Reads `ORTHO_DIR/rgb_<ix>_<iy>.jpg` and `mask_<ix>_<iy>.png` (the coverage: a covered pixel's
RGB is the scan's own mean there); writes `OUT_DIR/rgb_<ix>_<iy>.jpg`. Nothing else changes:
the world's tiles name their texture by file, so copying these over a world's `ortho/` (or
`world_usd.py ... OUT_DIR`) is the whole update.

Usage:
  ortho_refill.py ORTHO_DIR OUT_DIR [--tiles ix_iy,...] [--workers 6]
"""
import argparse
import os
import re
import time
from multiprocessing import Pool

import cv2
import numpy as np

SIGMAS = (0.75, 1.5, 3, 6, 12, 24, 48, 96, 192)  # px (2 cm): the scales, finest first
SUPPORT = 4.0   # points under a kernel (within ~1 sigma) for that scale to be used alone
PAD = 512       # px of each neighbour padded in (> 2.5 x the largest sigma that matters)
FULL = 24       # px: larger sigmas are blurred on a downsampled image


def tile_ids(d):
    return sorted({m.groups() for f in os.listdir(d)
                   if (m := re.match(r"rgb_(-?\d+)_(-?\d+)\.jpg$", f))}, key=lambda t: (int(t[0]), int(t[1])))


def load(d, ix, iy):
    rgb, msk = os.path.join(d, f"rgb_{ix}_{iy}.jpg"), os.path.join(d, f"mask_{ix}_{iy}.png")
    if not (os.path.exists(rgb) and os.path.exists(msk)):
        return None, None
    im, mk = cv2.imread(rgb), cv2.imread(msk, cv2.IMREAD_GRAYSCALE)
    assert im is not None and mk is not None, (rgb, msk)
    return im[..., ::-1].astype(np.float32), mk > 127


def padded(d, ix, iy):
    """The tile with PAD px of its neighbours around it (row 0 is north: iy + 1 is above)."""
    img, m = load(d, ix, iy)
    assert img is not None and m is not None
    n = img.shape[0]
    big = np.zeros((n + 2 * PAD, n + 2 * PAD, 3), np.float32)
    bm = np.zeros((n + 2 * PAD, n + 2 * PAD), bool)
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            im2, m2 = (img, m) if dx == dy == 0 else load(d, int(ix) + dx, int(iy) + dy)
            if im2 is None or m2 is None:
                continue
            # the neighbour's rows/cols that fall inside the padded canvas
            r0 = PAD - dy * n   # canvas row of the neighbour's row 0
            c0 = PAD + dx * n
            rs, re_ = max(r0, 0), min(r0 + n, n + 2 * PAD)
            cs, ce = max(c0, 0), min(c0 + n, n + 2 * PAD)
            big[rs:re_, cs:ce] = im2[rs - r0:re_ - r0, cs - c0:ce - c0]
            bm[rs:re_, cs:ce] = m2[rs - r0:re_ - r0, cs - c0:ce - c0]
    return big, bm, n


def blur(a, sigma):
    """Gaussian blur; large sigmas on an image downsampled to keep the kernel ~FULL px."""
    if sigma <= FULL:
        return cv2.GaussianBlur(a, (0, 0), sigma, borderType=cv2.BORDER_CONSTANT)
    f = int(2 ** np.floor(np.log2(sigma / FULL * 2)))
    h, w = a.shape[:2]
    small = cv2.resize(a, (w // f, h // f), interpolation=cv2.INTER_AREA)
    small = cv2.GaussianBlur(small, (0, 0), sigma / f, borderType=cv2.BORDER_CONSTANT)
    return cv2.resize(small, (w, h), interpolation=cv2.INTER_LINEAR)


def refill(img, mask):
    """Normalized convolution at the finest scale with SUPPORT points, coarse to fine blend."""
    m = mask.astype(np.float32)
    num = img * m[..., None]
    out = None
    for s in reversed(SIGMAS):
        w = blur(m, s)
        a = blur(num, s) / np.maximum(w, 1e-6)[..., None]
        # points under the kernel: its weight density times its area (2 pi sigma^2)
        alpha = np.clip(w * (2 * np.pi * s * s) / SUPPORT, 0.0, 1.0)[..., None]
        if out is None:
            mean = img[mask].mean(0) if mask.any() else np.full(3, 90.0, np.float32)
            out = np.where(w[..., None] > 1e-6, a, mean)
        else:
            out = alpha * a + (1 - alpha) * out
    return out


def _one(args):
    src, dst, ix, iy = args
    big, bm, n = padded(src, ix, iy)
    out = refill(big, bm)[PAD:PAD + n, PAD:PAD + n]
    cv2.imwrite(os.path.join(dst, f"rgb_{ix}_{iy}.jpg"),
                np.clip(out, 0, 255).astype(np.uint8)[..., ::-1], [cv2.IMWRITE_JPEG_QUALITY, 92])
    return ix, iy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ortho_dir")
    ap.add_argument("out_dir")
    ap.add_argument("--tiles", default="")
    ap.add_argument("--workers", type=int, default=6)
    a = ap.parse_args()
    os.makedirs(a.out_dir, exist_ok=True)
    ids = tile_ids(a.ortho_dir)
    if a.tiles:
        want = set(a.tiles.split(","))
        ids = [t for t in ids if f"{t[0]}_{t[1]}" in want]
    t0 = time.time()
    with Pool(a.workers) as pool:
        for k, (ix, iy) in enumerate(pool.imap_unordered(
                _one, [(a.ortho_dir, a.out_dir, ix, iy) for ix, iy in ids])):
            print(f"{k + 1}/{len(ids)} {ix}_{iy} {time.time() - t0:.0f}s", flush=True)


if __name__ == "__main__":
    main()
