"""build_meshes.poisson on a cloud whose points sit on an exact lattice (a voxel-downsampled
map, as COSS's): Qhull, inside the normals' orientation, refused it as cocircular."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import build_meshes  # noqa: E402


def test_poisson_takes_a_lattice():
    g = np.arange(0, 3.0, 0.03125)                    # 96 x 96, on 1/32 m
    xx, yy = np.meshgrid(g, g)
    P = np.c_[xx.ravel(), yy.ravel(), 0.5 + 0.0 * xx.ravel()]
    P = np.vstack([P, P + [0, 0, 0.5]])               # two parallel sheets, a box's top and bottom
    C = np.full((len(P), 3), 0.5)
    build_meshes.poisson(P, C)                        # raised QH6239 before


def test_poisson_takes_an_exactly_planar_sheet():
    # COSS tile 0_1: 3072 points all at one y, a vertical sheet 50 m x 19 m at the map's edge
    x = np.arange(0, 50.0, 0.125)
    z = np.arange(0, 19.0, 0.125)
    xx, zz = np.meshgrid(x, z)
    P = np.c_[xx.ravel(), np.full(xx.size, 52.0), zz.ravel()]
    C = np.full((len(P), 3), 0.5)
    build_meshes.poisson(P, C)
