#!/usr/bin/env python3
"""The map frame of a Lanelet2 map, as Autoware and SSv2 compute it from lat/lon.

Both load the map with `lanelet::load` and a TransverseMercatorProjector built from the map's
`map_projector_info.yaml`: on the WGS84 ellipsoid, about the declared origin, scale factor
`scale_factor` or 0.9996 when absent (Autoware's map_loader; SSv2's `lanelet_loader.cpp`).
Neither reads the nodes' `local_x`/`local_y`: Autoware only under a `Local` projector, SSv2
never. So lat/lon is the position, and every tool that writes or derives one goes through
`Projector` here. On the CARLA towns this frame is CARLA's with Y flipped (Town01's driving
lanelets within 0.21 m of OpenDRIVE's lane centres, `lanelet_opendrive_registration.py`).

The pack's own `local_x`/`local_y` were a spherical Mercator on R = 6 378 137 m, 0.7 % long
in y against this frame (2.2 m on Town01's y = -330 road); `repair_lanelet_local_coords.py`
rewrites them from `Projector.project`.

The forward projection is Snyder's series (Map Projections, 8-9 and 8-10), within a
millimetre of pyproj's tmerc over a town; `unproject` inverts it by Newton's method, to a
micrometre.

    Projector.for_map("data/carla-autoware-bridge/Town01/lanelet2_map.osm").project(lat, lon)
"""

import math
import re
from dataclasses import dataclass
from pathlib import Path

WGS84_A = 6378137.0
WGS84_F = 1 / 298.257223563
DEFAULT_K0 = 0.9996  # Autoware's and SSv2's default when the yaml names no scale_factor

_E2 = WGS84_F * (2 - WGS84_F)
_EP2 = _E2 / (1 - _E2)


def _meridian_arc(lat):
    e2, e4, e6 = _E2, _E2 ** 2, _E2 ** 3
    return WGS84_A * ((1 - e2 / 4 - 3 * e4 / 64 - 5 * e6 / 256) * lat
                      - (3 * e2 / 8 + 3 * e4 / 32 + 45 * e6 / 1024) * math.sin(2 * lat)
                      + (15 * e4 / 256 + 45 * e6 / 1024) * math.sin(4 * lat)
                      - (35 * e6 / 3072) * math.sin(6 * lat))


@dataclass(frozen=True)
class Projector:
    lat0: float = 0.0  # origin, degrees
    lon0: float = 0.0
    k0: float = DEFAULT_K0

    @classmethod
    def for_map(cls, osm):
        """the projector declared by the `map_projector_info.yaml` beside `osm` (a map file
        or its directory); refuses a type other than TransverseMercator"""
        osm = Path(osm)
        yaml = (osm if osm.is_dir() else osm.parent) / "map_projector_info.yaml"
        if not yaml.exists():
            raise SystemExit(f"{yaml}: missing; it declares the map's projection")
        text = yaml.read_text()

        def value(key):
            m = re.search(r"^\s*%s:\s*['\"]?([^'\"\s#]+)" % key, text, re.M)
            return m.group(1) if m else None

        kind = value("projector_type")
        if kind != "TransverseMercator":
            raise SystemExit(f"{yaml}: projector_type {kind!r}; only TransverseMercator is "
                             "handled (download_maps.sh rewrites the packs to it)")
        k0 = value("scale_factor")
        return cls(float(value("latitude") or 0.0), float(value("longitude") or 0.0),
                   float(k0) if k0 else DEFAULT_K0)

    def project(self, lat_deg, lon_deg):
        """(x east, y north) metres in the map frame"""
        lat = math.radians(lat_deg)
        s, c = math.sin(lat), math.cos(lat)
        n = WGS84_A / math.sqrt(1 - _E2 * s * s)
        t = (s / c) ** 2
        cc = _EP2 * c * c
        a = math.radians(lon_deg - self.lon0) * c
        x = self.k0 * n * (a + (1 - t + cc) * a ** 3 / 6
                           + (5 - 18 * t + t * t + 72 * cc - 58 * _EP2) * a ** 5 / 120)
        y = self.k0 * (_meridian_arc(lat) - _meridian_arc(math.radians(self.lat0))
                       + n * (s / c) * (a * a / 2 + (5 - t + 9 * cc + 4 * cc * cc) * a ** 4 / 24
                                        + (61 - 58 * t + t * t + 600 * cc - 330 * _EP2)
                                        * a ** 6 / 720))
        return x, y

    def unproject(self, x, y):
        """(lat, lon) degrees of a map-frame point; the inverse of `project`"""
        # metres per degree near the origin, to start and to step
        lat, lon = self.lat0 + y / 111000.0, self.lon0 + x / 111000.0
        h = 1e-7
        for _ in range(20):
            px, py = self.project(lat, lon)
            ex, ey = x - px, y - py
            if math.hypot(ex, ey) < 1e-7:
                break
            ax, ay = self.project(lat + h, lon)
            bx, by = self.project(lat, lon + h)
            j11, j12 = (ax - px) / h, (bx - px) / h  # d x / d lat, d x / d lon
            j21, j22 = (ay - py) / h, (by - py) / h
            det = j11 * j22 - j12 * j21
            lat += (j22 * ex - j12 * ey) / det
            lon += (j11 * ey - j21 * ex) / det
        return lat, lon
