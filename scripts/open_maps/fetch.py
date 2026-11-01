#!/usr/bin/env python3
"""Fetch a site's public map data into <out>/raw/ (roadmap 062, O1).

    scripts/open_maps/fetch.py <site.yaml> <out> [--only osm,overture,mapillary] [--force]

Sources, each cached (a rerun skips what is there unless --force):

- osm: every way, node and relation in the box with all its tags, by Overpass
  (`osm.json`; ODbL).
- overture: the transportation (segment, connector) and buildings themes for the box from
  Overture's latest release, by DuckDB over its public S3 bucket (`overture_<type>.parquet`).
- mapillary: map features (traffic signs, painted markings, traffic lights, poles) and image metadata (pose, time, camera), tiled because the
  API caps a request's area and count (`mapillary_features.json`, `mapillary_images.json`;
  CC-BY-SA). Needs MAPILLARY_TOKEN, a host setting in `.env`.
- nlsc: NLSC's orthophoto (WMTS layer PHOTO2) at z19 as one mosaic with its bounds
  (`nlsc_PHOTO2_z19.png` and `.json`); NLSC's terms, attribution 國土測繪中心.
- taipei: Taipei City's open data (data.taipei; 政府資料開放授權條款 v1): the road pavement
  blocks curb to curb (`taipei_pavement.geojson`, 新工處), the curb parking spaces
  (`taipei_parking.geojson`, 停管處), the 34 arterials' traffic signs (`taipei_signs.json`,
  交工處), and the signals' timing plans and day schedules (`taipei_signals.json`, 交通局) with
  each signal's 三合一報表 (`taipei_phases/<device>.pdf`: its phase diagrams, as of 2024-03).
  Only for a box in Taipei City.

The site file names the box: `bbox: [south, west, north, east]` (degrees, WGS84).

Run with the tool venv (`data/maps/.venv`: duckdb, requests, pyyaml, pillow, pyproj, pyshp).
"""
import argparse
import json
import os
import sys
import time
from pathlib import Path

import requests
import yaml

UA = {"User-Agent": "swb-open-maps/0.1 (research; NEWSLabNTU)"}
OVERPASS = "https://overpass-api.de/api/interpreter"
OVERTURE_BUCKET = "overturemaps-us-west-2"
MLY = "https://graph.mapillary.com"
FEATURE_FIELDS = "id,object_value,object_type,geometry,aligned_direction,first_seen_at,last_seen_at"
IMAGE_FIELDS = ("id,captured_at,compass_angle,computed_compass_angle,geometry,computed_geometry,"
                "computed_altitude,sequence,camera_type,make,model,is_pano,width,height")


def fetch_osm(bbox, out):
    s, w, n, e = bbox
    q = f"[out:json][timeout:300];(nwr({s},{w},{n},{e}););(._;>;);out body;"
    r = requests.post(OVERPASS, data={"data": q}, headers=UA, timeout=400)
    r.raise_for_status()
    d = r.json()
    (out / "osm.json").write_text(json.dumps(d))
    kinds = {}
    for el in d["elements"]:
        kinds[el["type"]] = kinds.get(el["type"], 0) + 1
    return f"{kinds}, base {d['osm3s']['timestamp_osm_base']}"


def overture_release():
    url = f"https://{OVERTURE_BUCKET}.s3.amazonaws.com/?list-type=2&prefix=release/&delimiter=/"
    r = requests.get(url, headers=UA, timeout=60)
    r.raise_for_status()
    import re
    rel = sorted(re.findall(r"<Prefix>release/([^/<]+)/</Prefix>", r.text))
    return rel[-1]


def fetch_overture(bbox, out):
    import duckdb
    s, w, n, e = bbox
    rel = overture_release()
    con = duckdb.connect()
    con.execute("INSTALL spatial; LOAD spatial; INSTALL httpfs; LOAD httpfs; SET s3_region='us-west-2';")
    counts = {}
    for theme, typ in (("transportation", "segment"), ("transportation", "connector"),
                       ("buildings", "building")):
        path = f"s3://{OVERTURE_BUCKET}/release/{rel}/theme={theme}/type={typ}/*"
        dest = out / f"overture_{typ}.parquet"
        con.execute(f"""
            COPY (SELECT * FROM read_parquet('{path}', hive_partitioning=1)
                  WHERE bbox.xmin <= {e} AND bbox.xmax >= {w}
                    AND bbox.ymin <= {n} AND bbox.ymax >= {s})
            TO '{dest}' (FORMAT PARQUET)""")
        counts[typ] = con.execute(f"SELECT count(*) FROM '{dest}'").fetchone()[0]
    (out / "overture_release.txt").write_text(rel + "\n")
    return f"release {rel}: {counts}"


def tiles(bbox, step):
    s, w, n, e = bbox
    y = s
    while y < n:
        x = w
        while x < e:
            yield (x, y, min(x + step, e), min(y + step, n))
            x += step
        y += step


def mly_get(path, params, token):
    params = dict(params, access_token=token)
    for attempt in range(5):
        r = requests.get(f"{MLY}/{path}", params=params, headers=UA, timeout=120)
        if r.status_code == 200:
            return r.json()
        if r.status_code in (429, 500, 502, 503):
            time.sleep(5 * (attempt + 1))
            continue
        raise RuntimeError(f"{path}: HTTP {r.status_code}: {r.text[:300]}")
    raise RuntimeError(f"{path} {params.get('bbox')} {params.get('layers')}: gave up after retries (last HTTP {r.status_code}: {r.text[:200]})")


def mly_tiled(path, bbox, step, params, token, cap):
    """Every item in the box, splitting a tile whose answer hits the API's cap."""
    found = {}
    todo = list(tiles(bbox, step))
    while todo:
        x0, y0, x1, y1 = todo.pop()
        d = mly_get(path, dict(params, bbox=f"{x0},{y0},{x1},{y1}", limit=cap), token)
        items = d.get("data", [])
        if len(items) >= cap and (x1 - x0) > 1e-4:
            mx, my = (x0 + x1) / 2, (y0 + y1) / 2
            todo += [(x0, y0, mx, my), (mx, y0, x1, my), (x0, my, mx, y1), (mx, my, x1, y1)]
            continue
        for it in items:
            found[it["id"]] = it
    return list(found.values())


def fetch_mapillary(bbox, out):
    token = os.environ.get("MAPILLARY_TOKEN")
    if not token:
        raise RuntimeError("MAPILLARY_TOKEN is not set (a host setting, in .env)")
    box = bbox  # tiles() takes [south, west, north, east] and yields (west, south, east, north)
    # `layers` does not filter (2026-10-30: both layers returned the same 17 196 features), so
    # one query takes signs, markings, lights and poles together
    feats = mly_tiled("map_features", box, 0.004, {"fields": FEATURE_FIELDS}, token, 2000)
    (out / "mapillary_features.json").write_text(json.dumps(feats))
    imgs = mly_tiled("images", box, 0.002, {"fields": IMAGE_FIELDS}, token, 2000)
    (out / "mapillary_images.json").write_text(json.dumps(imgs))
    seqs = {i.get("sequence") for i in imgs}
    return f"{len(feats)} map features, {len(imgs)} images in {len(seqs)} sequences"


NLSC_WMTS = "https://wmts.nlsc.gov.tw/wmts/{layer}/default/GoogleMapsCompatible/{z}/{y}/{x}"


def fetch_nlsc(bbox, out, layer="PHOTO2", z=19):
    """NLSC's orthophoto as one mosaic. z19 is its deepest real level (0.27 m per pixel at NTU,
    2026-10-30; z20 is z19 upsampled); tiles carry an "NLSC <year>" watermark."""
    import math
    from io import BytesIO
    from PIL import Image
    s, w, n, e = bbox
    N = 2 ** z

    def tx(lon):
        return (lon + 180) / 360 * N

    def ty(lat):
        return (1 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2 * N
    x0, x1, y0, y1 = int(tx(w)), int(tx(e)), int(ty(n)), int(ty(s))
    img = Image.new("RGB", ((x1 - x0 + 1) * 256, (y1 - y0 + 1) * 256))
    sess = requests.Session()
    for y in range(y0, y1 + 1):
        for x in range(x0, x1 + 1):
            r = sess.get(NLSC_WMTS.format(layer=layer, z=z, y=y, x=x), headers=UA, timeout=60)
            r.raise_for_status()
            if r.headers.get("content-type", "").startswith("image/jpeg"):
                img.paste(Image.open(BytesIO(r.content)).convert("RGB"), ((x - x0) * 256, (y - y0) * 256))
    img.save(out / f"nlsc_{layer}_z{z}.png")

    def lon(x):
        return x / N * 360 - 180

    def lat(y):
        return math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * y / N))))
    meta = {"layer": layer, "zoom": z, "crs": "EPSG:3857 tiles (GoogleMapsCompatible)",
            "tiles": [x0, y0, x1, y1], "west": lon(x0), "east": lon(x1 + 1),
            "north": lat(y0), "south": lat(y1 + 1), "size": list(img.size)}
    (out / f"nlsc_{layer}_z{z}.json").write_text(json.dumps(meta, indent=1))
    return f"{layer} z{z}: {img.size[0]} x {img.size[1]} px, {(x1 - x0 + 1) * (y1 - y0 + 1)} tiles"


TAIPEI = "https://data.taipei/api/dataset/{}/resource/{}/download"
TAIPEI_SETS = {     # data.taipei (dataset, resource); each is on data.gov.tw too
    "pavement": ("98b1b6a8-a092-423c-aab0-7dcb981d2a58", "1eb0cd43-3088-4d91-bbf3-f120469a955e"),   # 臺北市道路銑鋪區塊圖資, 新工處
    "parking": ("5a911ea5-1694-4301-808e-e1780d971611", "7e2f32a0-9201-4666-a0ed-11034e6c2b66"),    # 臺北市路邊停車格位, 停管處
    "signs": ("45fde530-6bd0-425b-98bc-de82c635f7ad", "c2cce902-c2bc-4301-baf5-52e538035d8b"),      # 臺北市標誌資料 (34 條主次要幹道), 交工處
    "signals": ("0d639f73-cbcc-42c3-aa53-20efac199701", "9522d9e8-131e-4890-9379-f17c523238a0"),    # 臺北市路口號誌時制計畫: the devices
}
TAIPEI_PLANS = "https://tcgbusfs.blob.core.windows.net/dotapp/timing_plan.json"           # the plans
TAIPEI_TABLE = "https://tcgbusfs.blob.core.windows.net/dotapp/timing_plan_table.json"     # the day's schedule
TAIPEI_PHASES = "https://www.ttcx.dot.gov.taipei/cpt/api/TimingPlan/pdf/{}"               # 三合一報表


def fetch_taipei(bbox, out, margin=0.001):
    """Taipei City's open data for the box (政府資料開放授權條款 v1; attribution to each agency):
    the road pavement blocks (curb to curb, per direction), the curb parking spaces, the arterials'
    traffic signs, and the signals' timing plans with each one's 三合一報表 (its phase diagrams)."""
    import csv
    import io
    import tempfile
    import zipfile
    import shapefile
    from pyproj import Transformer
    s, w, n, e = bbox
    s, w, n, e = s - margin, w - margin, n + margin, e + margin
    to97 = Transformer.from_crs(4326, 3826, always_xy=True)
    to84 = Transformer.from_crs(3826, 4326, always_xy=True)
    x0, y0 = to97.transform(w, s)
    x1, y1 = to97.transform(e, n)

    def get(url, timeout=600):
        r = requests.get(url, headers=UA, timeout=timeout)
        r.raise_for_status()
        return r.content

    def shapes(key, keep):
        """The data set's shapes touching the box, as GeoJSON in WGS84 with the fields named."""
        with tempfile.TemporaryDirectory() as tmp:
            zipfile.ZipFile(io.BytesIO(get(TAIPEI.format(*TAIPEI_SETS[key])))).extractall(tmp)
            shp = next(Path(tmp).rglob("*.shp"))
            feats = []
            with shapefile.Reader(str(shp), encoding="utf-8") as r:
                for sr in r.iterShapeRecords(bbox=(x0, y0, x1, y1)):
                    pts = sr.shape.points
                    if not pts:
                        continue
                    parts = list(sr.shape.parts) + [len(pts)]
                    rings = [[list(to84.transform(*p)) for p in pts[a:b]] for a, b in zip(parts, parts[1:])]
                    rec = sr.record.as_dict()
                    feats.append({"type": "Feature", "geometry": {"type": "Polygon", "coordinates": rings},
                                  "properties": {k: rec.get(k) for k in keep}})
        (out / f"taipei_{key}.geojson").write_text(json.dumps({"type": "FeatureCollection", "features": feats},
                                                              ensure_ascii=False))
        return len(feats)

    def rows(key):
        return list(csv.DictReader(io.StringIO(get(TAIPEI.format(*TAIPEI_SETS[key])).decode("utf-8-sig"))))

    def inside(lat, lon):
        try:
            return s <= float(lat) <= n and w <= float(lon) <= e
        except ValueError:
            return False
    counts = {"pavement": shapes("pavement", ["ROAD", "STA_ROAD", "END_ROAD", "WIDTH", "MAP_AREA", "NFCL_CODE"]),
              "parking": shapes("parking", ["pktype", "pktype1", "pkkind", "width", "len", "roadname", "pktime", "pknos"])}
    signs = [r for r in rows("signs") if inside(r["緯度座標Y-WGS84"], r["經度座標X-WGS84"])]
    (out / "taipei_signs.json").write_text(json.dumps(signs, ensure_ascii=False, indent=0))
    counts["signs"] = len(signs)
    devices = {r["設備編號"]: {"icid": r["icid"], "device": r["設備編號"], "name": " ".join(r["路口名稱"].split()),
                               "lat": float(r["緯度"]), "lon": float(r["經度"]), "plans": {}, "schedule": {}}
               for r in rows("signals") if inside(r["緯度"], r["經度"])}
    for p in json.loads(get(TAIPEI_PLANS)):
        if p["deviceid"] in devices:
            devices[p["deviceid"]]["plans"][str(int(p["planid（SeqNo）"]))] = {
                "phase_order": p["phaseorder"], "cycle": p["cycletime"], "offset": p["offset"],
                "phases": [{k: x[k] for k in ("green", "yellow", "allred", "mingreen", "maxgreen", "pedgreenflash", "pedred")}
                           for x in p["subplan"]],
                "info_time": p.get("InfoTime")}
    for t in json.loads(get(TAIPEI_TABLE)):
        if t["deviceid"] in devices:   # segmenttype: 1-7 Monday to Sunday (the 三合一報表's 一..日)
            devices[t["deviceid"]]["schedule"][str(t["segmenttype"])] = [
                [x["time"], str(int(x["planid（SeqNo）"]))] for x in t["subsegment"]]
    (out / "taipei_phases").mkdir(exist_ok=True)
    for d in devices:
        pdf = out / "taipei_phases" / f"{d}.pdf"
        if not pdf.exists():
            try:
                pdf.write_bytes(get(TAIPEI_PHASES.format(d), timeout=60))
            except requests.RequestException as ex:
                print(f"taipei: {d}'s 三合一報表: {ex}", file=sys.stderr)
    (out / "taipei_signals.json").write_text(json.dumps(list(devices.values()), ensure_ascii=False, indent=0))
    counts["signals"] = len(devices)
    return ", ".join(f"{v} {k}" for k, v in counts.items())


SOURCES = {
    "osm": (fetch_osm, "osm.json"),
    "overture": (fetch_overture, "overture_segment.parquet"),
    "mapillary": (fetch_mapillary, "mapillary_images.json"),
    "nlsc": (fetch_nlsc, "nlsc_PHOTO2_z19.png"),
    "taipei": (fetch_taipei, "taipei_signals.json"),
}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("site", type=Path)
    ap.add_argument("out", type=Path)
    ap.add_argument("--only", default=",".join(SOURCES))
    ap.add_argument("--force", action="store_true")
    a = ap.parse_args()
    site = yaml.safe_load(a.site.read_text())
    raw = a.out / "raw"
    raw.mkdir(parents=True, exist_ok=True)
    failed = False
    for name in a.only.split(","):
        fn, marker = SOURCES[name]
        if (raw / marker).exists() and not a.force:
            print(f"{name}: cached")
            continue
        t0 = time.time()
        try:
            print(f"{name}: {fn(site['bbox'], raw)} ({time.time() - t0:.0f} s)")
        except Exception as ex:  # one source failing leaves the others
            failed = True
            print(f"{name}: FAILED: {ex}", file=sys.stderr)
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
