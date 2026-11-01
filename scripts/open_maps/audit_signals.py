#!/usr/bin/env python3
"""The signal audit sheet (roadmap 062, O4): each signalised junction of a generated map with
Taipei's timing plan, for a person to say which approaches move in which phase.

    scripts/open_maps/audit_signals.py <site.yaml> <out> <sheet dir>

Reads `<out>/generated/signal_plans.json` (generate.py) and `<out>/raw/` (fetch.py: the NLSC
photo, `taipei_phases/<device>.pdf`). Per junction with a device, `<sheet dir>/<device>.jpg`:
the junction on NLSC's photo (north up), each approach's stop line red with an arrow along its heading and the label
the audit file takes (`heading` in degrees, counter-clockwise from east, and the road); and
`<device>_phases.png`, the phase diagrams of the device's 三合一報表 as the city printed it
(2024-03; north up, a row per phase order code), with the codes today's plans use. `<sheet dir>/sheet.json`
lists per junction the phase orders today's plans use (weekday schedule) and whether the
report's plans use the same: a diagram is only good for the codes it was drawn for.

Write what the diagram shows in the site's audit file (`sites/<site>_audit.yaml`):

    signals:
      - device: SDTJW10
        phase_order: "0A"        # the code the diagram is drawn for
        phases:                  # each phase: the approaches that move, by heading;
          - [-102, 78]           #   "<heading>:left" when only the left turn moves
          - ["-102:left"]
"""
import argparse
import json
import math
import re
import subprocess
import sys
import tempfile
from pathlib import Path

import yaml
from PIL import Image, ImageDraw, ImageFont, ImageOps

sys.path.insert(0, str(Path(__file__).parent))
from audit_junctions import Nlsc  # noqa: E402

RES, HALF = 0.12, 45.0
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"


def pdf_plans(pdf):
    txt = subprocess.run(["pdftotext", "-layout", str(pdf), "-"], capture_output=True, text=True).stdout
    return {str(int(m[0])): m[1] for m in re.findall(r"\b(\d\d)\s+\d+\s+\d+\s+\d\s+([0-9A-F]{2})\s+[\d ]+?\s*$", txt, re.M)}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("site")
    ap.add_argument("out", type=Path)
    ap.add_argument("sheet", type=Path)
    a = ap.parse_args()
    a.sheet.mkdir(parents=True, exist_ok=True)
    raw = a.out / "raw"
    photo = Nlsc(a.site, raw)
    plans = json.loads((a.out / "generated" / "signal_plans.json").read_text())
    try:
        font = ImageFont.truetype(FONT, 22)
    except OSError:
        font = ImageFont.load_default()
    summary = []
    for j in plans["signals"]:
        pts = [p for app in j["approaches"] for p in app["stop_line"]]
        cx, cy = sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts)
        half = max(HALF, max(max(abs(p[0] - cx), abs(p[1] - cy)) for p in pts) + 20)
        img = photo.render((cx - half, cy - half, cx + half, cy + half), RES * half / HALF)
        d = ImageDraw.Draw(img)
        P = lambda x, y: ((x - (cx - half)) / (RES * half / HALF), ((cy + half) - y) / (RES * half / HALF))
        for k, app in enumerate(j["approaches"]):
            (x0, y0), (x1, y1) = app["stop_line"]
            d.line([P(x0, y0), P(x1, y1)], fill=(255, 0, 0), width=5)
            mx, my = (x0 + x1) / 2, (y0 + y1) / 2
            h = math.radians(app["heading"])
            tail = (mx - 14 * math.cos(h), my - 14 * math.sin(h))
            d.line([P(*tail), P(mx, my)], fill=(0, 255, 255), width=5)
            for s in (-0.5, 0.5):
                q = (mx - 4 * math.cos(h + s), my - 4 * math.sin(h + s))
                d.line([P(*q), P(mx, my)], fill=(0, 255, 255), width=5)
            lx, ly = P(*tail)
            label = f"{app['heading']}° {app['road'] or '(unnamed)'} [j{app['junction']}]"
            d.rectangle(d.textbbox((lx, ly), label, font=font), fill=(0, 0, 0))
            d.text((lx, ly), label, fill=(0, 255, 255), font=font)
        d.text((8, 8), f"{j['device']} {j['name']}  junctions {j['junctions']}", fill=(255, 255, 0), font=font)
        pdf = raw / "taipei_phases" / f"{j['device']}.pdf"
        if pdf.exists() and not pdf.read_bytes()[:4] == b"%PDF":
            pdf = Path("/nonexistent")     # the city served an error page for it
        weekday = sorted({p for _, p in j["schedule"].get("1", [])}, key=int)
        today = {p: j["plans"][p]["phase_order"] for p in weekday if p in j["plans"]}
        drawn = pdf_plans(pdf) if pdf.exists() else {}
        rec = {"device": j["device"], "name": j["name"], "junctions": j["junctions"],
               "approaches": [(x["heading"], x["road"]) for x in j["approaches"]],
               "weekday_plans": today, "report_plans": {p: drawn.get(p) for p in weekday},
               "orders_now": sorted(set(today.values())), "orders_drawn": sorted(set(drawn.values()))}
        summary.append(rec)
        img.save(a.sheet / f"{j['device']}.jpg", quality=85)
        if pdf.exists():
            # the report's phase diagrams, from the first 「分相」 label down, at 200 dpi, trimmed
            with tempfile.TemporaryDirectory() as tmp:
                subprocess.run(["pdftoppm", "-r", "200", "-png", "-singlefile", str(pdf), f"{tmp}/p"], check=True)
                page = Image.open(f"{tmp}/p.png").convert("RGB")
            bbox = subprocess.run(["pdftotext", "-bbox", str(pdf), "-"], capture_output=True, text=True).stdout
            ys = [float(m) for m in re.findall(r'yMin="([\d.]+)"[^>]*>分相', bbox)]
            ph = re.search(r'<page width="[\d.]+" height="([\d.]+)"', bbox)
            top = (min(ys) - 12) / float(ph.group(1)) if ys and ph else 0.55
            low = page.crop((0, int(page.height * top), page.width, page.height))
            box = ImageOps.invert(low.convert("L")).point(lambda v: 255 if v > 40 else 0).getbbox()
            if box:
                low = low.crop((max(0, box[0] - 20), max(0, box[1] - 20), min(low.width, box[2] + 20), min(low.height, box[3] + 20)))
            dd = ImageDraw.Draw(low)
            dd.text((10, 5), f"now (weekday plan: phase order): {today}", fill=(200, 0, 0), font=font)
            low.save(a.sheet / f"{j['device']}_phases.png")
    (a.sheet / "sheet.json").write_text(json.dumps(summary, indent=1, ensure_ascii=False))
    print(f"{len(summary)} junctions with a Taipei signal in {a.sheet}")


if __name__ == "__main__":
    main()
