"""zones.yaml: read and write the offline zones (step 1's output) as czp's data model. Pure YAML."""

from __future__ import annotations

import yaml

from .model import Approach, Zone


def zone_from_dict(d):
    aps = tuple(Approach(id=str(a["id"]), lanelet=a.get("lanelet"), heading=float(a.get("heading", 0.0)),
                         order=int(a["order"]), priority=a.get("priority", "equal"),
                         stop_line=tuple(map(tuple, a["stop_line"])) if a.get("stop_line") else None,
                         entry_s=float(a.get("entry_s", 0.0)), exit_s=float(a.get("exit_s", 0.0)),
                         width=float(a.get("width", 3.5)), obstructed=bool(a.get("obstructed", False)),
                         centerline=tuple(tuple(p) for p in a.get("centerline", ())), exit_lanelet=a.get("exit_lanelet"))
                for a in d["approaches"])
    return Zone(id=str(d["id"]), kind=d["kind"], approaches=aps, polygon=tuple(tuple(p) for p in d.get("polygon", ())),
                lanelets=tuple(d.get("lanelets", ())),
                exit_box=tuple((str(k), tuple(tuple(p) for p in poly)) for k, poly in (d.get("exit_box") or {}).items()),
                shared_width=d.get("shared_width"))


def load_zones(path):
    """(the file's header, [Zone]) from a zones.yaml."""
    raw = yaml.safe_load(open(path))
    zones = [zone_from_dict(z) for j in raw.get("junctions", []) for z in j["zones"]]
    head = {k: v for k, v in raw.items() if k != "junctions"}
    return head, zones


def zone_to_dict(z: Zone):
    r = lambda x: round(float(x), 3)  # noqa: E731
    return {
        "id": z.id, "kind": z.kind, "lanelets": list(z.lanelets),
        "polygon": [[r(x), r(y)] for x, y in z.polygon],
        "shared_width": z.shared_width,
        "exit_box": {k: [[r(x), r(y)] for x, y in poly] for k, poly in z.exit_box},
        "approaches": [{"id": a.id, "lanelet": a.lanelet, "heading": r(a.heading), "order": a.order,
                        "priority": a.priority, "entry_s": r(a.entry_s), "exit_s": r(a.exit_s), "width": r(a.width),
                        "obstructed": a.obstructed, "exit_lanelet": a.exit_lanelet,
                        "stop_line": [[r(x), r(y)] for x, y in a.stop_line] if a.stop_line else None,
                        "centerline": [[r(x), r(y)] for x, y in a.centerline]} for a in z.approaches],
    }
