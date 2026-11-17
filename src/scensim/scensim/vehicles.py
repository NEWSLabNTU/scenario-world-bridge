"""A project's own cars (``vehicles:`` in swb.yaml), made for the stack (roadmap 077, G5).

Each listed spec (sim_vehicle_spec's format) is generated, when the stack starts, into
``<project>/state/vehicles/<name>/`` by ``sim_vehicle_spec.gen``: its SSv2 catalog entry,
its ``spec_hash.yaml``, and its CARLA files (``carla/carla_physics.yaml``,
``carla/carla_sensors.yaml``). Then one catalog holds the installed cars and the project's,
``state/vehicles/catalogs/vehicles.xosc``: a scenario names it by
``$(env SCENSIM_VEHICLES)`` (the stack sets it for every run, with or without own cars), the
bridge reads it (``vehicle_catalogs``), and a pilot's car is looked up there.

The car's name is the spec's ``name:``; it may not be an installed car's, and its body
(``body.catalogue_model``) may not be one an installed car uses (the bridge refuses two
specs on one body).
"""
import copy
import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

STATE = Path("state") / "vehicles"
CATALOG_DIR = "catalogs"
CATALOG = "vehicles.xosc"
ENV = "SCENSIM_VEHICLES"


class VehicleError(RuntimeError):
    pass


def installed_catalog() -> Path:
    """sim_vehicle_spec's catalog: the installed share's, else the workspace's."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("sim_vehicle_spec")) / "catalogs" / CATALOG
    except Exception:
        return Path(__file__).resolve().parents[2] / "sim_vehicle_spec" / "catalogs" / CATALOG


def installed_physics(car: str):
    """An installed car's CARLA physics (sim_vehicle_spec's generated/<car>/carla_physics.yaml,
    the installed share's, else the workspace's), or None when sim_vehicle_spec has no such
    car."""
    try:
        from ament_index_python.packages import get_package_share_directory
        root = Path(get_package_share_directory("sim_vehicle_spec"))
    except Exception:
        root = Path(__file__).resolve().parents[2] / "sim_vehicle_spec"
    f = root / "generated" / car / "carla_physics.yaml"
    return f if f.is_file() else None


def catalog_dir(project) -> Path:
    """What ``$(env SCENSIM_VEHICLES)`` names for a project: its merged catalog's directory when
    it has own cars, else the installed one's."""
    d = Path(project) / STATE / CATALOG_DIR
    return d if (d / CATALOG).is_file() else installed_catalog().parent


def _vehicles(catalog: Path) -> list:
    root = ET.parse(str(catalog)).getroot()
    cat = root.find("Catalog")
    return list(cat.findall("Vehicle")) if cat is not None else []


def installed_names(catalog: Path = None) -> dict:
    """``{name: model3d}`` of the installed cars."""
    return {v.get("name"): v.get("model3d") for v in _vehicles(catalog or installed_catalog())}


def spec_names(project, settings: dict) -> list:
    """``[(name, spec path)]`` of the project's ``vehicles:``; VehicleError naming a file
    that is missing or has no ``name:``."""
    out = []
    for v in settings.get("vehicles") or []:
        p = (Path(project) / str(v)).resolve()
        try:
            doc = yaml.safe_load(p.read_text()) or {}
        except (OSError, yaml.YAMLError) as e:
            raise VehicleError(f"vehicles: {v}: {e}") from None
        name = doc.get("name")
        if not isinstance(name, str) or not name:
            raise VehicleError(f"vehicles: {v}: the spec has no name:")
        out.append((name, p, ((doc.get("body") or {}).get("catalogue_model"))))
    return out


def problems(project, settings: dict, installed: dict = None) -> list:
    """``project check``'s words on ``vehicles:``: a name or body an installed car has, two
    specs of one name."""
    try:
        cars = spec_names(project, settings)
    except VehicleError as e:
        return [str(e)]
    installed = installed if installed is not None else installed_names()
    out, seen = [], set()
    bodies = {m: n for n, m in installed.items()}
    for name, path, body in cars:
        if name in installed:
            out.append(f"vehicles: {path.name}: {name} is an installed car's name; give yours "
                       "another")
        if body and body in bodies:
            out.append(f"vehicles: {path.name}: its body {body} is {bodies[body]}'s; give "
                       "your car a body of its own (the bridge refuses two specs on one body)")
        if name in seen:
            out.append(f"vehicles: two specs are named {name}")
        seen.add(name)
    return out


def generate(project, settings: dict, gen=None) -> dict:
    """Generate every own car (``gen(spec, out_dir) -> None``, default sim_vehicle_spec.gen)
    and the merged catalog. ``{name: {spec, dir, catalog, spec_hash_file, spec_hash,
    carla_physics, carla_sensors}}`` and ``"__catalog__"``: the merged catalog (None without
    own cars)."""
    cars = spec_names(project, settings)
    root = Path(project) / STATE
    out = {}
    for name, spec, _ in cars:
        d = root / name
        d.mkdir(parents=True, exist_ok=True)
        (gen or run_gen)(spec, d)
        hash_file = d / "spec_hash.yaml"
        (d / "description.urdf").write_text(urdf(yaml.safe_load(Path(spec).read_text())))
        out[name] = {
            "spec": str(spec), "dir": str(d), "catalog": str(d / CATALOG),
            "spec_hash_file": str(hash_file),
            "spec_hash": (yaml.safe_load(hash_file.read_text()) or {}).get("spec_hash"),
            "carla_physics": str(d / "carla" / "carla_physics.yaml"),
            "carla_sensors": str(d / "carla" / "carla_sensors.yaml"),
            "description": str(d / "description.urdf"),
            "pedal_maps": pedal_maps(spec, name),
        }
    merged = None
    if out:
        merged = root / CATALOG_DIR / CATALOG
        merge([installed_catalog()] + [Path(c["catalog"]) for c in out.values()], merged)
    out["__catalog__"] = str(merged) if merged else None
    return out


def run_gen(spec: Path, d: Path):
    argv = [sys.executable, "-m", "sim_vehicle_spec.gen", "--spec", str(spec),
            "--catalog", str(d / CATALOG), "--spec-hash", str(d / "spec_hash.yaml"),
            "--carla", str(d / "carla")]
    r = subprocess.run(argv, capture_output=True, text=True, timeout=120)
    if r.returncode != 0:
        raise VehicleError(f"generating {spec.name} failed: "
                           + (r.stderr.strip().splitlines() or ["(no output)"])[-1])


def merge(catalogs: list, out: Path) -> Path:
    """One ``vehicles`` catalog of every Vehicle in ``catalogs``, the first of a name kept."""
    first = ET.parse(str(catalogs[0]))
    cat = first.getroot().find("Catalog")
    names = {v.get("name") for v in cat.findall("Vehicle")}
    for c in catalogs[1:]:
        for v in _vehicles(c):
            if v.get("name") not in names:
                cat.append(copy.deepcopy(v))
                names.add(v.get("name"))
    out.parent.mkdir(parents=True, exist_ok=True)
    tmp = out.with_name(out.name + f".{os.getpid()}.tmp")
    first.write(str(tmp), encoding="UTF-8", xml_declaration=True)
    tmp.replace(out)
    return out


OPTICAL = {"roll": -1.5707963267948966, "pitch": 0.0, "yaw": -1.5707963267948966}


def urdf(spec: dict) -> str:
    """The car's frames as a URDF, for a pilot without the car's Autoware packages:
    base_link, sensor_kit_base_link at the kit's mount, each sensor's link at its mount (in
    the kit), and its message frame under the link (identity, or a camera's optical
    rotation). CARLA's vehicle bridge places the sensors at these frames."""
    def origin(m):
        m = m or {}
        xyz = " ".join(f"{float(m.get(k, 0.0)):.9g}" for k in ("x", "y", "z"))
        rpy = " ".join(f"{float(m.get(k, 0.0)):.9g}" for k in ("roll", "pitch", "yaw"))
        return f'<origin xyz="{xyz}" rpy="{rpy}"/>'

    links, joints = ["base_link", "sensor_kit_base_link"], []

    def joint(parent, child, m):
        joints.append(f'  <joint name="{child}_joint" type="fixed"><parent link="{parent}"/>'
                      f'<child link="{child}"/>{origin(m)}</joint>')

    joint("base_link", "sensor_kit_base_link", (spec.get("sensor_kit") or {}).get("mount"))
    for name, sen in (spec.get("sensors") or {}).items():
        link = sen.get("link") or name
        if link not in links:
            links.append(link)
            joint("sensor_kit_base_link", link, sen.get("mount"))
        frame = sen.get("frame")
        if frame and frame != link and frame not in links:
            links.append(frame)
            joint(link, frame, OPTICAL if sen.get("type") == "camera" else None)
    body = "\n".join([f'  <link name="{n}"/>' for n in links] + joints)
    return (f'<?xml version="1.0"?>\n<!-- {spec.get("name")}: generated by scensim from its spec '
            f'(frames only) -->\n<robot name="vehicle">\n{body}\n</robot>\n')


FALLBACK_MAPS_CAR = "tesla_model3"


def pedal_maps(spec: Path, name: str) -> dict:
    """The car's pedal maps (accel_map.csv, brake_map.csv: acceleration to throttle and
    brake, measured on the car, never generated): ``vehicles/<name>_maps/`` beside its spec
    when it has them, else the Tesla Model 3's, recorded as a fallback."""
    own = Path(spec).parent / f"{name}_maps"
    if (own / "accel_map.csv").is_file() and (own / "brake_map.csv").is_file():
        return {"dir": str(own), "fallback": False}
    try:
        from ament_index_python.packages import get_package_share_directory
        d = Path(get_package_share_directory(f"{FALLBACK_MAPS_CAR}_description")) / "config"
    except Exception:  # noqa: BLE001 (not installed)
        d = None
    return {"dir": str(d) if d else "", "fallback": True}
