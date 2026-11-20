"""A project's own cars (``vehicles:`` in testbench.yaml), made for the stack (roadmap 077, G5).

Each listed spec (sim_vehicle_spec's format) is generated, when the stack starts, into
``<project>/state/vehicles/<name>/`` by ``sim_vehicle_spec.gen``: its SSv2 catalog entry,
its ``spec_hash.yaml``, its CARLA files (``carla/carla_physics.yaml``,
``carla/carla_sensors.yaml``) and, for an Isaac testbench, its Isaac rig (``isaac/<name>.rig.yaml``,
by ``aib.gen``; roadmap 091 W5). Then one catalog holds the installed cars and the project's,
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


def carla_body(version):
    """The body a car drives on a CARLA server of ``version`` (sim_vehicle_spec's ``bodies``,
    roadmap 083): ``carla-1.0`` for 1.0, ``carla-0.9.16`` for 0.9.16; None when the version
    is not known (each car's default body)."""
    if not version:
        return None
    v = str(version).strip().split("-")[0]  # a source build reports "1.0-<commit>"
    return f"carla-{v}"


def body_file(directory: Path, body) -> Path:
    """``<directory>/<body>/vehicles.xosc`` when a car has a catalog for that body (sim_vehicle_
    spec writes a body other than the default in a directory of its own), else the default
    ``<directory>/vehicles.xosc``."""
    if body and (Path(directory) / body / CATALOG).is_file():
        return Path(directory) / body / CATALOG
    return Path(directory) / CATALOG


def _spec_share() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("sim_vehicle_spec"))
    except Exception:
        return Path(__file__).resolve().parents[2] / "sim_vehicle_spec"


def installed_catalog(body=None) -> Path:
    """sim_vehicle_spec's catalog (the installed share's, else the workspace's), for ``body``
    where it has one (``catalogs/<body>/vehicles.xosc``)."""
    return body_file(_spec_share() / "catalogs", body)


def autoware_vehicle_model(car: str, body) -> str:
    """Autoware's vehicle_model for ``car`` on ``body``: autoware_sim_vehicle's
    ``<car>_<body>`` (non-alphanumerics as ``_``: tesla_model3_carla_1_0) when that package
    is installed, else ``car`` (its default body, or a car with no such body)."""
    import re

    if not body:
        return car
    model = f"{car}_{re.sub('[^a-z0-9]+', '_', body.lower()).strip('_')}"
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory(f"{model}_description")
        return model
    except Exception:
        return car


def installed_physics(car: str):
    """An installed car's CARLA physics (sim_vehicle_spec's generated/<car>/carla_physics.yaml,
    the installed share's, else the workspace's), or None when sim_vehicle_spec has no such
    car. One path for every CARLA: built for 1.0, the world server and acb take its sibling
    carla_physics_1.0.yaml."""
    f = _spec_share() / "generated" / car / "carla_physics.yaml"
    return f if f.is_file() else None


def catalog_dir(project, body=None) -> Path:
    """What ``$(env SCENSIM_VEHICLES)`` names for a project: its merged catalog's directory when
    it has own cars, else the installed one's for ``body`` (the CARLA server's, ``carla_body``):
    the scenarios then declare the body the world server drives, which the bridge checks to
    2 cm."""
    d = Path(project) / STATE / CATALOG_DIR
    return d if (d / CATALOG).is_file() else installed_catalog(body).parent


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


def body_models(spec_path) -> set:
    """Every body a spec's car drives: ``body.catalogue_model`` and each simulator body's
    (``bodies.<body>.catalogue_model``), which the catalog for that body names instead."""
    try:
        doc = yaml.safe_load(Path(spec_path).read_text()) or {}
    except (OSError, yaml.YAMLError):
        return set()
    out = {(doc.get("body") or {}).get("catalogue_model")}
    for b in (doc.get("bodies") or {}).values():
        if isinstance(b, dict):
            out.add(b.get("catalogue_model"))
    return {m for m in out if isinstance(m, str) and m}


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
        for b in sorted(body_models(path) | ({body} if body else set())):
            if b in bodies:
                out.append(f"vehicles: {path.name}: its body {b} is {bodies[b]}'s; give your "
                           "car a body of its own, in body: and in each of bodies: (the "
                           "bridge refuses two specs on one body)")
        if name in seen:
            out.append(f"vehicles: two specs are named {name}")
        seen.add(name)
    return out


def generate(project, settings: dict, gen=None, body=None, rig_gen=None) -> dict:
    """Generate every own car (``gen(spec, out_dir) -> None``, default sim_vehicle_spec.gen)
    and the merged catalog. ``{name: {spec, dir, catalog, spec_hash_file, spec_hash,
    carla_physics, carla_sensors, isaac_rig}}`` and ``"__catalog__"``: the merged catalog (None
    without own cars). The merged catalog is ``body``'s where a car has a catalog for it (the
    CARLA server's, ``carla_body``). On an Isaac testbench each car's rig is made too
    (``rig_gen(spec, rig) -> None``, default aib.gen); elsewhere ``isaac_rig`` is None."""
    cars = spec_names(project, settings)
    root = Path(project) / STATE
    out = {}
    for name, spec, _ in cars:
        d = root / name
        d.mkdir(parents=True, exist_ok=True)
        (gen or run_gen)(spec, d)
        rig = None
        if settings.get("simulator") == "isaac":
            rig = d / "isaac" / f"{name}.rig.yaml"
            rig.parent.mkdir(parents=True, exist_ok=True)
            (rig_gen or run_rig_gen)(spec, rig)
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
            "isaac_rig": str(rig) if rig else None,
        }
    merged = None
    if out:
        merged = root / CATALOG_DIR / CATALOG
        merge([installed_catalog(body)]
              + [body_file(Path(c["dir"]), body) for c in out.values()], merged)
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


def run_rig_gen(spec: Path, rig: Path):
    """The Isaac rig of ``spec`` (aib.gen --spec), which names the spec by a path relative to
    the rig; aib loads the car's onboard computers' calibration from that spec."""
    argv = [sys.executable, "-m", "aib.gen", "--spec", str(spec), "-o", str(rig)]
    try:
        r = subprocess.run(argv, capture_output=True, text=True, timeout=120)
    except OSError as e:
        raise VehicleError(f"making {Path(spec).name}'s Isaac rig: {e}") from None
    if r.returncode != 0:
        last = (r.stderr.strip().splitlines() or ["(no output)"])[-1]
        hint = (" (aib is not importable: build and source scensim's workspace)"
                if "No module named 'aib'" in r.stderr else "")
        raise VehicleError(f"making {Path(spec).name}'s Isaac rig failed: {last}{hint}")


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
