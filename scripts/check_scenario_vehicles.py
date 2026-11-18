#!/usr/bin/env python3
"""Lint: scenario vehicles come from the vehicle catalog, and every other `<Vehicle>` named by a
CARLA blueprint declares that blueprint's box and axles (roadmaps 050, 051).

    scripts/check_scenario_vehicles.py [PATH ...] [--tolerance M]

The catalog (roadmap 051) is sim_vehicle_spec's `catalogs/vehicles.xosc`, generated from the
vehicle specs. In a scenario:

- an inline `<Vehicle>` whose body (`model3d`, else `name`) is a cataloged car fails: it must
  be a `CatalogReference` to the entry (`scripts/catalog_scenarios.py --rewrite` makes it one);
- a `CatalogReference` to the `vehicles` catalog must name an entry and assign exactly the
  entry's parameters, every one (the entry declares no defaults: in SSv2 a declared default
  would shadow the assignment), and the file's `CatalogLocations` must reach the catalog
  through `$(env SCENSIM_VEHICLES)`, the catalog the stack makes for the simulator's body
  (roadmap 083; scenario.launch.xml defaults it to the installed one), or through
  `$(find-pkg-share sim_vehicle_spec)/catalogs`; in a testbench (a directory with
  `testbench.yaml`) or a scenario package testbenches run (scensim_examples' scenarios,
  scensim_study_scenarios), only the former.

The catalog's own entries are checked against the measured table like any declaration, by
their `model3d`, and against their specs: an entry whose `scensim.spec_hash` is not the hash of
`sim_vehicle_spec/vehicles/<entry>.yaml` as it stands fails (a spec changed and the catalog
was not regenerated). So is each body's own catalog (`catalogs/carla-<version>/vehicles.xosc`,
roadmap 083), against that version's measured table (`blueprint_boxes_<version>.yaml`).

PATHs are `.xosc` files or directories searched for them (default: `src/scensim_examples`,
`src/scensim_study_scenarios` and `scenarios`). For each `<Vehicle name="...">` whose name is
a blueprint with axles in the measured table, the declared `BoundingBox` and `Axles` must equal what
`scripts/scenario_vehicle.py` derives from the table, within the tolerance (default 0.02 m):
Center x/y/z, Dimensions, both axles' wheelDiameter, trackWidth and positionZ, the front
axle's positionX, and the rear axle's positionX (0, the vehicle origin).

A vehicle that differs on purpose carries an OpenSCENARIO property naming why, and is
listed rather than failed:

    <Properties><Property name="scensim.box_exempt" value="why" /></Properties>

Exit status 1 on any failure, each named by file, entity, field, declared and measured value.
Run by `just test`.
"""

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import catalog_scenarios  # scripts/, beside this file
import scenario_vehicle

ROOT = scenario_vehicle.ROOT
DEFAULT_PATHS = [ROOT / "src/scensim_examples", ROOT / "src/scensim_study_scenarios", ROOT / "scenarios"]
EXEMPT = "scensim.box_exempt"

# (element path under <Vehicle>, attribute, geometry key; None = expected 0)
FIELDS = [
    ("BoundingBox/Center", "x", "center_x"),
    ("BoundingBox/Center", "y", "center_y"),
    ("BoundingBox/Center", "z", "center_z"),
    ("BoundingBox/Dimensions", "length", "length"),
    ("BoundingBox/Dimensions", "width", "width"),
    ("BoundingBox/Dimensions", "height", "height"),
    ("Axles/FrontAxle", "positionX", "front_x"),
    ("Axles/FrontAxle", "positionZ", "axle_z"),
    ("Axles/FrontAxle", "wheelDiameter", "wheel_diameter"),
    ("Axles/FrontAxle", "trackWidth", "track"),
    ("Axles/RearAxle", "positionX", None),
    ("Axles/RearAxle", "positionZ", "axle_z"),
    ("Axles/RearAxle", "wheelDiameter", "wheel_diameter"),
    ("Axles/RearAxle", "trackWidth", "track"),
]


def scenario_files(paths):
    for path in paths:
        path = Path(path)
        if path.is_dir():
            yield from sorted(path.rglob("*.xosc"))
        elif path.suffix == ".xosc":
            yield path


def entity_of(parents, vehicle):
    """The ScenarioObject (or CatalogEntry) name a <Vehicle> belongs to."""
    node = parents.get(vehicle)
    while node is not None:
        if node.tag in ("ScenarioObject", "Vehicle", "CatalogEntry") and node.get("name"):
            if node is not vehicle:
                return node.get("name")
        node = parents.get(node)
    return "?"


def catalog_parameters(path=catalog_scenarios.CATALOG):
    """{entry name: {parameter names its attributes refer to ($name)}} of the vehicle catalog."""
    root = ET.parse(path).getroot()
    return {v.get("name"): {value[1:] for node in v.iter() for value in node.attrib.values()
                            if value.startswith("$")}
            for v in root.iter("Vehicle")}


# A testbench's scenarios name the catalog the stack makes for the simulator it runs
VEHICLES_DIR = "$(env SCENSIM_VEHICLES)"
# The scenario packages testbenches run (pkg://...): their scenarios name it too, and
# scenario.launch.xml defaults it to the installed catalog for a run outside the stack
TESTBENCH_PACKAGES = ("src/scensim_examples/scenarios", "src/scensim_study_scenarios")


def check_references(path, root, parents, where_of, catalog):
    """Failures of the file's references to the vehicle catalog."""
    failures = []
    refs = [r for r in root.iter("CatalogReference")
            if r.get("catalogName") == catalog_scenarios.CATALOG_NAME]
    for ref in refs:
        where = where_of(ref)
        entry = ref.get("entryName")
        if entry not in catalog:
            failures.append(f"{where}: no catalog entry {entry!r} (have {sorted(catalog)})")
            continue
        assigned = [a.get("parameterRef")
                    for a in ref.findall("ParameterAssignments/ParameterAssignment")]
        for name in assigned:
            if name not in catalog[entry]:
                failures.append(f"{where}: {entry} has no parameter {name!r} "
                                f"(has {sorted(catalog[entry])})")
        missing = sorted(catalog[entry] - set(assigned))
        if missing:
            failures.append(f"{where}: {entry} needs {missing} assigned (the catalog declares "
                            "no defaults)")
    if refs:
        dirs = [d.get("path") for d in root.findall("CatalogLocations/VehicleCatalog/Directory")]
        # a testbench's scenario names the stack's catalog, for the simulator's body (roadmap
        # 083); any other in-repo scenario names the installed one
        in_testbench = (any((d / "testbench.yaml").is_file() for d in Path(path).parents)
                        or any(Path(path).resolve().is_relative_to(ROOT / p)
                               for p in TESTBENCH_PACKAGES))
        # elsewhere either: scenario.launch.xml defaults $(env SCENSIM_VEHICLES) to the
        # installed catalog for a run outside the stack
        wants = ([VEHICLES_DIR] if in_testbench
                 else [catalog_scenarios.CATALOG_DIR, VEHICLES_DIR])
        if len(dirs) != 1 or dirs[0] not in wants:
            failures.append(f"{where_of(root)}: VehicleCatalog is {dirs}, not one of "
                            f"{wants!r}")
    return failures


def check_catalog_specs(path=catalog_scenarios.CATALOG):
    """Failures for catalog entries whose spec_hash is not their spec's (a stale catalog)."""
    sys.path.insert(0, str(ROOT / "src/sim_vehicle_spec"))
    from sim_vehicle_spec import spec as specs

    failures = []
    for vehicle in ET.parse(path).getroot().iter("Vehicle"):
        name = vehicle.get("name")
        named = {p.get("name"): p.get("value") for p in vehicle.iter("Property")}
        source = ROOT / "src/sim_vehicle_spec/vehicles" / f"{name}.yaml"
        if "scensim.spec_hash" not in named or not source.exists():
            failures.append(f"{path.name}: {name}: no scensim.spec_hash or no spec {source.name}")
            continue
        want = specs.spec_hash(specs.load(source))
        if named["scensim.spec_hash"] != want:
            failures.append(f"{path.name}: {name} names spec {named['scensim.spec_hash'][:12]}, but "
                            f"vehicles/{name}.yaml is {want[:12]}: the catalog is stale "
                            f"(sim_vehicle_spec's `just generate`)")
    return failures


def check_file(path, table, tolerance, catalog=None, bodies=None):
    """(failures, exemptions, vehicles checked) for one file. `catalog` ({entry: parameters})
    and `bodies` ({model3d: entry}) are the vehicle catalog's; `bodies` None checks no
    cataloging (the catalog file itself)."""
    failures, exemptions, checked = [], [], 0
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError as e:
        return [f"{path}: not XML ({e})"], [], 0
    parents = {child: parent for parent in root.iter() for child in parent}
    rel = path.relative_to(ROOT) if path.is_relative_to(ROOT) else path

    def where_of(node):
        return f"{rel}: {entity_of(parents, node)}" if node is not root else str(rel)

    for prop in root.iter("Property"):
        old = prop.get("name") or ""
        if old.startswith("swb."):
            failures.append(f"{rel}: property {old} was renamed with the project (roadmap 080); "
                            f"name it scensim.{old[len('swb.'):]}")
    if catalog is not None:
        failures += check_references(path, root, parents, where_of, catalog)
    for vehicle in root.iter("Vehicle"):
        body = catalog_scenarios.body_of(vehicle)
        geometry = scenario_vehicle.geometry(body, table)
        entity = entity_of(parents, vehicle)
        where = f"{rel}: {entity}"
        exempt = [p.get("value", "") for p in vehicle.findall("Properties/Property")
                  if p.get("name") == EXEMPT]
        if exempt and (geometry is not None or body in (bodies or {})):
            exemptions.append(f"{where} ({body}): {exempt[0]}")
            continue
        if bodies and body in bodies:
            failures.append(f"{where}: an inline {body}; refer to the catalog "
                            f"(CatalogReference entryName=\"{bodies[body]}\", "
                            "scripts/catalog_scenarios.py --rewrite)")
            continue
        if geometry is None:
            continue
        checked += 1
        for element, attribute, key in FIELDS:
            node = vehicle.find(element)
            expected = 0.0 if key is None else geometry[key]
            if node is None or node.get(attribute) is None:
                failures.append(f"{where}: {element} has no {attribute} "
                                f"(measured {scenario_vehicle.fmt(expected)})")
                continue
            try:
                declared = float(node.get(attribute))
            except ValueError:
                failures.append(f"{where}: {element} {attribute}={node.get(attribute)!r} "
                                "is not a number")
                continue
            if abs(declared - expected) > tolerance:
                failures.append(f"{where}: {element} {attribute} declared "
                                f"{node.get(attribute)}, measured {scenario_vehicle.fmt(expected)}")
    return failures, exemptions, checked


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("paths", nargs="*", type=Path)
    ap.add_argument("--tolerance", type=float, default=0.02)
    ap.add_argument("--table", type=Path, default=scenario_vehicle.TABLE)
    args = ap.parse_args()
    table = scenario_vehicle.load_table(args.table)
    catalog, bodies = catalog_parameters(), catalog_scenarios.entries()
    failures, exemptions, checked, files = [], [], 0, 0
    f, e, c = check_file(catalog_scenarios.CATALOG, table, args.tolerance)
    failures, exemptions, checked, files = f + check_catalog_specs(), e, c, 1
    # each CARLA body's catalog against its own version's table (roadmap 083)
    for body_catalog in sorted(catalog_scenarios.CATALOG.parent.glob("carla-*/vehicles.xosc")):
        version = body_catalog.parent.name[len("carla-"):]
        body_table = args.table.with_name(f"blueprint_boxes_{version}.yaml")
        if not body_table.is_file():
            failures.append(f"{body_catalog}: no measured table {body_table.name} for it")
            continue
        f, e, c = check_file(body_catalog, scenario_vehicle.load_table(body_table),
                             args.tolerance)
        failures += f + check_catalog_specs(body_catalog)
        exemptions += e
        checked += c
        files += 1
    for path in scenario_files(args.paths or DEFAULT_PATHS):
        f, e, c = check_file(path.resolve(), table, args.tolerance, catalog, bodies)
        failures += f
        exemptions += e
        checked += c
        files += 1
    for line in exemptions:
        print(f"exempt: {line}")
    for line in failures:
        print(f"FAIL: {line}")
    print(f"scenario vehicles: {checked} checked in {files} files, {len(exemptions)} exempt, "
          f"{len(failures)} failure(s)")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
