"""scensim.vehicles (roadmap 077, G5): a project's own cars generated for the stack."""
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

from scensim import project as projectmod, stack, vehicles

SPEC = Path(__file__).resolve().parents[2] / "sim_vehicle_spec" / "vehicles" / "tesla_model3.yaml"
INSTALLED = Path(__file__).resolve().parents[2] / "sim_vehicle_spec" / "catalogs" / "vehicles.xosc"

pytestmark = pytest.mark.skipif(not SPEC.is_file(), reason="sim_vehicle_spec not checked out")


def own_spec(d: Path, name="my_lincoln", body="vehicle.lincoln.mkz_2020", bodies=False) -> Path:
    """A copy of the Model 3's spec as `name` on `body`; with `bodies`, its simulator bodies
    (bodies:) moved to `body` too."""
    doc = yaml.safe_load(SPEC.read_text())
    doc["name"], doc["body"]["catalogue_model"] = name, body
    if bodies:
        for b in (doc.get("bodies") or {}).values():
            b["catalogue_model"] = body
    p = d / "vehicles" / f"{name}.yaml"
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(yaml.safe_dump(doc, sort_keys=False))
    return p


def fake_gen(spec, d):
    """sim_vehicle_spec.gen's outputs, in their shapes."""
    doc = yaml.safe_load(Path(spec).read_text())
    name, body = doc["name"], doc["body"]["catalogue_model"]
    (d / "carla").mkdir(parents=True, exist_ok=True)
    (d / "spec_hash.yaml").write_text(f"spec_hash: hash-of-{name}\n")
    (d / "carla" / "carla_physics.yaml").write_text("x: 1\n")
    (d / "carla" / "carla_sensors.yaml").write_text("sensors: {}\n")
    (d / "vehicles.xosc").write_text(
        f'<OpenSCENARIO><Catalog name="vehicles"><Vehicle name="{name}" model3d="{body}"/>'
        "</Catalog></OpenSCENARIO>")


def test_merge_keeps_the_installed_cars_and_adds_the_projects(tmp_path):
    own = tmp_path / "own.xosc"
    own.write_text('<OpenSCENARIO><Catalog name="vehicles"><Vehicle name="my_lincoln" '
                   'model3d="vehicle.lincoln.mkz_2020"/><Vehicle name="tesla_model3" '
                   'model3d="x"/></Catalog></OpenSCENARIO>')
    out = vehicles.merge([INSTALLED, own], tmp_path / "m" / "vehicles.xosc")
    names = [v.get("name") for v in ET.parse(out).getroot().iter("Vehicle")]
    assert names == ["drift_mustang", "tesla_model3", "my_lincoln"]   # the first of a name


def test_problems_name_a_taken_name_or_body(tmp_path):
    own_spec(tmp_path, "tesla_model3", "vehicle.lincoln.mkz_2020")
    own_spec(tmp_path, "my_tesla", "vehicle.tesla.model3")
    s = {"vehicles": ["vehicles/tesla_model3.yaml", "vehicles/my_tesla.yaml"]}
    p = vehicles.problems(tmp_path, s, installed=vehicles.installed_names(INSTALLED))
    assert any("is an installed car's name" in x for x in p)
    assert any("vehicle.tesla.model3 is tesla_model3's" in x for x in p)
    # the Model 3's spec names its CARLA bodies too (bodies:): a copy that renames only body:
    # still drives the Model 3's body there, which the bridge refuses (roadmap 091 W5)
    own_spec(tmp_path)
    p = vehicles.problems(tmp_path, {"vehicles": ["vehicles/my_lincoln.yaml"]},
                          installed=vehicles.installed_names(INSTALLED))
    assert p and all("vehicle.tesla.model3 is tesla_model3's" in x for x in p)
    spec = own_spec(tmp_path, bodies=True)
    assert vehicles.body_models(spec) == {"vehicle.lincoln.mkz_2020"}
    assert vehicles.problems(tmp_path, {"vehicles": ["vehicles/my_lincoln.yaml"]},
                             installed=vehicles.installed_names(INSTALLED)) == []


def test_urdf_has_the_kit_and_every_sensor_frame():
    u = vehicles.urdf(yaml.safe_load(SPEC.read_text()))
    root = ET.fromstring(u)
    links = {l.get("name") for l in root.iter("link")}
    assert {"base_link", "sensor_kit_base_link", "top_base_link", "top", "camera6/camera_link",
            "camera6/camera_optical_link", "imu_link", "gnss_link"} <= links
    kit = next(j for j in root.iter("joint") if j.find("child").get("link") == "sensor_kit_base_link")
    assert kit.find("origin").get("xyz") == "0.9 0 2"
    opt = next(j for j in root.iter("joint")
               if j.find("child").get("link") == "camera6/camera_optical_link")
    assert opt.find("origin").get("rpy").startswith("-1.57079633")


def test_generate_writes_each_car_and_the_merged_catalog(tmp_path, monkeypatch):
    monkeypatch.setattr(vehicles, "installed_catalog", lambda body=None: INSTALLED)
    own_spec(tmp_path)
    cars = vehicles.generate(tmp_path, {"vehicles": ["vehicles/my_lincoln.yaml"]}, gen=fake_gen)
    c = cars["my_lincoln"]
    assert c["spec_hash"] == "hash-of-my_lincoln"
    assert Path(c["description"]).read_text().startswith("<?xml")
    merged = Path(cars["__catalog__"])
    assert merged == tmp_path / "state" / "vehicles" / "catalogs" / "vehicles.xosc"
    assert "my_lincoln" in merged.read_text() and "tesla_model3" in merged.read_text()
    assert vehicles.catalog_dir(tmp_path) == merged.parent


def test_own_car_args_per_kind():
    car = {"carla_sensors": "/s.yaml", "spec_hash_file": "/h.yaml", "spec": "/c.yaml",
           "description": "/d.urdf", "pedal_maps": {"dir": "/m", "fallback": True}}
    a = stack.own_car_args(car, "simple", "carla")
    assert a == {"vehicle_config": "/s.yaml", "autoware_spec_hash_file": "/h.yaml",
                 "spec_hash_file": "/h.yaml", "vehicle_maps_dir": "/m", "spec": "/c.yaml",
                 "description": "/d.urdf"}
    assert stack.own_car_args(car, "autoware", "carla") == {"vehicle_config": "/s.yaml"}


def test_project_check_names_an_autoware_pilot_without_the_cars_packages(tmp_path, monkeypatch):
    monkeypatch.setattr(vehicles, "installed_names", lambda catalog=None: {"tesla_model3": "vehicle.tesla.model3"})
    monkeypatch.setattr(projectmod, "_package_installed", lambda name: False)
    own_spec(tmp_path, bodies=True)
    s = {"simulator": "carla", "vehicles": ["vehicles/my_lincoln.yaml"],
         "pilots": {"ego": {"kind": "autoware", "vehicle": "my_lincoln"},
                    "lead": {"kind": "simple", "vehicle": "my_lincoln"}}}
    found = projectmod.own_car_problems(tmp_path, s)
    assert len(found) == 1 and "my_lincoln_description" in found[0].what and found[0].refused
    # on Isaac the stack makes the car's rig (roadmap 091 W5): the same one finding
    s["simulator"] = "isaac"
    found = projectmod.own_car_problems(tmp_path, s)
    assert len(found) == 1 and "my_lincoln_description" in found[0].what


def test_pedal_maps_beside_the_spec_else_the_teslas(tmp_path):
    spec = own_spec(tmp_path)
    m = vehicles.pedal_maps(spec, "my_lincoln")
    assert m["fallback"] is True
    d = spec.parent / "my_lincoln_maps"
    d.mkdir()
    (d / "accel_map.csv").write_text("x")
    (d / "brake_map.csv").write_text("x")
    assert vehicles.pedal_maps(spec, "my_lincoln") == {"dir": str(d), "fallback": False}


def test_the_body_follows_the_carla_version(tmp_path, monkeypatch):
    # roadmap 083: CARLA 1.0's Model 3 is another body, its catalog in a directory of its own
    assert vehicles.carla_body(None) is None
    assert vehicles.carla_body("1.0") == "carla-1.0"
    assert vehicles.carla_body("1.0-82c52a7c2") == "carla-1.0"  # a source build's version
    assert vehicles.carla_body("0.9.16") == "carla-0.9.16"
    cats = INSTALLED.parent
    assert vehicles.body_file(cats, "carla-1.0") == cats / "carla-1.0" / "vehicles.xosc"
    # a body with no catalog of its own, or none: the default
    assert vehicles.body_file(cats, "carla-0.9.16") == INSTALLED
    assert vehicles.body_file(cats, None) == INSTALLED
    one = (cats / "carla-1.0" / "vehicles.xosc").read_text()
    assert 'length="4.694"' in one and 'length="4.792"' in INSTALLED.read_text()
    # Autoware's car on a body it has no package for is the car itself
    assert vehicles.autoware_vehicle_model("tesla_model3", None) == "tesla_model3"
    assert vehicles.autoware_vehicle_model("no_such_car", "carla-1.0") == "no_such_car"


def test_the_merged_catalog_is_the_bodys(tmp_path, monkeypatch):
    monkeypatch.setattr(vehicles, "installed_catalog",
                        lambda body=None: vehicles.body_file(INSTALLED.parent, body))
    own_spec(tmp_path)
    cars = vehicles.generate(tmp_path, {"vehicles": ["vehicles/my_lincoln.yaml"]}, gen=fake_gen,
                             body="carla-1.0")
    text = Path(cars["__catalog__"]).read_text()
    assert 'length="4.694"' in text and "my_lincoln" in text
    # no own cars: the scenarios read the installed catalog of the body
    assert vehicles.catalog_dir(tmp_path / "empty", "carla-1.0") == INSTALLED.parent / "carla-1.0"


def test_on_isaac_each_car_gets_its_rig(tmp_path, monkeypatch):
    """Roadmap 091 W5: an Isaac testbench's own car is made into an aib rig beside its other
    files; a CARLA testbench's is not."""
    monkeypatch.setattr(vehicles, "installed_catalog", lambda body=None: INSTALLED)
    own_spec(tmp_path)
    made = []
    rig_gen = lambda spec, rig: made.append((Path(spec).name, rig)) or rig.write_text("x: 1\n")  # noqa: E731
    s = {"vehicles": ["vehicles/my_lincoln.yaml"]}
    assert vehicles.generate(tmp_path, s, gen=fake_gen, rig_gen=rig_gen)["my_lincoln"][
        "isaac_rig"] is None and made == []
    c = vehicles.generate(tmp_path, {**s, "simulator": "isaac"}, gen=fake_gen,
                          rig_gen=rig_gen)["my_lincoln"]
    rig = tmp_path / "state" / "vehicles" / "my_lincoln" / "isaac" / "my_lincoln.rig.yaml"
    assert c["isaac_rig"] == str(rig) and made == [("my_lincoln.yaml", rig)]


def test_the_rig_is_aibs_from_the_spec(tmp_path):
    """The real generator (aib.gen --spec), where aib imports: a rig naming the spec and its
    hash, with the PhysX vehicle's fields."""
    import importlib.util
    if importlib.util.find_spec("aib") is None:
        pytest.skip("aib not importable (scensim's workspace not sourced)")
    spec = own_spec(tmp_path)
    rig = tmp_path / "isaac" / "my_lincoln.rig.yaml"
    rig.parent.mkdir()
    vehicles.run_rig_gen(spec, rig)
    doc = yaml.safe_load(rig.read_text())
    assert (rig.parent / doc["spec"]).resolve() == spec.resolve()
    assert doc["spec_hash"] and doc["vehicle"]["wheel_base"] > 0


def test_what_carla_does_not_apply_is_read_from_the_cars_file(tmp_path):
    # roadmap 091 W24: sim_vehicle_spec names what CARLA cannot apply in the file's not_applied
    carla = tmp_path / "carla"
    carla.mkdir()
    f = carla / "carla_physics.yaml"
    f.write_text("not_applied: ['body.contact: CARLA has no per-actor physics material']\n")
    (carla / "carla_physics_1.0.yaml").write_text(
        "not_applied: ['suspension (frequency): the asset''s', 'body.contact: none']\n")
    assert vehicles.not_applied(f) == ["body.contact: CARLA has no per-actor physics material"]
    assert vehicles.not_applied(f, "0.9.16") == vehicles.not_applied(f)
    assert vehicles.not_applied_fields(vehicles.not_applied(f, "1.0-abc")) == [
        "suspension (frequency)", "body.contact"]
    f.write_text("vehicle: {}\n")
    assert vehicles.not_applied(f) == []
    assert vehicles.not_applied(None) == vehicles.not_applied("none") == []
    assert vehicles.not_applied(tmp_path / "missing.yaml") == []


def test_check_notes_a_contact_carla_does_not_apply(tmp_path):
    p = own_spec(tmp_path)
    s = {"simulator": "carla", "vehicles": ["vehicles/my_lincoln.yaml"]}
    assert projectmod.not_applied_notes(tmp_path, s) == []
    doc = yaml.safe_load(p.read_text())
    doc["body"]["contact"] = {"restitution": 0.3}
    p.write_text(yaml.safe_dump(doc, sort_keys=False))
    notes = projectmod.not_applied_notes(tmp_path, s)
    assert len(notes) == 1 and "my_lincoln" in notes[0] and "body.contact" in notes[0]
    s["simulator"] = "isaac"  # Isaac applies it
    assert projectmod.not_applied_notes(tmp_path, s) == []
