from pathlib import Path

import pytest

from swb import assets, project
from swb.config import ConfigError

EXAMPLES = Path(__file__).resolve().parents[2] / "swb_examples" / "scenarios"

DRIVE = (project.TEMPLATES / "carla-first-drive" / "scenarios" / "drive_to_goal.xosc").read_text()


def whats(findings):
    return [f.what for f in findings]


@pytest.mark.parametrize("template", ["carla-first-drive", "multi-pilot", "isaac"])
def test_templates_check_clean_after_fetch(host, maps_hub, template, capsys):
    d = host["tmp"] / "p"
    assert project.main(["new", str(d), "--template", template]) == 0
    assert (d / ".gitignore").read_text().split()[-4:] == ["assets/", "state/", "runs/", ".swb/"]
    assert "@PROJECT@" not in (d / "README.md").read_text()
    assert "# p:" in (d / "swb.yaml").read_text()
    # before fetching: refused, the map not fetched
    assert project.main(["check", str(d)]) == 3
    assets.fetch(d, out=lambda s: None)
    assert project.check_project(d) == []
    assert project.main(["check", f"project:={d}"]) == 0


def test_templates_listed():
    assert {"carla-first-drive", "multi-pilot", "isaac"} <= set(project.templates())


def test_new_refuses_non_empty_and_unknown(tmp_path):
    (tmp_path / "x").mkdir()
    (tmp_path / "x" / "f").write_text("")
    assert project.main(["new", str(tmp_path / "x")]) == 3
    with pytest.raises(ConfigError, match="no template"):
        project.new_project(tmp_path / "y", "nope")
    project.new_project(tmp_path / "empty")  # an empty directory is fine
    assert (tmp_path / "empty" / "swb.yaml").is_file()


def make(tmp, swb_yaml, scenarios=None, maps=("Town01",)):
    d = tmp / "proj"
    (d / "scenarios").mkdir(parents=True, exist_ok=True)
    (d / "swb.yaml").write_text(swb_yaml)
    for m in maps:
        (d / "assets" / "maps" / m).mkdir(parents=True, exist_ok=True)
    for name, text in (scenarios or {"drive.xosc": DRIVE}).items():
        (d / "scenarios" / name).parent.mkdir(parents=True, exist_ok=True)
        (d / "scenarios" / name).write_text(text)
    return d


def scenario_checks(d):
    """Findings other than the assets' (the tests' maps are plain directories)."""
    return [f for f in project.check_project(d) if not f.file.startswith(("swb.lock", "assets/"))]


BASE = "simulator: carla\nassets: []\npilots:\n  ego: {kind: autoware, vehicle: tesla_model3}\n"


def test_clean(host):
    assert scenario_checks(make(host["tmp"], BASE)) == []


def test_schema(host):
    d = make(host["tmp"], """simulator: carla
assets: []
pilot: {}
pilots:
  ego: {kind: autowar}
  b: {kind: carla_basic, run: [x]}
  c: {kind: command}
  e: {kind: container}
  f: {kind: simple, lifetime: forever, inputs: [lidar]}
logging: {capture: lots, keep: {passed: result, flaky: full}}
""")
    w = whats(scenario_checks(d))
    assert "unknown key pilot" in w
    assert "pilots.ego: unknown kind autowar" in w
    assert "pilots.b: unknown key run for kind carla_basic" in w
    assert "pilots.c: kind command needs run:" in w
    assert "pilots.e: kind container is not available yet" in w
    assert "pilots.f: lifetime forever is not stack or run" in w
    assert any(x.startswith("pilots.f: inputs ['lidar']") for x in w)
    assert "logging.capture: lots is not a level" in w
    assert "logging.keep: unknown verdict flaky" in w
    # the ego's entity now has no valid pilot kind, but it matches a key
    f = [x for x in project.check_project(d) if x.what == "unknown key pilot"][0]
    assert str(f).startswith("problem: swb.yaml: unknown key pilot (fix: use only simulator,")


def test_required(host):
    d = make(host["tmp"], "simulator: carla\n")
    w = whats(scenario_checks(d))
    assert "pilots: is required" in w and "assets: is required" in w


def test_carla_basic_is_carla_only(host):
    d = make(host["tmp"], "simulator: isaac\nassets: []\npilots:\n  ego: {kind: carla_basic}\n")
    assert any("runs on CARLA only" in x for x in whats(scenario_checks(d)))


def test_old_map_variable(host):
    d = make(host["tmp"], BASE, {"a.xosc": DRIVE.replace("SWB_MAPS", "CARLA_MAPS")})
    [f] = scenario_checks(d)
    assert f.file == "scenarios/a.xosc"
    assert "does not name a project map" in f.what and "$(env SWB_MAPS)/Town01" in f.fix


def test_map_not_fetched_is_refused(host):
    d = make(host["tmp"], BASE, {"a.xosc": DRIVE.replace("Town01", "Town05")})
    [f] = scenario_checks(d)
    assert f.refused and "map Town05 is not fetched" in f.what
    assert "carla-town-autoware-maps" in f.fix


def test_entity_without_pilot(host):
    d = make(host["tmp"], "simulator: carla\nassets: []\npilots: {other: {kind: none}}\n")
    [f] = scenario_checks(d)
    assert f.what == "entity ego (controller Autoware) has no pilot"
    assert project.main(["check", str(d)]) == 1


TWO_AV = (EXAMPLES / "multi_av" / "town01_two_av.xosc")


@pytest.mark.skipif(not TWO_AV.is_file(), reason="swb_examples not here")
def test_patterns_and_isaac_limit(host):
    text = TWO_AV.read_text().replace("CARLA_MAPS", "SWB_MAPS")
    pilots = "pilots:\n  ego: {kind: autoware}\n  bg_av_*: {kind: autoware}\n"
    d = make(host["tmp"], "simulator: carla\nassets: []\n" + pilots, {"two.xosc": text})
    assert scenario_checks(d) == []
    (d / "swb.yaml").write_text("simulator: isaac\nassets: []\n" + pilots)
    [f] = scenario_checks(d)
    assert "2 externally driven entities (ego, bg_av_1); Isaac drives one" == f.what


def test_pilot_car_must_be_the_scenario_car(host):
    d = make(host["tmp"], BASE.replace("tesla_model3", "golf_cart"))
    [f] = scenario_checks(d)
    assert f.what == "entity ego is a tesla_model3, its pilot ego drives a golf_cart"


def test_bad_xml_and_wrong_root(host):
    d = make(host["tmp"], BASE, {"a.xosc": "<OpenSCENARIO>", "b.xosc": "<Other/>"})
    w = whats(scenario_checks(d))
    assert any(x.startswith("not valid XML") for x in w)
    assert "the root element is <Other>, not <OpenSCENARIO>" in w


def test_yaml_scenario_with_is_ego(host):
    text = """OpenSCENARIO:
  RoadNetwork:
    LogicFile: {filepath: $(env SWB_MAPS)/Town01}
  Entities:
    ScenarioObject:
      - name: ego
        ObjectController:
          Controller:
            name: ''
            Properties:
              Property:
                - {name: isEgo, value: 'true'}
      - name: npc
        ObjectController:
          Controller: {name: '', Properties: {Property: []}}
"""
    d = make(host["tmp"], "simulator: carla\nassets: []\npilots: {}\n",
             {"awf/s.yaml": text, "poses.yaml": "ego: [1, 2]\n"})
    [f] = scenario_checks(d)
    assert f.file == "scenarios/awf/s.yaml" and f.what == "entity ego (isEgo) has no pilot"


def test_suites(host):
    d = make(host["tmp"], BASE)
    (d / "suites").mkdir()
    (d / "more").mkdir()
    (d / "more" / "x.xosc").write_text(DRIVE.replace("Town01", "Town02"))
    (d / "suites" / "s.yaml").write_text(
        "scenarios: [scenarios/drive.xosc, more/, scenarios/gone.xosc]\nrepetition: 3\n")
    found = scenario_checks(d)
    assert {(f.file, f.what) for f in found} == {
        ("suites/s.yaml", "unknown key repetition"),
        ("suites/s.yaml", "scenario scenarios/gone.xosc: no such file or directory"),
        ("more/x.xosc", "map Town02 is not fetched (assets/maps/Town02)"),
    }


def test_no_project(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    [f] = project.check_project(None)
    assert f.refused and "project new" in f.what
    assert project.main(["check"]) == 3


def test_vehicles_listed_must_exist(host):
    d = make(host["tmp"], BASE + "vehicles: [vehicles/my_car.yaml]\n")
    assert whats(scenario_checks(d)) == ["vehicles: vehicles/my_car.yaml: no such file"]


@pytest.mark.skipif(not EXAMPLES.is_dir(), reason="swb_examples not here")
def test_every_example_scenario_parses():
    files = project._scenarios_in(EXAMPLES)
    assert len(files) > 10
    for f in files:
        sc = project.read_scenario(f)
        assert sc.logic_file, f
    two = project.read_scenario(TWO_AV)
    assert [e.name for e in two.entities if e.external] == ["ego", "bg_av_1"]
    awf = next(f for f in files if f.suffix == ".yaml")
    assert [e.name for e in project.read_scenario(awf).entities if e.external] == ["ego"]


# --- the example projects (src/swb_examples/projects) and `new --from` -----------------------

PROJECTS = EXAMPLES.parent / "projects"
EXAMPLE_PROJECTS = ["first_drive", "multi_pilot", "safety_suite", "isaac_first_drive",
                    "road_physics"]
CATALOG = {"tesla_model3", "drift_mustang"}  # sim_vehicle_spec's catalogs/vehicles.xosc


@pytest.fixture
def examples_hub(maps_hub):
    """maps_hub plus the road physics stages, as far as the examples take them."""
    maps_hub.commit("jerry73204/isaac-road-physics-stages",
                    {"procedural/spikes/road_town/road_detail.yaml": "corridors: []\n",
                     "procedural/spikes/road_town/surfaces.yaml": "zones: []\n",
                     "materials_ground.usda": "#usda 1.0\n", "README.md": "# stages\n"},
                    manifest=True)
    return maps_hub


@pytest.mark.skipif(not PROJECTS.is_dir(), reason="swb_examples not here")
def test_every_example_project_is_tested():
    assert sorted(p.name for p in PROJECTS.iterdir() if p.is_dir()) == sorted(EXAMPLE_PROJECTS)


@pytest.mark.skipif(not PROJECTS.is_dir(), reason="swb_examples not here")
@pytest.mark.parametrize("name", EXAMPLE_PROJECTS)
def test_example_projects_check_clean_after_fetch(host, examples_hub, name):
    src = PROJECTS / name
    assert not (src / "swb.lock").exists(), "the examples ship without a lock"
    d = host["tmp"] / name
    assert project.main(["new", str(d), "--from", str(src)]) == 0
    assert (d / ".gitignore").read_text().split()[-4:] == ["assets/", "state/", "runs/", ".swb/"]
    assert (d / "README.md").is_file()
    files = project._scenarios_in(d / "scenarios")
    assert files
    for f in files:
        sc = project.read_scenario(f)
        assert sc.logic_file.startswith(project.MAPS_PREFIX), f
        for e in sc.entities:
            assert e.catalog_entry in (None, *CATALOG), (f, e.name, e.catalog_entry)
    assert project.main(["check", str(d)]) == 3  # nothing fetched yet: refused
    assets.fetch(d, out=lambda s: None)
    assert (d / "swb.lock").is_file()
    assert project.check_project(d) == []


def test_new_from_copies_a_project_without_host_state(tmp_path):
    src = tmp_path / "src"
    for rel, text in {"swb.yaml": BASE, ".gitignore": "assets/\n", "swb.lock": "version: 1\n",
                      "scenarios/drive.xosc": DRIVE, "pilots/me.py": "print(1)\n",
                      "assets/maps/Town01/x": "", "state/Town01/y": "",
                      "runs/r/result.json": "", ".swb/local.yaml": "seat: 3\n",
                      "pilots/__pycache__/me.pyc": ""}.items():
        (src / rel).parent.mkdir(parents=True, exist_ok=True)
        (src / rel).write_text(text)
    (src / "pilots" / "me.py").chmod(0o555)  # read-only and executable, as installed
    dest = project.copy_project(src, tmp_path / "copy")
    got = sorted(p.relative_to(dest).as_posix() for p in dest.rglob("*") if p.is_file())
    assert got == [".gitignore", "pilots/me.py", "scenarios/drive.xosc", "swb.lock", "swb.yaml"]
    mode = (dest / "pilots" / "me.py").stat().st_mode
    assert mode & 0o200 and mode & 0o100  # the user's own: writable, still executable


def test_new_from_follows_links(tmp_path):
    real = tmp_path / "real"
    real.mkdir()
    (real / "swb.yaml").write_text(BASE)
    src = tmp_path / "share"
    src.mkdir()
    (src / "swb.yaml").symlink_to(real / "swb.yaml")  # a --symlink-install share
    dest = project.copy_project(src, tmp_path / "copy")
    assert not (dest / "swb.yaml").is_symlink()
    assert (dest / "swb.yaml").read_text() == BASE


def test_new_from_refuses(tmp_path):
    src = tmp_path / "src"
    src.mkdir()
    (src / "swb.yaml").write_text(BASE)
    with pytest.raises(ConfigError, match="no swb.yaml"):
        project.copy_project(tmp_path / "nothing", tmp_path / "a")
    (tmp_path / "full").mkdir()
    (tmp_path / "full" / "f").write_text("")
    assert project.main(["new", str(tmp_path / "full"), "--from", str(src)]) == 3
    with pytest.raises(ConfigError, match="inside the project"):
        project.copy_project(src, src / "sub")
    with pytest.raises(SystemExit):  # one or the other
        project.main(["new", str(tmp_path / "b"), "--from", str(src), "--template", "isaac"])


def test_new_from_an_installed_package(tmp_path, monkeypatch):
    import sys
    import types
    share = tmp_path / "share" / "swb_examples"
    (share / "projects" / "first_drive").mkdir(parents=True)
    (share / "projects" / "first_drive" / "swb.yaml").write_text(BASE)
    fake = types.ModuleType("ament_index_python.packages")
    fake.get_package_share_directory = lambda pkg: str(tmp_path / "share" / pkg)
    monkeypatch.setitem(sys.modules, "ament_index_python", types.ModuleType("ament_index_python"))
    monkeypatch.setitem(sys.modules, "ament_index_python.packages", fake)
    dest = tmp_path / "mine"
    assert project.main(["new", str(dest), "--from",
                         "pkg://swb_examples/projects/first_drive"]) == 0
    assert (dest / "swb.yaml").read_text() == BASE
    with pytest.raises(ConfigError, match="installs no swb.yaml"):
        project.source_project("pkg://swb_examples/projects/nope")
