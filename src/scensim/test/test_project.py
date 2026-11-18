from pathlib import Path

import pytest

from scensim import assets, project
from scensim.config import ConfigError

EXAMPLES = Path(__file__).resolve().parents[2] / "scensim_examples" / "scenarios"

DRIVE = (project.TEMPLATES / "carla-first-drive" / "scenarios" / "drive_to_goal.xosc").read_text()


def whats(findings):
    return [f.what for f in findings]


@pytest.mark.parametrize("template", ["carla-first-drive", "multi-pilot", "isaac"])
def test_templates_check_clean_after_fetch(host, maps_hub, template, capsys):
    d = host["tmp"] / "p"
    assert project.main(["new", str(d), "--template", template]) == 0
    assert (d / ".gitignore").read_text().split()[-5:] == ["assets/", "state/", "runs", ".scensim/", "play_log/"]
    assert "@PROJECT@" not in (d / "README.md").read_text()
    assert "# p:" in (d / "testbench.yaml").read_text()
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
    assert (tmp_path / "empty" / "testbench.yaml").is_file()


def make(tmp, testbench_yaml, scenarios=None, maps=("Town01",)):
    d = tmp / "proj"
    (d / "scenarios").mkdir(parents=True, exist_ok=True)
    (d / "testbench.yaml").write_text(testbench_yaml)
    for m in maps:
        (d / "assets" / "maps" / m).mkdir(parents=True, exist_ok=True)
    for name, text in (scenarios or {"drive.xosc": DRIVE}).items():
        (d / "scenarios" / name).parent.mkdir(parents=True, exist_ok=True)
        (d / "scenarios" / name).write_text(text)
    return d


def scenario_checks(d):
    """Findings other than the assets' (the tests' maps are plain directories)."""
    return [f for f in project.check_project(d) if not f.file.startswith(("scensim.lock", "assets/"))]


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
    assert str(f).startswith("problem: testbench.yaml: unknown key pilot (fix: use only simulator,")


def test_required(host):
    d = make(host["tmp"], "simulator: carla\n")
    w = whats(scenario_checks(d))
    assert "pilots: is required" in w and "assets: is required" in w


def test_carla_basic_is_carla_only(host):
    d = make(host["tmp"], "simulator: isaac\nassets: []\npilots:\n  ego: {kind: carla_basic}\n")
    assert any("runs on CARLA only" in x for x in whats(scenario_checks(d)))


def test_old_map_variable(host):
    d = make(host["tmp"], BASE, {"a.xosc": DRIVE.replace("SCENSIM_MAPS", "CARLA_MAPS")})
    [f] = scenario_checks(d)
    assert f.file == "scenarios/a.xosc"
    assert "does not name a project map" in f.what and "$(env SCENSIM_MAPS)/Town01" in f.fix
    assert "CARLA_MAPS is the developers' map directory" in f.what


def test_the_old_names_are_refused_naming_their_successors(host):
    """roadmap 080: $(env SWB_MAPS) and the swb.* scenario properties name scensim's."""
    old = DRIVE.replace("SCENSIM_MAPS", "SWB_MAPS").replace(
        '<Property name="maxJerk"', '<Property name="swb.zone.pit.class" value="gravel" />'
        '<Property name="maxJerk"', 1)
    d = make(host["tmp"], BASE, {"a.xosc": old})
    fs = scenario_checks(d)
    w = whats(fs)
    assert any("SWB_MAPS was renamed SCENSIM_MAPS" in x for x in w), w
    [f] = [f for f in fs if "property" in f.what]
    assert f.refused and "swb.zone.pit.class was renamed" in f.what
    assert f.fix == 'name="scensim.zone.pit.class"'


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


def test_kind_none_on_an_autoware_controller_is_refused(host, capsys):
    """G34: kind none starts no vehicle side, which SSv2's Autoware ego waits for."""
    d = make(host["tmp"], "simulator: carla\nassets: []\npilots: {ego: {kind: none}}\n")
    [f] = scenario_checks(d)
    assert f.refused and f.file == "scenarios/drive.xosc"
    assert f.what.startswith("entity ego (controller Autoware) is bound to pilot ego of kind"
                             " none, which starts no vehicle side")
    assert "pilots: {ego: {kind: autoware}}" in f.fix and 'Controller name="agent"' in f.fix
    assert project.main(["check", str(d)]) == 3
    assert "entity ego (controller Autoware)" in capsys.readouterr().out
    # a pattern binds it too; the finding names the pilot's key
    (d / "testbench.yaml").write_text("simulator: carla\nassets: []\npilots: {'*': {kind: none}}\n")
    [f] = scenario_checks(d)
    assert "bound to pilot * of kind none" in f.what
    # the same entity under controller agent: kind none is the user's own process, fine
    (d / "testbench.yaml").write_text("simulator: carla\nassets: []\npilots: {ego: {kind: none}}\n")
    (d / "scenarios" / "drive.xosc").write_text(DRIVE.replace('name="Autoware"', 'name="agent"'))
    assert scenario_checks(d) == []
    # managed_ego: false: SSv2 hands its ego to nobody, so nothing waits for a vehicle side
    (d / "scenarios" / "drive.xosc").write_text(DRIVE)
    (d / "testbench.yaml").write_text("simulator: carla\nassets: []\npilots: {ego: {kind: none}}\n"
                                "ssv2: {managed_ego: false}\n")
    assert scenario_checks(d) == []


TWO_AV = (EXAMPLES / "multi_av" / "town01_two_av.xosc")


@pytest.mark.skipif(not TWO_AV.is_file(), reason="scensim_examples not here")
def test_patterns_and_isaac_limit(host):
    text = TWO_AV.read_text()
    pilots = "pilots:\n  ego: {kind: autoware}\n  bg_av_*: {kind: autoware}\n"
    d = make(host["tmp"], "simulator: carla\nassets: []\n" + pilots, {"two.xosc": text})
    assert scenario_checks(d) == []
    (d / "testbench.yaml").write_text("simulator: isaac\nassets: []\n" + pilots)
    [f] = scenario_checks(d)
    assert "2 externally driven entities (ego, bg_av_1); Isaac drives one" == f.what


AUTOPILOT = (EXAMPLES / "basic" / "town01_simulator_autopilot.xosc")


@pytest.mark.skipif(not AUTOPILOT.is_file(), reason="scensim_examples not here")
def test_simulator_autopilot_is_refused_on_isaac(host):
    """swb 241 (T3 on Isaac, G66): check passed it, and the run ended in SSv2's SemanticError."""
    text = AUTOPILOT.read_text()
    d = make(host["tmp"], BASE, {"autopilot.xosc": text})
    assert not [f for f in scenario_checks(d) if "simulator_autopilot" in f.what]
    (d / "testbench.yaml").write_text(BASE.replace("simulator: carla", "simulator: isaac"))
    f = next(f for f in scenario_checks(d) if "simulator_autopilot" in f.what)
    assert f.refused and "runs on CARLA only, the simulator is isaac" in f.what


def test_pilot_car_must_be_the_scenario_car(host):
    d = make(host["tmp"], BASE.replace("tesla_model3", "golf_cart"))
    [f] = [f for f in scenario_checks(d) if not f.host]   # its car's packages: the host's
    assert f.what == "entity ego is a tesla_model3, its pilot ego drives a golf_cart"


def test_bad_xml_and_wrong_root(host):
    d = make(host["tmp"], BASE, {"a.xosc": "<OpenSCENARIO>", "b.xosc": "<Other/>"})
    w = whats(scenario_checks(d))
    assert any(x.startswith("not valid XML") for x in w)
    assert "the root element is <Other>, not <OpenSCENARIO>" in w


def test_yaml_scenario_with_is_ego(host):
    text = """OpenSCENARIO:
  RoadNetwork:
    LogicFile: {filepath: $(env SCENSIM_MAPS)/Town01}
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


@pytest.mark.skipif(not EXAMPLES.is_dir(), reason="scensim_examples not here")
def test_every_example_scenario_parses():
    files = project._scenarios_in(EXAMPLES)
    assert len(files) > 10
    for f in files:
        sc = project.read_scenario(f)
        # every example names its map as a project's scenarios do (roadmap 076 step 10)
        assert sc.logic_file and sc.logic_file.startswith(project.MAPS_PREFIX), f
    two = project.read_scenario(TWO_AV)
    assert [e.name for e in two.entities if e.external] == ["ego", "bg_av_1"]
    awf = next(f for f in files if f.suffix == ".yaml")
    assert [e.name for e in project.read_scenario(awf).entities if e.external] == ["ego"]


# --- the example projects (src/scensim_examples/projects) and `new --from` -----------------------

PROJECTS = EXAMPLES.parent / "projects"
EXAMPLE_PROJECTS = ["first_drive", "multi_pilot", "safety_suite", "isaac_first_drive",
                    "road_physics", "simple_drive"]
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


@pytest.mark.skipif(not PROJECTS.is_dir(), reason="scensim_examples not here")
def test_every_example_project_is_tested():
    assert sorted(p.name for p in PROJECTS.iterdir() if p.is_dir()) == sorted(EXAMPLE_PROJECTS)


@pytest.mark.skipif(not PROJECTS.is_dir(), reason="scensim_examples not here")
@pytest.mark.parametrize("name", EXAMPLE_PROJECTS)
def test_example_projects_check_clean_after_fetch(host, examples_hub, name):
    src = PROJECTS / name
    assert not (src / "scensim.lock").exists(), "the examples ship without a lock"
    d = host["tmp"] / name
    assert project.main(["new", str(d), "--from", str(src)]) == 0
    assert (d / ".gitignore").read_text().split()[-5:] == ["assets/", "state/", "runs", ".scensim/", "play_log/"]
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
    assert (d / "scensim.lock").is_file()
    assert project.check_project(d) == []


def test_new_from_copies_a_project_without_host_state(tmp_path):
    src = tmp_path / "src"
    for rel, text in {"testbench.yaml": BASE, ".gitignore": "assets/\n", "scensim.lock": "version: 1\n",
                      "scenarios/drive.xosc": DRIVE, "pilots/me.py": "print(1)\n",
                      "assets/maps/Town01/x": "", "state/Town01/y": "",
                      "runs/r/result.json": "", ".scensim/local.yaml": "seat: 3\n",
                      "pilots/__pycache__/me.pyc": ""}.items():
        (src / rel).parent.mkdir(parents=True, exist_ok=True)
        (src / rel).write_text(text)
    (src / "pilots" / "me.py").chmod(0o555)  # read-only and executable, as installed
    dest = project.copy_project(src, tmp_path / "copy")
    got = sorted(p.relative_to(dest).as_posix() for p in dest.rglob("*") if p.is_file())
    assert got == [".gitignore", "pilots/me.py", "scenarios/drive.xosc", "scensim.lock", "testbench.yaml"]
    mode = (dest / "pilots" / "me.py").stat().st_mode
    assert mode & 0o200 and mode & 0o100  # the user's own: writable, still executable


def test_new_from_follows_links(tmp_path):
    real = tmp_path / "real"
    real.mkdir()
    (real / "testbench.yaml").write_text(BASE)
    src = tmp_path / "share"
    src.mkdir()
    (src / "testbench.yaml").symlink_to(real / "testbench.yaml")  # a --symlink-install share
    dest = project.copy_project(src, tmp_path / "copy")
    assert not (dest / "testbench.yaml").is_symlink()
    assert (dest / "testbench.yaml").read_text() == BASE


def test_new_from_refuses(tmp_path):
    src = tmp_path / "src"
    src.mkdir()
    (src / "testbench.yaml").write_text(BASE)
    with pytest.raises(ConfigError, match="no testbench.yaml"):
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
    share = tmp_path / "share" / "scensim_examples"
    (share / "projects" / "first_drive").mkdir(parents=True)
    (share / "projects" / "first_drive" / "testbench.yaml").write_text(BASE)
    fake = types.ModuleType("ament_index_python.packages")
    fake.get_package_share_directory = lambda pkg: str(tmp_path / "share" / pkg)
    monkeypatch.setitem(sys.modules, "ament_index_python", types.ModuleType("ament_index_python"))
    monkeypatch.setitem(sys.modules, "ament_index_python.packages", fake)
    dest = tmp_path / "mine"
    assert project.main(["new", str(dest), "--from",
                         "pkg://scensim_examples/projects/first_drive"]) == 0
    assert (dest / "testbench.yaml").read_text() == BASE
    with pytest.raises(ConfigError, match="installs no testbench.yaml"):
        project.source_project("pkg://scensim_examples/projects/nope")


def test_variants_counted(tmp_path):
    y = tmp_path / "t.yaml"
    y.write_text("ScenarioModifiers:\n  ScenarioModifier:\n"
                 "    - {name: a, list: [1, 2, 3]}\n"
                 "    - {name: b, start: 0, step: 5, stop: 10}\n"
                 "OpenSCENARIO:\n  FileHeader: {}\n")
    assert project.read_scenario(y).variants == 9
    x = tmp_path / "d.xosc"
    x.write_text('<OpenSCENARIO><ParameterValueDistribution><ScenarioFile filepath="s.xosc"/>'
                 '<Deterministic><DeterministicSingleParameterDistribution parameterName="v">'
                 '<DistributionSet><Element value="1"/><Element value="2"/></DistributionSet>'
                 '</DeterministicSingleParameterDistribution>'
                 '<DeterministicSingleParameterDistribution parameterName="w">'
                 '<DistributionRange stepWidth="1"><Range lowerLimit="0" upperLimit="2"/>'
                 '</DistributionRange></DeterministicSingleParameterDistribution>'
                 '</Deterministic></ParameterValueDistribution></OpenSCENARIO>')
    assert project.read_scenario(x).variants == 6
    awf = (Path(__file__).resolve().parents[2] / "scensim_examples" / "projects" / "safety_suite"
           / "scenarios" / "awf")
    for f in awf.glob("UC-ACC*.yaml"):
        assert project.read_scenario(f).variants == 27


def test_ego_geometry_against_the_catalog(tmp_path):
    import xml.etree.ElementTree as ET
    cat = tmp_path / "vehicles.xosc"
    cat.write_text('<OpenSCENARIO><Catalog name="vehicle_catalog"><Vehicle name="tesla_model3">'
                   '<BoundingBox><Dimensions width="2.163" height="1.488" length="4.792"/>'
                   '</BoundingBox><Axles><FrontAxle positionX="3.005" trackWidth="1.667" '
                   'wheelDiameter="0.74"/><RearAxle positionX="0" trackWidth="1.667" '
                   'wheelDiameter="0.74"/></Axles></Vehicle></Catalog></OpenSCENARIO>')
    good = project.catalog_geometry("tesla_model3", cat)
    assert good["front_x"] == 3.005 and project.catalog_geometry("nope", cat) is None
    sample = ET.fromstring('<Vehicle><BoundingBox><Dimensions length="$l" width="2.163"/>'
                           '</BoundingBox><Axles><FrontAxle positionX="1.385" '
                           'trackWidth="1.63" wheelDiameter="0.78"/></Axles></Vehicle>')
    declared = project.vehicle_geometry(sample)
    assert "length" not in declared  # a parameter: not compared
    bad = project.geometry_problems(declared, good)
    assert bad == ["front_track 1.63, the catalog's 1.667", "front_wheel_diameter 0.78, the "
                   "catalog's 0.74", "front_x 1.385, the catalog's 3.005"]
    assert project.geometry_problems(project.vehicle_geometry(ET.fromstring(
        '<Vehicle><Axles><FrontAxle positionX="3.004"/></Axles></Vehicle>')), good) == []


def test_logging_keys_not_built_are_refused_by_name():
    f = project._check_logging("testbench.yaml", "logging", {"levels": {"bridge": "info"},
                                                       "debug": ["bridge"], "capture": "full"})
    assert len(f) == 2 and "not built yet" in str(f[0]) and "not built yet" in str(f[1])


# --- roadmap 077 gaps: world, isaac, ssv2, gap trace, the profile's arguments ---------------

def test_world_isaac_ssv2_keys_checked(host):
    d = make(host["tmp"], BASE + "world: {weather: ClearNoon, npc_motion: fast, state_dir: /x,"
             " config_file: gone.yaml}\nisaac: {render: headless}\n"
             "ssv2: {frame_rate: 0, managed_ego: maybe}\n"
             "logging: {gap_trace: yes please}\n"
             "judges: [{name: g, run: [x], needs: [video]}]\n")
    w = whats(scenario_checks(d))
    assert "world.npc_motion: 'fast' is not one of tracked, kinematic" in w
    assert any(x.startswith("world: state_dir is scensim's") for x in w)
    assert "world.config_file: gone.yaml: no such file" in w
    assert "isaac: is for simulator: isaac, this project's is carla" in w
    assert "ssv2.frame_rate: 0 is not a positive number" in w
    assert "ssv2.managed_ego: 'maybe' is not true or false" in w
    assert "logging.gap_trace: 'yes please' is not true or false" in w
    assert "judges[0]: needs: a list of gap_trace, context" in w
    clean = make(host["tmp"], BASE + "world: {weather: {preset: ClearNoon, cloudiness: 30}}\n"
                 "ssv2: {global_timeout: 900, frame_rate: 100}\nlogging: {gap_trace: true}\n")
    assert scenario_checks(clean) == []


def test_suite_items_and_managed_ego(host):
    """A suite's scenario may carry its own run settings; with managed_ego: false its ego
    needs no pilot."""
    d = make(host["tmp"], "simulator: carla\nassets: []\npilots: {other: {kind: none}}\n")
    (d / "suites").mkdir()
    (d / "suites" / "s.yaml").write_text(
        "ssv2: {global_timeout: 600}\n"
        "scenarios: [{path: scenarios/drive.xosc, ssv2: {managed_ego: false}}]\n")
    assert scenario_checks(d) == []
    assert project.unmanaged_scenarios(d, project.config.project_settings(d)) == {
        (d / "scenarios" / "drive.xosc").resolve()}
    # managed (the default): the ego needs its pilot
    (d / "suites" / "s.yaml").write_text("scenarios: [{path: scenarios/drive.xosc}]\n")
    assert whats(scenario_checks(d)) == ["entity ego (controller Autoware) has no pilot"]
    # testbench.yaml's ssv2 covers every scenario
    (d / "testbench.yaml").write_text("simulator: carla\nassets: []\npilots: {other: {kind: none}}\n"
                                "ssv2: {managed_ego: false}\n")
    assert scenario_checks(d) == []
    (d / "suites" / "s.yaml").write_text(
        "scenarios: [{file: scenarios/drive.xosc}, {path: scenarios/drive.xosc, "
        "ssv2: {frame_rate: fast}}]\nlogging: {gap_trace: true}\n")
    w = whats(scenario_checks(d))
    assert "scenarios[0]: unknown key file (use only path, ssv2)" in w
    assert "scenarios[0]: path: is required (a file or a directory)" in w
    assert "scenarios[1].ssv2.frame_rate: 'fast' is not a positive number" in w
    assert any(x.startswith("logging.gap_trace: the stack records the gap trace") for x in w)


PROFILE = (Path(__file__).resolve().parents[2] / "autoware_sim_vehicle" / "autoware_sim_profile"
           / "scripts" / "check_launch_args")


@pytest.mark.skipif(not PROFILE.is_file(), reason="autoware_sim_vehicle not checked out")
def test_profile_arguments_refused_at_check(host):
    """G12: what the Autoware profile refuses at start is refused by project check."""
    d = make(host["tmp"], "simulator: carla\nassets: []\npilots:\n"
             "  ego: {kind: autoware, args: {carla_control: false, ndt: '{param:ndt}'}}\n")
    w = whats(scenario_checks(d))
    assert w == ["pilots.ego.args: carla_control:= was renamed; pass sim_control:=false instead"]
    (d / "testbench.yaml").write_text("simulator: carla\nassets: []\npilots:\n"
                                "  ego: {kind: autoware, args: {ndt: cuda}}\n")
    (d / "suites").mkdir()
    (d / "suites" / "s.yaml").write_text(
        "scenarios: [scenarios/drive.xosc]\n"
        "pilot_args: {ego: {localization_error_monitor_param_path: /x.yaml}, other: {ndt: x}}\n")
    [f] = scenario_checks(d)
    assert f.file == "suites/s.yaml"
    assert f.what.startswith("pilot_args.ego: localization_error_monitor_param_path:= does not"
                             " reach ndt:=cuda")


@pytest.mark.skipif(not AUTOPILOT.is_file(), reason="scensim_examples not here")
def test_a_carla_only_scenario_refuses_its_runs_not_the_stack(host):
    """swb 241 (G66): with autopilot.xosc in an Isaac project, the other scenarios still run."""
    from scensim.project import check_stack, scenario_problems
    d = make(host["tmp"], BASE.replace("simulator: carla", "simulator: isaac"),
             {"autopilot.xosc": AUTOPILOT.read_text()})
    assert not [f for f in check_stack(d) if "simulator_autopilot" in f.what]
    found = scenario_problems(d, [d / "scenarios" / "autopilot.xosc"])
    assert any("simulator_autopilot" in f.what for f in found)


# --- issue 051 (077 G84): SSv2's frame against Isaac's physics step -----------------------------

ISAAC = "simulator: isaac\nassets: []\npilots:\n  ego: {kind: simple}\n"


def test_an_isaac_frame_rate_that_does_not_divide_physics_hz_is_refused(host):
    from scensim import world
    d = make(host["tmp"], ISAAC + "ssv2: {frame_rate: 100}\n")
    [why] = [w for w in whats(scenario_checks(d)) if "isaac.physics_hz" in w]
    assert why.startswith("ssv2.frame_rate 100 is not a whole number of isaac.physics_hz 60 "
                          "steps per frame")
    assert "60, 30, 20, 15, 12, 10, 6, 5, 4, 3, 2, 1" in why
    assert "a physics_hz that is a multiple of 100" in why
    # what fits: SSv2's default 20 on the default 60; 100 on 100, 200 or 300
    for fits in ("", "ssv2: {frame_rate: 20}\n", "ssv2: {frame_rate: 100}\nisaac: {physics_hz: 300}\n",
                 "ssv2: {frame_rate: 30}\nisaac: {physics_hz: 240}\n"):
        d = make(host["tmp"], ISAAC + fits)
        assert not [w for w in whats(scenario_checks(d)) if "isaac.physics_hz" in w], fits
    # CARLA has no such rule
    assert world.isaac_frame_problem({"simulator": "carla", "ssv2": {"frame_rate": 100}}) is None


def test_a_suites_frame_rate_is_judged_against_the_projects_physics():
    from scensim import world
    settings = {"simulator": "isaac", "isaac": {"physics_hz": 100}}
    assert world.isaac_frame_problem(settings, {"frame_rate": 100}) is None
    assert world.isaac_frame_problem(settings, {"frame_rate": 50}) is None
    why = world.isaac_frame_problem(settings, {"frame_rate": 40})
    assert why and "isaac.physics_hz 100" in why and "100, 50, 25, 20" in why
    assert world.frame_rate_problem(120, 60)   # shorter than one step
