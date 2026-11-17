"""simple_pilot without a simulator: routing on synthetic lanelet maps (a straight road, a
curve), the map frame from lat/lon, the controller driving a kinematic car to the goal, the
agent's hooks and phases, and `scensim-agent check`. Skipped where python3 `lanelet2` is missing
(ROS 2 Humble's ros-humble-lanelet2), except the check, which needs no map."""

import math
import os
from pathlib import Path

import pytest

from scensim_agent import Episode
from scensim_agent import protocol as P
from scensim_agent.adapters import simple_route as R
from scensim_agent.adapters.simple_pilot import (DEFAULTS, Controller, FakeWorld, SimplePilotAgent,
                                             pose_dict, vehicle_geometry)
from scensim_agent.check import run_check

try:
    import lanelet2
    from lanelet2.core import AttributeMap, Lanelet, LaneletMap, LineString3d, Point3d, getId
except ImportError:  # pragma: no cover - no ROS environment
    lanelet2 = None
needs_lanelet2 = pytest.mark.skipif(lanelet2 is None, reason="python3 lanelet2 not installed")

WIDTH = 3.5
RADIUS = 30.0
TOWN01 = Path(os.environ.get("SCENSIM_TEST_TOWN01", "/home/aeon/repos/carla-scenario-bridge/data/"
                                                "carla-autoware-bridge/Town01"))


# --- a synthetic map: 0-50-100 m straight along x, then a 90 deg left curve of R 30 m ---------

def _pt(x, y, local=False):
    attrs = AttributeMap({"local_x": f"{x:.4f}", "local_y": f"{y:.4f}"} if local else {})
    return Point3d(getId(), x, y, 0.0, attrs)


def _centre(s_total, n):
    """(x, y, heading) along the road's centre: straight 100 m, then the curve."""
    out = []
    for i in range(n + 1):
        s = s_total[0] + (s_total[1] - s_total[0]) * i / n
        if s <= 100.0:
            out.append((s, 0.0, 0.0))
        else:
            a = (s - 100.0) / RADIUS
            out.append((100.0 + RADIUS * math.sin(a), RADIUS * (1 - math.cos(a)), a))
    return out


def _bounds(start, end, n, local):
    left, right = [], []
    for x, y, h in _centre((start, end), n):
        nx, ny = -math.sin(h), math.cos(h)
        left.append(_pt(x + nx * WIDTH / 2, y + ny * WIDTH / 2, local))
        right.append(_pt(x - nx * WIDTH / 2, y - ny * WIDTH / 2, local))
    return left, right


def synthetic_map(local=False, limits=(None, 30.0, None)):
    """Three lanelets in a row (0-50, 50-100, the curve to s = 100 + pi/2 R), joined at shared
    points; `limits` their speed_limit tags (km/h)."""
    sections = [(0.0, 50.0, 10), (50.0, 100.0, 10), (100.0, 100.0 + math.pi / 2 * RADIUS, 24)]
    lmap = LaneletMap()
    prev = None
    for (a, b, n), kmh in zip(sections, limits):
        left, right = _bounds(a, b, n, local)
        if prev is not None:  # share the join, as lanelet2's succession needs
            left[0], right[0] = prev
        prev = (left[-1], right[-1])
        attrs = {"type": "lanelet", "subtype": "road", "location": "urban",
                 "one_way": "yes"}
        if kmh is not None:
            attrs["speed_limit"] = f"{kmh:.1f}"
        ll = Lanelet(getId(), LineString3d(getId(), left, AttributeMap({"type": "line_thin",
                                                                       "subtype": "solid"})),
                     LineString3d(getId(), right, AttributeMap({"type": "line_thin",
                                                               "subtype": "solid"})),
                     AttributeMap(attrs))
        lmap.add(ll)
    return lmap


def router(**kw):
    return R.Router(*R.finish_map(synthetic_map(**kw)))


GOAL_END = (100.0 + RADIUS, RADIUS - 2.0, math.pi / 2)  # near the curve's end


# --- routing --------------------------------------------------------------------------------

@needs_lanelet2
def test_straight_route():
    path = router().route((5.0, 0.2, 0.0), [(90.0, 0.0, 0.0)])
    assert abs(path.end - 85.0) < 0.5
    assert all(abs(y) < 1e-6 for y in path.y)
    assert abs(path.point(0.0)[0] - 5.0) < 0.5
    assert path.limit[path.index(20.0)] == pytest.approx(30.0 / 3.6)  # the default
    assert path.limit[path.index(60.0)] == pytest.approx(30.0 / 3.6, rel=1e-3)  # tagged 30


@needs_lanelet2
def test_curved_route_and_curvature():
    path = router(limits=(50.0, 50.0, 50.0)).route((5.0, 0.0, 0.0), [GOAL_END])
    arc = math.pi / 2 * RADIUS
    assert abs(path.end - (95.0 + arc - 2.0)) < 1.5
    mid = path.index(95.0 + arc / 2)
    assert path.curvature[mid] == pytest.approx(1.0 / RADIUS, rel=0.1)  # left: positive
    assert abs(path.curvature[path.index(40.0)]) < 1e-3
    gx, gy = path.point(path.end)
    assert math.hypot(gx - GOAL_END[0], gy - GOAL_END[1]) < 0.5


@needs_lanelet2
def test_goal_against_the_lane_is_refused():
    with pytest.raises(R.RouteError, match="the goal"):
        router().route((5.0, 0.0, 0.0), [(90.0, 0.0, math.pi)])
    with pytest.raises(R.RouteError, match="the vehicle"):
        router().route((5.0, 30.0, 0.0), [(90.0, 0.0, 0.0)])


@needs_lanelet2
def test_goal_behind_on_a_dead_end_is_refused():
    with pytest.raises(R.RouteError):
        router().route((40.0, 0.0, 0.0), [(10.0, 0.0, 0.0)])


@needs_lanelet2
def test_waypoints():
    path = router().route((5.0, 0.0, 0.0), [(60.0, 0.0, 0.0), GOAL_END])
    assert path.end > 100.0


# --- the map frame --------------------------------------------------------------------------

def test_transverse_mercator_at_the_origin():
    tm = R.TransverseMercator()
    x, y = tm.project(0.0, 0.0)
    assert abs(x) < 1e-9 and abs(y) < 1e-9
    x, _ = tm.project(0.0, 0.003454829852879983)  # a Town01 node: 384.4 m east at k0 0.9996
    assert x == pytest.approx(0.003454829852879983 * math.pi / 180 * R.WGS84_A * 0.9996,
                              rel=1e-6)


def _osm(path, nodes):
    lines = ["<?xml version='1.0' encoding='UTF-8'?>", "<osm version='0.6'>"]
    for i, (lat, lon, local) in enumerate(nodes, 1):
        tags = "".join(f'<tag k="{k}" v="{v}"/>' for k, v in (local or {}).items())
        lines.append(f'<node id="{i}" lat="{lat}" lon="{lon}">{tags}</node>')
    lines.append("</osm>")
    path.write_text("\n".join(lines))


def test_positions_follow_the_projector(tmp_path):
    osm = tmp_path / "lanelet2_map.osm"
    _osm(osm, [(-0.001, 0.002, {"local_x": "1.0", "local_y": "2.0"}), (0.0, 0.0, None)])
    (tmp_path / "map_projector_info.yaml").write_text(
        "projector_type: TransverseMercator\nvertical_datum: WGS84\n"
        "map_origin: {latitude: 0.0, longitude: 0.0, altitude: 0.0}\n")
    pos = R.map_frame_positions(osm, R.read_projector_info(tmp_path))
    assert pos[1] == pytest.approx(R.TransverseMercator().project(-0.001, 0.002))
    assert pos[2] == pytest.approx((0.0, 0.0))
    (tmp_path / "map_projector_info.yaml").write_text("projector_type: Local\n")
    with pytest.raises(R.RouteError, match="local_x"):
        R.map_frame_positions(osm, R.read_projector_info(tmp_path))
    (tmp_path / "map_projector_info.yaml").write_text("projector_type: Cassini\n")
    with pytest.raises(R.RouteError, match="Cassini"):
        R.map_frame_positions(osm, R.read_projector_info(tmp_path))


def _write_local_map(folder):
    from lanelet2.io import Origin
    from lanelet2.projection import UtmProjector
    lanelet2.io.write(str(folder / "lanelet2_map.osm"), synthetic_map(local=True),
                      UtmProjector(Origin(0.0, 0.0)))
    (folder / "map_projector_info.yaml").write_text("projector_type: Local\n")
    return folder


@needs_lanelet2
def test_load_map_from_a_file(tmp_path):
    r = R.Router.load(_write_local_map(tmp_path))
    path = r.route((5.0, 0.0, 0.0), [GOAL_END])
    assert path.end > 120.0


@needs_lanelet2
@pytest.mark.skipif(not (TOWN01 / "lanelet2_map.osm").is_file(), reason="no Town01 map")
def test_town01_drive_to_goal():
    """first_drive's drive_to_goal: 190.8, -130.1 heading west, to 120.0, -129.8."""
    r = R.Router.load(TOWN01)
    path = r.route((190.8, -130.1, math.pi), [(120.0, -129.8, math.pi)])
    assert 65.0 < path.end < 76.0
    assert all(abs(y + 130.0) < 2.5 for y in path.y)
    # and a longer one, through Town01's junctions, driven by the fake car
    path = r.route((190.8, -130.1, math.pi), [(300.0, -2.0, 0.0)])
    ctl, world = controller_and_car(path, start=(190.8, -130.1, math.pi))
    trace = drive(ctl, world, 200.0)
    assert trace[-1][3].arrived
    st = world.state()
    assert math.hypot(st.x - 300.0, st.y + 2.0) < 1.5
    for x, y, _, _ in trace:
        px, py = path.point(path.locate(x, y, 0, len(path.s) - 1))
        assert math.hypot(px - x, py - y) < 1.2


# --- the controller -------------------------------------------------------------------------

def drive(ctl, world, seconds, dt=0.05):
    """Advance the fake car, the controller commanding it; the trace of (x, y, v, cmd)."""
    trace = []
    for _ in range(int(seconds / dt)):
        st = world.state()
        cmd = ctl.step(st.x, st.y, st.yaw, st.v, dt)
        world.publish(cmd)
        world.advance(dt)
        trace.append((st.x, st.y, st.v, cmd))
        if cmd.arrived:
            break
    return trace


def controller_and_car(path, start=(0.0, 0.0, 0.0), **cfg):
    config = {**DEFAULTS, **cfg}
    ctl = Controller(config, 3.0, 0.7)
    ctl.set_path(path)
    world = FakeWorld(config, 3.0, None, realtime=False)
    world.place(pose_dict(start[0], start[1], 0.0, start[2]))
    return ctl, world


def test_straight_drive_stops_at_the_goal():
    path = R.Path.from_points([(0.0, 0.0), (80.0, 0.0)], 10.0)
    ctl, world = controller_and_car(path, start=(0.0, 1.0, 0.1))  # off the line, askew
    trace = drive(ctl, world, 60.0)
    assert trace[-1][3].arrived
    st = world.state()
    assert abs(st.x - 80.0) < 1.0 and abs(st.y) < 0.2 and st.v < 0.3
    assert max(v for _, _, v, _ in trace) < 10.5
    assert max(c.velocity for *_, c in trace) <= 10.0 + 1e-9
    assert all(c.accel >= -DEFAULTS["brake_max"] - 1e-9 for *_, c in trace)


def test_speed_limit_is_obeyed():
    path = R.Path.from_points([(0.0, 0.0), (200.0, 0.0)], 15.0)
    ctl, world = controller_and_car(path)
    ctl.limit = 5.0
    trace = drive(ctl, world, 25.0)
    assert max(v for _, _, v, _ in trace) < 5.5
    assert trace[-1][2] > 4.0  # it does drive


@needs_lanelet2
def test_curve_is_followed_and_slowed_for():
    path = router(limits=(50.0, 50.0, 50.0)).route((0.5, 0.0, 0.0), [GOAL_END])
    ctl, world = controller_and_car(path, start=(0.5, 0.0, 0.0))
    trace = drive(ctl, world, 90.0)
    assert trace[-1][3].arrived
    st = world.state()
    assert math.hypot(st.x - GOAL_END[0], st.y - GOAL_END[1]) < 1.5
    v_curve = math.sqrt(DEFAULTS["lateral_accel"] * RADIUS)
    in_curve = [(x, y, v) for x, y, v, _ in trace if x > 105.0 and y > 2.0 and y < RADIUS - 5]
    assert in_curve and max(v for *_, v in in_curve) < v_curve + 0.7
    # never off the lane: the distance from the curve's centre stays R within half a lane
    for x, y, _ in in_curve:
        if x > 100.0:
            assert abs(math.hypot(x - 100.0, y - RADIUS) - RADIUS) < WIDTH / 2
    for x, y, _, _ in trace:
        if x < 95.0:
            assert abs(y) < 0.5


def test_stop_holds():
    path = R.Path.from_points([(0.0, 0.0), (200.0, 0.0)], 10.0)
    ctl, world = controller_and_car(path)
    drive(ctl, world, 8.0)
    ctl.clear()
    trace = drive(ctl, world, 10.0)
    assert world.state().v < 0.05
    assert all(c.velocity == 0.0 and c.accel < 0 for *_, c in trace)


# --- the agent ------------------------------------------------------------------------------

def test_passes_check_with_the_fake_world():
    report = run_check(SimplePilotAgent, {"world": "fake"}, 10.0)
    assert report.passed, report.text()


@needs_lanelet2
def test_agent_drives_to_arrived_on_a_map(tmp_path):
    agent = SimplePilotAgent({"world": "fake", "fake_realtime": False,
                              "map_path": str(_write_local_map(tmp_path))})
    try:
        world = agent.world
        world.advance(0.05)
        agent.on_reset(Episode(pose=pose_dict(5.0, 0.0, 0.0, 0.0)))
        assert agent.phase == P.IDLE
        agent.on_goal(pose_dict(*GOAL_END[:2], 0.0, GOAL_END[2]), [])
        assert agent.phase == P.DRIVING
        for _ in range(int(90 / 0.05)):
            world.advance(0.05)
            if agent.phase == P.ARRIVED:
                break
        assert agent.phase == P.ARRIVED
        st = world.state()
        assert math.hypot(st.x - GOAL_END[0], st.y - GOAL_END[1]) < 1.5
        assert world.commands > 100
        assert world.command.velocity == 0.0  # holding
    finally:
        agent.close()


@needs_lanelet2
def test_agent_refuses_a_goal_off_the_map(tmp_path):
    import scensim_agent
    agent = SimplePilotAgent({"world": "fake", "fake_realtime": False,
                              "map_path": str(_write_local_map(tmp_path))})
    try:
        agent.world.advance(0.05)
        agent.on_reset(Episode(pose=pose_dict(5.0, 0.0, 0.0, 0.0)))
        with pytest.raises(scensim_agent.Failed, match="no route"):
            agent.on_goal(pose_dict(500.0, 500.0, 0.0, 0.0), [])
    finally:
        agent.close()


def test_ros_world_needs_a_map_to_route():
    import scensim_agent
    agent = SimplePilotAgent({"world": "fake", "route": "lanelet", "fake_realtime": False})
    try:
        agent.world.advance(0.05)
        with pytest.raises(scensim_agent.Failed, match="map_path"):
            agent.on_goal(pose_dict(50.0, 0.0, 0.0, 0.0), [])
    finally:
        agent.close()


def test_vehicle_geometry_overrides_and_spec(tmp_path):
    spec = tmp_path / "car.yaml"
    spec.write_text("geometry: {wheel_base: 2.5, max_steer_angle: 0.6}\n")
    wb, steer, _ = vehicle_geometry({"spec": str(spec)})
    assert (wb, steer) == (2.5, 0.6)
    wb, steer, _ = vehicle_geometry({"spec": str(spec), "wheel_base": 2.8})
    assert (wb, steer) == (2.8, 0.6)
    wb, _, _ = vehicle_geometry({"vehicle": ""})
    assert wb == pytest.approx(3.005)
