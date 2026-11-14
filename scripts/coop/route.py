"""074: a lane's curvature at a point, from CARLA's waypoints (no ROS, no agent).

kappa > 0 where the lane turns toward increasing yaw, in the same frame as the cars' logged
yaw, so v^2 kappa is the lateral acceleration a car needs just to follow the lane (measures'
lateral comfort subtracts it). Smoothed over `span` m: the heading change between the points
span/2 behind and ahead, over the arc between them. Through a junction it follows the branch
nearest the lane's heading, as cutin_agent's tracking does.
"""
import math


def _yaw_off(wp, ref_yaw):
    """Degrees between a waypoint's heading and `ref_yaw`."""
    return abs((wp.transform.rotation.yaw - ref_yaw + 180.0) % 360.0 - 180.0)


def _walk(wp, dist, forward, step=1.0):
    """The waypoint `dist` m on (or back) along wp's lane, straight on through junctions."""
    ref = wp.transform.rotation.yaw
    left = dist
    while left > 1e-6:
        d = min(step, left)
        cands = wp.next(d) if forward else wp.previous(d)
        if not cands:
            return wp, dist - left
        wp = min(cands, key=lambda w: _yaw_off(w, ref))
        ref = wp.transform.rotation.yaw
        left -= d
    return wp, dist


def lane_kappa(wp, span=5.0):
    """The curvature (1/m) of wp's lane at wp, smoothed over `span` m."""
    a, da = _walk(wp, span / 2, True)
    b, db = _walk(wp, span / 2, False)
    s = da + db
    if s < 0.5:
        return 0.0
    dyaw = (a.transform.rotation.yaw - b.transform.rotation.yaw + 180.0) % 360.0 - 180.0
    return math.radians(dyaw) / s


def kappa_at(cmap, x, y, z=0.0, span=5.0):
    """The curvature of the driving lane nearest (x, y), CARLA's frame."""
    import carla

    wp = cmap.get_waypoint(carla.Location(x=x, y=y, z=z))
    return 0.0 if wp is None else lane_kappa(wp, span)


def straight(cmap, x, y, tol=3.0, step=2.0, max_len=800.0):
    """How far the lane at (x, y) runs straight ahead and behind (m): until its heading leaves
    the heading at (x, y) by more than `tol` degrees (a curve, or a lane shift)."""
    import carla

    wp0 = cmap.get_waypoint(carla.Location(x=x, y=y))
    ref = wp0.transform.rotation.yaw
    out = []
    for forward in (True, False):
        wp, s = wp0, 0.0
        while s < max_len:
            cands = wp.next(step) if forward else wp.previous(step)
            if not cands:
                break
            wp = min(cands, key=lambda w: _yaw_off(w, ref))
            if _yaw_off(wp, ref) > tol:
                break
            s += step
        out.append(s)
    return out[0], out[1]


def fit_on_straight(cmap, cars, margin=5.0):
    """cars: [(x, y, need)] in CARLA's frame, need the metres each must drive. Returns
    (shift, short): how far to move every start back along the road so that each car's need fits
    ahead of it (limited by the straight behind them), and by how much the worst one still falls
    short (0: it fits)."""
    ahead, behind = zip(*(straight(cmap, x, y) for x, y, _ in cars))
    want = max(0.0, max(need - a for (_, _, need), a in zip(cars, ahead)))
    room = max(0.0, min(behind) - margin)
    shift = min(want, room)
    return shift, want - shift
