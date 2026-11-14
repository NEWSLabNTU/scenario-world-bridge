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
