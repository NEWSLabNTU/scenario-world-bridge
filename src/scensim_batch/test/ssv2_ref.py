"""SSv2's default vehicle behaviour for one vehicle, transcribed from the C++ (traffic_simulator's
longitudinal_speed_planning.cpp; behavior_tree_plugin's follow_lane_action.cpp and
follow_front_entity_action.cpp) in plain floats: the reference the batched form is tested
against. Only the speed logic: the distance to the front entity is an input."""

C = dict(max_acceleration=10.0, max_acceleration_rate=3.0, max_deceleration=10.0,
         max_deceleration_rate=3.0, max_speed=50.0)


def clamp(x, lo, hi):
    return min(max(x, lo), hi)


def plan_linear_jerk(target, v, a, c, dt):
    if v <= target:
        new = clamp(a + dt * c["max_acceleration_rate"], 0.0,
                    min(c["max_acceleration"], (target - v) / dt))
    else:
        new = clamp(a - dt * c["max_deceleration_rate"],
                    max(c["max_deceleration"] * -1, (target - v) / dt), 0.0)
    return (new - a) / dt


def dynamic_states(target, c, v, a, dt):
    if target > c["max_speed"]:
        target = c["max_speed"]
    jerk = plan_linear_jerk(target, v, a, c, dt)
    a1 = clamp(a + dt * jerk, c["max_deceleration"] * -1, c["max_acceleration"])
    v1 = clamp(v + a1 * dt, -1 * c["max_speed"], c["max_speed"])
    a1 = (v1 - v) / dt
    jerk = (a1 - a) / dt
    return v1, a1, jerk


def running_distance(target, c, v, a, jerk, dt):
    if abs(target - v) <= 0.01:
        return 0.0
    ret, state = 0.0, (v, a, jerk)
    while True:
        state = dynamic_states(target, c, state[0], state[1], dt)
        ret = ret + state[0] * dt + state[1] * dt * dt / 2.0 + state[2] * dt * dt * dt / 6.0
        if abs(target - state[0]) <= 0.01:
            return ret


def local_target(target, v, a, jerk, length, d_front, v_front, dt, c=C, see_around=True):
    """The speed the tree asks for this step: FollowLane's, or FollowFrontEntity's."""
    stop = running_distance(0.0, c, v, a, jerk, dt)
    if not see_around or d_front is None or d_front > stop + length + 5.0:
        return target
    if target <= v_front:
        return target
    stable = stop + length + 5.0
    requested = 0.0 if d_front <= stop else (v_front + 2.0 if d_front >= stable else v_front)
    return clamp(requested, 0.0, target)


def front_distance(ds, gap_len_self, len_other, corner_laterals, width_self):
    """splineDistanceToBoundingBox for a box ahead on a straight path: None when the bumper gap
    is negative; the gap plus half the own length when a corner is within half the own width
    of the path (distanceToSpline), or else when the path crosses the box
    (getCollisionPointIn2D: corners on both sides); else None."""
    gap = ds - gap_len_self / 2 - len_other / 2
    if gap < 0:
        return None
    if min(abs(d) for d in corner_laterals) <= width_self / 2:
        return gap + gap_len_self / 2
    if min(corner_laterals) <= 0 <= max(corner_laterals):
        return gap + gap_len_self / 2
    return None
