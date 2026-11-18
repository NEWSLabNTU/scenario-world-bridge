"""Issue 053: the agent protocol sends a non-finite float as null, and a key present with null
defeats ``dict.get``'s default. Autoware 2.1.0's rtc_interface publishes non-finite start and
finish distances, so the relay died on ``float(None)`` in its RTC status."""
import math

import pytest

from scenario_agent_relay import mapping as M
from scensim_agent import protocol as P


def test_given_takes_the_default_for_an_absent_or_null_key_only():
    assert M.given({}, "a", 1) == 1
    assert M.given({"a": None}, "a", 1) == 1
    assert M.given({"a": 0.0}, "a", 1) == 0.0      # a real zero stays zero
    assert M.given({"a": False}, "a", True) is False


def rtc_state_over_the_wire():
    """A 2.1.0-shaped RTC status, as the agent sends it and the relay reads it back."""
    status = {"module": "LANE_CHANGE_LEFT", "uuid": "11" * 16, "safe": True,
              "requested": False, "command_status": "ACTIVATE", "state": "WAITING_FOR_EXECUTION",
              "auto_mode": True, "start_distance": math.inf, "finish_distance": math.nan}
    line = P.encode(P.state(P.DRIVING, rtc_status=[status]))
    return P.decode(line)["rtc_status"]


def test_the_wire_turns_non_finite_distances_to_null():
    s = rtc_state_over_the_wire()[0]
    assert s["start_distance"] is None and s["finish_distance"] is None


def test_the_relay_publishes_a_null_distance_as_unknown_not_zero():
    pytest.importorskip("tier4_rtc_msgs.msg")
    relay_node = pytest.importorskip("scenario_agent_relay.relay_node")
    from builtin_interfaces.msg import Time
    array = relay_node._rtc_status(Time(), rtc_state_over_the_wire())
    s = array.statuses[0]
    assert math.isinf(s.start_distance) and s.start_distance > 0
    assert math.isinf(s.finish_distance) and s.finish_distance > 0
    assert s.module.type == M.RTC_MODULES["LANE_CHANGE_LEFT"]
    assert s.auto_mode and s.safe and not s.requested


def test_the_relay_reads_a_status_whose_other_fields_are_null():
    pytest.importorskip("tier4_rtc_msgs.msg")
    relay_node = pytest.importorskip("scenario_agent_relay.relay_node")
    from builtin_interfaces.msg import Time
    keys = ("module", "uuid", "safe", "requested", "auto_mode", "state", "command_status",
            "start_distance", "finish_distance")
    s = relay_node._rtc_status(Time(), [dict.fromkeys(keys)]).statuses[0]
    assert s.module.type == 0 and bytes(s.uuid.uuid) == bytes(16)
    assert math.isinf(s.start_distance)


def test_a_null_phase_reads_as_unavailable():
    assert M.adapi_view({"phase": None}) == M.adapi_view(None)
