"""stack.launch.py refuses ROS domains the stack cannot run in (roadmap 075)."""
import importlib.util
import os

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SPEC = importlib.util.spec_from_file_location(
    "stack_launch", os.path.join(HERE, "..", "launch", "stack.launch.py"))
stack = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(stack)


def test_defaults_pass():
    stack.check_domains("1", "9")


def test_highest_usable_domain_passes():
    stack.check_domains("57", "0")


def test_domain_past_cyclone_ports_refused():
    with pytest.raises(RuntimeError, match="ego_domain .* is 58: use 0-57"):
        stack.check_domains("58", "9")


def test_scenario_domain_past_ports_refused():
    with pytest.raises(RuntimeError, match="scenario_domain .* is 77"):
        stack.check_domains("1", "77")


def test_negative_refused():
    with pytest.raises(RuntimeError, match="is -1"):
        stack.check_domains("-1", "9")


def test_not_a_number_refused():
    with pytest.raises(RuntimeError, match="not a number"):
        stack.check_domains("", "9")


def test_same_domain_refused():
    with pytest.raises(RuntimeError, match="both 5"):
        stack.check_domains("5", "5")
