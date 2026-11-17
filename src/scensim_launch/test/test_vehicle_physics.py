"""`just run`'s vehicle_physics (carla.launch.xml): spec, the default, is the spec car's
generated carla_physics.yaml, so the vehicle agents' cars of the ego's model run it too."""
import importlib.util
import os
import xml.etree.ElementTree as ET

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
LAUNCH = os.path.join(HERE, "..", "launch")

spec = importlib.util.spec_from_file_location("stack_launch_physics",
                                              os.path.join(LAUNCH, "stack.launch.py"))
stack = importlib.util.module_from_spec(spec)
spec.loader.exec_module(stack)


def share_with(tmp_path, *cars):
    for car in cars:
        d = tmp_path / "generated" / car
        d.mkdir(parents=True)
        (d / "carla_physics.yaml").write_text("{}\n")
    return str(tmp_path)


def test_carla_launch_defaults_to_the_spec():
    root = ET.parse(os.path.join(LAUNCH, "carla.launch.xml")).getroot()
    args = {a.get("name"): a.get("default") for a in root.iter("arg")}
    assert args["vehicle_physics"] == "spec"


def test_spec_is_the_model_s_generated_file(tmp_path):
    share = share_with(tmp_path, "tesla_model3", "drift_mustang")
    assert stack.spec_physics("spec", "", "", share) == \
        os.path.join(share, "generated", "tesla_model3", "carla_physics.yaml")
    assert stack.spec_physics("spec", "drift_mustang", "", share) == \
        os.path.join(share, "generated", "drift_mustang", "carla_physics.yaml")


def test_a_path_and_none_are_kept(tmp_path):
    assert stack.spec_physics("none", "", "", str(tmp_path)) == "none"
    assert stack.spec_physics("/p.yaml", "", "/h.yaml", str(tmp_path)) == "/p.yaml"


def test_spec_refuses_a_car_it_does_not_have(tmp_path):
    share = share_with(tmp_path, "tesla_model3")
    with pytest.raises(RuntimeError, match="no car 'my_lincoln'"):
        stack.spec_physics("spec", "my_lincoln", "", share)
    with pytest.raises(RuntimeError, match="spec_hash_file"):
        stack.spec_physics("spec", "", "/elsewhere/spec_hash.yaml", share)
