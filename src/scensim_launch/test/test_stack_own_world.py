"""stack.launch.py starts a project's own world server (world.server, roadmap 081 step 5)
through scensim_launch's world_server.launch.py, on the seat's world port."""
import importlib.util
import os

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SPEC = importlib.util.spec_from_file_location(
    "stack_launch", os.path.join(HERE, "..", "launch", "stack.launch.py"))
stack = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(stack)


def test_the_own_world_server_is_world_server_launch_with_its_words():
    path, args = stack.own_world_launch("my_world my_server", "--level 'two words'", "5791",
                                        "/share/scensim_launch")
    assert path == "/share/scensim_launch/launch/world_server.launch.py"
    assert args == {"package": "my_world", "executable": "my_server", "port": "5791",
                    "args": "--level 'two words'"}
    assert os.path.isfile(os.path.join(HERE, "..", "launch", "world_server.launch.py"))


def test_a_malformed_world_server_or_no_port_is_refused():
    with pytest.raises(RuntimeError, match="'<package> <executable>'"):
        stack.own_world_launch("my_world", "", "5791", "/s")
    with pytest.raises(RuntimeError, match="needs world_port"):
        stack.own_world_launch("my_world my_server", "", "", "/s")
