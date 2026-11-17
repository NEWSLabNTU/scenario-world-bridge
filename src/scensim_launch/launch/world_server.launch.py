"""A world server from a support package (roadmap 081 step 5): a testbench's
``world: {server: [<package>, <executable>, args...]}``, started in place of the stock one.

scensim gives it the seat's world port as ``--port <port>`` after its own arguments, and in
SCENSIM_WORLD_PORT; it serves the world protocol on 127.0.0.1:<port> (scensim/own_world.py:
the contract). Started with ``ros2 run``, not as a ``Node``: a node gets ROS arguments
appended, which a server's own argument parser need not take. Respawned, as the stock servers.

    ros2 launch scensim_launch world_server.launch.py package:=my_world \\
        executable:=my_world_server port:=5591 [args:="--flag value"]
"""
import shlex

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.actions import SetEnvironmentVariable


def _server(context):
    get = context.perform_substitution
    from launch.substitutions import LaunchConfiguration as C
    package, executable = get(C("package")), get(C("executable"))
    port = get(C("port"))
    args = shlex.split(get(C("args")))
    if "--port" in args or any(a.startswith("--port=") for a in args):
        raise RuntimeError("world_server: --port is scensim's to give (port:=): drop it from args")
    return [SetEnvironmentVariable("SCENSIM_WORLD_PORT", port),
            ExecuteProcess(cmd=["ros2", "run", package, executable, *args, "--port", port],
                           name="world_server", output="screen", respawn=True,
                           respawn_delay=5.0)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("package", description="the support package holding the world server"),
        DeclareLaunchArgument("executable", description="its executable (ros2 run's second word)"),
        DeclareLaunchArgument("port", description="the world protocol's port on 127.0.0.1"),
        DeclareLaunchArgument("args", default_value="",
                              description="its own arguments (shell words), before --port"),
        OpaqueFunction(function=_server),
    ])
