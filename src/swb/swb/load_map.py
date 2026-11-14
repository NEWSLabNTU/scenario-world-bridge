"""The bridge's ``~/load_map`` from outside (roadmap 077, G9): load a map's world now, between
scenarios, as SSv2's Initialize would.

    python3 -m swb.load_map <map dir | lanelet2 file> [--wait 10] [--timeout 120]

Run in the bridge's ROS domain (the stack's scenario domain) with the stack's DDS settings.
Prints one JSON line, ``{"success": bool, "message": str}``, and exits 0 when the world was
loaded, 1 when the bridge refused or failed, 2 when it was not reached. The stack's keeper
runs it on a CARLA map switch before it starts the vehicle stacks again
(docs/design/projects-and-pilots.md, "Maps and the keeper").
"""
import argparse
import json
import sys

SERVICE = "/scenario_world_bridge/load_map"
NOT_REACHED = 2


def call(map_path: str, wait: float = 10.0, timeout: float = 120.0):
    """``(success, message, reached)``: the bridge's answer, or why there was none."""
    try:
        import rclpy
        from swb_interfaces.srv import LoadMap
    except ImportError as e:
        return False, f"cannot call {SERVICE}: {e} (ROS and this workspace not sourced?)", False
    rclpy.init(args=None)
    node = rclpy.create_node("swb_load_map")
    try:
        client = node.create_client(LoadMap, SERVICE)
        if not client.wait_for_service(timeout_sec=wait):
            return (False, f"no {SERVICE} within {wait:.0f} s (the bridge is not up, or not "
                    "in this ROS domain)", False)
        future = client.call_async(LoadMap.Request(map_path=str(map_path)))
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done():
            return (False, f"{SERVICE} did not answer within {timeout:.0f} s (the bridge may "
                    "still be loading)", False)
        reply = future.result()
        return bool(reply.success), str(reply.message), True
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python3 -m swb.load_map")
    ap.add_argument("map_path", help="a map directory (its lanelet2_map.osm) or the lanelet2 file")
    ap.add_argument("--wait", type=float, default=10.0, help="for the service to appear, s")
    ap.add_argument("--timeout", type=float, default=120.0, help="for the load's answer, s")
    a = ap.parse_args(argv)
    ok, message, reached = call(a.map_path, a.wait, a.timeout)
    print(json.dumps({"success": ok, "message": message}), flush=True)
    return 0 if ok else (1 if reached else NOT_REACHED)


if __name__ == "__main__":
    sys.exit(main())
