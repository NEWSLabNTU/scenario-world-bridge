"""A world server from a support package, as `world: {server: [fixture_world, fake_world]}`
names it (scensim roadmap 081 step 5): the protocol shell's minimal world (core only, kinematic
bodies posed exactly), started by scensim with `--port <port>`."""
import argparse
import logging
import sys


def main(argv=None):
    ap = argparse.ArgumentParser(prog="fake_world")
    ap.add_argument("--port", type=int, required=True, help="scensim gives the seat's world port")
    args = ap.parse_args(argv)
    try:
        from scensim_world_server import lifetime
        from scensim_world_server.server import bind, run
        from scensim_world_server.world import World
    except ImportError as e:
        sys.exit(f"fake_world: {e}: the world protocol's Python shell is not on this Python's "
                 "path (scensim_world_server)")

    class FakeWorld(World):
        simulator_name = "fake"
        capabilities = ()
        max_physics_dt = 0.0

        def __init__(self):
            self.poses = {}

        def load(self, world, ground_paths, signals, request=None):
            self.poses.clear()

        def spawn(self, handle, name, kind, subtype, center, dims, pose, model="",
                  motion="KINEMATIC"):
            self.poses[handle] = pose
            return None

        def despawn(self, handle):
            del self.poses[handle]

        def step(self, drives, substeps, dt, render, sim_time):
            for handle, drive in drives.items():
                if drive.pose is not None:
                    self.poses[handle] = drive.pose
            return [(h, p, None) for h, p in self.poses.items()], []

    logging.basicConfig(level=logging.INFO)
    lifetime.die_with_parent()
    run(FakeWorld(), bind(args.port))


if __name__ == "__main__":
    main()
