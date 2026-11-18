"""Test fixtures for ``testbench check --live`` (roadmap 081 step 5): a world server on the
scensim_world_server shell, core only, whose bodies go exactly where they are put.

    python3 kinematic_world.py [--broken] --port <port>

``--broken`` breaks the protocol the way a careless server would: a step leaves a kinematic
body where it was instead of putting it at its pose, so the conformance check fails.
"""
import argparse

from scensim_world_server.server import bind, run
from scensim_world_server.world import World


class KinematicWorld(World):
    simulator_name = "fixture"
    capabilities = ()
    max_physics_dt = 0.0

    def __init__(self, broken=False):
        self.broken = broken
        self.bodies = {}

    @property
    def simulator_version(self):
        return "fixture-1"

    def load(self, world, ground_paths, signals=(), request=None):
        self.bodies.clear()

    def spawn(self, handle, name, kind, subtype, center, dims, pose, model="",
              motion="KINEMATIC"):
        self.bodies[handle] = pose

    def despawn(self, handle):
        del self.bodies[handle]

    def step(self, drives, substeps, dt, render, sim_time):
        for handle, drive in drives.items():
            if drive.pose is not None and not self.broken:
                self.bodies[handle] = drive.pose
        return [(h, pose, None) for h, pose in self.bodies.items()], []


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, required=True)
    ap.add_argument("--broken", action="store_true")
    args = ap.parse_args()
    run(KinematicWorld(args.broken), bind(args.port))


if __name__ == "__main__":
    main()
