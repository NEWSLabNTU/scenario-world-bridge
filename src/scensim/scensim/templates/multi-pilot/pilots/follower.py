"""follower: a pilot of kind `command`, an agent on the scensim_agent SDK.

scensim starts it as `python3 pilots/follower.py --relay <url> --entity follower --seed <n>
--run-dir <dir>` (the SDK's options: ``self.entity``, ``self.seed``, ``self.run_dir``). It
registers with the relay for its entity and is told the scenario's goal; fill in the hooks
with your autopilot.
"""
import sys

import scensim_agent


class Follower(scensim_agent.Agent):

    def on_reset(self, ep):
        # A scenario boundary: re-localize at ep.pose; return once ready for a goal.
        self.log.info(f"reset for episode {ep.episode!r} at {ep.pose}")

    def on_goal(self, goal, waypoints):
        # Plan to `goal` and drive; report READY, DRIVING and ARRIVED as it goes.
        self.log.info(f"goal {goal}")

    def on_speed_limit(self, mps):
        pass

    def on_stop(self):
        pass


def main(argv=None):
    # --relay, --entity, --seed and --run-dir: self.entity, self.seed, self.run_dir
    return scensim_agent.main(Follower, argv)


if __name__ == "__main__":
    sys.exit(main())
