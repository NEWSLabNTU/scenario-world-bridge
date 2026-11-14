"""follower: a pilot of kind `command`, an agent on the swb_agent SDK.

swb starts it as `python3 pilots/follower.py --relay <url> --entity follower --seed <n>
--run-dir <dir>`. It registers with the relay for its entity and is told the scenario's
goal; fill in the hooks with your autopilot.
"""
import argparse
import sys

import swb_agent


class Follower(swb_agent.Agent):

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
    parser = argparse.ArgumentParser()
    parser.add_argument("--relay")
    parser.add_argument("--entity", default="follower")
    parser.add_argument("--seed")
    parser.add_argument("--run-dir")
    args, rest = parser.parse_known_args(sys.argv[1:] if argv is None else argv)
    sdk = ["--entity", args.entity] + (["--relay", args.relay] if args.relay else [])
    swb_agent.main(Follower, sdk + rest)


if __name__ == "__main__":
    main()
