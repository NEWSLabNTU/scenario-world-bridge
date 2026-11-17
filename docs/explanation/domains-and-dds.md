# ROS domains and DDS

A stack uses several ROS domains on purpose, and every process in it must use the same DDS
setup. This page says why, which process lives where, and how your own terminal joins
one. The numbers come from your seat ([Several stacks on one
host](../how-to/several-stacks.md)).

## Why more than one domain

- **The scenario simulator talks to an ADAPI that is not Autoware's.** SSv2 drives the ego
  through Autoware's API (`/api/routing/set_route_points`, `/api/operation_mode/state`,
  ...). The agent relay serves those names to SSv2, in the scenario domain, and forwards
  each call over TCP to the entity's agent. The ego's real Autoware serves the same names
  in its own domain. In one domain the two would collide.
- **Every Autoware uses the same topic names** (`/tf`, `/clock`, `/sensing/...`), none
  namespaced. Each pilot therefore runs in a domain of its own.
- **Domain 0 stays empty**, so that a ROS process started without any setup on the host
  cannot join a run.

The scenario reaches each vehicle only through the relay's TCP port, so nothing has to
cross between domains.

## Which process lives where

| Group | Domain (seat n of the default table) |
|---|---|
| The bridge, the agent relay, SSv2 | the scenario domain, `1 + 8 n` |
| Each pilot: its vehicle side, its driver, its agent | the next pilot domain, in the order the pilots start (the ego first) |
| The world server (CARLA's or Isaac's) | the first pilot domain; Isaac's publishes the ego's sensors, status and clock there, and takes Autoware's commands |

The stack sets every process's domain itself; nothing is read from `ROS_DOMAIN_ID` in your
shell. `stack.json` lists each pilot's domain ([Results and
logs](../reference/results-and-logs.md#where)).

## Domains 0 to 57

The stack's DDS profile sets `DomainGain 1000`: domain d uses ports from `7400 + 1000 d` up
to about `7400 + 1000 d + 911`. Domain 57 ends at 65311; domain 58 would need ports past
65535, and every node would die at start with only `failed to create domain`. A seat table
with a domain outside 0 to 57, or two seats sharing one, is refused when it is read.

## The DDS: Cyclone, with scensim's profile

Every process of a stack runs CycloneDDS with the profile `scensim_launch` installs,
`cyclonedds-localhost.xml`: discovery by unicast on the loopback interface, which works on
hosts where loopback multicast is off. The stack sets both variables for everything it
starts. Your own terminals (`ros2 topic`, RViz, an agent, a test script) need both too, and
the domain:

```bash
eval "$(ros2 run scensim seat env scenario)"   # the bridge, the relay, SSv2
eval "$(ros2 run scensim seat env 0)"      # the first pilot (the ego)
ros2 topic list
```

`seat env` prints `export ROS_DOMAIN_ID=... RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CYCLONEDDS_URI=file://.../cyclonedds-localhost.xml` for your seat.

- Without `RMW_IMPLEMENTATION` a terminal runs Fast DDS, ignores the profile and sees
  nothing the stack publishes.
- Autoware's `setup.bash` exports a `CYCLONEDDS_URI` of its own on some hosts: run the
  `eval` after sourcing `~/scensim.bash`.
- If a list stays empty although the stack runs, the `ros2` daemon may still run with other
  settings: `ros2 daemon stop`, or add `--no-daemon`.

## One host for DDS

The localhost profile keeps DDS on one machine. On Isaac the ego's Autoware therefore runs
on the world server's host. On CARLA the vehicle bridge talks to CARLA over CARLA's own
port, and every agent reaches the relay over TCP, so a CARLA vehicle side and its driver
may run on another machine, with a profile of their own.
