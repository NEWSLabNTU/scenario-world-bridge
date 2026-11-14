# ROS domains and DDS

A run uses several ROS domains on purpose, and every process in it must use the same DDS
setup. This page says why, which process lives where, and how to look into each domain from
your own terminal.

## Why more than one domain

- **The scenario simulator talks to an ADAPI that is not Autoware's.** SSv2 drives the ego
  through Autoware's API (`/api/routing/set_route_points`, `/api/operation_mode/state`, ...).
  In swb the agent relay serves those names to SSv2, in the scenario domain, and forwards
  each call over TCP to the ego's vehicle agent. The ego's real Autoware serves the same
  names in its own domain. In one domain the two would collide.
- **Every Autoware uses the same topic names** (`/tf`, `/clock`, `/sensing/...`), none
  namespaced. Each autonomous vehicle therefore needs a domain of its own: the ego one,
  each background AV another (2, 3, ...).
- **Domain 0 stays empty**, so that a ROS process started without any setup on the host
  cannot join a run.

The scenario reaches each vehicle only through the relay's TCP port, so nothing has to
cross between domains.

## Which process lives where

| Group | Domain variable | Default |
|---|---|---|
| The world server (CARLA's or Isaac's), the ego's autopilot, its vehicle agent | `EGO_ROS_DOMAIN_ID` | 1 |
| The bridge, the agent relay, SSv2 (the scenario) | `SCENARIO_ROS_DOMAIN_ID` | 9 |
| Background AV number n (CARLA only) | the `ROS_DOMAIN_ID` you start it with | 2, 3, ... |

The stack launch (`carla.launch.xml`, `isaac.launch.xml`, `stack.launch.py`) reads the two
variables from the environment and sets each group's domain itself. Its launch arguments
override the environment for one run:

```bash
play_launch launch swb_launch carla.launch.xml \
    map_path:=$CARLA_MAPS/Town01 \
    ego_domain:=3 \
    scenario_domain:=13
```

`ros2 launch` takes the same arguments. A background AV is a launch of its own, so you
give it its domain with `ROS_DOMAIN_ID=2` in front of its command.

## Choose domains from 0 to 57

The stack's DDS profile sets `DomainGain 1000`: domain d uses ports from
`7400 + 1000 d` up to about `7400 + 1000 d + 911`. Domain 57 ends at 65311; domain 58
would need ports past 65535, and every node then dies at start with only
`failed to create domain`.

The stack refuses such a domain before anything starts, naming the variable:

```
stack: scenario_domain (SCENARIO_ROS_DOMAIN_ID) is 60: use 0-57 (the localhost Cyclone
profile's DomainGain 1000 puts higher domains' ports past 65535)
```

It also refuses the same domain for both groups:

```
stack: ego_domain and scenario_domain are both 9: the autopilot's ADAPI and the relay's
would meet; give them different domains
```

A background AV is not checked: keep its domain at 57 or below, and different from every
other domain in use.

## The DDS: Cyclone, with swb's profile

Every process of a run must use CycloneDDS with the profile `swb_launch` installs,
`cyclonedds-localhost.xml`: discovery by unicast on the loopback interface, which works on
hosts where loopback multicast is off. The launches set both variables themselves, unless
the environment already names others. Your own terminals (`ros2 topic`, RViz, your agent,
a test script) need both exported:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
SWB_SHARE=$(ros2 pkg prefix --share swb_launch)
export CYCLONEDDS_URI=file://$SWB_SHARE/config/cyclonedds-localhost.xml
```

- Without `RMW_IMPLEMENTATION` a terminal runs Fast DDS, ignores the profile and sees
  nothing the stack publishes.
- Autoware 1.5.0's `setup.bash` exports a `CYCLONEDDS_URI` of its own on some hosts. Export
  swb's **after** sourcing Autoware, in every terminal, including the one you start the
  stack from: the launches keep a `CYCLONEDDS_URI` they find.
- Autoware's `setup.bash` also unsets `ROS_LOCALHOST_ONLY`, which conflicts with a Cyclone
  profile that names its own interface. If you set it again afterwards, unset it.

Put the lines in the environment file you source in every terminal
([Build your own stack](build-your-own-stack.md#1-the-environment-file)).

## Looking into each domain

Prefix a `ros2` command with the domain it should see:

```bash
# the ego's side: the world server, Autoware, the vehicle agent
ROS_DOMAIN_ID=${EGO_ROS_DOMAIN_ID:-1} ros2 topic list
ROS_DOMAIN_ID=${EGO_ROS_DOMAIN_ID:-1} ros2 run swb_launch ego_ready --timeout 600

# the scenario's side: the bridge, the relay, SSv2
ROS_DOMAIN_ID=${SCENARIO_ROS_DOMAIN_ID:-9} ros2 node list
ROS_DOMAIN_ID=${SCENARIO_ROS_DOMAIN_ID:-9} \
    ros2 param get /scenario_world_bridge config_file

# a background AV started with ROS_DOMAIN_ID=2
ROS_DOMAIN_ID=2 ros2 topic list
```

On CARLA the world server's node is in the ego domain:
`ROS_DOMAIN_ID=${EGO_ROS_DOMAIN_ID:-1} ros2 param get /swb_carla_world config_file`.

Write `${EGO_ROS_DOMAIN_ID:-1}`, not `$EGO_ROS_DOMAIN_ID`: an unset variable gives an
empty `ROS_DOMAIN_ID`, which is domain 0, where nothing runs.

If a list stays empty although the stack runs, the `ros2` daemon may still be running from
a terminal with other DDS settings. Add `--no-daemon` to the command, or stop the daemon
once with `ros2 daemon stop`.

## Isaac Sim: one host

On Isaac the world server itself publishes the ego's sensors, status and `/clock` in the
ego domain and takes Autoware's commands there. The localhost profile keeps DDS on one
machine, so Isaac's Autoware runs on the world server's host.

On CARLA the ego's vehicle bridge talks to CARLA over CARLA's own port, and every vehicle
agent reaches the relay over TCP, so a CARLA vehicle side may run on another machine. That
holds for CARLA only.

## Two stacks on one host

A second stack on the same machine moves every shared number, and keeps every domain
distinct from the first stack's:

| Setting | First stack | Second stack, for example |
|---|---|---|
| `EGO_ROS_DOMAIN_ID` | 1 | 11 |
| `SCENARIO_ROS_DOMAIN_ID` | 9 | 19 |
| Background AVs | 2, 3 | 12, 13 |
| `SSV2_PORT` (the bridge also uses the next two ports) | 5555 | 5565 |
| `AGENT_PORT` | 5560 | 5570 |
| `WORLD_PORT` (CARLA's world server) | 5591 | 5601 |
| `ISAAC_PORT` (Isaac's world server) | 5590 | 5600 |
| `CARLA_PORT` (one world server per CARLA: start a second CARLA) | 2000 | 3000 |
| play_launch `--web-addr` | 127.0.0.1:8084 | 127.0.0.1:8094 |
| `SWB_LOG_ROOT` | `./play_log` | a directory of its own |

Export the second set in that stack's environment file. Commands that take a port as an
argument need it too: `port:=` on `scenario.launch.xml`, `relay:=` on
`background_av.launch.xml`, `--relay` and `-- port:=` on `run_suite`
([Build your own stack](build-your-own-stack.md#10-two-stacks-on-one-host)).
