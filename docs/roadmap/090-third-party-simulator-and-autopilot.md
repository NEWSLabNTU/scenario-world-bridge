# 090: A Third-Party Simulator and Autopilot (AWSIM, Apollo)

**Status**: step 1 done 2026-11-19; decided **not now** for both (user): findings recorded below, revisit when a user or study needs one.

## Question

scensim has one interface for simulators (a world server speaking `swb_world.proto`) and one
for autopilots (pilots: `autoware`, `launch`, `command`, with the agent protocol). Can a user
add a simulator and an autopilot scensim does not ship, AWSIM and Apollo as the test cases,
as support packages in their own ROS workspace, built together with their testbench?

```
my_ws/src/
  my_testbench/            # testbench.yaml, scenarios, package.xml (exec_depend on the two below)
  awsim_world_server/      # the simulator's support: speaks scensim's world protocol
  apollo_scensim_pilot/    # the autopilot's support: agent registration and message adapter
```

## What is expected to fit

- **Simulator:** `world: {server: [<package>, <executable>, args]}` runs a world server from
  the workspace (`docs/how-to/your-world-server.md`); `scensim_world_check conformance`
  checks it; the Python shell `scensim_world_server` carries the loop.
- **Autopilot:** `kind: launch` or `kind: command` with `agent: own`, through the agent SDK
  (`scensim_agent`); `overlays:` and `{workspace:<name>}` for things built elsewhere.
- **Car:** `vehicles: [<spec>.yaml]` for a car scensim does not ship.

## Known or suspected gaps

1. **Lockstep time (AWSIM).** SSv2 drives frames and scensim's time is `elapsed + 1000 s`.
   Whether AWSIM can be stepped (through its SSv2 ZMQ connection or otherwise) decides whether
   it can be a backend at all.
2. **A simulator-native ego (AWSIM).** AWSIM simulates the ego and publishes Autoware's sensor
   and vehicle-interface topics itself: no vehicle bridge, the Autoware pilot with
   `launch_vehicle_interface:=false`. The bridge's 2 cm body check needs AWSIM's car as a spec.
3. **`simulator: none`.** The world-server how-to uses it; the project-file reference lists
   only `carla | isaac` (being reconciled, 2026-11-19).
4. **Apollo is not ROS.** Cyber RT channels, Bazel, its Docker image: the ROS adapter builds in
   the workspace, Apollo itself does not. Sensors, localization and chassis in, control out,
   converted between Cyber and the vehicle side's interface.
5. **Apollo's HD map.** Its own format, from OpenDRIVE with Apollo's converter: a map per town
   in the testbench's assets.
6. **Binary simulators.** AWSIM ships as a Unity binary, not a ROS package: its path from the
   host settings (`workspaces:` or a new key).

## Steps

1. **Paper study** (no code): AWSIM's external control and time (its SSv2 connection, ROS 2
   clock, any step API), its vehicle and sensor topics, its car; Apollo's Cyber/ROS bridges,
   its routing and engage, its map conversion, its Docker build. For each gap: what it takes,
   and the effort.
2. **AWSIM spike**: a minimal world server that spawns and steps AWSIM and passes
   `scensim_world_check conformance`.
3. **Apollo plan**: the adapter's design and effort, or a spike if step 1 shows a cheap path.
4. **The guide**: "Your simulator and autopilot, end to end", from whichever spike works.

## Exit criteria

A decision per case (support, support with named limits, or not now), each with its effort,
and the scensim changes either needs.

## Step 1 result (241, 2026-11-19)

Report with a source for every claim: NAS `logs/2026 SSv2 on CARLA/2026-11-19 090 step 1 paper
study AWSIM and Apollo (241)/`. **Neither can run in scensim's lockstep time as it is.**

**AWSIM** (autowarefoundation/AWSIM main 7b803d8, v2.0.1; AWSIM-Labs is archived). It serves
SSv2's ZMQ interface itself (SSv2 PR #1099); by default Unity runs freely and `update_frame`
only copies SSv2's time; its "step execution" raises `timeScale` for a wall-time sleep, so
physics steps per frame depend on timing; there is no `Physics.Simulate` or step API.
Lockstep needs a fork of AWSIM's C# (script-driven physics, sensors on sim time), about 2-4
weeks; then either AWSIM speaks `swb_world.proto` (+2 weeks) or the bridge gets an SSv2
pass-through (~1 week). AWSIM is the ego's vehicle side itself (a Lexus RX450h; its own
scenes, no CARLA towns, ~1-2 weeks for a first map); its connector settings live in the scene
(an editor build of our own). About 1-2 months to parity.

**Apollo**. Cyber RT's mock clock has no clock channel, its timers sleep in wall time, and
Control is a timer component; prior art (carla_apollo_bridge) runs on wall time; the official
ROS 2 bridge existed only in v10. What works: a `cyber_py3` adapter in Apollo's container
talking to scensim's agent protocol over TCP (1-2 weeks), headless engage, Apollo maps of
several CARLA towns from carla_apollo_bridge, the Docker build (8.7 GB, CUDA 11.8). Lockstep
needs a Cyber fork (mock clock from `/clock`, timers on it, a per-step barrier), 4-8 weeks; a
pilot paced at 1x wall time with named limits (not stepped, not reproducible step for step)
is about 3-4 weeks.

**What scensim would need for either**: an SSv2 pass-through (or world-server mode) for a
simulator that speaks SSv2; a simulator-native ego (the simulator is the vehicle side, the
Autoware pilot's vehicle interface off, the car's spec for the body check); a host key for a
binary simulator; a paced wall-clock run mode with stamp mapping (scensim is lockstep
throughout); a container pilot speaking the agent protocol over TCP; per-autopilot map assets.

Gap 3 (`simulator: none` in the reference) is closed (a6f14eb).
