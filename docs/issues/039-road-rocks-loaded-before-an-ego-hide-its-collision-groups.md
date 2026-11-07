# 039: A road with rocks, loaded before any ego, keeps the ego out of PhysX

**Severity**: high (every world-server run on a `road_detail.yaml` with `rocks:` fails at
its first ego). **Status**: fixed in swb_isaac_world `5444614` (see below). **Found**: 2026-11-07, in
066's acceptance spike (`spikes/multi_ego_road.py` on Town01 with a rough corridor and
rocks), Isaac Sim 6.1.

## What happens

The world server's preload spawns its throwaway ego and dies reading the ego's body:

```
[Error] [omni.physx.plugin] SimulationInterface function could did not locate any objects at the specified path.
[Error] [omni.physx.plugin] Error executing getRigidBodyTransformation.
KeyError: 'position'   (aib.isaac.vehicle.Vehicle.sample)
```

The same Town01 without the road detail preloads and drives.

## Why

`rock_layer.RockStream.__init__` defines its collision groups (`rocks`, `rockTiles`,
`roadDetail`, `roadBase`) under `/World/vehicleCollisionGroups`. aib's
`ensure_vehicle_scene` makes the vehicles' own groups there, `chassis` and `groundQuery`,
only `if not stage.GetPrimAtPath(GROUPS)`: when the scope is missing. The world server loads
the road (and so the rocks' groups) with the world, before any ego exists, so the scope is
already there when the first ego comes, aib makes neither group, and the ego's colliders and
wheel queries name groups that do not exist. PhysX does not create the vehicle.

`spikes/rock_world.py` and `rock_field.py` make the car before the rock stream, which is why
R4's runs never met it: R4's third slice said rock_world drives "exactly this code", and for
the order of creation it did not.

## Fix

`RockStream.__init__` calls aib's `ensure_vehicle_scene(stage)` before defining its own
groups, so the vehicles' groups exist first whatever the order (swb_isaac_world, with 066
R4's close). aib's check of the scope rather than of each group is still fragile for any
other group a world defines first; changing it to check `chassis` is aib's.

Checked (2026-11-07, `play_log/multi_ego_road/2026-11-07_066-fix/rough.log`): the rough Town01
preloads, and one and two egos spawn and drive the whole corridor, no rock lost.
