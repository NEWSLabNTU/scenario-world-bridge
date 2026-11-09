# 039: A road with rocks, loaded before any ego, keeps the ego out of PhysX

**Severity**: high (every world-server run on a `road_detail.yaml` with `rocks:` fails at
its first ego). **Status**: fixed in swb_isaac_world `5444614` and `daca90c` (two causes; see the end). **Found**: 2026-11-07, in
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

## Still failing sometimes (2026-11-08)

After the fix, 10 rough Town01 runs passed and then 2 failed the same way, back to back
(`play_log/multi_ego_road/2026-11-07_066-rep3/rough.log`, `-rep3-bare/rough.log`, the second
without the render mesh or the rock stream's updates): the preload's warm-up ego has no PhysX
body (`getRigidBodyTransformation` finds nothing at its path), 59 s and 585 s after Kit
started. No GPU, CUDA or memory error in either log. The flat Town01 has not failed in 6 runs.
Both the passing and the failing runs warn once, before the ego, that
`collection:colliders:includes` is missing on `/World/vehicleCollisionGroups/chassis`.

Not yet known: what else the rock road changes before the first ego (the pool's 992 parked
prims and the groups are made at the road's load, all in one stage edit before PhysX's first
step). Next: a run that, on the failure, prints whether the ego's prims carry their physics
APIs and the groups their includes, and a stepped world (one `world.step` after the road
loads, before the ego) to see whether parsing the load's edits first removes it.

## The second cause, fixed (2026-11-09)

16 preload-only runs (`multi_ego_road.py`, `ROAD_CASES=`): 8 plain, 1 failed; 8 with one
physics step after the world's load, none failed. At the failure every collision group was
in place (`chassis` with its colliders, `groundQuery`, the rocks' four), so the groups were
not the cause this time. The road's load makes the rock pool's parked prims (992 here) in one
stage edit, and the preload's ego was made in the same batch, before PhysX had stepped: now
and then PhysX made no body for it.

Fix (swb_isaac_world `daca90c`): `IsaacWorld._load_road` steps the physics once after a road
with rocks loads, before any ego. With it, 16 of 16 preloads from the world server passed;
24 of 24 with the step in all, against 3 failures in 20 without it.
