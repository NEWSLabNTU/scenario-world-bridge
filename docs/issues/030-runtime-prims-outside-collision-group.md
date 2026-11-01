# 030: Collider prims made after the simulation starts miss a collision group that includes them by their parent's path

**Severity**: high for roadmap 066 R4 and for any streamed collider. The wheels' ground query
hit rocks it must not see, and threw the car (issue 029's catapult). **Status**: fixed in
`spikes/rock_field.py`; each streamed prim is added to the group by its own path. The world
server's rock streaming (R4, to come) must do the same.
**Found**: 2026-11-01 on newslab-server-241, 066 R4 second slice
(`swb_isaac_world/spikes/rock_field.py`), Isaac Sim 6.1 (omni.physx 110.3.2).

## What happens

In R4 the rocks are in a collision group, `rocks`, that filters aib's `groundQuery` group, so
that PhysX's suspension raycast sees only the base road. The tire contact (`road_contact`)
reads the rocks instead. The group's colliders collection included `/World/rocks`, the parent
of every rock prim.

The first slice (`rock_fixtures.py`) and the first rock field made every rock before
`world.play()`. That worked: at 40 km/h the car drove 2 km of 8531 rocks with no launch.

Streaming the rocks makes the prims while the simulation runs, only for the tiles near the car.
After that change the car was thrown wherever a front wheel met a loose rock:

| run | rock | what the car did |
|---|---|---|
| 60 km/h | 2.4 kg, 8.7 cm tall, at 170 m | speed 16.7 to 14.7 m/s within a few steps; body up about 1 m; stuck in the ditch at 214 m |
| 80 km/h | 9.9 kg, 13.7 cm, at 70 m | body up 0.39 m, yaw 15 deg; off the road at 106 m |
| 40 km/h | at 770 m | body up 0.2 m, yaw 14 deg; stuck in the ditch at 1104 m |

The trace showed no force from the tire contact's rock reactions at those steps. The shape
was R0's raycast catapult: a sudden loss of speed and a launch.

## Why

omni.physx resolves a collider's collision group when it creates the shape. A collection that
includes a parent path covers the prims under it when they exist at parse time. A prim made
later under that parent is not in the group. So it collides with everything, the wheels'
ground query included. The PhysX vehicle then put the wheel on top of the rock within one
step, which is issue 029's mechanism.

This was not tested directly in PhysX: the evidence is the run that made its prims before play
against the runs that streamed them, and the fix below.

## Fix

Add each streamed prim to the group's includes by its own path when it is made, and remove the
target when the prim is removed (`rock_field.group`). The collision tiles of the road detail
were added that way from the start.

With the fix the same road at 40 and 60 km/h runs its 2 km with no launch. 19 and 21 rocks
went under the tires, none faster than 5.2 m/s, none lost. At 80 km/h the car grounds its
skid plate on the washboard and stops in the ditch at 794 m: R4's geometry, not this issue.
