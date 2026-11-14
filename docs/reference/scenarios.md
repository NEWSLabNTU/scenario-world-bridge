# What a scenario may use

Scenarios are standard OpenSCENARIO 1.x files (or TIER IV's YAML form), run by SSv2. This
page lists what is particular to running them in CARLA or Isaac Sim through swb: what each
kind of road user does, and what fails. Writing one step by step:
[Tutorial 2](../tutorials/02-write-a-scenario.md) and
[Tutorial 3](../tutorials/03-other-road-users.md).

## The map

`RoadNetwork/LogicFile` names the map directory as `$(env SWB_MAPS)/<map>`; the stack sets
`SWB_MAPS` to the project's `assets/maps/`. On CARLA the directory's name is the CARLA town,
and the bridge loads that town when CARLA holds another. The file itself is never changed.

## The ego

The entity whose controller is named `Autoware` (or that has the property `isEgo=true`).
Its pilot runs in a ROS domain of its own; the scenario reaches it through the agent relay,
so what SSv2 offers for an ego works as usual:

- `TeleportAction` in the Init places the vehicle and re-initializes its localization there.
- `AcquirePositionAction` or `AssignRouteAction` gives it its goal; it plans a route and
  engages.
- `maxSpeed` (a controller property) and an absolute `SpeedAction` set its speed limit.
- `ego.currentState` (`WAITING_FOR_ROUTE`, `DRIVING`, `ARRIVED_GOAL`, ...), minimum-risk
  maneuver and turn-indicator conditions read the state its agent reports.
- `RequestToCooperateCommandAction` works with Autoware; an agent without it answers
  "unsupported" and the action fails, after SSv2 retries it for about 90 s.

It cannot be told to change lanes or follow a relative speed: SSv2 refuses both for an ego.

**Its body is its catalog car's.** Keep the ego's `CatalogReference` to the vehicle
catalog (`tesla_model3`, `drift_mustang`): the bridge refuses an ego whose body in the
simulator is more than 2 cm off the catalog's box.

## Road users SSv2 drives

Every vehicle, pedestrian and object whose controller is SSv2's own (the default). SSv2
computes each pose and the bridge places the simulator's body there every frame. Every SSv2
action works on them. On CARLA they are physics bodies that follow SSv2's pose: one the ego
hits is pushed, and one driven into the ego keeps pushing for as long as SSv2 drives it on,
so have it brake once the collision is seen.

## Vehicles the simulator drives: `simulator_autopilot` (CARLA)

A vehicle whose controller is `simulator_autopilot` is driven by CARLA's Traffic Manager,
with physics on: it brakes for vehicles ahead and for red lights, and it can be pushed.

- **Start.** It is spawned parked until SSv2's road users start moving, then handed to
  Traffic Manager.
- **Goal.** `AcquirePositionAction` (or `AssignRouteAction`, the last waypoint the goal):
  the bridge plans the lane route on CARLA's roads; the vehicle stops at the goal and stays
  there until the next one. Without a goal it roams the town.
- **Speed.** An absolute `SpeedAction` target is its desired speed; it accelerates and
  brakes as Traffic Manager does, not with the action's dynamics. It accelerates at about
  5.2 m/s^2 from rest, so declare a `Performance` that allows it.
- **Pose.** SSv2 takes the pose and speed CARLA reports every frame, so conditions and
  `CollisionCondition` see where the vehicle really is.
- **Repeatability.** Traffic Manager is reseeded at every scenario start: the same scenario
  on the same CARLA drives the same way.

Each of these is a scenario error, not a silent no-op:

- `LaneChangeAction`, a relative `SpeedAction` and `FollowTrajectoryAction`;
- switching to or from `simulator_autopilot` with `AssignControllerAction` during a run;
- a goal the lane route cannot reach (on another lane of the same road, or behind a one-way
  end);
- a pedestrian or an object with this controller: vehicles only.

A `TeleportAction` after the start is ignored. The route keeps its lane, so a stopped
vehicle ahead on a single-lane road holds it up. Isaac Sim refuses this controller.

## Vehicles a pilot drives: `agent`

A vehicle whose controller is `agent` is driven by a pilot of the project, often a second
Autoware ([Tutorial 4](../tutorials/04-several-pilots.md)). This is how a scenario has more
than one automated vehicle; conditions, distances and `CollisionCondition` see it as any
other entity. `swb.yaml` binds it to a pilot by its name.

- **Start.** The bridge spawns it parked, with physics on, and tells its agent where it is.
- **Goal.** `AcquirePositionAction` (or `AssignRouteAction`) becomes the agent's goal once
  the scenario's road users start and the agent has localized; the pilot plans and drives.
- **Speed.** An absolute `SpeedAction` target is its speed limit.
- **End.** The end of the scenario tells the agent to stop; the pilot serves the next run.

Fails, saying why: a goal for an entity with no registered agent; a goal the autopilot
refuses (for example Autoware's "The planned route is empty"); `LaneChangeAction`, a
relative `SpeedAction`, `FollowTrajectoryAction`, or switching the controller during a run.
On Isaac Sim only the ego can have a pilot.

## Traffic lights

At every scenario start the bridge sets every mapped light green and freezes CARLA's own
light cycle. A light the scenario never commands stays green for the whole run; a light it
commands (`TrafficSignalStateAction`) shows what it commands until the next command. To
stop the ego at an intersection, command that light. "Mapped" means the map's signal table
links it to a Lanelet2 traffic light; a CARLA light the map does not reference keeps its
frozen state, and Autoware cannot see it. The ego learns the lights' state from the
simulator, not from SSv2, so every mapped light is visible to it.

## Collisions

SSv2's `CollisionCondition` (bounding boxes) gives the verdict. On CARLA swb also records
CARLA's own contacts for the ego as a cross-check
([Verdicts](../explanation/verdicts.md)). That check sees only contacts that last more than
one frame, so a scenario that ends on the first frame of a collision ends before CARLA sees
the bodies touch: delay the exit (for example `delay="3"` on the `CollisionCondition`'s
`Condition`) when CARLA's account matters.

## Isaac Sim

The same files run as far as Isaac can drive them: SSv2's road users the same way,
`simulator_autopilot` refused, one entity with a pilot (the ego), one world per stack.
Every limit in one place: [Supported and not supported](limits.md).
