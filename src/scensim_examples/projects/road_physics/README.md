# road_physics

**Goal**: drive one Autoware ego on Isaac Sim over mud, gravel and a wet road on Town01,
the ground given by the project, not by the map.

| File | Holds |
|---|---|
| `swb.yaml` | Isaac Sim, Town01 (map, world, USD), the road physics stages, the pilot of `ego`, and `ground:` |
| `ground/town01.yaml` | Town01's surface map for this project: a wet stretch, 12.8 m of wet mud that ruts (a soil zone with its own ground), a gravel patch |
| `scenarios/mud_and_gravel.xosc` | the ego drives 70 m west over all three, on a dry road, to its goal |
| `scenarios/after_rain.xosc` | the same drive with the whole road moist and the gravel patch turned to dirt by the scenario (`RoadCondition`) |

The ground is a project file: `ground: {Town01: {surfaces: ground/town01.yaml}}` names it
for the map; the stack links it into the project's `state/Town01/`, where the world server
reads it, so the fetched map, shared and read-only, is never edited. A scenario sets the road's state
(wetness, a zone's class or water depth) through OpenSCENARIO's `RoadCondition`; it cannot
choose the surface map.

Check the ground without Isaac (from a checkout of `scensim_isaac_world`):

```bash
python3 -m scensim_isaac_world.surfaces check ground/town01.yaml
```

It warns that the mud's soil parameters are not validated (the map opts in with
`allow_unvalidated: [mud_wet]`) and counts 3 zones, 1 with soil.

## Hardware

As for Isaac on Town01: an NVIDIA RTX GPU with 24 GB, 64 GB of RAM or more, Isaac Sim
installed (`isaac.root`). The soil zone's ruts cost memory: about 100 MB per km of tracks
in it, for as long as the world server runs.

## Commands

```bash
ros2 run scensim project new ~/road_physics --from pkg://scensim_examples/projects/road_physics
cd ~/road_physics
ros2 run scensim assets fetch
ros2 run scensim project check
play_launch launch scensim_launch scensim.launch.xml
```

and, in a second terminal in the project:

```bash
ros2 run scensim run scenarios/mud_and_gravel.xosc
ros2 run scensim run scenarios/after_rain.xosc
```

The world server logs one line per wheel that moves onto other ground (its class, zone,
wetness and friction): look for `mud_road` and `gravel_patch` in the simulator's log
(`runs/<id>/logs/simulator/`).

## Expected result

`project check` prints `project: ok`; each run prints `passed  <run id>  <run directory>`.
The rear-drive Model 3 slows in the mud and crosses it; a stop inside fresh wet mud is
another matter (it barely starts again, and Autoware's longitudinal controller does not
push harder at low speed), so neither scenario stops there.

## Exercises

1. Add the rough gravel road the stages dataset carries: uncomment `road_detail:` in
   `swb.yaml`. Check: the world server's log names the corridor at load.
2. Make the mud softer: set `mud_road`'s class to `mud_soft` in `ground/town01.yaml`.
   Check: `mud_and_gravel.xosc` now ends `failed` on its 180 s timeout, the car stalled in
   the mud.

`assets/`, `state/` and `runs/` are made on each host and are not committed.
