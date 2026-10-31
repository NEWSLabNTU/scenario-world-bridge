# 029: PhysX's wheel query misses short obstacles (raycast) or throws the car off them (sweep)

**Severity**: high for roadmap 066 (no PhysX setting gives a real car's response to a rock or a
step at the live rate); none today, since no world has such obstacles yet. **Status**: open.
**Found**: 2026-10-31 on newslab-server-241, 066 R0 (`swb_isaac_world/spikes/road_fixtures.py`,
`spikes/fixture_course.py`), aib's Model 3 rig (sim_vehicle_spec 3773783), Isaac Sim 6.1.

## What happens

aib's Model 3 drove the R0 fixture course at a held speed: cleats 5, 10 and 15 cm high and
0.2 m long with vertical edges, an 8 cm pothole, washboard, a 0.5 m kicker, a 20 cm boulder
and a loose 10 kg rock. Every wheel was logged at each PhysX step. The reference was a quarter
car with a 42 kg wheel on a tire spring, the tire's geometric envelope and a bump stop
(`fixture_course.reference`).

**The ground is queried once per PhysX step (60 Hz), not per vehicle substep.** At 20 m/s a
wheel moves 0.33 m between queries. With raycast, the front-left wheel never touched the 10 cm
cleat in 2 of 5 runs that started 0.07 m apart; at 240 Hz it touched it in all 5.

**Raycast barely feels a step.** Within one step the wheel is placed on top of the cleat
(jounce 0.095 to 0.181 m), and the suspension force hardly rises:

| 10 cm cleat at 10 m/s | peak wheel load | body rise |
|---|---|---|
| reference | 20.2 kN | 7.8 cm |
| raycast, 60 Hz | 4.4 kN | 1.6 cm |
| raycast, 240 Hz | 4.5 kN | 2.0 cm |

**Sweep sees the step early, then throws the car.** The wheel's cylinder meets the cleat's
vertical face ahead of the wheel. PhysX reports a ground plane tilted 40-50 degrees (normal
(-0.75, 0, 0.66)) and a jounce at full travel. Its rigid travel limit then lifts the body at
10-11 m/s within one step. At 20 m/s, a 10 cm cleat threw the car 1.7-3.9 m up and pitched it
to 90 degrees in 2-3 of 5 runs.

These fixes do not remove it:
- More travel (0.30 m instead of 0.20) still threw the car in 2 of 5 runs.
- The same case run on its own did not throw the car; run after the others it did. So the
  outcome depends on the state the earlier cases leave.

At 240 Hz the sweep threw the car in 1 of 5 runs (0.41 m) and was otherwise closest to the
reference (boulder: 30.6 kN against 29.9; the 10 cm cleat at 10 m/s, 26.0 kN against 20.2).

**`limitSuspensionExpansionVelocity`** lets a wheel drop and unload, as the reference does. With
the sweep on the boulder, the run diverged: the body rose by kilometres.

**A loose rock is not pushed.** The wheel query reported `/World/env/rock` as its ground, and
the rock moved 2 mm while the car drove over it.

**Cost.** One PhysX step took 0.42-0.45 ms p50 with one vehicle, whatever the query type. 240 Hz
is 12 steps per 50 ms frame instead of 3, and everything run per PhysX step (the soil layer)
runs four times as often.

## Why it matters

066's rule is what a real car does, within the live budget. At 60 Hz, neither query type gives
a real response to a step or a rock, and the sweep's failure is violent. A 240 Hz scene rate
with the sweep is close, but it quadruples the scene's physics and the soil layer, and it still
throws the car now and then. None of the variants has wheel hop.

## Next

066 R0, "Where R0 leaves the plan": take the vertical tire contact over from PhysX. PhysX's
query sees the smooth base road. The tire model (envelope, tire spring, bump stop) acts on the
one height function at the vehicle substep rate, and its contact forces go to the body and to
the rock. Measure it on the same fixtures before any further PhysX tuning.
