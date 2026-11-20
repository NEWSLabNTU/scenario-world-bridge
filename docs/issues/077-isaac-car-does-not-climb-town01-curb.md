# 077: The Isaac car does not climb Town01's curb: isaac_verge never reaches the sidewalk

**Severity**: medium (056 step 5's scenario fails on Isaac; a car on Isaac cannot leave the
road over a curb). **Status**: a fix for scenarios that opt in (`isaac.wheel_query: sweep`, 118, 2026-11-20). `isaac_verge`'s
own run on Isaac Town01 waits for a host that renders Town01 (241 or 139). The sweep on soil is
open (below).
**Found**: 2026-11-20, swb 241, `isaac_verge` (main 7d5bb9d) on Isaac Town01, 2 of 2 runs.

## What happens

`town01_verge.xosc` has scensim's `simple` pilot drive Town01's straight at y -130 westward,
turn right at x 184 and stop on the walkway lanelet beside the road, (175.5, -125.2), 0.1 m
above the road. On CARLA (118) it passes: 26 m, 19 s, no contact. On Isaac both runs fail on
the scenario's 90 s limit:

- the right wheels reach the curb line at y -127.4 and stay there; the world server's surface
  log flips them between `asphalt` and `gravel` (the walkway's class in the scenario) many
  times a second at the same point: the wheels sit on the curb's edge;
- the car slides west along the curb at 0.05 to 0.07 m/s and never reaches the walkway.

## Cause (confirmed 2026-11-20, 118)

PhysX's raycast wheel query cannot take Town01's curb. Town01 has no road detail, so the road
layer's tyre contact (066 R5) is not there, and the curb meets PhysX's query alone.

- **The curb is a sharp step.** Town01's collision mesh at x 182-190 has a vertical face at
  y -127.419 up to 0.053 m, rounded to the 0.103 m top within 13 mm. Its lidar point cloud
  shows a soft edge, but that is the beam's footprint, not the collider.
- **The raycast does not climb it.** In a headless test of aib's vehicle against that profile
  (flat ground, straight at the curb, a held speed), the raycast fails in most cases:
  - 10 of 15 at 15 to 38 degrees off the curb line at 1 to 3 m/s;
  - 8 of 16 at 45 to 90 degrees at 1 to 4 m/s.

  In a failure, the near front wheel's ray lands on the face or its round-over, and PhysX
  reports that face as the ground plane (normal z 0 to 0.25). The tyre's force on it pushes
  the wheel back and sideways. The car then yaws parallel to the curb and creeps along it or
  stops. That is what isaac_verge shows: the front-right wheel's hit stays on y -127.4 from
  x 184.1 for 52 s while the car moves west.
- **The sweep query climbs every one of those cases.** But issue 029 found that it throws the
  car off 10 cm cleats at 10 to 20 m/s, so it was not chosen (066 R0).
- **The query type cannot change while the car runs.** PhysX reads it only when it creates
  the vehicle; set 1 s into a run, the run stays the raycast's.
- **No workaround in the scenario.** This road has no curb cut, and no approach angle is
  dependable: only 60 degrees climbed at every speed tried, while 45 and 75 failed at some.

## Fix (roadmap 086): the sweep with a bump stop, for scenarios that opt in

The fix applies to scenarios that opt in with `isaac.wheel_query: sweep` in `testbench.yaml`.
Every other scenario keeps the raycast, unchanged. `isaac_verge` is the first to opt in.

**What changed.** In aib (`VehicleParams`, `aib.bump_stop`):
- `wheel_query: sweep` makes PhysX query the ground with the wheel's cylinder.
- PhysX's suspension travel becomes 2.5 times the real one (`physx_travel`; 0.50 m on the
  Model 3), its frame raised so the droop stays the real one.
- A progressive bump stop holds the wheel at the real travel. Each PhysX step it applies a
  force at the wheel's attachment, from PhysX's jounce:
  - it engages 0.06 m before full compression;
  - its rate is 5 times the axle's spring;
  - its force is capped at rate times engagement;
  - its damper sees at most 0.2 m/s.

  The sweep's jounce jumps by most of a step's height within one PhysX step at the step's
  face, so the stop acts on the jounce as read: extrapolated, the first version gave 420 kN
  and threw the car.

On road-detail maps, scensim_isaac_world's road layer sets PhysX's longer travel through
`physx_travel`, and turns aib's stop off: its own corner has one.

**Why not the sweep alone.** 029's fixtures, sweep with no stop:

| PhysX travel | passes | cases thrown (rise above 0.3 m or pitch above 10 deg) |
|---|---|---|
| 0.20 m (the rig's) | 1 | 6, up to 4.5 m |
| 0.30 m | 1 | 2 |
| 0.40 m | 2 | 1 in both (4.6 m) |
| 0.50 m | 5 | none |

At 0.50 m, though, the wheel compressed up to 0.24 m past rest against the Model 3's 0.11, so
the stop is needed.

**Checks of the sweep with the stop** (118):
- **029's fixtures:** no throws. Rise is at most 0.13 m and pitch at most 2.9 deg, with the
  kicker ramp left out. Three passes are identical, and every case stayed on its lane.
- **The curb:** all 30 cases climb (Town01's collision profile and a box; 15, 25, 38, 60 and
  90 deg; 1, 2 and 4 m/s). The body rises at most 0.144 m on the 0.103 m curb. With the
  raycast, most fail.
- **aib's slope matrix:** identical (start and hold on downhill, flat and uphill). So are the
  launch, regen, coast and step responses. Braking past the tire moves: `brake_100` stops in
  39.59 m against 40.01, with fewer locked frames.
- **055's suite on dry and wet asphalt:** small changes. Examples: dry drift peak sideslip
  -13.7 against -14.8 deg; split-mu peak sideslip 3.1 against 2.5 deg; ABS share up 2 to 3
  points.
- **The road layer with today's car:** rock_world.py, 2 km of R3's gravel at 40, 60 and
  80 km/h. Every result matches main's, with only frame times differing.

**Known limits.**
- **Cleats:** on 10 to 15 cm cleats at 10 to 20 m/s the wheel still goes past the real travel,
  to a jounce of 0.28 to 0.39 m against 0.20. The rigid sweep puts the wheel on the cleat's
  edge before a tire would envelop it, and no force applied a step later can catch it. The
  body's motion stays as above.
- **Pothole and fast cleat loads:** peak loads are high (26.4 kN against the reference's
  10.1 on the pothole; 46 kN against 24 on a 10 cm cleat at 20 m/s).

**Open: the sweep on soil.** On 055's mud pad the sweep is a different car:

| case | metric | raycast | sweep |
|---|---|---|---|
| `mud_circle` | ends off its line | -1.1 m | 21.3 m |
| `mud_dlc` | peak sideslip | 20.8 deg | 10.7 deg |
| `mud_sine` | heading change | -42 deg | -32 deg |

On a straight stop on mud (`mud_brake`) the sweep's yaw rate chatters by about 0.5 deg/s and
the left and right wheel slips differ, with the car's track within 2 mm. The raycast shows
neither. The soil classes (047 S5, 053, 054) were fitted under the raycast, so a scenario on
soil should not opt in. Switching every car to the sweep and refitting mud under it is a
separate decision (asked of the user through 133, 2026-11-20).

**For roadmap 091's small car.** A RoboRacer (a 1/10-scale car) has wheels of about 0.05 m radius, so door sills,
floor seams and cable covers are steps of a third of the wheel or more. A raycast takes each of
these as a wall, as it takes Town01's curb. Such a car opts in the same way.

## Evidence

NAS logs `2026 SSv2 on CARLA/2026-11-20 077 curb climb, raycast and sweep (118)/` (the test,
per-step traces, Town01's mesh section) and `2026-11-20 077 sweep recheck, 029 fixtures (118)/`
(the sweep without a stop), `2026-11-20 077 aib bump stop, fixtures and curb (118)/` and
`2026-11-20 077 aib bump stop, 055 suite and slope matrix (118)/` (the stop).


NAS logs `2026 SSv2 on CARLA/2026-11-20 086 C4 isaac_verge on Isaac Town01 (241)/`.
