# 085: Track B: Fast Driving, Drifting and the Race Planner

One race planner and one car (the Mustang) on the race courses: drifting on Town05's ring, grip or drift chosen live, the style layer, whether the planner generalises to any route, a mountain pass from open data, and localization by camera at those speeds.

**Status**: consolidated 2026-11-18 from 057, 061, 065, 071, 058, 063.

**Research rule** (user, 2026-11-18): a research track does not wait for scensim releases.
Its work runs on current scensim main, and it moves to each new scensim release when that
release is published (rebuild, rerun the track's reference runs), as a deliberate step.

**Consolidated** 2026-11-18 (user: "Have A-D"): the phases below keep their own documents as
history and detail; this document is the track's plan and status from now on, and new work
is planned here.

## Parts

| phase | what | status | left |
|---|---|---|---|
| [057](057-drift-campaign.md) | drifting on Town05's highway | in progress 2026-10-28: a full lap with Autoware and one drift, no contact | option 3 built and run |
| [061](061-grip-or-drift-mode-choice.md) | grip or drift, chosen live | in progress (M4c done 2026-10-31, gate not passed) | pass the gate |
| [065](065-drift-style-layer.md) | drift style chosen by a layer over the solvers | in progress: M1-M4 done; S2 not adopted yet | adopt or drop S2 |
| [071](071-planner-route-generalisation.md) | does the race planner generalise to any route | planned, design only | build the route set and measure |
| [058](058-touge-course-from-open-data.md) | a mountain pass from open data (Haruna) | in progress: C1-C8 offline done; 077 items 24-25: Haruna under Autoware stops before the first hairpin, Tsukuba not run | C9 on a GPU; the hairpin stop |
| [063](063-vision-only-localization.md) | vision-only localization on a fast car (the speed ladder) | steps 1-3 done; the ladder project built; G84 and speed found 2026-11-17 | items 1-3 (G84 fix, camera at 30 Hz, the speed band; 241, 2026-11-18), the full ladder on a quiet GPU host, steps 4-8 |

## Folded in

- 063 could stand alone (its question is localization, not planning); it stays here because it shares the car, the courses and the GPU host.

## Next

1. The speed ladder's items 1-3 (241).
2. Then the full ladder (063 step 4) on a host with a free GPU.
3. Re-plan 057/061/065/071 together: one planner question list.
