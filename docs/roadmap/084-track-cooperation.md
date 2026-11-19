# 084: Track A: Cooperation Between Automated Vehicles

How automated vehicles share conflict zones: the cases as games whose payoffs scensim measures (074), the strategy turned into a node beside Autoware and tested (078), several Autoware egos on one world for the multi-car runs (039).

**Status**: consolidated 2026-11-18 from 074, 078, 039. 2026-11-19: 078 P2g is done on its
branch; its rerun on a quiet host is the open item (Next 1).

**Research rule** (user, 2026-11-18): a research track does not wait for scensim releases.
Its work runs on current scensim main, and it moves to each new scensim release when that
release is published (rebuild, rerun the track's reference runs), as a deliberate step.

**Consolidated** 2026-11-18 (user: "Have A-D"): the phases below keep their own documents as
history and detail; this document is the track's plan and status from now on, and new work
is planned here.

## Parts

| phase | what | status | left |
|---|---|---|---|
| [074](074-cooperation-study.md) | the cooperation study: games, arena, strategy atlas | started 2026-11-13; most site types done in the kinematic model and the arena (S1-S6, U1-U3, Y merge); stock-Autoware runs for the Y merge and the left turn; atlas v11 | Autoware on the four-way stop (U3); the reflectance sweep (user's call) |
| [078](078-conflict-zone-protocol.md) | the conflict-zone protocol: library, node, evaluation | phases 0-1 done; phase 2 on the crossing. P2e: G0 passes in the arena. P2f (2026-11-19): the scensim grid fails on Autoware, 5 contacts in 20 against stock's 0, the mutual-rest case. P2g (2026-11-19, branch `p2g-launch`, not on main): Autoware's launch measured; no fixed guard is enough, so the crossing probes every go from a mutual rest; the arena's G0 passes on 4 sets; the grid had 2 contacts in 32 against stock's 0. One was the `self` contract handing a committed go to stock (issue 066, fixed at 1c84d14), the other late perception. `role_time_error_e` stays 0.55: the formal check covers the measured 0.65 | the grid's left t1.5p5 cells again at `p2g-launch` 3505a08 on a quiet host (the bundle is ready; it waits for 133's renice), then land `p2g-launch`; phases 3-4 |
| [039](039-isaac-multi-autoware.md) | several Autoware instances on Isaac Sim, V2X | planned | one ROS domain per ego on the world server; V2X |

## Folded in

- **Isaac lidar against CARLA (139, 2026-11-19).** The intensity fix (the user's option 1: an intensity table in the car spec, times the cosine, by object id and normal; spawned bodies take their entity box's class) landed as aib a37d23e / scensim_isaac_world c3d0e2f; 084b, the lidar firing the spec's pattern instead of Ouster OS1's, as aib 68f350f. At a matched pose (`scripts/sim_compare/`; an earlier 1.39 m rear-axle offset in the comparison tooling gave a spurious 35 m gap, withdrawn) Isaac sits at CARLA's numbers on every detector within the intervals: TransFusion 35 m 0.48 vs 0.41, side car 0.41 vs 0.36, FP 0.27 vs 0.36; CenterPoint 20/35 m 0.86/0.13 vs 0.83/0.14, FP 0.94 vs 0.96; 110.4k vs 112.0k points a scan. Standing differences: CARLA's ray cast returns nothing from props without collision (a pergola: 0 points against Isaac's 116). Both simulators fire identical azimuths every scan, so a thin object's hits depend on a sub-column placement. 084e: phase drift, opt-in (aib fe9c802: a rig profile's `phase_jitter: {seed: N}` turns Isaac's lidar prim within a column a frame and its points back; CARLA's lidar at 20.00268 Hz, `scripts/sim_compare/carla_record.py`). Over five ego positions within +-1.4 m it halves the 35 m car's TransFusion recall spread (CARLA 0.49 to 0.21, Isaac 0.39 to 0.17; both means 0.36) and puts CityLamp115's pole at 16-31 hits a scan instead of 6-39. 084f: under drift Isaac's side car scores lower (TransFusion 0.31 vs 0.45, intervals apart). Splicing the car between the bags moves the recall with it. The cause is the car's own points: CARLA's collision hull against Isaac's render mesh, 7 % fewer points, mostly on the roof. Class is CAR on both. TransFusion itself is non-deterministic: 140 byte-identical clouds gave 0.29-0.41 over four replays, with fixed stamps too, so a single replay resolves recall differences above ~0.1 only. The rule for every detector-dependent verdict we quote, comparisons and T3 alike: a recall difference below ~0.1 needs three or more replays (or runs) per condition (`scripts/sim_compare/README.md`). Beam divergence (VLS-128 manual Table F-1: 2.09/1.05 mrad) cannot act on either: Isaac 6.1's IDEALIZED rays ignore it and GAUSSIAN_BEAM crashes the RTX sensor plugin (issue 063); CARLA has no such attribute. Reports on the NAS, "2026-11-18 084 ..." and "2026-11-19 084b-084f ... (139)".
- CARLA 1.0's deterministic collisions ([083](083-carla-1-0.md)): contact verdicts that repeat between runs.
- Deferred: Taiwan's scooters and the night market (reactive NPC populations); the heavy-vehicle spec (built when an Autoware test needs a truck).

## Next

1. 078 Phase 2 (P2g): the grid's left t1.5p5 cells, czp and stock, on fresh seeds 8-11 at
   `p2g-launch` 3505a08, on a quiet host. Bundle: NAS `home/2026-11-19 078 P2g rerun left t1.5p5
   for 133 (241)`. Its `f4_summary.md` reads the decision's CPU time against wall time and the
   `self` contract's failing ticks per run. `p2g-launch` lands, rebased, with issue 066, when:
   - G0 holds there: czp is no worse than stock, with no contact where stock has none;
   - on czp's runs, no decision is handed to stock by a contract after the warm-up
     (`f4_summary.md`'s stock-by-contract count is 0);
   - no committed go changes hands (its committed-go-handed-over count is 0).

   Then phase 3.
2. 074 U3 (four-way stop) on Autoware.
3. Candidate for the next deliberate spec change, batched with other spec edits so `spec_hash` churns once: drift by default (`carla.attributes.rotation_frequency` 20.00268 and an Isaac `phase_jitter` flag in the spec's lidar). Until then drift is opt-in, for comparison work.
4. Low priority, not started: a run seed handed to the world server and on to the lidar's `phase_jitter`, so "seeded per run" holds literally (today the seed is the profile's).
