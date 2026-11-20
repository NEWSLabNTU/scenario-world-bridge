# Vehicle risk taxonomy, and scenario design for coverage and realism (study)

Status: study for phase 092, 2026-11-20. It extends
[survey-risk-categories.md](survey-risk-categories.md), which covers the purposes (a risk
rate, and failure-mode search), the realism rules and two worked families. This note lists the
whole space of vehicle risk and asks, for each family, what a scenario must express, where
realism comes from, and how coverage is measured.

## 1. Sources for the space

| Source | Its organising idea |
|---|---|
| NHTSA light-vehicle pre-crash scenarios, 2011-2015 data ([ROSA P 41932](https://rosap.ntl.bts.gov/view/dot/41932)); the 2007 typology of 37 scenarios ([DOT HS 810 767](https://rosap.ntl.bts.gov/view/dot/6281)) | 36 scenarios in nine groups: control loss, road departure, animal, pedestrian, pedalcyclist, lane change, opposite direction, rear-end, crossing paths |
| NHTSA, a framework for ADS testable cases ([DOT HS 812 623](https://www.nhtsa.gov/document/framework-automated-driving-system-testable-cases-and-scenarios)) | a test scenario has four elements: the tactical manoeuvre, the object and event detection and response (OEDR) behaviour, fault and failure behaviour, and the ODD |
| ISO 34503 ([ISO](https://www.iso.org/standard/78952.html)), BSI PAS 1883 | the ODD taxonomy: scenery (zones, junctions, lanes, temporary structures), environmental conditions (weather, particulates, illumination, connectivity), dynamic elements (traffic agents, the subject vehicle) |
| JAMA / SAKURA and ISO 34502 (see the companion study) | traffic, perception and vehicle-motion disturbance |
| ISO 21448, SOTIF (see the companion study) | functional insufficiencies and their triggering conditions |
| PBCAT, Euro NCAP, UN R157, Waymo CAT (see the companion study) | VRU crash types; test grids; reference drivers; responder-role conflicts |

The families below merge these sources. A risk can sit in more than one family. A wet-road
rear-end, for example, is traffic (rear-end) × vehicle motion (friction) × perception (spray);
that overlap is what the coverage cells in section 4 capture.

## 2. The families

### R1 Traffic conflicts with other vehicles

| Category | Subtypes (examples) |
|---|---|
| rear-end | lead decelerating, lead stopped, lead slower; stop-and-go waves; the ego as the lead (struck from behind) |
| lane change and merge | cut-in, cut-out revealing a slower car, two cars into one gap, on-ramp merge, lane drop, zipper merge |
| crossing paths | straight crossing at junctions, left turn across path (opposite direction), right turn into path, red-light runner, unsignalised negotiation |
| opposite direction | oncoming car drifting over the centre line, overtaking on a two-lane road, wrong-way driver |
| special vehicles | emergency vehicle needing right of way, bus pulling out, truck with a trailer off-tracking, heavy vehicle blind spots, motorcycle filtering between lanes |
| stationary and parked | door opening into the lane, double-parked car, a car reversing out of a parking space |
| control loss of another vehicle | a car spinning or sliding into the ego's path |

### R2 Vulnerable road users

| Category | Subtypes (examples) |
|---|---|
| walker crossing | near side, far side, dart-out from occlusion, hesitating or turning back, at a crosswalk on a turn, mid-block, a group, a child, a wheelchair |
| walker along or in the road | walking along the edge, standing in the road (a worker, a broken-down driver), lying down |
| cyclist and micromobility | crossing, being overtaken (side clearance), coming the wrong way, dooring, filtering, scooters |
| animals | crossing; large animals (a deer: a collision with severity) and small ones (a dog: unpredictable) |

### R3 Infrastructure, rules and maps

| Category | Subtypes (examples) |
|---|---|
| signals | the yellow-light dilemma zone, a signal outage or flashing, a signal hidden by a truck, conflicting signals |
| signs and markings | missing, occluded or temporary signs; worn or conflicting lane markings |
| work zones | lane closure, shifted lanes, cones, a flagger, police hand signals |
| special places | school zones, railway crossings, tolls, parking structures, tunnels, bridges |
| map against reality | the HD map stale (a closed lane still on the map, a moved stop line), a map with defects (orphan stop lines, issue 081), a projection or origin error (issue 084) |
| localisation | GNSS loss, a featureless stretch where NDT degenerates (issue 002), a wrong initial pose |

### R4 Perception disturbance (SOTIF triggering conditions)

| Category | Subtypes (examples) |
|---|---|
| occlusion | static (parked cars, buildings, vegetation), dynamic (trucks), crest and curve occlusion |
| illumination | low sun ahead, night, tunnel entry and exit, oncoming headlights, flickering lights |
| weather and particulates | rain, fog, snow, spray behind trucks, dust, smoke |
| sensor-specific | lidar on black or glossy cars and glass, radar multipath in tunnels and under bridges, a camera dirty or wet, sensor misalignment |
| rare and odd objects | debris, a fallen load, an overturned vehicle, unusual vehicles (a tractor, a horse cart), costumes, advertising images of people |
| ghosts | reflections in puddles and glass, a billboard showing a car, a steam plume |

### R5 Vehicle motion disturbance

| Category | Subtypes (examples) |
|---|---|
| friction | wet, ice, snow, gravel, mud, split friction (each side differs), a friction change mid-manoeuvre |
| geometry | steep grades and crests (Haruna), sharp curves, off-camber, banked turns |
| roughness | potholes, curbs and verges (issue 077), speed bumps, rail tracks |
| forces | crosswind gusts, trailer sway, load shift |
| the car's own limits | actuator saturation, control latency (one study found Autoware's tracking broke past about 40 ms injected latency; see the companion study's sources), brake fade, tyre wear |

### R6 The ego's own faults and degraded modes

| Category | Subtypes (examples) |
|---|---|
| sensor faults | dropout, frozen frames, a noisy or biased IMU (issue 086's kind), clock skew between sensors |
| compute and timing | overload, latency spikes, a stalled node (the log-disk stall in CLAUDE.md), message loss |
| actuators | steering stuck or weak, a brake lost on one wheel, a powertrain fault |
| software | a process crash and restart, a planner without a path, contradictory module outputs |
| fallback | the minimal risk manoeuvre itself: does it stop safely, and where? |

### R7 Interaction and social behaviour

| Category | Subtypes (examples) |
|---|---|
| negotiation | unsignalised junctions and narrow passages (the crossing protocol, roadmap 078), four-way stops, merging courtesy |
| aggressive or erratic drivers | tailgating, brake-checking, sudden swerves, racing |
| ambiguous intent | a turn signal without a turn, a turn without a signal, a car hesitating |
| cooperative traffic | platoons, V2X messages that are late, lost or false (V2X is optional, roadmap 074) |

### R8 ODD boundaries

| Category | Subtypes (examples) |
|---|---|
| leaving the ODD | weather starting mid-run, the road type changing, the map ending |
| detours and closures | unexpected closures, a police-directed detour |
| handover | a takeover request and its timing (Level 3), no driver response |

## 3. What a scenario must express, per family

| Family | Actors and behaviour | World knobs | Events during a run | Today in OpenSCENARIO and scensim |
|---|---|---|---|---|
| R1 traffic | vehicles with car-following and lane-change models; cause-driven braking | road layout (the world), signal timing | cut-ins, braking, a red-light runner | storyboard actions; behaviour models are 092 examples |
| R2 VRUs | walkers and cyclists with gap-acceptance and route models | crosswalks, occluders | dart-out triggered by time to conflict | pedestrian entities; behaviour models are examples |
| R3 infrastructure | flaggers, police | signals, signs, work zones (props), the map variant | signal outage, a sign occluded | traffic signal actions exist; **the map against reality**: needs a world knob that loads a deliberately different map for the pilot than the world it drives in |
| R4 perception | occluding traffic | illumination, weather, materials (glossy and black cars), props | weather onset, tunnel transitions | `EnvironmentAction` exists; the world server applies weather; per-sensor effects depend on the simulator |
| R5 vehicle motion | none required | surfaces, grade, roughness, wind | friction changes along the route | surfaces per scenario exist (Track C); wind and mid-run friction changes are new knobs |
| R6 ego faults | none required | none | sensor dropout, latency, actuator faults at a chosen time | **new**: fault injection through the pilot interface (`UserDefinedAction`, which SSv2 parses, routed to a fault hook) |
| R7 interaction | agents with strategy models (aggressive, cooperative, hesitant) | none | intent signals, V2X messages | the crossing protocol's agents (078); V2X as an attachment |
| R8 ODD | none required | ODD attributes per region of the route | weather onset, road type change, map end | environment actions; route segments tagged with ODD attributes |

The new capabilities this asks for are three:

1. **Fault injection (R6).** A `UserDefinedAction` names a fault, and the pilot interface
   delivers it to a fault hook on the vehicle side: drop a sensor, add latency, weaken an
   actuator.
2. **A pilot map that differs from the world (R3).** The world and the pilot's map are
   separate inputs, so a stale-map case is a deliberate variant.
3. **Knobs that change during a run (R4, R5, R8).** Weather onset, friction changes along the
   route, and crosswind.

Everything else is either there today or is a behaviour model shipped as an example.

## 4. Coverage

Coverage has three layers, each measurable from the sample records.

**1. Category coverage.** Which families and categories from section 2 have at least one
logical scenario in the testbench. The report lists the uncovered ones by name. The aim is to
show the gaps honestly, not to fill every row.

**2. Condition coverage (combinatorial).** Each category is crossed with its relevant ODD and
disturbance attributes, which are discrete: day or night, dry, wet or icy, occluded or not,
straight or curve. Every pair is not run exhaustively; t-wise combinatorial coverage (pairs,
triples) gives a bounded set that still meets each combination of t attributes. The report
gives the t-wise coverage achieved per category.

**3. Criticality coverage (continuous).** Within each cell, sampling covers the encounter's
critical region: purpose A with importance weights, purpose B with adaptive search. The report
gives, per cell:

- the samples;
- the density near the boundary (for example, time to conflict below the reference driver's
  reaction);
- the failures and their clusters;
- the rate with its interval.

A result then reads like: "rear-end × wet × night: 4,000 weighted samples, failure rate 2e-4
(1e-4 to 4e-4), two failure modes (late braking on spray; false stop on a bridge shadow)".

## 5. Realism, per family

| Family | Where realistic distributions come from (public) | What to avoid |
|---|---|---|
| R1 | highD, exiD (highway), inD and rounD (junctions), INTERACTION, NGSIM; NHTSA's crash frequencies for weights | independent uniform speeds and gaps |
| R2 | inD and rounD (walkers, cyclists), measured walking-speed studies; PBCAT and Euro NCAP for subtypes | walkers on fixed paths that ignore the car |
| R3 | public signal timing standards; work-zone layout manuals; for map errors, the kinds we found ourselves (issues 081, 084) | random sign removal with no relation to real defects |
| R4 | meteorological records for the region (how often fog or low sun occurs), illumination by time of day | weather sampled as uniform over presets |
| R5 | road-surface statistics, friction measurements, our measured surfaces (Track C) and terrain (Haruna) | friction outside physical limits |
| R6 | rarely public; faults are injected at stated rates, and a result is reported per fault rather than folded into one rate unless rates are known | blending invented fault rates into a single risk number |
| R7 | interaction datasets (INTERACTION, inD) for gap acceptance and yielding | perfectly rational or perfectly aggressive agents only |
| R8 | the ODD definition itself (ISO 34503 attributes) | ODD exits that never happen on a real route |

Rules that hold across families:

- **Encounter-relative parameters.** Parameters are relative to the encounter.
- **Joint bases.** Distributions are joint, fitted from public data (user, 2026-11-20).
- **Behaviour models.** Other road users run behaviour models, not fixed paths.
- **Feasibility constraints,** with the rejection rate reported.
- **Weights per family.** Where weights are unknown (R6, some of R3), the family reports per
  category and does not merge into an overall rate.

## 6. Consequences for phase 092

| Need | Work item |
|---|---|
| category ids from section 2 in the sample record, and the cell (attribute values) | S1 |
| t-wise selection over discrete attributes as a sampler plug-in, beside the joint, importance and adaptive ones | K1 |
| a coverage report with the three layers of section 4 | a new component, reading records |
| fault injection through the pilot interface (R6) | a new spec, with the vehicle bridges' hooks |
| a pilot map differing from the world (R3) | a world-interface knob; the compiler binds it |
| knobs that change during a run (R4, R5, R8) | world-interface knobs with time or position |
| behaviour models (car-following, gap acceptance, strategy agents) | examples, per D1 |
| starter logical scenarios, one per category the lab studies first | examples |

## 7. Open questions

1. **Which families first?** R1, R2 and R5 fit what scensim already does best (traffic,
   pedestrians, physics). R4 depends on each simulator's sensor effects. R6 needs the fault
   hook.
2. **What t for combinatorial coverage?** Pairs are cheap; triples catch more but grow fast.
3. **Reporting R6.** Per fault, or folded in with stated assumed rates?
