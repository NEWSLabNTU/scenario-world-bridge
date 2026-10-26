# Phase 054: Sliding on Mud and Wet Ground

Make the Isaac ego slide the way a car slides on mud, wet asphalt and soft soil, at a cost a
live simulation with several vehicles can carry. Today's model already reaches the grip limit
and slides in the right directions (052, "Today's model at the grip limit on mud"), but it
lets go sharply where a tire in soft ground builds its force slowly, the soil pushes nothing
sideways, and the car has no stability control. This phase adds those in tiers, cheapest
first, each measured for what it changes and what it costs.

**Status**: in progress, 2026-10-24: steps 1-3 done (ESC in aib, off unless a rig enables it). 2026-10-25: the ESC, with the powertrain, ABS and traction control, moves out of aib into `sim_vehicle_ecu` and onto the car's own sensors ([055](055-onboard-computers.md), issue 007); step 4 waits for it. 2026-10-25: 055 done; step 4's design explored and decided ("T1 in detail" below). 2026-10-25: step 4 (T1) done, with issue 009 fixed in `sim_vehicle_ecu`. 2026-10-25: step 5 (T2) done. 2026-10-26: step 6 done, with issue 010 fixed in swb_isaac_world; step 7 measured (T2 +1.3 % on four egos by estimate, the frame time to repeat on a quiet host); step 8 done (two of three pass; the mud stop does not drive off, explained). Open: the suite side-by-side table, rut crossing at a shallow angle, the four-ego frame time.

**Source**: user direction 2026-10-24: "I expect that the muddy road would tilt the steering",
then "I'm looking for proper physics modeling and see if we can simulate the effect in a
realistic way", then "I prefer to explore the design that balances the fidelity and
computation effort for live simulation. we may check game and physics simulation studies for
vehicle, plane, etc.", then "write up a phase doc".

**Depends on**: [047](047-off-road-surfaces.md) (the surface and soil layers),
[052](052-isaac-physics-completeness.md) (the limit test, `spikes/limit_mud.py`; the
several-ego cost spike, `spikes/multi_ego_soil.py`). **Feeds**:
[053](053-terramechanics-calibration.md) (the new soil terms' parameters are fitted there),
[039](039-isaac-multi-autoware.md), [040](040-ntu-offroad-field.md).

Not in this phase: fitting any parameter to measurements (053; values here are engineered and
say so); CARLA, whose tire model is its own; particle or MPM soil; changing the soil grid's
normal-pressure model.

## Where it starts

Measured 2026-10-24 (052), one ego without Autoware, 40 km/h, open loop:

| case | asphalt | mud (`mud_wet`, friction 0.23 of dry) |
|---|---|---|
| straight full braking | 8.8 m, 7.1 m/s^2 | 16.9 m, 3.55 m/s^2, ABS 92 % of the time, straight |
| split-mu braking, left wheels in mud | | yaws 7.6 deg toward the asphalt side |
| 5 deg tire-angle step, speed held | yaw rate 0.99 of the kinematic, 3.54 m/s^2 lateral | power oversteer: rear slip angle 0.3 to 6.5 deg in 1.2 s, yaw rate not settling, 1.25-1.35 m/s^2 lateral |
| 1.5 deg turn, then full braking | heading 9.4 deg at the stop | heading 22.9 deg, rear slip angle to 16 deg |

What the model has and lacks:

| | today |
|---|---|
| friction circle (braking and cornering share one grip) | PhysX's tire, friction written per wheel every substep from its ground |
| per-wheel ground (split-mu) | yes |
| ABS, traction control | aib's powertrain |
| stability control (ESC) | **none**; the real Model 3 has one |
| tire on mud | asphalt's lateral stiffness and friction-vs-slip curve, only the friction value scaled: PhysX refuses edits to the curve while playing. The onset of a slide is sharp |
| soil forces | resistance along the rolling direction only (`SurfaceLayer._soil`, `apply_force` at the contact); no lateral shear, no sidewall bulldozing, no force from rut walls |
| soil cost | the grid, 0.45 ms per wheel and substep in Python; 5.5 ms per ego per frame (052 step 5) |

## Design

What the survey found (2026-10-24; games, vehicle dynamics, flight and rover simulators,
PhysX's tire):

- **Games get convincing soft-surface slides from the shape of each surface's force curve,
  not from soil mechanics**: a drop from static to sliding friction (BeamNG's mud 0.55 to
  0.35), one friction budget for both directions, and either a high update rate (BeamNG at
  2 kHz) or a detailed tire run offline into tables (rFactor 2).
- **Flight simulators layer tabulated physics over a few measured anchors**: a Level D
  simulator's measured stopping-distance check is on a dry runway; wet and contaminated
  runway braking may come from tables.
- **On soft soil the lateral force has two parts** besides the tire's own: shear under the
  wheel along the slip direction and the reaction of soil bulldozed against the wheel's side,
  growing with sinkage (Ishigami et al. 2007, *J. Field Robotics* 24(3):233-250).
- **Isaac exposes no custom tire.** PhysX 5's vehicle SDK can replace the tire component, but
  `omni.physx.vehicle`'s Python binding (110.3.2 here) has no tire callback. The hooks are the
  per-wheel friction value and a force at a position on the body. Enough for a tire of our own
  on soil: PhysX's friction near zero there, our force at the contact, and the soil's reaction
  taken off the wheel's drive torque, which aib already sets every substep.
- **A lateral force applied from the last state is unstable at low speed at 1/60 s.** Our own
  estimate from the rig's numbers (cornering stiffness about 154 kN/rad per axle, about 920 kg
  on it): explicit damping is stable only above about 1.4 m/s, about 2.8 m/s with the force
  path's one-substep lag. Fixes: the lateral velocity damping semi-implicit
  (v_y / (1 + C dt / (m v))), the slip through a relaxation-length lag (time constant
  sigma / v, sigma 0.5-1.5 m, stable to far above road speeds and the soft-soil feel besides),
  and below about 3 m/s the soil's lateral force faded out and PhysX's low-speed tire in.
  Sub-cycling buys little: our hook runs once per PhysX step.

Three tiers, chosen per zone and per vehicle:

| tier | where | model | cost per wheel and substep |
|---|---|---|---|
| **T1** shaped PhysX tire | all rigid ground, every vehicle | PhysX's tire; per surface class the friction written each substep as its peak times its own curve over the wheel's combined slip (static to sliding drop, a Stribeck speed term, the water film already in `hydroplane_factor`); ESC in `sim_vehicle_ecu` (055) | microseconds (one value written, as today) |
| **T2** brush on soil | soil zones, the ego and physics vehicles near it | our tire: a brush of 10-20 points along the contact patch, Janosi shear (c + sigma tan phi)(1 - exp(-j/K)) along the combined slip direction with sigma from the grid's sinkage under the wheel, giving F_x, F_y and M_z; sidewall bulldozing growing with sinkage against lateral slip; a rut-wall term from the grid's height gradient across the tire; relaxation lag, semi-implicit damping, PhysX below 3 m/s | 50-100 us in Python per wheel, about 30 us batched with numpy (estimate) |
| **T3** grid-integrated shear | reference and calibration only | each grid node's shear along its own slip velocity, as Chrono SCM | 0.45 ms today, +20-40 % with direction (estimate) |

T2 is the balance point: it reuses what the grid already computes (pressure, sinkage, the
ruts, the part with history) and replaces only the shear integration, so it adds every missing
mud effect for about a tenth of the grid's cost.

The table is the plan as first written. T2 as decided is narrower: PhysX keeps the shear, and
T2 adds the soil against the tire's side ("T2 in detail" below).


### T1 in detail (explored and decided 2026-10-25)

**What already exists.** A class may carry a `slip:` curve, friction over longitudinal slip
(mud: 0.7 at no slip, 1.0 at 0.25, 0.9 locked), applied through the wheel's friction value as
class(slip) / vehicle(slip) (`surfaces.WheelSurfaces.update`). So the longitudinal half of T1
is in place; nothing acts sideways.

**Why a slide on mud snaps.** PhysX's default tire (as PhysX 3 and 4 wrote it; step 4a
confirms it on 5) builds force linearly with slip at the tire's stiffness C and saturates at
the friction value times the load, near a slip of 2 mu F_z / C, flat beyond. The stiffness is
the rig's on every ground, so only mu moves the peak, and on low grip it moves to a tiny slip.
With the rig's tire (lateral stiffness about 69 kN/rad, longitudinal 45 kN, about 4300 N per
wheel):

| ground | friction, relative to dry | slip angle at the peak | longitudinal slip at the peak |
|---|---|---|---|
| dry asphalt | 1.0 | about 7 deg | about 0.19 |
| wet mud | 0.35 | about 2.5 deg | about 0.07 |
| packed snow | 0.24 | about 1.7 deg | about 0.05 |
| ice | 0.12 | about 0.8 deg | about 0.02 |

A stiff tire on a slippery surface: ice, wherever the class says mud. And past the peak the
force stays flat, because PhysX's friction-vs-slip graph reads longitudinal slip only: a wheel
sliding sideways keeps its peak friction. Shaping the friction value alone (the plan before
this exploration) would add the sliding drop but leave mud's peak at 2.5 deg, still snapping.

**Decided: friction value and stiffness, per class** (user, 2026-10-25, against friction
alone and against our own tire force, which stays T2's).

- **Stiffness puts PhysX's own peak where the class's is.** Per class, `peak_slip` and
  `peak_slip_angle`; each wheel's longitudinal stiffness and lateral stiffness graph scaled
  by (class mu / reference mu) x (reference peak / class peak), so that 2 mu F_z / C lands on
  the class's peak. The reference ground's peaks are the rig's own (from its stiffness and
  nominal load), so dry asphalt's linear range, everything Autoware and the ESC are tuned in,
  does not change. Written only when the wheel's blended ground changes by more than a few per
  cent (step 1: both attributes take while playing), never every substep.
- **The friction value falls past the peak, in both directions.** With kappa the wheel's
  longitudinal slip, alpha its slip angle (`TIRE_LATERAL_SLIP`, which PhysX reports and aib
  does not read yet) and rho = sqrt((kappa / kappa_peak)^2 + (tan alpha / tan alpha_peak)^2)
  the combined slip against the class's own peaks: g(rho) = 1 up to rho = 1, then
  s + (1 - s) / rho, s the class's `sliding` share of the peak (mud seeded from BeamNG's
  0.55 to 0.35, about 0.64). The value written is tire_friction x peak x g(rho) / vehicle
  graph(kappa), dividing out the curve PhysX applies itself. This replaces `slip:` (no
  shims: the key is refused by name, not read alongside).
- **Dry asphalt gets the sideways drop too** (user, 2026-10-25: realism past the limit over
  keeping the dry numbers). The reference class gets `sliding` like any other, and a world
  without a surface map runs every wheel on the reference class, so a map of only asphalt
  still drives as no map at all. Everything measured on dry ground past the limit changes:
  the 054 suite, 055's table, vehicle-check and the Autoware scenarios are run again (step 4).
- **Sliding speed (Stribeck), optional per class** (user, 2026-10-25): `stribeck: {speed,
  sliding}` lowers the sliding share with the sliding speed v_s = |v| sqrt(kappa^2 +
  tan^2 alpha): s(v_s) = sliding_fast + (s - sliding_fast) exp(-(v_s / speed)^2). Meant for
  wet asphalt and ice; engineered values, `unvalidated:`.
- **Relaxation in T1 delays the drop, not the force.** T1 cannot lag PhysX's force, so the
  falling part of g passes a first-order lag with time constant `relaxation` / |v| (sigma
  0.5-1.5 m): a softer onset, and damping for the loop a friction written one substep late
  closes (the drop raises the slip, which lowers the friction). Below 1 m/s no lag.
- **aib** reads the lateral slip into its `WheelState` and gets a per-wheel stiffness setter
  beside `set_wheel_friction`. The onboard computers (`sim_vehicle_ecu`) do not change: they
  find the new limits through their estimates. ABS and TC release at 0.25 slip, about mud's
  peak and before sand's (0.3); the suite shows what that costs.
- **Cost**: as today, one value per wheel and substep; stiffness writes at ground changes only.

Per class, then: `mu`, `peak_slip`, `peak_slip_angle` (deg), `sliding`, `relaxation` (m),
optional `stribeck`. The reference class leaves the peaks out (the rig's). The first values
come from the literature's ranges (dry asphalt peaks at 4-8 deg and 0.10-0.15 slip, sliding
0.75-0.85 of the peak; soft and loose ground at larger slips and lower sliding shares; ice at
small ones), each class `unvalidated:` until 053.

### T2 in detail (explored and decided 2026-10-25)

Explored against the code after T1:

- **T1 already shapes the shear on soil.** A soil wheel's friction is the soil's traction
  limit over its load, and since T1 it follows the class's curve: mud peaks at 12.8 deg. So
  the step steer on mud already builds over a larger slip angle than on asphalt.
- **The rut's floor is already PhysX's.** Where a zone has `ground_z`, each wheel rides on a
  plate fitted to the ruts over its footprint (`soil_tiles`, 047 S3): a tire half over a rut
  sits half in it, tilted toward it. A rut-wall force on top would count it twice.
- **Our own tire on soil (the brush as first written) needs our own wheel.** With PhysX's
  friction near zero, the brush's F_x must come back on the wheel's spin as a torque the host
  sets a substep late. Mud's shear stiffness (about 10 kN per unit slip) on a 1.4 kg m^2 wheel
  at 1/60 s is explicit-stable only above about 4-8 m/s (the slip's denominator is |v| + 4):
  it would oscillate at the speeds Autoware takes mud at (13 km/h). It needs the wheel
  integrated implicitly on soil, a spike of its own.
- **What no tier has yet: the soil against the tire's side.** Pushed sideways, a sunk tire
  drives its side into the soil, which answers with its passive earth pressure (Ishigami et
  al. 2007's bulldozing, Bekker's resistance per unit width): on fresh soil at the sinkage, in
  an old rut at the height its wall stands above the wheel's bottom.

**Decided (user, 2026-10-25): T2 adds to PhysX (option B), on every physics vehicle on soil.**

- **The sidewall force**, per wheel on soil: `soil.sidewall`, Bekker's bulldozing resultant
  over the contact length, `L (0.67 c z K_pc + 0.5 gamma z^2 K_pgamma)` with
  `K_pc = (N_c - tan phi) cos^2 phi`, `K_pgamma = (2 N_gamma / tan phi + 1) cos^2 phi`
  (Terzaghi's factors, Vesic's N_gamma), z the height the soil beside the leading side stands
  above the wheel's bottom (`SoilGrid.side_heights`: the nodes one and two spacings outside
  the tire, along its contact; the closed-form wheel: the sinkage), L the contact chord.
  Mud at 4 cm of sinkage: about 210 N a wheel; at 8 cm, 740 N.
- **Against the contact's lateral speed**, `tanh(v_lat / 0.1 m/s)` (none rolling straight, full
  sliding), v_lat from PhysX's lateral slip; through the class's relaxation length as T1's
  drop; at the contact, beside the resistance, in one force. A body force, so the wheel's spin
  is untouched and nothing new runs explicit on a stiff loop: its damping, 2 kN s/m a wheel,
  is two orders under the step's limit.
- **New soil key**: `unit_weight` (N/m^3, default 16000), for the weight term.
- **Our own tire on soil (option A)** stays an open question below, for when T2's numbers say
  the shear needs more than T1's curve.

## Steps

- [x] **1. What PhysX lets us change while playing.** A spike setting each tire attribute on a
  running vehicle (lateral stiffness graph, longitudinal stiffness, friction-vs-slip graph,
  camber stiffness) and reading the effect back in a step steer. If the lateral stiffness
  takes, T1 can soften the tire on mud by itself; write down which do and which do not.
- [x] **2. A limit-test suite.** `spikes/limit_mud.py` grown into the standard open-loop
  manoeuvres on asphalt, wet asphalt and mud, every frame kept: step steer (ISO 7401),
  steady-state circle (ISO 4138), double lane change (ISO 3888), split-mu braking (ISO 14512),
  braking in a turn (ISO 7975), sine with dwell (FMVSS 126). Today's numbers are the baseline
  every later step is compared with.
- [x] **3. ESC in aib.** Beside ABS and traction control in `aib.powertrain`: a yaw-rate
  target from the bicycle model capped at mu g / v, braking one wheel against the error; rig
  parameters with `esc_enabled`, so it can be compared off and on. Checked with sine with
  dwell on dry asphalt against FMVSS 126's criteria (yaw rate at most 35 % of its peak 1 s
  after the steering ends, at most 20 % at 1.75 s; at least 1.83 m lateral displacement at
  1.07 s).
- [x] **4. T1: each surface's own curve** (design: "T1 in detail" above).
  - [x] **4a. PhysX's tire, measured** (below). A spike sweeping the slip angle (a slowly growing steer
    at constant speed) and the slip (a ramped brake) on dry asphalt and on mud, today's
    tire: `TIRE_FORCE` over the load against PhysX's reported slips. Confirms the peak at
    2 mu F_z / C and the flat top; if not, the stiffness rule changes before anything is
    built.
  - [x] **4b. The slip sweep in the suite.** 4a's two sweeps as manoeuvres of the suite, per
    class, so every later change shows each class's realized curve.
  - [x] **4c. T1.** The class table's new keys (`slip:` refused), the lateral slip and the
    stiffness setter in aib, the stiffness rule and g(rho) with the lag in the surface layer,
    the reference class applied without a surface map; unit-tested like `surfaces.py`.
  - [x] **4d. Measure.** The suite (now with the sweeps) against step 2's and 055's numbers; aib's
    vehicle-check; town01_mud with Autoware, recorded. On mud the step steer's lateral force
    builds over a larger slip angle than on asphalt and falls toward the sliding share past
    it.
- [x] **5. T2: the soil against the tire's side** (design: "T2 in detail" above).
  `soil.sidewall`, `SoilGrid.side_heights`, the force in `SurfaceLayer._soil` beside the
  resistance, `unit_weight`; unit-tested like the grid; the suite against T1's numbers.
- [x] **6. Low-speed and edge checks.** Creeping across a zone edge at 0.5-3 m/s, stopping and
  holding on soil, starting from rest in a rut, one side on soil: no oscillation in yaw rate or
  lateral velocity, no jump at the hand-back speed.
- [x] **7. Cost.** `spikes/multi_ego_soil.py` with T2 on: one, two and four egos, frame p50 and
  p95 and the soil layer's share beside 052 step 5's (29.2 / 45.5 / 77.4 ms at p50).
- [x] **8. With Autoware.** New scenarios on Isaac, each recorded with `record:=` for the video:
  an emergency stop on mud (a pedestrian stepping out), braking with mud on only the right
  half of the lane, a curve in mud taken at the speed limit. Each passes, or fails for a
  reason written down (a slide Autoware cannot hold is a finding, not a bug).

## Acceptance

- [x] Which tire attributes PhysX accepts while playing, written down (step 1).
- [ ] The manoeuvre suite runs from one command on asphalt, wet asphalt and mud, and its
  results for today's model, T1 and T2 sit side by side here.
- [x] ESC meets FMVSS 126's sine-with-dwell criteria on dry asphalt, and off it is today's car.
- [ ] On mud, the step steer's lateral force builds over a larger slip angle than on asphalt
  (the slide no longer snaps), and a car crossing ruts at a shallow angle feels a lateral force
  from them.
- [x] No oscillation below 3 m/s on soil, at zone edges or at the hand-back (step 6).
- [ ] Four egos on soil with T2 within 10 % of 052 step 5's frame time (step 7: +1.3 % by the per-wheel measurement; the frame time to repeat on a quiet host).
- [x] Step 8's scenarios pass on Isaac with Autoware, or their failures are explained (`town01_mud_stop` stops without contact but does not drive off: the car on mud_wet and Autoware's controller, step 8).

## Progress

### Step 1: what PhysX accepts while playing (2026-10-24)

`spikes/tire_runtime.py`: one ego without Autoware on Town01's asphalt at 40 km/h, each case
a fresh vehicle with one attribute of every wheel's `PhysxVehicleTireAPI` edited on the
playing stage before the manoeuvre (and once during the run-up), against the unedited car.
Frames in `play_log/tire_runtime/2026-10-24_22-39-11/`.

| attribute | edit | effect | verdict |
|---|---|---|---|
| `lateralStiffnessGraph` | value x 0.25 (68976 to 17244 N/rad) | 5 deg step: slip angles 1.6 to 4.1 (rear) and 4.7 deg (front), yaw rate at 0.3 s 15.9 to 8.2 deg/s, steady 18.3 to 16.0 deg/s, lateral 3.55 to 2.21 m/s^2; the same when edited mid run-up | **takes**, any time |
| `longitudinalStiffness` | x 0.25 (45249 to 11312) | braking at 4 m/s^2 asked: mean wheel slip 0.043 to 0.173, deceleration unchanged (3.87, 3.96) | **takes** |
| `frictionVsSlipGraph` | values halved | full braking: 7.95 m/s^2 either way; PhysX logs `Attribute "physxVehicleTire:frictionVsSlipGraph": modification not allowed once the simulation has been started. Changes will be ignored.` | **refused** (as 047 found) |
| `camberStiffness` | 0 to 1000 | none (the car has no camber), no complaint | accepted, untestable here |

So T1 can soften a tire on mud directly: the cornering and longitudinal stiffness per wheel,
written like the friction value when its ground changes. The curve's shape over slip (the drop
from peak to sliding) still has to come through the friction value, from the wheel's slip.

### Step 2: the manoeuvre suite, today's car (2026-10-24)

`spikes/manoeuvres.py` on a flat proving ground it makes at its first run (one 660 x 560 m mesh
labelled Road) with `spikes/proving_ground.yaml`'s pads: dry asphalt, wet asphalt
(`wetWithPuddles`, friction 0.75), wet mud (`mud_wet`, the soil grid with no `ground_z`, so the
wheels stay on the flat ground and there are no render tiles) and a split pad (the left wheels
in mud). One command runs it all (`PADS=`, `MANOEUVRES=` narrow it; `RIG=` overrides the rig's
vehicle fields); frames and `summary.json` under `play_log/manoeuvres/<time>/`. The driver
and inputs are fixed: the numbers compare models with each other, not with a real Model 3.

The mud pad first used the closed-form wheel, to spare the grid's memory: it meets fresh soil
under every wheel at every step, and held the car to 26 km/h at 3 m/s^2 asked (braking read
6.5 m/s^2 against the grid's 3.5). On the grid the pad agrees with Town01's mud: straight
braking 16.1 m and 3.52 m/s^2 (Town01 16.9 m, 3.55), split-mu braking yaws 7.65 deg (7.6).

### Step 3: ESC (2026-10-24)

`aib.esc` (aib `1d49569`): the bicycle model's yaw rate at the present speed and tire angle,
capped at 0.85 mu g / v, against the measured one, with a dead band of 0.035 rad/s plus 10 % of
the target; a sideslip past 0.07 rad counts as oversteer whatever the yaw rate. Oversteer
brakes the outer front wheel and cuts the motor; understeer brakes the inner rear. The yaw
moment (12000 N m per rad/s of error, 30000 per rad of sideslip) becomes one wheel's brake
torque (at most 2000 N m) through the hydraulics' lag, added ahead of ABS, so ABS still keeps
the wheel turning. The friction is the one the host writes under the wheels: an idealised ESC,
where a real one estimates it. A rig's `esc_*` fields, from a spec's `brakes.esc`; off by
default, and the shipped Model 3 spec has none yet. Unit tests in `test/test_esc.py`.

The suite without and with it (`RIG='{"esc_enabled": true}'`; `play_log/manoeuvres/
2026-10-24_22-44-02` dry and wet, `2026-10-24_22-50-32` mud and split, `2026-10-24_22-55-47-esc`):

| case | without ESC | with ESC |
|---|---|---|
| dry sine with dwell, 80 km/h (6.67 deg) | yaw rate 43 % of its peak 1 s after the steering ends, 0.7 % at 1.75 s: **fails** FMVSS 126 (35 %, 20 %); heading 51 deg, sideslip 13 deg; 2.77 m lateral at 1.07 s | 0 % and 0 %: **passes**; heading 15 deg, sideslip 6.2 deg; 2.69 m (at least 1.83) |
| wet sine with dwell, 80 km/h | 92 % and 88 %: a spin, heading 82 deg, sideslip 34 deg | 0 % and 0 %; heading 12.6 deg, sideslip 5.3 deg; 2.32 m |
| dry / wet double lane change, 80 km/h | path error 0.77 / 0.86 m | 0.66 / 0.71 m, 78 km/h at the end |
| mud step steer, 40 km/h | sideslip 10.8 deg and growing, yaw rate 1.15 of the kinematic | sideslip 2.6 deg, yaw rate 0.67 of the kinematic (held to what the mud carries) |
| mud double lane change, 40 km/h | spins: sideslip 42 deg, path error 3.5 m | path error 0.19 m, 31 km/h at the end |
| dry / wet / mud circle, 40 m | the most lateral acceleration held 8.41 / 7.35 / 1.7 m/s^2 | 7.26 / 6.49 / 1.51: ESC steps in at 0.85 mu g |
| straight braking, steps on dry and wet | | unchanged (to the centimetre) |
| mud braking in a turn | heading 18.3 deg, sideslip 5.4 deg | 18.8, 5.7: unchanged |
| split-mu braking | yaws 7.65 deg | 7.66: unchanged |

Two things it does not do. On split-mu braking it asks for a moment against the yaw by braking
the front wheel on the mud side, where ABS already holds the brake at the mud's limit, so
nothing changes; a real ESC (with ABS's yaw-moment limitation) lowers the brake on the
high-grip side instead. And braking in a turn on mud is unchanged for the same reason: every
wheel is at ABS's limit. Both are for a later step (taking brake off rather than adding it);
neither matters with ESC off, which stays the default until it is decided for the shipped car.

The circle also shows the car steering neutrally: an understeer gradient of 0.06 deg/g on dry
asphalt and -0.04 on wet (the yaw rate 0.99 of the kinematic in the step). A road car
understeers; this is the rig's tires (equal stiffness per load front and rear), for T1.

### Step 4a: PhysX's tire, measured (2026-10-25)

`spikes/tire_curve.py` on the proving ground, ABS off for the braking sweep: a steering ramp
(0 to 25 deg at 4 deg/s, 8 m/s held) and a braking ramp (0 to 10 m/s^2 asked over 2 s), each
on dry asphalt, on dry asphalt with the rig's tire friction at 0.35 (`dry035`: the same tire,
only the friction lower) and on the mud pad. Every frame reads each wheel's `TIRE_FORCE`,
both slips, the suspension force and `TIRE_FRICTION` from PhysX
(`play_log/tire_curve/2026-10-25_4a/`, `curves.png`).

| | dry | dry035 | mud (soil friction 0.17-0.19) |
|---|---|---|---|
| lateral stiffness C (small slip), N/rad | 62 000 front, 69 000 rear | 58 000, 66 000 | 54 000 front |
| slip angle where the force tops out | not reached at 25 deg steering (5.8 deg front at 0.94 of the cap) | 2.7-3.4 deg | 1.6-1.8 deg |
| 2.3 mu F_z / C | 8.6 deg | 3.2 deg | 1.7 deg |
| longitudinal stiffness, N per unit slip | 40 000-44 000 | 39 000-41 000 | 36 000-38 000 |
| longitudinal slip where it tops out | 0.25 front | 0.09-0.11 | about 0.05 |
| 2.3 mu F_z / C | 0.23 | 0.085-0.09 | 0.045-0.05 |
| past the top | flat to 0.4 slip | flat to 17 deg and 0.7 slip | flat to 18 deg and slip 3 |

What it settles:

- **The design's premise holds.** The force rises at the tire's stiffness, bends over and
  saturates at a slip of about 2.3 mu F_z / C in both directions, then stays flat: no drop
  at all, sideways or lengthways. The peak's slip is proportional to the friction: the same
  tire peaks at 3 deg on 0.35 and at 1.7 deg on mud's 0.17, where a real tire on mud peaks
  far later.
- **The cap is about 1.15 mu times the suspension force**, not 1.0: the tire's load includes
  more than the suspension reports (the wheel's own weight is 196 N of it). A constant; the
  rule takes the measured 2.3 instead of 2.
- **The stiffness follows the load**, C/F_z about 15 sideways (the graph's 17 per N of rest
  load, a little less in use), so the scaling works on the graph's value and keeps its shape.
- **PhysX's friction-vs-slip graph reads the longitudinal slip only**: `TIRE_FRICTION` fell
  from 0.35 to 0.31 as the wheel locked (the rig's graph, 1.0 to 0.8), and stayed at 0.35
  through the whole steering sweep. So the sideways drop has to come through the friction
  value, as designed.
- **The rig's own peaks on dry asphalt**, the reference T1 keeps: about 8.6 deg and 0.24
  slip, softer than the 4-8 deg and 0.10-0.15 the literature gives for a road tire.
  Not changed here.

So the stiffness rule is: to put a class's peak at slip angle alpha_c with friction mu_c,
scale the lateral stiffness by mu_c tan(alpha_rig) / tan(alpha_c), alpha_rig the rig's peak at
friction 1.0 (tan 0.15); the same lengthways with the rig's 0.24.

### Steps 4b-4d: T1, built and measured (2026-10-25)

**Built.** aib (`9416d80`): each wheel's slip angle and tire force read back,
`set_wheel_stiffness` (shares of the rig's, written on a change of more than 2 %).
swb_isaac_world: the class table's `slip:` graphs replaced by `peak_slip`, `peak_slip_angle`,
`sliding`, `relaxation`, `stribeck` (the old key refused by name); `surfaces.WheelSurfaces`
scales the stiffness and lowers the friction past the peak as designed; a world without a
surface map runs on the reference class; the suite gains `sweep_lat` (from 10 m/s, the tire
angle to 35 deg at 5 deg/s) and `sweep_long` (ABS off, 15 m/s^2 asked over 1 s), each
summarising each axle's realized curve. First values, engineered (`surface_classes.yaml`):

| class | peak slip | peak slip angle | sliding | relaxation |
|---|---|---|---|---|
| asphalt, concrete | the rig's (0.24) | the rig's (8.6 deg) | 0.8 | 0.5 m |
| gravel | 0.3 | 12 deg | 0.9 | 1.0 m |
| dirt | 0.2 | 10 deg | 0.85 | 1.0 m |
| grass | 0.15 | 9 deg | 0.75 | 1.0 m |
| sand | 0.3 | 14 deg | 0.95 | 1.5 m |
| mud_wet | 0.25 | 12 deg | 0.64 | 1.2 m |
| snow_packed | 0.15 | 6 deg | 0.8 | 0.8 m |
| snow_loose | 0.3 | 12 deg | 1.0 | 1.5 m |
| ice | 0.08 | 3 deg | 0.6, Stribeck to 0.4 past 3 m/s | 0.5 m |

**The curves come out as designed** (`play_log/manoeuvres/2026-10-25_t1-sweeps2/curves.png`).
Sideways: dry asphalt peaks at 8.8 deg (the rig's 8.6) and falls to 0.88 of the peak at
24 deg (0.87 predicted); wet asphalt alike, at 0.75 of the friction (stiffness 0.75); mud
peaks at 12.8 deg (the class's 12; before T1, 1.7 deg) and falls to 0.78 of the peak at 34 deg
(0.75 predicted). Lengthways the drop shows less (dry 0.93 of the peak at 0.64 slip, wet 0.94,
mud's rear 0.83 at 1.9): the brakes take the wheels only to about 0.6 slip, and the lag holds
at low speed.

**Mud grips more sideways than before, by design.** The old `slip:` graphs started below 1 at
zero slip (mud 0.7, sand 0.6), and PhysX reads them by longitudinal slip only, so a wheel
rolling straight while cornering had its sideways grip cut by that share (off-road-surfaces.md
noted it at 047). T1 gives a wheel below its peak the class's full friction in every
direction: mud's sideways peak rose from 0.167 to 0.237 of the load (the soil layer's own
0.237 both times; checked with and without T1's stiffness, `play_log/tire_curve/2026-10-25_soil-{on,off}`).

**The suite with T1** (`2026-10-25_t1-ecu-{off,esc}`, against 055's estimator runs):

| case | 055, ESC off | T1, ESC off | 055, ESC on | T1, ESC on |
|---|---|---|---|---|
| dry: brake, step, lane change, circle | | unchanged (below the peak) | | unchanged |
| dry sine with dwell, yaw ratio at 1 s | 0.432 | 1.005 (spins: heading -100 deg) | 0.0 | 0.0 (passes) |
| wet brake, distance / deceleration | 9.21 m / 6.94 | 9.30 m / 6.46 | 9.21 m / 6.94 | 9.30 m / 6.46 |
| wet sine, yaw ratio at 1 s / lateral at 1.07 s | 0.918 / 2.42 m | 1.0 (spins) / 2.08 m | 0.0 / 2.16 m | 0.001 / 1.87 m (limit 1.83) |
| wet circle, lateral acceleration held | 7.41 | 6.68 | 7.39 | 6.63 |
| mud brake, distance / deceleration | 16.89 m / 3.54 | 17.96 m / 3.15 | 16.89 m / 3.54 | 17.96 m / 3.15 |
| mud step, steady sideslip | -10.27 deg | -9.25 deg | -5.6 deg | -6.66 deg |
| mud sine, heading change | -4.7 deg | -63.5 deg (spins) | -4.9 deg | -18.4 deg |
| mud lane change, path error | 3.39 m | 3.45 m (spins: heading -238 deg) | 0.21 m | 1.02 m |
| mud circle, lateral acceleration held | 1.78 | 0.28 (cannot hold the speed) | 1.59 | 0.28 |
| split-mu brake, heading | -7.72 deg | -8.5 deg | -7.68 deg | -8.5 deg |

- **Past the limit the car now lets go the way a car does**: without ESC the dry, wet and mud
  sines and the mud lane change spin; with ESC the dry and wet sines still pass FMVSS 126
  (the wet one's lateral displacement now 1.87 m, close to its 1.83 m floor) and the mud lane
  change holds the path to 1.0 m. The mud sine was a non-event before (the car barely turned
  on 0.167 of sideways grip); now it can turn, and spin.
- **On mud traction control starved the car, and the cause was 055's slip estimate, not the
  thresholds** (issue 009). The mud circle slowed from 4.5 to 0.1 m/s against a driver asking
  for 3 m/s^2, traction control's factor at 0.15-0.33, and the mud stop was 1.1 m longer. The
  brake module divided the wheel's slip by `max(|v|, 4 m/s)` where PhysX divides by
  `|v| + 4 m/s`, so it read 1.9 times PhysX's slip at 4.5 m/s (0.197 against 0.103 driving
  straight on mud) and ABS and traction control, tuned against PhysX's slip, acted at about
  half their slip. T1's softer mud tire only made it show. With the estimate on PhysX's
  definition (`sim_vehicle_ecu` `8742540`; `2026-10-25_009-ecu-{off,esc}`):

  | case | T1, ESC off | T1 + 009, ESC off | T1, ESC on | T1 + 009, ESC on |
  |---|---|---|---|---|
  | mud circle, lateral acceleration held | 0.28 | 1.50 (to 24 km/h) | 0.28 | 1.35 (to 29 km/h) |
  | mud brake, distance / deceleration | 17.96 m / 3.15 | 15.71 m / 3.65 | 17.96 m / 3.15 | 15.71 m / 3.65 |
  | mud brake in a turn, distance | 17.68 m | 14.93 m | 17.62 m | 14.75 m |
  | mud lane change, path error / end speed | 3.45 m (spins) | 4.93 m (spins) | 1.02 m / 0.3 km/h | 0.99 m / 20.6 km/h |
  | mud sine, yaw ratio at 1 s | | | 0.094 | 0.048 |
  | split-mu brake, distance / heading | 11.32 m / -8.5 deg | 10.76 m / -7.9 deg | 11.32 m / -8.5 deg | 10.75 m / -7.86 deg |

  Dry and wet are unchanged within 2 % but for the wet stop's ABS share. Traction control now
  works the rear wheels at 0.22-0.28 of PhysX's slip on the mud circle, mud's peak. The fixed
  thresholds (0.25, 0.15) still sit past the peak on ice (0.08) and packed snow (0.15) and
  before it on sand and gravel (0.3); slip targets that follow the ground are a follow-up for
  `sim_vehicle_ecu`, with ice and snow cases in the suite to measure them.
- Then each wheel is held against the road speed its place on the car gives in a turn (the
  yaw rate times half the tread, the steered fronts along their heading; `sim_vehicle_ecu`
  `03b7065`, `2026-10-25_009b-ecu-{off,esc}`): the inner and outer wheels no longer read
  slips 0.02 apart that are not there. Within the runs' scatter past the limit otherwise (mud
  circle 1.38 / 1.45 m/s^2 held, ESC off / on); the mud stop's false 6.6 deg peak sideslip
  goes to 0.4; the sines unchanged.
- With both fixes aib's vehicle-check passes (`play_log/vehicle_check/2026-10-25_009`) and
  `town01_mud.xosc` with Autoware on Isaac passes, recorded
  (`play_log/record/2026-10-25_19-17-54-ego/run.mp4`): across the mud at 13.0-14.4 km/h,
  Autoware asking up to 0.67 m/s^2; the rear wheels at up to 0.086 of PhysX's slip, which the
  brake module now reads as 0.087 (before the fix it would have read about twice that);
  traction control never acts.
- aib's vehicle-check passes (`play_log/vehicle_check/2026-10-25_t1`; aib's own runner has no
  surface layer, so it checks the aib change only).
- `town01_mud.xosc` with Autoware on Isaac passes with T1, recorded
  (`play_log/record/2026-10-25_15-43-56-ego/run.mp4`): the car crosses the mud at 13-14 km/h
  as before, Autoware asking up to 0.64 m/s^2; the rear wheels run at 0.06-0.08 slip on the
  mud (0.02 before: the softer tire), under traction control's threshold, so it never acts.

### Step 5: T2, built and measured (2026-10-25)

**Built** (swb_isaac_world): `soil.sidewall` (with `bearing_factors`, `contact_length`),
`SoilGrid.side_heights` and `WheelSoil.side`, the force in `SurfaceLayer._soil` beside the
resistance (`_sidewall`: lagged over the class's relaxation length, `side_force` per wheel, the
recorder's `side_n`), the soil key `unit_weight`; five new unit tests (124 in all).

**The suite with T2** (`2026-10-25_t2-ecu-{off,esc}`, against T1 with issue 009's fix,
`2026-10-25_009b-*`). Dry and wet unchanged (no soil); on mud:

| case | T1, ESC off | T2, ESC off | T1, ESC on | T2, ESC on |
|---|---|---|---|---|
| mud step, steady sideslip / yaw rate ratio | -10.3 deg / 1.16 | -5.8 deg / 0.84 | -6.7 deg / 0.86 | -5.3 deg / 0.82 |
| mud circle, lateral acceleration held | 1.38 | 1.87 | 1.45 | 1.73 |
| mud sine, yaw ratio at 1 s / lateral at 1.07 s | 0.955 / 0.38 m | 0.838 / 0.51 m | 0.048 / 0.39 m | 0.002 / 0.54 m |
| mud lane change, path error | 4.2 m (spins) | 5.4 m (spins) | 0.99 m | 0.88 m, 17.0 s (21.0) |
| mud brake in a turn, peak sideslip / distance | -3.8 deg / 14.9 m | -0.6 deg / 15.4 m | -4.2 deg / 14.7 m | -0.7 deg / 15.3 m |
| mud lateral sweep, peak sideslip | -81 deg | -25 deg | -11.3 deg | -9.9 deg |
| split-mu brake, heading | -7.9 deg | -6.2 deg | -7.9 deg | -6.1 deg |
| mud brake, distance | 15.7 m | 15.7 m | 15.7 m | 15.7 m |

The sunk tires' sides hold the car on mud: about a third more lateral acceleration held on
the circle, half the step steer's sideslip, a slide past the limit far slower to grow (the
lateral sweep's tail). Braking straight is unchanged (no lateral speed); in a turn the stop is
0.5 m longer, the soil's side force taking some of the tires' budget off the deceleration's
direction. Without ESC the mud lane change still spins. Rut crossing at a shallow angle (the
acceptance's second half) is not in the suite: the mud pad has no plates, and step 8's
scenarios carry it.

### Step 6: low speed and edges (2026-10-26)

**Built**: `spikes/low_speed.py`, one ego without Autoware on manoeuvres' proving ground, every
substep kept, each case beside the same one on dry asphalt: crossing into the soil at 0.5, 1,
2 and 3 m/s; the split pad's lane at those speeds (left wheels in soil); a 15 deg tire angle
held at 0.5, 1 and 2 m/s from rest; a speed ramp 0 to 4 to 0 m/s at 0.25 m/s^2 with 5 deg of
tire angle (through `LAG_MIN_SPEED`, `RESIST_V0` and the brake module's slip offset, both
ways); a stop in the soil held 10 s, then a start; a start from rest 0.15 m beside that stop's
ruts. Oscillation is the rms of yaw rate, lateral velocity and acceleration after a 0.5 s
moving average, a jump the largest change between two substeps; `SOIL=dirt` runs the pads as
dirt. The speed is held by a PI driver (near Autoware's longitudinal controller's gains): the
drive unit is feedforward only, so the soil's extra resistance is the driver's to overcome.

**It found issue [010](../issues/010-soil-digs-at-physx-low-speed-slip.md)**: the soil's slip
sinkage read the wheel's real slip, where PhysX's tire needs about 0.5 m/s of overspeed for
any drive (its 4 m/s slip denominator); at a creep that reads as spinning in place, and the
driven wheels dug from 4 to 9 cm. Fixed in swb_isaac_world: the soil digs at the real slip
with its denominator held at PhysX's 4 m/s (`soil.tire_slip`), so nothing changes above
4 m/s. On mud_wet with the PI driver, before (`SLIP=real`, `play_log/low_speed/
2026-10-26_real-mud_wet`) and after (`_010b-mud_wet`): into the mud at 0.5 m/s it sank to
0.06 m/s before and holds 0.33-0.51 after; the 15 deg turn at 0.5 m/s 0.06-0.09 against
0.28-0.51 m/s; the start after the stop 0.11 against 0.89 m/s at most; the rear wheels' ruts
9.0 against 6.2 cm deep. The manoeuvre suite's mud pads with the fix
(`play_log/manoeuvres/2026-10-26_010b-ecu-{off,esc}`) match step 5's table; only the last
0.15 s of a stop, below 4 m/s, is a little longer (the mud stop still 15.72 m).

**With the fix and the PI driver** (`2026-10-26_010b-{mud_wet,dirt}`):

| | dirt | mud_wet | dry asphalt |
|---|---|---|---|
| yaw rate wobble, rms | 0.08 deg/s or less | 0.06 deg/s or less | 0.0003 deg/s straight, 0.04-0.11 turning in |
| lateral velocity wobble, rms | 0.002 m/s or less | 0.002 m/s or less | 0.0002 m/s or less |
| sidewall force, most / sign flips | 55 N / none | 44 N / none | |
| acceleration wobble rolling on the soil, rms | 0.05-0.11 m/s^2 | 0.05-0.13 m/s^2 (0.26 under traction control) | 0.007-0.02 m/s^2 |
| into the soil at 0.5-3 m/s: speed dip / largest step in acceleration | 0.17-0.25 m/s / 0.39 m/s^2 | 0.19-0.33 m/s / 0.41 m/s^2 | none / 0.14 m/s^2 |
| held 10 s on the hold brake: drift | 1 mm | 1 mm | 0 |
| start after the stop: to 2.9 m/s | 1.7 s | not in 60 s (0.89 m/s at most), traction control throughout | 1.4 s |
| start beside the ruts | 1.8 s to 2.9 m/s | 0.16 m/s at most in 9 s | |
| 15 deg held from rest at 0.5 / 1 / 2 m/s | 0.5 / 1.0 / 2.0 m/s | 0.5 / 0.11 / 0.12 m/s | 0.5 / 1.0 / 2.0 m/s |
| ramp to 4 m/s and back | follows | 0.39 m/s at most | follows |

No oscillation anywhere below 3 m/s, at an edge or through the low-speed thresholds: the
sidewall force never chatters, the yaw rate and lateral velocity wobble stays two orders
under anything a passenger would feel, and the ramp shows no step where the lag and the
resistance fade hand over (the one jump in yaw rate and acceleration, at the instant the car
comes to rest, is the hold brake's and the same on asphalt). The acceleration wobble on soil
is the grid's 5 cm nodes passing under the tires, about 0.1 m/s^2.

**Mud_wet is at the edge of what this car can start in.** A rear-driven Model 3 on Wong's
wettest clay row: on fresh soil a wheel's compaction resistance at its static sinkage
(about 3.9 cm, 500-750 N) against a driven wheel's traction limit (about 830 N, mu 0.2 of
its load) leaves almost nothing to accelerate with. Rolling, it holds any speed from 0.5 m/s up; from
rest it barely gets away, the less the more it is asked (with PhysX's own slip as the soil's,
a version tried first, the start reached 2.9 m/s in 15 s at a steady 1 m/s^2 and 40 s with the
PI driver's up to 3 m/s^2: more torque, more spin, deeper ruts), and in a turn or from beside
its own ruts not at all. On dirt every case runs as on asphalt. This is the
soil as engineered, not a defect: whether a road car should be able to start in town01_mud's
mud is 053's to settle with measured soil, and step 8's emergency stop on mud expects not to
drive off again.

### Step 7: cost (2026-10-26)

`spikes/multi_ego_soil.py` (052 step 5's: Town01, 88 m of dirt, every rig sensor on, egos held
to 4 m/s) with `MULTI_T2=ab`: each case run with the sidewall force off and on in turn, three
times, in one world server (`play_log/multi_ego_soil/2026-10-26_t2b`; the first run,
`_t2`, before the change below).

**The host was not 052's quiet machine.** Another project's simulations and a language-model
evaluation held the CPUs at a load of 50-62 on 24 cores and one GPU at 100 %. The same case
repeated spread two to three times: one ego 35-119 ms a frame at p50 (052: 29.2), four egos
244-305 ms with the force off. Frame times from today cannot be held against 052's within
10 %, and a T2 cost of a few per cent is under the noise between repeats (the soil layer's
share, paired: +0.1 to +3.5 ms a frame for one or two egos, and for four egos the "on" runs
came out faster than the "off" ones).

**So T2's cost was measured where the noise allows**: the grid step of one wheel rolled 4 m
at 4 m/s on fresh soil with T2's work (`SoilGrid.side_heights` and the force's arithmetic)
off and on, alternated, the least of seven (`soil` alone, no Isaac):

| | wheel and substep, off | on | T2 |
|---|---|---|---|
| T2 as built in step 5 | 235-245 us | 300-302 us | +55-67 us (22-28 %) |
| `side_heights` as plain loops on floats | 307-308 us | 327-329 us | +20 us (6.7 %) |

The first version read the 20 nodes beside a tire through `depth()` and `np.linspace` and
compared numpy scalars; as plain loops on Python floats they cost a third. With it, T2 adds
about 20 us a wheel and substep, 0.25 ms an ego and frame (12 wheel steps): on 052's four
egos, 77.4 ms at p50, about 1.0 ms or 1.3 %, inside the acceptance's 10 % by estimate. The
frame time itself is to be measured again on a quiet host.

Memory over eight cycles of four egos with T2 (`_t2`): RSS 14265 to 14334 MB (+8.6 MB a
cycle, against 052's +4.8), GPU 6642-6646 MB throughout.

### Step 8: with Autoware (2026-10-26)

Three scenarios on Town01, Isaac only, each with its surface map beside it in
`scenarios/surfaces/` (copied to the map directory for a run), recorded with the world
server's `record:=` and made into a video by `scripts/record_video.py`. Autoware drives at
its profile's limit, 15 km/h (`max_vel` 4.17 m/s; `max_velocity:=` raises it). The mud is
`mud_wet` with `ground_z` (its own plates), fresh at every run.

- `town01_mud_stop`: the west road's southbound lane in mud over y -232..-184 (48 m); a
  pedestrian darts from the right kerb into the lane at y -214.4 when the ego is 20 m short
  of it (demo incident 2's dart), waits 4 s, crosses; no contact, then the goal at y -285.
- `town01_mud_right`: the same with only the lane's right half in mud (x -4..-2, the right
  wheels' track): the stop is split-mu.
- `town01_mud_curve`: the south-west corner, a 90 deg left bend of about 12 m radius, in mud
  (99 tiles, the bend alone); it passes once the ego is at x 60 on the south road, before the next junction's
  traffic light.

`just scenario <xosc> sim:=isaac worlds:=Town01 initialize_duration:=300 global_timeout:=1200`
(`initialize_duration:=` new to the recipe here: SSv2's budget for Autoware from start to
engage, 120 s by default).

| scenario | verdict | what happened | recording |
|---|---|---|---|
| `town01_mud_stop` | fails (its 240 s timeout), for the reason below | from 3.8 m/s braked to rest in the mud at up to 2.1 m/s^2, 9 m short of the pedestrian's line, no contact (both runs); then crawled at 0.06-0.07 m/s, 12.7 m in 170 s | `play_log/record/2026-10-26_13-54-52-ego` |
| `town01_mud_right` | PASS (twice: the dart at 40 m and at 20 m) | at 20 m Autoware asked -4.06 m/s^2: 4.5 m/s^2 at the peak, ABS working the mud side (0.63), the heading within 0.53 deg and the line within 0.07 m; stopped 9 m short, no contact; away again to 3 m/s in 14.8 s, the left wheels on asphalt | `2026-10-26_13-03-46-ego` |
| `town01_mud_curve` | PASS | Autoware slowed from 4.1 to 2.6 m/s into the bend (about 0.56 m/s^2 sideways on 12 m); sideslip at most 0.65 deg, slip angles 2.25 deg, the sidewall 97 N at most; ESC and traction control never acted | `2026-10-26_14-33-30-ego` |

**Why the stop on mud does not drive off.** Two things together. The car: a rear-driven
Model 3 on fresh mud_wet barely starts (step 6): at rest each wheel meets 350-450 N of
compaction resistance, 1650 N in all, against a driven wheel's traction limit of about 880 N.
And Autoware: its longitudinal controller (`pid.param.yaml`, stock) has
`enable_integration_at_low_speed: false` below 0.5 m/s, so it holds its request at its
feedforward, 0.51 m/s^2 here (about 950 N of drive), and never raises it against a resistance
it does not model. With the left wheels on asphalt (`town01_mud_right`) the same request
starts the car. Whether a road car should get going in this mud is 053's (measured soil);
whether Autoware should integrate at low speed is Autoware's tuning, not changed here.

**What the runs cost besides.** The host was loaded by other work (load 36-85 on 24 cores)
and the stack started fresh for every run (the surface map is read when the world server
starts), which failed about a third of the starts (6 of 17) before the scenario began: Autoware still
INITIALIZING when SSv2's budget ran out (NDT's first align took 5-130 s, normally about 2;
hence `initialize_duration:=`), and once the ego's spawn taking 95 s against SSv2's 90 s
response limit. Two other things found on the way: once, with the curve's first surface map (221 tiles
over the bend and 10 m either side), RTX ran out of resource descriptors and the world server
hung instead of exiting, and another start sat in Kit for 16 minutes (issue
[011](../issues/011-world-server-hangs-on-a-kit-fatal.md): not the zone's size, which loads
when tried again; the world server now exits on either and is respawned); and a goal
at y -332.8 on the south road, which lanelet 7768's bounds put at the lane's centre, made
Autoware's route empty, while it drives that road at y -330.5 (the demo tour's line), where
the goal now is.

Not covered: a car crossing old ruts at a shallow angle (the acceptance's second half). Every
run starts on fresh soil; that needs a scenario that drives the same mud twice, or one started
from a saved rut set (`ruts`, 047 S6).

## Open questions

- Whether T2 later needs our own tire on soil (option A): PhysX's wheel with near-zero
  friction there, the brush's F_x on an implicitly integrated wheel spin. Only if T1's curve
  over the soil's traction limit proves short of the shear's behaviour.
- Where a vehicle hands T2 back to T1 when several are on soil and the frame budget is short
  (distance from the ego, or the ego only).
- Mud has no measured lateral data that the survey found; El-Razaz 1988, Ishigami 2007 and
  Virginia Tech's soft-soil tire tests are on sand and loam, so mud's T2 values stay engineered
  after 053 too.

## Sources

- Ishigami, Miwa, Nagatani, Yoshida, "Terramechanics-based model for steering maneuver of
  planetary exploration rovers on loose soil", *J. Field Robotics* 24(3):233-250, 2007.
- Senatore, "Prediction of mobility, handling, and tractive efficiency of wheeled off-road
  vehicles", Virginia Tech thesis, 2010 (https://vtechworks.lib.vt.edu/handle/10919/37781).
- ISTVS, "Systematic tests for study of tire tractive performance on soft soil, part I"
  (https://www.istvs.org/publication-news/systematic-tests-for-study-of-tire-tractive-performance-on-soft-soil-part-i-experimental-data-collection).
- Wong, *Theory of Ground Vehicles*; Pacejka, *Tire and Vehicle Dynamics* (brush model,
  relaxation length).
- ISO 7401, 4138, 3888, 14512, 7975; FMVSS 126 (ESC, sine with dwell).
- NVIDIA PhysX 5 vehicle SDK documentation (tire component); `omni.physx.vehicle` 110.3.2.
