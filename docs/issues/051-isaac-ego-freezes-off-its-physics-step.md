# 051: An Isaac ego freezes when SSv2's frame is not a whole number of physics steps (077 G84)

**Severity**: high for any project whose `ssv2.frame_rate` does not divide `isaac.physics_hz`
(the ego never moves, and the run times out). **Status**: fixed on branch `g84-frame-rate`,
which refuses such a pairing by name. **Found**: 2026-11-17 by swb 118 (item 26, rung 10); the
cause was read by 139 and the gap numbered G84 by 133 (077's known limits). Written up here by
241 on 2026-11-18, before the fix.

## What happens

speed_ladder runs at `ssv2.frame_rate: 100` with the Isaac world server's default
`physics_hz` of 60. The relay reports DRIVING and AUTONOMOUS, the gear is DRIVE, and Autoware
commands +0.5 m/s^2. Yet the car never moves: 0.0 m of ground truth, in three runs out of
three. The same rung at frame_rate 20 drove 591 m. With `isaac.physics_hz: 100` at frame_rate
100 it drove 1077 m. Both are in 077 G84.

## Cause

1. The Isaac world server builds its PhysX world at `1/physics_hz` and tells the bridge
   `max_physics_dt = 1/physics_hz`.
2. The bridge splits each SSv2 frame into `n = ceil(frame * physics_hz)` substeps of
   `frame / n` (`scenario_world_bridge` `remote/mod.rs` `physics_steps`).
   - At 20 Hz on 60 that is 3 x 1/60 s, the world's own step.
   - At 100 Hz on 60 it is 1 x 10 ms. So `IsaacWorld.step` calls
     `set_simulation_dt(0.01)` at run time (`scensim_isaac_world` `world.py`), after the ego
     spawned and warmed up at 1/60.
3. From then on, the PhysX vehicle does not move. Why a runtime step change freezes it is not
   shown. No run of this world server has driven after such a change. 057's and 063's 100 Hz
   laps used aib's runner, which builds its World at a fixed 1/240 s.

## Fix: refuse, rather than change the step

Two ways were open (077 G84):
- refuse a frame that is not a whole number of the world's physics steps; or
- refresh the vehicle after `set_simulation_dt`.

The first was taken, for three reasons:

- **It never puts the car on a step it was not calibrated on.** The ego's drivetrain, tyres and
  sim_vehicle_ecu were tuned and measured at the world's physics rate (aib's 240 Hz rig;
  physics_hz in the project). A refreshed vehicle at 10 ms would drive, but on other dynamics
  than the ones every measurement of the car was taken at.
- **It is fully explained.** A refresh depends on why PhysX stops, which is not known. It would
  need an Isaac experiment first, and it could fail in other ways (the soil, the wheels topic,
  the ECU's own dt).
- **The fix for the user is one setting,** named in the refusal: a frame rate that divides
  physics_hz, or a physics_hz that is a multiple of the frame rate. At 60 Hz the frame rates
  that work are 60, 30, 20, 15, 12, 10, 6, 5, 4, 3, 2 and 1. At 240 Hz, 100 is still not one;
  physics_hz 100, 200 or 300 suits frame_rate 100.

What refuses:

- **The world server** (`scensim_isaac_world`):
  - `load` refuses a `LoadWorld.step_time` that is not a whole number of its physics steps;
  - `step` refuses any `dt` other than its own, rather than calling `set_simulation_dt`.

  Each message names the step, the physics rate and the frame rates that divide it. This holds
  for every client: `just scenario`, scensim, or another bridge.
- **scensim**, before anything starts:
  - `testbench check` / `project check` give a finding for the project's
    `ssv2.frame_rate` against its `isaac.physics_hz`;
  - a run refuses a suite or scenario whose `ssv2.frame_rate` does not fit, naming the
    scenario, the two rates and the fix.

## Test

- **The world server's pure check:** for each (frame, physics_hz) pair, the number of
  substeps or the refusal's text.
- **scensim:** the project finding and the run's refusal, with a pair that fits and one that
  does not.
