# 086: aib's body acceleration is noise at a high physics rate

**Severity**: medium. The IMU's specific force and the brake module's acceleration sensor read
it. At 60 Hz, today's rate for the full-size cars, the noise is 0.003 m/s^2 and harmless. At
240 Hz, the rate roadmap 091 W7 chose for a 1/10 car, it is 0.021 m/s^2, about a real IMU's own.
At 600 Hz it is 0.16 m/s^2. The noise is about the square of the rate, and it grows with the
car's distance from the world's origin.
**Status**: fixed (aib 00b5db6), by 241.
**Found**: 2026-11-20, swb 118, roadmap 091 W7 (the physics rate for the RoboRacer).

## What happens

The RoboRacer example cruised at a steady 3 m/s on a flat plane. The run used
`scenarios/projects/roboracer/tools/smallcar.py`'s `cruise` test, the ideal powertrain and TGS,
with 0.05 s frames. The table gives the standard deviation of the body's longitudinal
acceleration over the last 5 s, taken once each physics step. The last column is the same
deviation for the speed.

| physics rate | acceleration, m/s^2 | the same differenced over 1/60 s | speed, m/s |
|---|---|---|---|
| 60 Hz | 0.0027 | 0.0026 | 0.0011 |
| 100 Hz | 0.0059 | 0.0042 | 0.0012 |
| 120 Hz | 0.0062 | 0.0042 | 0.0012 |
| 200 Hz | 0.0153 | 0.0082 | 0.0012 |
| 240 Hz | 0.0210 | 0.0072 | 0.0012 |
| 300 Hz | 0.0215 | 0.0067 | 0.0012 |
| 400 Hz | 0.0387 | 0.0116 | 0.0011 |
| 600 Hz | 0.1596 | 0.0274 | 0.0013 |

The speed's deviation does not move with the rate, so the car itself is no less steady at
600 Hz. Only the acceleration that aib computes from it gets noisier.

## Cause

`aib/isaac/vehicle.py` `Vehicle.sample` reads only the body's pose from PhysX
(`get_rigidbody_transformation`). It differences that pose twice, each over one physics step:
- the velocity is (position - last position) / dt;
- the acceleration is (velocity - last velocity) / dt.

PhysX keeps positions in float32. Around x = 20 m, where the cruise ran, a float32 is
resolved to about 2e-6 m. A position error e becomes about e / dt^2 in a second difference.
That is 2e-6 x 600^2 = 0.7 m/s^2 at worst at 600 Hz, and 0.007 at 60 Hz. The measured
deviations sit below those bounds and grow as they do.

Two checks that it is quantisation:
- **The lateral acceleration is exactly 0 at every rate.** The car drove along y = 0, which a
  float32 holds exactly.
- **The noise scales with the step, not with the motion.** The speed's deviation is the same
  at every rate (the table's last column).

A float32's resolution doubles with each doubling of distance from the origin. On a map whose
roads lie hundreds of metres out (Haruna's start is at (135, 198)), the noise at a given rate
should be several times the table's. That is predicted, not measured.

What reads it:
- the IMU's specific force (`aib/sensing.py` `imu_reading`, published by `aib/ros.py`);
- the component's sensors (`accel_x`, `accel_y`), which the brake module's estimator and a
  user's component read.


## Fix options

1. **Difference over a fixed window, not over one step.** Use the speed change over 1/60 s
   (k = round(1 / (60 dt)) steps): (v_now - v_k_steps_ago) / (k dt). At 60 Hz and below, k is 1,
   so nothing changes for the shipped cars. Measured offline on the same runs, the window
   brings 600 Hz from 0.16 to 0.027 m/s^2 and 240 Hz from 0.021 to 0.007. The cost is a lag of
   half the window, about 8 ms, which an IMU's own filter has anyway. The velocity itself stays
   a one-step difference, still float32-limited, but its error is only e / dt.
2. **Read the velocities from PhysX.** The velocities are PhysX's own state, not a difference
   of positions, so they carry no position quantisation. scensim_isaac_world already reads them
   for the egos (`_ego_view`, a rigid-prim view's `get_velocities`). The acceleration is then
   one difference of the true velocity, with an error of about the velocity's float32
   resolution / dt. That is about 1e-6 m/s / dt, well under the window's at every rate. It
   needs a tensor view of the body in aib, created after the stage plays, with a fallback for a
   host without one.

Either way, a check belongs with the fix. Re-run `smallcar.py --only cruise` at 60, 240 and
600 Hz, at the origin and a few hundred metres out. The noise at 240 Hz should be no more than
60 Hz's is today.

## Fix

Option 2, with option 1 as the fallback (aib 00b5db6). `Vehicle.sample` reads the body's linear and
angular velocities from PhysX through a tensor view (`omni.physics.tensors`, one rigid-body view
per vehicle, made on the first sample after the stage plays). PhysX reports them at the centre of
mass, so the linear one is moved to base_link (v - w x r). The acceleration is their one-step
difference. A host without a tensor view differences the poses as before, but over a fixed 1/60 s
window for both the velocity and the acceleration. `Vehicle.velocity_source` names which path ran.

### The noise, before and after

`accel_noise.py` (NAS, below) builds the Model 3 as `aib.steptest` does, on a flat box, and
takes each physics step's acceleration and velocity. The noise is the standard deviation about a
0.5 s centred mean. Phases: standing on the hold brake, cruising at 10 m/s, and a gentle ramp
(0.3 m/s^2 asked). The table gives the ramp's longitudinal acceleration in m/s^2, and the
speed's noise in m/s in brackets.

| rate | at | before | after | fallback (no view) |
|---|---|---|---|---|
| 60 Hz | origin | 0.031 (0.0003) | 0.0000 (0.00000) | 0.031 (0.0003) |
| 240 Hz | origin | 0.45 (0.0011) | 0.0001 (0.00001) | 0.034 (0.0004) |
| 600 Hz | origin | 1.63 (0.0028) | 0.0002 (0.00002) | 0.14 (0.0012) |
| 60 Hz | x = 1 km | 0.21 (0.0022) | 0.0000 (0.00000) | 0.21 (0.0022) |
| 240 Hz | x = 1 km | 1.42 (0.0085) | 0.0001 (0.00001) | 0.53 (0.0072) |
| 600 Hz | x = 1 km | 3.56 (0.0206) | 0.0002 (0.00002) | not run |

The steady cruise hides the problem. At 1 km a float32 cannot resolve the step's displacement at
all, so the differenced acceleration there is exactly 0 until the speed changes.

The RoboRacer cruise (`smallcar.py --only cruise`, the table under What happens) has a
longitudinal acceleration deviation of 0.0027, 0.0211 and 0.169 m/s^2 at 60, 240 and 600 Hz
before. After, it is 0.0025 at all three, which is the car's own motion.

### PhysX's own solver at 1 km

The velocities PhysX reports at x = 1 km match those at the origin to the table's last digit at
every rate. The speed's mean and the acceleration's mean are the same too. So the solver itself
loses no velocity accuracy at 1 km. Only aib's differencing of float32 positions did.

### The references this can move

Each was rerun with the fix (the same change as aib 00b5db6, on aib e5e3fba, before W24) and compared
with a run without it, on the same host (241).

- **`aib.steptest`**: 48 metrics are unchanged. The slope matrix is identical. Two moved by more than
  rounding: the +1.5 m/s^2 step's t90 went from 0.114 to 0.112 s, and the mean absolute acceleration
  error from 0.042 to 0.040 m/s^2. Both are read from the now quieter acceleration.
- **The 055 suite (`vehicle_check`)**: 17 metrics moved by more than 2 %. The dry cases are unchanged.
  - `wet_brake_turn` peak lateral acceleration: 1.104 to 1.066 m/s^2.
  - `split_brake` peak lateral acceleration: -3.74 to -1.21 m/s^2. The old value came from the last
    frame's differenced acceleration, a spike. The trajectory is the same within 1 mm.
  - `mud_brake_turn`: peak sideslip -0.72 to -0.52 deg, mean deceleration 3.82 to 4.01 m/s^2.
  - `mud_circle` is chaotic: the path differs by up to 8.6 m, the mean sideways acceleration goes
    from -1.14 to +1.46 m/s^2, and the understeer gradient from 12.02 to 10.8. The ESC and the brake
    module act on the acceleration they read, so a quieter reading changes their interventions, and
    on mud a small change grows.
- **The T3 Isaac suites**: `isaac_t2` (autoware, simple, none, launch_own, command), `isaac_basic`
  (towns, rear_contact), `isaac_road_physics` road and `isaac_rough_road` rough all pass, as their
  references do (`t3.py compare`: as the reference). The rough suite's first start never became
  ready. Its behavior_path_planner did not load, after a lost `load_node` response in its container.
  That is an Autoware start failure outside this change, and the rerun passed.
- **isaac_soil** (soil, mud, snow, zones) and **isaac_verge** pass, as they did before the fix the same
  day. Their results hold only timing metrics, which follow the host's load. Each scenario's frame
  count is the same.
- **haruna_down**: passes with and without the fix, run from a copy of the `courses` project (issue 087). Both runs drove 1473.1 m in 5200 frames, and the NVTL judge's p05 is 3.119 in both.

None of the T3 references needs re-recording: no verdict moved.

## Evidence

NAS logs `2026 SSv2 on CARLA/2026-11-20 091 W7 RoboRacer physics rate (118)/`: `out/tgs<rate>/
cruise.csv`, which holds every physics step's speed and acceleration at each rate. The table
above was computed from those files.

The fix's measurements are under NAS logs `2026 SSv2 on CARLA/2026-11-21 086 aib tensor velocities
(241)/`:
- `accel_noise.py`, with `out/{before,after,fallback}/` and `table.py`;
- the steptest and 055 runs, in `steptest/` and `man/`;
- the RoboRacer cruise, in `smallcar/`;
- the reference reruns' logs: `run.log` for T3, `rest.log` for rough, soil and verge, and
  `haruna.log`.
