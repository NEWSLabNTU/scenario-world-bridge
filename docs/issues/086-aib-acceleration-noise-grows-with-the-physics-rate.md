# 086: aib's body acceleration is noise at a high physics rate

**Severity**: medium. The IMU's specific force and the brake module's acceleration sensor read
it. At 60 Hz, today's rate for the full-size cars, the noise is 0.003 m/s^2 and harmless. At
240 Hz, the rate roadmap 091 W7 chose for a 1/10 car, it is 0.021 m/s^2, about a real IMU's own.
At 600 Hz it is 0.16 m/s^2. The noise is about the square of the rate, and it grows with the
car's distance from the world's origin.
**Status**: open; diagnosis only. The fix is 241's, who owns the IMU outputs (roadmap 091 W19).
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

## Evidence

NAS logs `2026 SSv2 on CARLA/2026-11-20 091 W7 RoboRacer physics rate (118)/`: `out/tgs<rate>/
cruise.csv`, which holds every physics step's speed and acceleration at each rate. The table
above was computed from those files.
