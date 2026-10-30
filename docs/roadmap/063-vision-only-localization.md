# Phase 063: Vision-Only Localization on a Fast Car

Autoware drives 057's Mustang around Town05's ring in Isaac on a pose from a camera, the IMU and
the wheel speed alone: rig_localization in place of cuda_ndt, no lidar and no point-cloud map.
The question is how fast, and how far, a car can go on that before it leaves its lane, and what
stops it: drift, lost tracking, latency or compute.

**Status**: planned, 2026-10-30.

**Source**: user direction 2026-10-30: "run pure rig localization on speedy car ... see if the rig
can replace the cuda_ndt if we tweak the message rate and improve the computation so it can run
on live car"; "employ pure vision alternative to lidar and test the extreme performance to see
if it can work as very fast vehicle". Decisions (user, same day): a lanelet2 map for route
planning, no point-cloud map; the pose seeded once, from ground truth in simulation (on the real
car: a fixed start point whose pose NDT measures once); Autoware drives on rig's pose (closed
loop); the target computer is the Jetson AGX Orin, tested in simulation for now; no anchor (no
visual map, no lane-line matching) for now. After the design review (user, same day): Autoware's
localization checks widened to the lane, rig's covariance kept honest; the Isaac runs on 139 when
its GPU is free; the camera a copy of the golf cart's ZED X left camera with the 2.2 mm lens.

**Depends on**: [057](057-drift-campaign.md) (the Mustang, `drift_mustang`; Town05's ring route,
its lanelet repairs; `jerry73204/autoware-drift`'s high-speed planning), rig_localization
(NEWSLabNTU, the golf cart's twist estimator: cuVSLAM mono tracking, a GTSAM fixed-lag smoother
with IMU and wheel speed, the wheel-seeded initial alignment of 2026-10-29). [060](060-live-race-planner.md)'s race planner in autoware-drift drives
on whatever pose localization gives; this phase uses Autoware's own planning, as 057's laps
did, and leaves the race planner on rig's pose to a later run.

## What is measured

A speed ladder: 10, 15, 20, 25, 30 m/s, each a run from the seed with the drift controller off.
Per run, against the Isaac recorder's ground truth:

- **distance to the first lane departure** (base_link's footprint over a lane boundary of the
  lanelet), and the time;
- **pose error over distance**: lateral, along-track and heading, at 100 m, 250 m, 500 m, 1 lap;
- **what ended it**: drift (error grows smoothly), tracking lost (rig's state leaves tracking),
  latency (the pose's age at the EKF), an MRM from Autoware's own checks, or none (a full lap);
- **rig's cost**: per-frame step time (median, p95, max) and the pose's capture-to-output age.

Success is that map, not a particular speed: "holds X m at Y m/s, ends by Z".

## What is expected

Camera-IMU odometry with no anchor drifts by roughly 0.5-2 % of distance; a lap is ~1.5 km and a
lane 3.5 m wide, so the car will likely leave its lane within a few hundred metres, well inside
the first lap. That is a result, not a defect: it says what an anchor would have to buy. The
wheel speed fixes the scale; what is left is heading drift, which turns into lateral error.

## What changes, and where

1. **Pose out of rig** (rig_localization, its own spec). Today rig publishes a twist into
   Autoware's EKF and NDT gives the pose; `/rigloc/odom` is its own `odom` frame, used by
   nothing. New: the seed (an `/initialpose` from ground truth) fixes `map -> odom` once, and rig's
   pose reaches Autoware in the map frame. Without NDT the EKF reports "pose is not updated" and
   the diagnostic graph takes autonomous mode away (057 lap 24, issue 017), so either rig's pose
   becomes the EKF's pose input at a rate and covariance the EKF and its checks accept, or rig
   owns `map -> base_link` and `/localization/kinematic_state` and the EKF is not run. Rig also
   takes `sensor_msgs/Image` (Isaac publishes raw images; rig takes only `CompressedImage` and
   decodes with nvJPEG).
2. **The camera on the Isaac Mustang** (sim_vehicle_spec, aib). `camera6` is off on Isaac and
   10 Hz; at 30 m/s that is 3 m between frames. A new camera in the Mustang's spec, a copy of the
   ZED X left camera with the 2.2 mm lens (110 x 80 deg; a ZED X mode's resolution and rate, the
   rate as high as the lockstep affords, 30 Hz the first guess; global shutter, as the ZED X),
   mounted where a ZED sits on a car's windshield. Measure what rendering it costs the lockstep
   (057: the world already stalls localization above a host load of ~120).
3. **Open loop first.** Isaac drives with cuda_ndt (as in 057), recording camera, IMU, wheel speed
   and ground truth at each speed; rig replays the recordings (`rigloc_replay`, real time) and
   is scored as above. The first ceiling, with no crash risk, and a fixed set of inputs for 4.
4. **Rate and compute.** Raise the camera and output rates; profile rig's step on the recordings
   (on x86 on the golf cart: whole step 9 ms median, 22 ms p95; smoother p95 19 ms; covariance
   4-15 ms; tracking 0.5 ms) and cut it to fit the frame at the Orin's speed. No Orin is measured
   yet: the budget is x86 time times a stated factor (3-5x is the starting guess), checked on an
   Orin when one is available.
5. **Closed loop.** Autoware drives on rig's pose, cuda_ndt and the lidar off; the speed ladder.
   Autoware's localization checks (the EKF's covariance-ellipse diagnostics, the localization
   error monitor) widened to the lane: their thresholds at the lane's half-width, 1.75 m, so a
   run ends when the honest uncertainty passes the lane or the car leaves it; rig's covariance
   is not clamped. Both are logged.
6. **Write-up.** The map, and what an anchor would buy (a cuVSLAM visual map, or lane lines
   matched to the lanelet2 boundaries): the next phase if wanted.

## Steps

1. [x] **Docs.** This plan (2026-10-30).
2. [x] **Rig spec**: pose output, the seed, raw images (rig_localization `docs/superpowers/specs/2026-10-30-pose-output-design.md`, landed `a080019`, off by default). Replay on a golf-cart drive: at the anchor 0.0 mm / 0.04 deg; 0.48 m / -3.0 deg at +10 s, 1.30 m / -3.6 deg at +60 s; the early yaw step (anchor taken on init-window estimates up to ~2 s old) is for step 5.
3. [ ] **Camera on**: the Mustang's camera on Isaac at the chosen rate; rendering cost measured.
4. [ ] **Recordings**: one Isaac run per speed driven on the ground-truth pose, ground truth recorded.
5. [ ] **Open-loop ladder**: rig on the recordings, scored.
6. [ ] **Rate and compute**: profile, cut, re-score.
7. [ ] **Closed-loop ladder**.
8. [ ] **Write-up**.

## Decisions taken here (not the user's)

- The Autoware project is `autoware-drift` (057's laps: its planning holds 25-30 m/s), with
  rig's localization launch in place of the NDT one; the drift controller off.
- The drift controller stays off: a drift's sideslip is a separate question.
- Open loop before closed loop: it finds the first ceiling without a crashed run and gives the
  compute work fixed inputs.
- **A rung's speed is Autoware's `max_velocity`.** The planner slows the car in the bends by its
  lateral-acceleration limit (Town05's ~100 m bends take ~28 m/s at 0.79 g), so the error is
  scored per stretch (straight, bend) as well as per run, and a departure in a bend is checked
  against grip (the ground truth's sideslip) before it counts as localization's.
- **The seed is a hand-over.** Autoware cannot engage without a pose, and rig initialises only
  once the car moves (~5 s after the first wheel sample). So a run starts on the ground-truth
  pose; when rig reports tracking, `map -> odom` is tied to the ground truth at that instant and
  the pose source switches to rig for the rest of the run. On the real car the same hand-over
  is from the one NDT fix at the start point.
- **Recordings are driven on the ground-truth pose**, not cuda_ndt: issue 017 holds cuda_ndt to
  28 m/s, and the open-loop ladder needs 30. Lidar and NDT still run, for comparison.
- **Wheel speed**: the ideal Isaac wheel in every run; open-loop replays also run with it
  spoiled (scale +-3 %, the golf cart's whole-km/h quantisation, rear slip from the recorder's
  wheel and body speeds) to show how much of the result the ideal wheel buys.
- **Rig's compute in closed loop**: not gated by the lockstep (its pose ages as it would on a
  car); closed-loop runs at x86 speed, plus one rung repeated with rig's output delayed by the
  Orin budget's extra time, so the map says something about the Orin before one is measured.
- **A run ends at the first lane departure** in the ground truth: a harness monitor stops it
  (the ring is raised and walled; 25-30 m/s past the lane ends in a wall).

## Risks

- **The host.** The Isaac runs are on 139, only when its GPU is free (the rig runs' quiet-host
  gate: 5 min of quiet first, then a gate on each run). Isaac's SimReady Town05 needs a GPU that
  renders and ~17.5 GB of RAM (057 ran on server-241); 139's GPU is shared (another user's vLLM
  has held ~29 GB of 32 GB), and Isaac, rig and Autoware's GPU nodes share it. Runs may wait
  long; open-loop replays need no Isaac. A camera at 30 Hz adds rendering per frame to a
  lockstep that already strains. Never Isaac and CARLA at once on 139.
- **Sim images are too easy or too hard**: no motion blur or rolling shutter, but repetitive
  highway texture; the result is a bound for this world, not for a real road.
- **Lockstep time vs. compute time**: in lockstep the world waits for nothing in Autoware, so a
  slow rig ages its pose without slowing the world; the age has to be measured, not assumed.
- **Autoware without NDT**: its localization checks (error monitor, EKF diagnostics) were tuned
  for a pose that does not drift; some may need retuning for a closed loop to run at all, and
  that retuning must not hide the drift being measured.
