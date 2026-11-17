# Your Autoware

Change how the `autoware` pilot runs Autoware: its arguments, your own launch or
parameters, your own build, the fixes Autoware 1.5.0 lacks, and its TensorRT. It assumes a
project with a working stack ([Tutorial 1](../tutorials/01-first-drive.md)).

## What the pilot runs

An `autoware` pilot runs scensim's **Autoware profile** (`autoware_sim_profile`'s
`autoware.launch.xml`, in `autoware_sim_vehicle`) around a stock `autoware.launch.xml`, with
the car's packages (`vehicle_model`, `sensor_model`), plus the vehicle side (on CARLA, the
vehicle bridge) and scensim's vehicle agent, all in the pilot's ROS domain.

## Arguments

A pilot's `args` reach the profile as launch arguments:

```yaml
pilots:
  ego:
    kind: autoware
    vehicle: tesla_model3
    rviz: true
    args: {max_velocity: "11.11", lidar_detection_model: centerpoint}
```

| Argument | Default | Effect |
|---|---|---|
| `autoware_launch` | Autoware's | your own top-level `autoware.launch.xml` |
| `max_velocity` | 4.17 (15 km/h) | Autoware's planning speed limit, m/s (`sim_planning`) |
| `lidar_detection_model` | `clustering` | `centerpoint` uses TensorRT |
| `gpu` | inherit | `CUDA_VISIBLE_DEVICES` for this Autoware |
| `extra_launch` | none | a launch file of yours beside Autoware, in its domain |
| `sim_localization` | `true` | `false`: your NDT, EKF and pose-initializer parameters |
| `sim_perception` | `true` | `false`: your perception (traffic lights from a camera, not the simulator) |
| `sim_system` | `true` | `false`: your component-state topics, MRM parameters and diagnostic graph |
| `sim_control` | `true` | `false`: upstream's MPC parameters (the profile turns off steering-offset learning) |
| `sim_planning` | `true` | `false`: your planning parameters; `max_velocity` no longer applies |
| `sim_clock_guard` | `true` | `false`: no clock guard (it ends the stack when `/clock` goes back) |
| `launch_deprecated_api` | `true` | `false`: no deprecated API, which serves the velocity limit of a scenario's `SpeedAction` (the AWF use cases fail without it) |

A launch ignores an argument no file declares, so an old name is dropped without a word.
Check yours first; it exits 2 naming each problem and its replacement:

```bash
ros2 run autoware_sim_profile check_launch_args \
    max_velocity:=11.11 \
    sim_localization:=false
```

The lidar detector defaults to `clustering` because it passes on CARLA: the AWF
standing-pedestrian use case went 9/9 with it and 2/9 with `centerpoint` (2026-10-10, same
host and build).

To change the arguments per run (a suite comparing two detectors), make the pilot
`lifetime: run` and give `pilot_args` in the suite, as the `safety_suite` example's
`suites/centerpoint.yaml` does.

## Your own build

Build your Autoware of the 1.5.0 release and source its `setup.bash` in `~/scensim.bash`
instead of `/opt/autoware/1.5.0/setup.bash`, then rebuild this workspace against it. A
different Autoware release is not supported.

## Fixes Autoware 1.5.0 lacks

Three fixes, in four package directories from two NEWSLabNTU forks, built from source over
Autoware 1.5.0 into a workspace of their own:

| Fork, branch | Package directory | Fixes |
|---|---|---|
| `autoware_universe`, `csb/pose-instability-empty-clip-0.48.0` | `planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module` | `blind_spot` never stops for a cyclist inside a right turn (autoware_universe#13509) |
| same | `localization/autoware_pose_instability_detector` | the node dies on a twist stamped at the previous odometry's time (`negative time point`) |
| `autoware_core`, `csb/ndt-step-reversal-1.5.0` | `localization/autoware_ndt_scan_matcher`, `localization/autoware_localization_util` | NDT's two-point cycle runs to the iteration limit, and its warning is an emergency stop |

With Autoware sourced:

```bash
mkdir -p ~/autoware_overlay/src
cd ~/autoware_overlay/src
git clone --filter=blob:none --sparse \
    -b csb/pose-instability-empty-clip-0.48.0 \
    https://github.com/NEWSLabNTU/autoware_universe.git
git -C autoware_universe sparse-checkout set \
    planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module \
    localization/autoware_pose_instability_detector
git clone --filter=blob:none --sparse \
    -b csb/ndt-step-reversal-1.5.0 \
    https://github.com/NEWSLabNTU/autoware_core.git
git -C autoware_core sparse-checkout set \
    localization/autoware_ndt_scan_matcher \
    localization/autoware_localization_util
cd ~/autoware_overlay
colcon build \
    --allow-overriding $(colcon list -n) \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

Then source `~/autoware_overlay/install/local_setup.bash` in `~/scensim.bash`, right after
Autoware's setup. Without it the ego runs stock Autoware, with the right-turn bug and an
emergency stop now and then.

## TensorRT 10.8.0

Autoware 1.5.0 was compiled against TensorRT 10.8.0 and rebuilds any cached engine that
another version made. If the first `libnvinfer.so.10` your loader finds is another version
(CUDA 12.8's 10.9 under `/usr/local/cuda`, say), CenterPoint rebuilds its engines at every
start: minutes, with `Plan was created with a different version of TensorRT` in its log.
Fetch NVIDIA's `tensorrt-cu12-libs==10.8.0.43` wheel (3.1 GB) and unpack its libraries
into a directory of your own, from the workspace:

```bash
bash -c 'source scripts/autoware_tensorrt.bash; \
    scensim_tensorrt_setup ~/tensorrt-10.8.0/lib'
```

and name that directory in your host settings; every autoware pilot then puts it first on
its library path:

```yaml
# ~/.config/scensim/host.yaml
autoware: {tensorrt_lib: ~/tensorrt-10.8.0/lib}
```

The first start on 10.8 rebuilds once; after that the engine loads (CenterPoint ready in
about 20 s instead of minutes). On the default detector (`clustering`) no engine is built.
`project check` warns when a pilot runs CenterPoint and this is not set, and a pilot is not
ready while its detector is still loading, so no run starts with the ego blind.
