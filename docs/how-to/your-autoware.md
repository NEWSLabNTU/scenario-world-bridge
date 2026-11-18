# Your Autoware

Change how the `autoware` pilot runs Autoware: its arguments, your own launch or
parameters, your own build, the fixes Autoware 1.5.0 lacks, and its TensorRT. It assumes
your Autoware is connected ([Connect your Autoware](connect-autoware.md)).

## What the pilot runs

An `autoware` pilot runs the vehicle side (on CARLA, the vehicle bridge) and scensim's
vehicle agent, and Autoware with the car's packages (`vehicle_model`, `sensor_model`), all in
the pilot's ROS domain. Autoware is configured one of two ways
([Connect your Autoware](connect-autoware.md#two-ways-to-configure-autoware)):

- `autoware_profile: stock`, the default: your `autoware.launch.xml` as it is, told only
  what a simulator needs;
- `autoware_profile: scensim`: scensim's **Autoware profile** (`autoware_sim_profile`'s
  `autoware.launch.xml`, in `autoware_sim_vehicle`) around it.

### The car's body on each simulator

A car's vehicle specification describes the real car, and each simulator drives a body of its
own: an asset whose wheel base, track, wheel radius and box are measured from that simulator.
Autoware must plan with the body the simulator drives, so each body has its own Autoware
vehicle model. The pilot's `vehicle` names the car; the stack selects the vehicle model and the
scenarios' vehicle catalog (`$(env SCENSIM_VEHICLES)`) from the simulator's version, so a
testbench names the same `vehicle` on every simulator:

| `vehicle` | Simulator | `vehicle_model` | Body |
|---|---|---|---|
| `tesla_model3` | CARLA 0.9.16, Isaac Sim | `tesla_model3` | CARLA 0.9.16's `vehicle.tesla.model3`: 3.005 m wheel base, 0.370 m wheel |
| `tesla_model3` | CARLA 1.0 | `tesla_model3_carla_1_0` | CARLA 1.0's `vehicle.tesla.model3`, the production car's dimensions: 2.876 m wheel base, 0.336 m wheel |

The sensor kit (`sensor_model`, `tesla_model3_sensor_kit`) is the same on every body. When the
stack cannot read the CARLA server's version (a remote server, or `simulator_start:=external`
before one answers), it uses the car's first body, CARLA 0.9.16's. A testbench's scenarios
name the catalog as `<Directory path="$(env SCENSIM_VEHICLES)" />`, as the templates of
`scensim testbench new` do. A scenario that names `$(find-pkg-share sim_vehicle_spec)/catalogs`
directly declares CARLA 0.9.16's body on every simulator: `scensim testbench check` reports it,
and on CARLA 1.0 the scenario bridge refuses its ego with a message naming the catalog it needs.

## Arguments

A pilot's `args` reach Autoware as launch arguments. With `stock` they are your
`autoware.launch.xml`'s; the table below is the profile's, with `autoware_profile: scensim`:

```yaml
pilots:
  ego:
    kind: autoware
    autoware_profile: scensim
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

Source your Autoware build's `setup.bash` in `~/scensim.bash` instead of
`/opt/autoware/1.5.0/setup.bash`, then rebuild the car's Autoware packages against it
([Connect your Autoware](connect-autoware.md#import-and-build)). Its release must be one the
packages support ([Supported Autoware releases](connect-autoware.md#supported-autoware-releases));
the profile runs 1.5.0, 2.0.0 and 2.1.0.

## NEWSLab's Autoware 1.5.0 binaries

If you use NEWSLab's Autoware 1.5.0 binary packages with `autoware_profile: scensim`, as
scensim's own testbed does, the fixes and the TensorRT runtime it adds are on the
[testbed page](../design/autoware-testbed.md). Your own Autoware needs neither.
