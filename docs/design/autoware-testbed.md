# NEWSLab testbed: Autoware 1.5.0's binaries

**NEWSLab testbed, not needed for your Autoware.** This page is for scensim's own testbed,
which runs NEWSLab's Autoware 1.5.0 binary packages with scensim's Autoware profile
(`autoware_profile: scensim`). A user's Autoware needs none of it: the car's Autoware
packages are all it needs ([Connect your Autoware](../how-to/connect-autoware.md)).

Two pieces make the testbed's Autoware 1.5.0 drive scensim's scenarios: three fixes 1.5.0
lacks, built as an overlay, and the TensorRT runtime 1.5.0 was compiled against. In a scensim
checkout the developer recipes are `just build-autoware-overlay` and `just setup-tensorrt`;
`scensim autoware overlay` and `scensim autoware tensorrt` run the same scripts, and
`scensim autoware where` prints where each is placed.

## The fixes, in short

They are built once, in a few minutes:

```bash
scensim autoware overlay
```

Without them a stack runs stock Autoware and its log says so. With CenterPoint as the
lidar detector, Autoware 1.5.0 also needs TensorRT 10.8.0 (below). The default detector,
`clustering`, needs neither. The fixes are for Autoware 1.5.0: with another release the
overlay is not sourced, and the pilot's log says so.

## The fixes, by hand

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
`testbench check` warns when a pilot runs CenterPoint and this is not set, and a pilot is not
ready while its detector is still loading, so no run starts with the ego blind.
