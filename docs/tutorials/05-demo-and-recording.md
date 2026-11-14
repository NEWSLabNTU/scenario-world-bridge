# Tutorial 5: Demo and recording

A screencast of a scenario like the one in the [README](../../README.md): CARLA from behind
the ego, and each vehicle's RViz beside it. It builds on the stack on CARLA
([Build your own stack](../build-your-own-stack.md), sections 1 to 5) and on the two
Autoware vehicles of [Tutorial 3](03-other-road-users.md#3-a-background-autonomous-vehicle).

Commands use `play_launch launch`; `ros2 launch` takes the same arguments, but play_launch
must still be installed, because the stack starts its ego and scenario as play_launch
processes of their own.

## 1. Windows for the demo

The README's showreel shows three windows: CARLA from behind the ego, and each vehicle's
RViz. All three come with the workspace.

**A chase camera** on the ego (the CARLA vehicle named `hero`), in a window of its own:

```bash
SDL_VIDEO_WINDOW_POS=70,28 VIEW_W=740 VIEW_H=416 \
    ros2 run swb_launch chase_view
```

It is a free camera moved behind the vehicle every frame, not attached to it, so a scenario
ending never leaves a sensor behind; it waits for the next ego when there is none. It needs
CARLA's Python API (`pip install carla==0.9.16`) and reads `CARLA_PORT`.

**RViz layouts** that place the two Autoware windows on the right half of a 1920x1080
screen (ego top, `bg_av_1` bottom), each following its own vehicle from behind. Start the
stack and the background AV with them:

```bash
RVIZ=$(ros2 pkg prefix --share swb_launch)/rviz
play_launch launch --web-addr 127.0.0.1:8084 \
    swb_launch carla.launch.xml \
    map_path:=$CARLA_MAPS/Town01 \
    side_args:="weather:=ClearNoon" \
    ego_launch_args:="rviz:=true rviz_config:=$RVIZ/ego_autoware.rviz"
```

```bash
RVIZ=$(ros2 pkg prefix --share swb_launch)/rviz
ROS_DOMAIN_ID=2 play_launch launch \
    --container-mode isolated \
    --composable-respawn on-crash \
    --web-addr 127.0.0.1:8083 \
    swb_launch background_av.launch.xml \
    entity:=bg_av_1 \
    map_path:=$CARLA_MAPS/Town01 \
    relay:=tcp://localhost:${AGENT_PORT:-5560} \
    launch_rviz:=true \
    rviz_config:=$RVIZ/bg_av_1_autoware.rviz
```

For a single vehicle, `autoware_cast.rviz` is the same view over the whole right half.

## 2. Record

Any screen recorder works. With ffmpeg, the whole X display, stopped with `q` when the
scenario has ended:

```bash
ffmpeg -f x11grab \
    -framerate 15 \
    -video_size 1920x1080 \
    -i $DISPLAY \
    -vf scale=1280:720 \
    -c:v libx264 \
    -preset veryfast \
    -crf 23 \
    -pix_fmt yuv420p \
    demo.mp4
```

Start the recording, then the scenario in a terminal at the bottom left:

```bash
S=$(ros2 pkg prefix --share swb_examples)/scenarios
ROS_DOMAIN_ID=${SCENARIO_ROS_DOMAIN_ID:-9} play_launch launch \
    --web-addr 127.0.0.1:8081 \
    swb_launch scenario.launch.xml \
    scenario:=$S/multi_av/town01_two_av.xosc \
    port:=${SSV2_PORT:-5555} \
    output_directory:=/tmp/two_av
```

Two Autoware stacks and two RViz windows load a machine heavily; a recorded run is a demo,
not a measurement of Autoware's performance. Record on a quiet machine: Autoware's motion
planning stalls when other work saturates the disk, and the ego then stops for no reason on
camera.

**A GIF** for a README or a slide: `scripts/make_showreel.sh` in the source tree cuts the
showreel this way. It crops each window out of the 1280x720 recording, stacks them, speeds
the whole up 6 to 8 times and writes a palette GIF. Its crop rectangles are for the layout
above recorded as above (1920x1080 screen, scaled to 1280x720); adjust them for your own.

## Where next

- [User Guide](../user-guide.md): every launch and its arguments, the environment, your own
  Autoware, timeouts, troubleshooting.
- [Writing scenarios](../design/scenario-authoring.md): vehicles CARLA's Traffic Manager
  drives (`simulator_autopilot`), traffic lights, collisions.
- [Developer Guide](../development.md): if you want to change the bridge itself.
