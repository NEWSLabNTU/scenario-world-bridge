# Record a demo

A screencast like the ones in the README: CARLA from behind the ego, and each Autoware's
RViz beside it. It assumes a CARLA project with a working stack
([Tutorial 1](../tutorials/01-first-drive.md)); the two-Autoware layout uses a project
like [Tutorial 4](../tutorials/04-several-pilots.md)'s.

A recorded run is a demo, not a measurement: two Autowares, two RViz windows and a screen
recorder load a machine heavily. Record on a quiet machine: Autoware's planning stalls when
other work saturates the disk, and the ego then stops for no reason on camera.

## The windows

**A chase camera** on the ego (CARLA's vehicle named `hero`), in a window of its own:

```bash
SDL_VIDEO_WINDOW_POS=70,28 VIEW_W=740 VIEW_H=416 \
    ros2 run swb_launch chase_view
```

It is a free camera moved behind the vehicle every frame, not attached to it, so a
scenario ending never leaves a sensor behind; it waits for the next ego when there is none.
It needs CARLA's Python client and reads CARLA's port from `CARLA_PORT` (default 2000).

**RViz layouts** that put the Autoware windows on the right half of a 1920x1080 screen,
each following its own vehicle from behind, ship with `swb_launch`. Give them to the
pilots in `swb.yaml`:

```yaml
pilots:
  ego:
    kind: autoware
    rviz: true
    args: {rviz_config: "$(find-pkg-share swb_launch)/rviz/ego_autoware.rviz"}
  bg_av_*:
    kind: autoware
    rviz: true
    args: {rviz_config: "$(find-pkg-share swb_launch)/rviz/bg_av_1_autoware.rviz"}
```

For a single vehicle, `autoware_cast.rviz` is the same view over the whole right half.

**The weather.** CARLA's Town01 is overcast and raining by default. With the stack up:

```bash
eval "$(ros2 run swb seat env scenario)"
ros2 service call /scenario_world_bridge/set_weather \
    swb_interfaces/srv/SetWeather "{preset: ClearNoon}"
```

The world server keeps it across map loads ([Configure the stack](configure.md#the-weather)).

## Record

Any screen recorder works. With ffmpeg, the whole X display, stopped with `q` when the
run has ended:

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

Start the recording, then the run, in a terminal at the bottom left:

```bash
ros2 run swb run scenarios/two_av.xosc
```

A GIF for a README or a slide: crop each window out of the recording with ffmpeg's `crop`
filter, stack them, speed the whole up 6 to 8 times (`setpts=PTS/6`) and write a palette
GIF (`palettegen`, `paletteuse`).

## Record the data, not the screen

`ros2 run swb run <scenario> --capture full` records the pilots' key topics (ego state,
control command, vehicle status, detected objects, trajectory) as a rosbag per pilot, into
`runs/<id>/record/<entity>/`, and keeps it whatever the verdict; `--capture debug` records
every topic. Play one back beside RViz with `ros2 bag play`
([Results and logs](../reference/results-and-logs.md)).
