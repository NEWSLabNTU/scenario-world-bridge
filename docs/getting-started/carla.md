# Getting started on CARLA

This page installs CARLA 0.9.16 and makes it known to scensim. The CARLA version is fixed when
scensim is built (a build feature of its CARLA client), and 0.9.16 is the version that build
targets and has been tested with. Most of the time goes to the download: the unpacked
release measures 30 GB. [Install](install.md) comes first.

## CARLA itself

Download the packaged release, `CARLA_0.9.16.tar.gz`, from
[CARLA's releases](https://github.com/carla-simulator/carla/releases/tag/0.9.16) and unpack
it, for example into `~/CARLA_0.9.16`
([CARLA's quick start](https://carla.readthedocs.io/en/0.9.16/start_quickstart/)). Its
Python client is used by `carla_poses`, which finds poses on a town
([Tutorial 2](../tutorials/02-write-a-scenario.md)), and by CARLA's BasicAgent pilot
([Tutorial 4](../tutorials/04-several-pilots.md)); it is installed for the user into
Humble's Python (the system `python3`, 3.10), as are the tools of
[Install](install.md#recommended-play_launch):

```bash
python3 -m pip install --user carla==0.9.16
python3 -c 'import carla; print(carla.__file__)'
```

## Let the stack start it (recommended)

The host settings (`~/.config/scensim/host.yaml`) record where CARLA is installed:

```yaml
carla: {root: ~/CARLA_0.9.16, host: localhost, port: 2000}
```

With `carla.root` set, the stack starts CARLA itself when nothing answers on the port
(`carla.start: auto`, the default), waits for it (up to `carla.ready_timeout`, 300 s) and
stops it with the stack. It runs CARLA off-screen, on the NVIDIA card, at the default
quality (Epic), and writes CARLA's output to
`<logs>/<project>/seat-<n>-current/proc/simulator/carla.log` (`logs:` of the host
settings, `~/.local/state/scensim/logs` by default). This mode is recommended for a first
drive: no separate process is started, and none remains afterwards. The BasicAgent pilot of [Tutorial 4](../tutorials/04-several-pilots.md)
needs `carla.root` anyway.

Off-screen means there is no CARLA window. To watch a drive, see
[Tutorial 1, step 4](../tutorials/01-first-drive.md#4-run-the-testbench).

## Or start it yourself

To keep one CARLA up between stacks, or to see CARLA's own window, start it in a terminal
of its own; the stack then uses it as it is and never stops it:

```bash
VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json \
    ~/CARLA_0.9.16/CarlaUE4.sh \
    -RenderOffScreen \
    -nosound \
    -carla-rpc-port=2000
```

The `VK_ICD_FILENAMES` line makes CARLA render on the NVIDIA card on a laptop or a host
with two GPUs; without it CARLA may pick another and take minutes to answer. Check the file
exists (`ls /usr/share/vulkan/icd.d/`); if there is no `nvidia_icd.json`, the NVIDIA
driver's Vulkan part is missing or names its file differently, and CARLA will not render on
the NVIDIA card until it is there. Leave out `-RenderOffScreen` to get CARLA's window (it
needs a display).

It serves its port after one to three minutes. **Check**:

```bash
ss -lnt | grep ':2000 '
```

prints one `LISTEN` line.

## Another port or another machine

`localhost:2000` is the default. For another, set it in `~/.config/scensim/host.yaml`, for
example `carla: {host: localhost, port: 3000}`. On a host with several stacks, the seat
table can give every seat its own CARLA port
([Several stacks on one host](../how-to/several-stacks.md#one-carla-per-stack)).

## If it does not work

| Symptom | Fix |
|---|---|
| The stack says CARLA did not answer within `carla.ready_timeout` | read the stack's `proc/simulator/carla.log`; a first start on a new host can take minutes: raise `carla.ready_timeout` |
| CARLA takes minutes to serve its port, then freezes | it came up on another GPU or a software renderer: keep the `VK_ICD_FILENAMES` line |
| A second CARLA on the same port crashes at start | the first one still holds it: stop it by its pid (SIGKILL if it ignores SIGTERM) |
| CARLA crashes when a scenario changes towns | it runs at `-quality-level=Low`: keep the default, Epic ([Supported and not supported](../reference/limits.md#running-carla)) |
| The bridge logs `CARLA version mismatch` | CARLA and the build are different versions: use 0.9.16 ([Supported and not supported](../reference/limits.md#platform)) |

## Next

[Tutorial 1: first drive](../tutorials/01-first-drive.md): a project, its map, the stack
and a first run with Autoware.
