# Getting started on CARLA

CARLA 0.9.16 installed, running, and found by swb. About 20 minutes, most of it the
download. Do [Install](install.md) first.

## CARLA itself

Download the packaged release, `CARLA_0.9.16.tar.gz`, from
[CARLA's releases](https://github.com/carla-simulator/carla/releases/tag/0.9.16) and unpack
it, say into `~/CARLA_0.9.16`
([CARLA's quick start](https://carla.readthedocs.io/en/0.9.16/start_quickstart/)). Then its
Python client, which the pose helper and CARLA's BasicAgent pilot use:

```bash
pip install carla==0.9.16 networkx shapely
```

## Start it

CARLA runs in a terminal of its own and stays up between runs and between projects:

```bash
VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json \
    ~/CARLA_0.9.16/CarlaUE4.sh \
    -RenderOffScreen \
    -nosound \
    -carla-rpc-port=2000
```

The `VK_ICD_FILENAMES` line makes CARLA render on the NVIDIA card on a laptop or a host
with two GPUs; without it CARLA may pick another and take minutes to answer. Leave out
`-RenderOffScreen` to get CARLA's window.

It serves its port after one to three minutes. **Check**:

```bash
ss -lnt | grep ':2000 '
```

prints one `LISTEN` line.

## Tell swb where it is

swb finds CARLA through the host settings. `localhost:2000` is the default; for another
port or another machine, set it in `~/.config/swb/host.yaml`:

```yaml
carla: {host: localhost, port: 2000}
```

swb does not start CARLA yet: keep it running as above. Its quality level matters when a
scenario changes towns: at `-quality-level=Low` CARLA 0.9.16 crashes on a town change, so
keep the default (Epic) unless every run stays on the town CARLA started with.

## If it does not work

| Symptom | Fix |
|---|---|
| CARLA takes minutes to serve its port, then freezes | it came up on another GPU or a software renderer: keep the `VK_ICD_FILENAMES` line |
| A second CARLA on the same port crashes at start | the first one still holds it: stop it by its pid (SIGKILL if it ignores SIGTERM) |
| The bridge logs `CARLA version mismatch` | CARLA and the build are different versions: use 0.9.16, or build for yours (README, Other CARLA versions) |

## Next

[Tutorial 1: first drive](../tutorials/01-first-drive.md): a project, its map, the stack
and a first run with Autoware.
