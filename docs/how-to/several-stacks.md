# Several stacks on one host

Two stacks on one machine (yours and a colleague's, or two projects of yours, or a lab's
students) must not share a ROS domain, a port or a CARLA. A **seat** is one stack's share
of the host: its domains, its ports, its web UI port and its GPU, all from one number. This
page sets seats up and uses them. Why the domains are split: [ROS domains and
DDS](../explanation/domains-and-dds.md).

## The seat table

```bash
ros2 run swb seat show
```

```
seats (defaults); locks in /tmp/swb-seats
seat 0: scenario domain 1, pilot domains 2-8, SSv2 5555 (+5556, 5557), relay 5560,
    world server 5591, web 8080 (run 8081), GPU 0; free
seat 1: scenario domain 9, pilot domains 10-16, SSv2 5655 (+5656, 5657), relay 5660,
    world server 5691, web 8090 (run 8091), GPU 0; free
...
your seat: auto (the first free one when the stack starts)
```

(Each seat is one line; wrapped here.) Seat `n` of the default table:

| Resource | Value |
|---|---|
| scenario domain | `1 + 8 n` |
| pilot domains | the next 7, in the order the pilots start (the ego first) |
| SSv2 port | `5555 + 100 n`, and the next two ports |
| relay port | SSv2 port + 5 |
| world server port | SSv2 port + 36 |
| web UI | `8080 + 10 n` for the stack, +1 for a run's scenario |
| GPU | `gpus[n]` of the table, else 0 |

A stack takes a seat when it starts and holds it while it is up (a lock file in
`/tmp/swb-seats`, naming the user, the process and the project); a second stack on a held
seat is refused, naming the holder. With `seat: auto` (the default) a stack takes the
first free seat.

## Take a seat

```bash
ros2 run swb seat take 2       # writes seat: 2 to ~/.config/swb/host.yaml
ros2 run swb seat release      # back to auto
```

A seat for one checkout only goes in its `.swb/local.yaml` (`seat: 2`), and `seat:=2` on
`swb.launch.xml` or `--seat 2` on `swb run` set it for one command. Start the stack with
the seat's web port: `play_launch` binds its web UI before the launch file is read, so the
port is the command line's:

```bash
play_launch launch --web-addr 127.0.0.1:8100 \
    swb_launch swb.launch.xml
```

The stack's first lines say which seat and web port it took. A terminal of your own joins
a seat's domain with `eval "$(ros2 run swb seat env scenario)"` (the bridge, the relay,
SSv2) or `seat env <k>` (the k-th pilot, from 0).

## One CARLA per stack

A CARLA world server drives one CARLA (a lock on CARLA's host and port), so each CARLA
stack needs a CARLA of its own. Start each on its own port, leaving the two after it free
(CARLA uses three), and give each user's host settings theirs:

```bash
VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json \
    ~/CARLA_0.9.16/CarlaUE4.sh -RenderOffScreen -nosound -carla-rpc-port=3000
```

```yaml
# ~/.config/swb/host.yaml of the second user
seat: 1
carla: {host: localhost, port: 3000}
```

CARLA's Traffic Manager takes port CARLA + 6000 (8000 for 2000, 9000 for 3000). Each CARLA
holds about 2.4 GB of RAM and 2 GB of video memory on Town01.

An Isaac stack needs an RTX GPU with 24 GB to itself: one Isaac stack per GPU.

## A table of your own

An administrator sets the host's table in `/etc/swb/host.yaml`, for every user; a user's
`~/.config/swb/host.yaml` can override it:

```yaml
# /etc/swb/host.yaml
seats:
  domains: {base: 1, stride: 8}     # seat n: scenario domain base + stride n
  ports: {base: 5555, stride: 100}
  web: {base: 8080, stride: 10}
  count: 6
  gpus: [0, 0, 0, 1, 1, 1]          # seat -> GPU index
seat_locks: /tmp/swb-seats          # a directory every user can write
logs: ~/.local/state/swb/logs       # per user, on a fast disk
```

Every domain must be 0 to 57 (the DDS profile's limit), and seats must not overlap; a
table that breaks either is refused when read. `stride` is the number of domains a seat
holds: the scenario's and `stride - 1` pilot domains, one per pilot (an Autoware each).

## Resources

Each Autoware is about 90 processes and several GB of GPU memory. On one GPU, two Autowares
beside one CARLA is a comfortable limit; three Autowares starting together saturated a
48-core host. Start stacks one after another, each once the previous one is ready.
