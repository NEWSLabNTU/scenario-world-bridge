# Several stacks on one host

Two stacks on one machine (yours and a colleague's, or two testbenches of yours, or a lab's
students) must not share a ROS domain, a port or a CARLA. A **seat** is one stack's share
of the host: its domains, its ports, its web UI port and its GPU, all from one number. This
page sets seats up and uses them. Why the domains are split: [ROS domains and
DDS](../explanation/domains-and-dds.md).

## The seat table

```bash
scensim seat show
```

```
seats (defaults); locks in /tmp/scensim-seats
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

A stack takes a seat when it starts and holds it while it is up (a lock on a file in
`/tmp/scensim-seats`, naming the user, the process and the testbench); a second stack on a held
seat is refused, naming the holder. With `seat: auto` (the default) a stack takes the
first free seat. The lock is the kernel's: when the stack's process ends, however it ends,
the seat is free again, so a crashed stack leaves no stale lock. What a stack killed by
hand can leave behind are its child processes, still holding the seat's ports: the next
stack on that seat then fails with `Address already in use`. Find them with `ps -u $USER
-o pid,ppid,etimes,comm` (orphans have parent 1) and stop them by pid.

A free lock is not enough: before a stack starts, scensim also checks that nothing listens on
the seat's ports (SSv2 and its two sensor-release sockets, the relay, the world server, and
the web UI when `scensim run` starts the stack). A process outside the seat table, such as a
stack a developer started by hand on its own ports, may hold them. `auto` then takes the
next seat; a seat you named is refused, the message naming each port in use. `ss -ltnp`
names the process.

## Take a seat

```bash
scensim seat take 2       # writes seat: 2 to ~/.config/scensim/host.yaml
scensim seat release  # back to auto
```

A seat for one testbench on this host goes in its work directory's `.scensim/local.yaml`
(`<logs>/.testbenches/<testbench>/`, `seat: 2`), and `--seat 2` on `scensim run` or
`seat:=2` on `scensim.launch.xml` sets it for one command. Under `ros2 launch`
the stack needs nothing more. Under play_launch, which binds its web interface before the
launch file is read, give it the seat's web port on its command line, for a stack in the
foreground of a terminal (its `project:=` is the testbench's work directory):

```bash
play_launch launch --web-addr 127.0.0.1:8100 \
    scensim_launch scensim.launch.xml project:=<logs>/.testbenches/<testbench>
```

The stack's first lines say which seat and web port it took. A terminal of your own joins
a seat's domain with `eval "$(scensim seat env scenario)"` (the bridge, the relay,
SSv2) or `seat env <k>` (the k-th pilot, from 0).

## One CARLA per stack

A CARLA world server drives one CARLA (a lock on CARLA's host and port), so each CARLA
stack needs a CARLA of its own. CARLA uses its port and the two after it, and its Traffic
Manager takes port CARLA + 6000 (8000 for 2000). Each CARLA held about 2.4 GB of RAM and
2 GB of video memory on Town01.

The simplest way: give every seat its own CARLA port in the seat table, and let each stack
start its own CARLA there. In `/etc/scensim/host.yaml` (or a user's `~/.config/scensim/host.yaml`):

```yaml
seats:
  carla: {base: 2000, stride: 10}   # seat n: CARLA on 2000 + 10 n
carla:
  root: /opt/CARLA_0.9.16           # where CARLA is installed, readable by every user
```

`seat show` then prints each seat's CARLA port, and a stack on seat `n` uses CARLA on
`2000 + 10 n`, starting it there when nothing answers (`carla.start: auto`) and stopping it
with the stack. `stride` must be at least 3, and a table whose CARLA ports meet the seats'
other ports is refused.

Without `seats.carla`, every stack uses `carla.port` (2000): give each user or checkout its
own in its host settings or `.scensim/local.yaml`, for example `carla: {port: 3000}`, and start
that CARLA, or let the stack start it from `carla.root`:

```bash
VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json \
    ~/CARLA_0.9.16/CarlaUE4.sh -RenderOffScreen -nosound -carla-rpc-port=3000
```

An Isaac stack needs an RTX GPU with 24 GB to itself: one Isaac stack per GPU.

## A table of your own

An administrator sets the host's table in `/etc/scensim/host.yaml`, for every user; a user's
`~/.config/scensim/host.yaml` can override it:

```yaml
# /etc/scensim/host.yaml
seats:
  domains: {base: 1, stride: 8}     # seat n: scenario domain base + stride n
  ports: {base: 5555, stride: 100}
  web: {base: 8080, stride: 10}
  count: 6
  gpus: [0, 0, 0, 1, 1, 1]          # seat -> GPU index
seat_locks: /tmp/scensim-seats      # a directory every user can write
logs: ~/.local/state/scensim/logs   # per user, on a fast disk
```

Every domain must be 0 to 57 (the DDS profile's limit), and seats must not overlap; a
table that breaks either is refused when read. `stride` is the number of domains a seat
holds: the scenario's and `stride - 1` pilot domains, one per pilot (an Autoware each).

## Resources

Each Autoware is about 90 processes; one Autoware with the default lidar detector held
about 1 to 2 GB of video memory on one host. On one GPU, two Autowares beside one CARLA is
a comfortable limit; three Autowares starting together saturated a 48-core host. Start
stacks one after another, each once the previous one is ready.
