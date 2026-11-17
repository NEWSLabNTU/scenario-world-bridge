# Example testbenches

Working scensim testbenches, one use case each. Every one is a directory with a
`testbench.yaml` (the simulator, the assets, who drives each entity), its scenarios and, where
useful, suites.

| Example | Simulator | Shows |
|---|---|---|
| [first_drive](first_drive/) | CARLA | one Autoware ego: a drive to a goal, a red light, a pedestrian crossing |
| [multi_pilot](multi_pilot/) | CARLA | four pilots of four kinds in one run: Autoware, a second Autoware, CARLA's BasicAgent and your own agent cutting in |
| [safety_suite](safety_suite/) | CARLA | the Autoware Foundation's ACC and AEB use cases as a nightly suite, with an expected failure and a contact judge |
| [simple_drive](simple_drive/) | CARLA | the ego driven by `simple_pilot`, no Autoware: lane following to a goal, a turn at a corner |
| [isaac_first_drive](isaac_first_drive/) | Isaac Sim | one Autoware ego on CARLA's Town01 in USD |
| [road_physics](road_physics/) | Isaac Sim | mud, gravel and a wet road under the ego's wheels |

## Make your own copy

The installed examples are read-only (`share/scensim_examples/projects/`). Copy one into
a colcon workspace as a testbench package of your own, with its own `package.xml`:

```bash
mkdir -p ~/my_ws/src && cd ~/my_ws/src
scensim testbench new first_drive --from pkg://scensim_examples/projects/first_drive
cd ~/my_ws && colcon build --symlink-install && source install/setup.bash
scensim assets fetch first_drive
scensim testbench check first_drive
```

`--from` takes any testbench or project directory, too (a colleague's checkout); it copies
the files and leaves out what each host makes.

## The lock

The examples ship without `scensim.lock`. The first `assets fetch` of your copy resolves each
dataset to its current commit and writes the lock with every file's checksum; commit it in
your copy, and every later fetch, on any host, gets exactly those files.

Then follow the copy's own README: it names the commands, what a passing run prints, and
the hardware it needs.
