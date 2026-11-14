# Example projects

Working scenario-world-bridge projects, one use case each. Every one is a directory with a
`swb.yaml` (the simulator, the assets, who drives each entity), its scenarios and, where
useful, suites.

| Project | Simulator | Shows |
|---|---|---|
| [first_drive](first_drive/) | CARLA | one Autoware ego: a drive to a goal, a red light, a pedestrian crossing |
| [multi_pilot](multi_pilot/) | CARLA | four pilots of four kinds in one run: Autoware, a second Autoware, CARLA's BasicAgent and your own agent cutting in |
| [safety_suite](safety_suite/) | CARLA | the Autoware Foundation's ACC and AEB use cases as a nightly suite, with an expected failure and a contact judge |
| [isaac_first_drive](isaac_first_drive/) | Isaac Sim | one Autoware ego on CARLA's Town01 in USD |
| [road_physics](road_physics/) | Isaac Sim | mud, gravel and a wet road under the ego's wheels |

## Make your own copy

The installed examples are read-only (`share/swb_examples/projects/`), and a project is
written to as it runs: `assets fetch` writes `swb.lock` and the `assets/` links, the stack
writes `state/`, runs write `runs/`. So copy one into a directory of your own first:

```bash
ros2 run swb project new ~/first_drive --from pkg://swb_examples/projects/first_drive
cd ~/first_drive
ros2 run swb assets fetch
ros2 run swb project check
```

`--from` takes any project directory, too (a colleague's checkout); it copies the files and
leaves out what each host makes (`assets/`, `state/`, `runs/`, `.swb/`).

## The lock

The examples ship without `swb.lock`. The first `assets fetch` in your copy resolves each
dataset to its current commit and writes the lock with every file's checksum; commit it in
your copy, and every later fetch, on any host, gets exactly those files.

Then follow the copy's own README: it names the commands, what a passing run prints, and
the hardware it needs.
