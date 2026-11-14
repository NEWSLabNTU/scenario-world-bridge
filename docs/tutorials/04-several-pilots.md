# Tutorial 4: Several pilots

**Goal**: one run on Town01 in which four vehicles are driven by four pilots of three
kinds: the ego and a second car by Autoware, each in its own ROS domain, a lead car by
CARLA's BasicAgent, and a car that cuts in by a Python program of your own.

```
$ ros2 run swb run scenarios/four_pilots.xosc
passed   2026-11-15T14-40-02_four_pilots_r0  .../multi_pilot/runs/2026-11-15T14-4...
```

## You will learn

- what a pilot is (a vehicle side plus a driver) and the kinds swb starts:
  `autoware`, `carla_basic`, `launch`, `command`, `none`;
- to bind scenario entities to pilots in `swb.yaml`, by name or by pattern;
- to check an agent of your own without a simulator;
- the difference between a pilot that lives with the stack and one started per run.

## Before you start

- [Tutorial 3](03-other-road-users.md), so that the stack, runs and controllers are
  familiar. Stop that project's stack first (Ctrl-C): this tutorial has its own.
- Two Autowares and CARLA on one host: one NVIDIA GPU with room for both Autowares'
  perception beside CARLA, and 32 GB of RAM or more.
- CARLA's install named in `~/.config/swb/host.yaml`, for the two pilots that use CARLA's
  Python API:

  ```yaml
  carla: {root: ~/CARLA_0.9.16, host: localhost, port: 2000}
  ```

- About 45 minutes.

The example is new: its junction and poses come from Town01's lanelet map, and it has not
yet been run end to end on a live stack (2026-11-15). If a run fails on the cut-in's
timing, exercise 2 is where to start.

## 1. Copy the example

```bash
ros2 run swb project new ~/multi_pilot \
    --from pkg://swb_examples/projects/multi_pilot
cd ~/multi_pilot
ros2 run swb assets fetch
cat swb.yaml
```

**What you see**: the fetch as in Tutorial 1, then the pilots:

```yaml
pilots:
  ego: {kind: autoware, vehicle: tesla_model3}
  bg_av_*: {kind: autoware, vehicle: tesla_model3}   # a second Autoware, its own domain
  lead: {kind: carla_basic}                          # CARLA's BasicAgent
  cut_in:                                     # your own program: pilots/cut_in.py
    kind: command
    run: [python3, pilots/cut_in.py, --trigger-m, "40"]
    lifetime: run                             # a fresh process per run
```

`scenarios/four_pilots.xosc` has the four entities, `ego` with the controller `Autoware`
and the other three with `agent`: driven from outside SSv2. Each key of `pilots:` is an
entity name or a pattern (`bg_av_*` matches `bg_av_1`).

**Check**: `ros2 run swb project check` prints `project: ok`. It refuses an `agent`
entity with no pilot, and a pilot kind the simulator cannot run (`carla_basic` on Isaac).

## 2. Check your own pilot without a simulator

`pilots/cut_in.py` is an agent on swb's SDK: a subclass of the SDK's `carla_basic` adapter
that holds the goal the scenario gives it until the ego is within its trigger distance,
then goes. swb starts it as

```
python3 <project>/pilots/cut_in.py --trigger-m 40 \
    --relay <url> --entity cut_in --seed <n> --run-dir <dir>
```

The SDK's conformance check runs it over a stand-in world, with no CARLA:

```bash
echo '{"world": "fake"}' > /tmp/fake.json
ros2 run swb_agent swb-agent check pilots/cut_in.py:CutIn --config /tmp/fake.json
```

**What you see**: one line per rule of the agent protocol, each `PASS`, ending

```
  PASS  registers again after a dropped connection
PASS: 16/16 rules
```

**Check**: `echo $?` prints `0`.

## 3. Bring up the stack

```bash
CARLA_DIR=~/CARLA_0.9.16 play_launch launch --web-addr 127.0.0.1:8080 \
    swb_launch swb.launch.xml \
    map:=Town01
```

`CARLA_DIR` in front of the command tells the BasicAgent pilot where CARLA's Python
`agents` package is; the stack does not pass it on from `carla.root` yet. Keep it out of
`~/swb.bash`: a `colcon build` with `CARLA_DIR` set fails.

The stack starts the pilots that live with it: both Autowares and the BasicAgent, each
nested play_launch in its own pilot domain of the seat. `cut_in` is `lifetime: run`: each
run starts it afresh.

**What you see**: `swb stack: pilot lead ready on Town01` almost at once,
`pilot ego ready` and `pilot bg_av_1 ready` once each Autoware can drive, then
`swb stack ...: ready`.

**Check**: which domain each pilot got, from the stack's record:

```bash
grep -E '"(entity|domain)"' \
    ~/.local/state/swb/logs/multi_pilot/seat-0-current/stack.json
```

(with another `logs:` in your host settings, that directory instead). Each entity has its
own domain.

## 4. Run the scenario

In a second terminal, in the project:

```bash
ros2 run swb run scenarios/four_pilots.xosc
```

`ego` comes down the north arm of a T junction, turns right and drives west; `lead`
drives west ahead of it; `bg_av_1` drives east in the oncoming lane; `cut_in` waits at the
south arm's stop line and turns into the ego's road just ahead of it.

**What you see**: `passed   <run id>  <run directory>` when the ego yields to the car
cutting in and both Autowares reach their goals; `failed` when the ego touches any of the
others.

**Check**:

```bash
cat runs/*four_pilots*/logs/pilot-cut_in/cut_in.json
```

shows the trigger distance the cut-in chose for this run.

## 5. A fresh pilot per run, with a seed

```bash
ros2 run swb run suites/cut_in_seeds.yaml
```

The suite runs the scenario five times, run k with seed 1000 + k. `cut_in.py` draws its
trigger distance from the seed (40 m, plus up to 5 m), so each repetition cuts in at
another gap, and a run's seed repeats it exactly.

**What you see**: five run lines, `..._r0` to `..._r4`, then `summary: .../summary.csv`.

**Check**: the five `cut_in.json` files hold five different distances.

## What happened

A **pilot** is what drives one entity: a **vehicle side** swb starts (on CARLA, the
vehicle bridge with the car's sensors and vehicle interface, in the pilot's domain) plus a
**driver**, plus the agent that registers the entity with the relay. The kinds:

| kind | Driver | Agent |
|---|---|---|
| `autoware` | Autoware with swb's profile | swb's vehicle agent |
| `carla_basic` | CARLA's BasicAgent; no vehicle side (CARLA only) | the SDK's `carla_basic` |
| `launch` | a launch file of yours, given the pilot contract | swb's vehicle agent, unless `agent: own` |
| `command` | a program of yours, given `--relay --entity --seed --run-dir` | the program itself |
| `none` | started by you | yours |

Every pilot gets its own ROS domain, because every Autoware uses the same topic names;
the scenario reaches each through the relay's TCP port. A `lifetime: stack` pilot lives
as long as the stack and serves run after run; a `lifetime: run` pilot starts with each
run and can take per-run values (`{seed}`, `{run_dir}`, `{param:<name>}`).
[Your autopilot as a pilot](../how-to/your-autopilot.md) shows `launch` and `command` for
your own driver; the [project file reference](../reference/project-file.md#pilots) lists
every key.

## Exercises

1. **Easy.** Drive `lead` with Autoware instead: rename it `bg_av_2` in the scenario (all
   its references), and restart the stack. Check: the stack console prints
   `pilot bg_av_2 ready`, and `stack.json` lists a third Autoware domain.
2. **Medium.** Make the cut-in later: `--trigger-m` 30 in `swb.yaml`, then restart the
   stack (a run refuses a stack started from another `swb.yaml`). Check: `cut_in.json`
   shows a distance between 30 and 35 m.
3. **Open.** Write a pilot of your own: `ros2 run swb_agent swb-agent new my-pilot` makes
   a skeleton that passes `swb-agent check`; give it to an entity as a `command` pilot.
   Check: `swb-agent check` passes, and a run with your pilot ends `passed` or `failed`,
   not `error`.

## If it does not work

| Symptom | Fix |
|---|---|
| `pilot lead` restarts again and again; its log says `No module named 'agents'` | the stack's terminal lacked `CARLA_DIR` (step 3) |
| `swb run: refused: the stack ... was started from another project state: ... differ` | `swb.yaml` (or the lock) changed since the stack started: restart the stack |
| One Autoware never becomes ready | two Autowares need GPU memory and RAM: close other GPU work; read `runs why` and the pilot's log under the stack's `proc/pilot-<entity>/` |
| `refused: entity bg_av_1 ... has no pilot in the stack` | the stack started before the pilot was added: restart it |

## Next

[Tutorial 5: suites](05-suites.md): parameters, repetitions and the results table.
