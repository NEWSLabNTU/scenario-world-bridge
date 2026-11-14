# Tutorial 2: Run and judge

Run many scenarios as one suite, add CARLA's own collision check to SSv2's verdict, and find
your way around the results. Uses the stack on CARLA with Autoware on Town01
([Build your own stack](../build-your-own-stack.md), sections 1 to 5) and the scenario from
[Tutorial 1](01-prepare-a-scenario.md).

## 1. A suite

`run_suite` runs scenarios one after another, each through `scenario.launch.xml` in its
own output directory, and merges the verdicts into one JUnit file. It runs in the scenario
domain, against the live stack:

```bash
S=$(ros2 pkg prefix --share swb_examples)/scenarios
ROS_DOMAIN_ID=${SCENARIO_ROS_DOMAIN_ID:-9} ros2 run swb_launch run_suite \
    $S/basic/town01_*.xosc \
    ~/my_scenarios \
    --output ~/swb_results/2026-11-14 \
    --relay tcp://localhost:${AGENT_PORT:-5560} \
    -- port:=${SSV2_PORT:-5555}
```

Arguments are scenario files (`.xosc`, `.yaml`) or directories (every scenario file in
them, sorted, recursively). Arguments after `--` go to every `scenario.launch.xml`. Keep a
suite to the town of the stack's `map_path`: a Town01 Autoware fails the other towns'
starters. It prints one line per scenario and a total (the six Town01 starters and
Tutorial 1's `follow_to_goal`):

```
[run_suite] 7 scenario(s), domain 9, results in .../swb_results/2026-11-14
[run_suite] (1/7) town01_ego_drive: .../basic/town01_ego_drive.xosc
[run_suite] PASS town01_ego_drive: 1/1 passed in 41 s
...
[run_suite] 7/7 scenario(s) passed; .../swb_results/2026-11-14/suite.junit.xml
```

Before each scenario it asks the relay whether the scenario's vehicles are registered, and
fails that scenario at once if one is not (`pre-flight: no vehicle agent registered ...`)
instead of letting it time out. If play_launch is installed it runs each scenario with it;
otherwise with `ros2 launch`.

## 2. CARLA's contacts in the verdict

SSv2 judges collisions in its own world: the declared boxes, in 2D, and nothing static. The
ego can clip a pole, a curb, or an NPC whose CARLA body is bigger than its declared box, and
SSv2 still passes the run. The bridge records what CARLA's collision sensor saw for every
run; `--physical-verdict` puts it next to SSv2's verdict:

```bash
ROS_DOMAIN_ID=${SCENARIO_ROS_DOMAIN_ID:-9} ros2 run swb_launch run_suite \
    ~/my_scenarios \
    --output ~/swb_results/physical \
    --physical-verdict \
    -- port:=${SSV2_PORT:-5555}
```

Each scenario now has two testcases, SSv2's and `carla_contact`:

```
[run_suite] PASS follow_to_goal: 2/2 passed in 52 s
```

| `carla_contact` | When |
|---|---|
| pass | no contact, or only contacts the scenario declares |
| failure | CARLA saw the ego touch something else; the message names it, the frame and the impulse |
| error | no contact report for the run (bridge too old, reports turned off) |

A scenario that is *meant* to make contact declares with whom (SSv2 ignores the parameter):

```xml
<ParameterDeclaration name="csbExpectedContacts" parameterType="string"
                      value="npc_rear" />
```

A comma-separated list of entity names; `static` allows contact with the static world. The
starter `town01_rear_contact` is an example. When the two testcases disagree (SSv2 passed,
`carla_contact` failed), the scenario passed only because SSv2 did not see what CARLA did.

## 3. Bodies that do not fit

The bridge gives every NPC the CARLA body closest in size to its declared box. When CARLA has
nothing close (a 12 m truck: the longest CARLA truck is 8.5 m), the `carla_contact`
testcase's output lists it, without failing anything:

```
body outside tolerance: 'Npc1' declared 4.00 x 1.80 x 2.50 m, spawned as
'vehicle.jeep.wrangler_rubicon' 3.87 x 1.91 x 1.88 m
(difference -0.13 / +0.11 / -0.62 m)
```

It tells you where the two worlds' geometry differ. Set `model3d` on the entity to choose the
body yourself.

## 4. The results

```
~/swb_results/physical/
  suite.junit.xml                       every scenario, merged (any JUnit viewer)
  follow_to_goal/
    launch.log                          the scenario launch's console
    play_log/                           per-node logs (only when play_launch ran it)
    scenario_test_runner/
      result.junit.xml                  SSv2's verdict for this scenario
      execution_timer.json              how long SSv2's frames took
      follow_to_goal/                   SSv2's per-run files
```

`suite.junit.xml` is what a CI job reads. The contact reports themselves are JSON under
`~/.ros/swb/contacts/`, one per run; `ros2 run swb_launch physical_verdict --help` judges a
single run's result outside a suite.

## 5. Scenarios with variants

SSv2's YAML scenarios can sweep parameters (`ScenarioModifiers`): one file, one run per
combination. The AWF use cases do this:

```bash
S=$(ros2 pkg prefix --share swb_examples)/scenarios
ROS_DOMAIN_ID=${SCENARIO_ROS_DOMAIN_ID:-9} ros2 run swb_launch run_suite \
    $S/awf/UC-AEB-001-0001_Standing-pedestrian.yaml \
    --output ~/swb_results/aeb \
    --physical-verdict \
    -- port:=${SSV2_PORT:-5555}
# [run_suite] PASS UC-AEB-001-0001_Standing-pedestrian: 10/10 passed in 537 s
```

Nine variants (ego speed x pedestrian distance) plus the contact check. SSv2 bounds each
variant's time itself; `--timeout-per-scenario S` caps a whole file if you need a hard limit.

Next: [Tutorial 3, other road users and more AVs](03-other-road-users.md).
