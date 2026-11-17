# Verdicts

What `passed`, `failed` and `error` mean, why CARLA's own contacts are checked beside
SSv2's verdict, and why a judge's verdict never replaces the run's.

## Three verdicts

| Verdict | When | Exit code |
|---|---|---|
| `passed` | SSv2 passed the scenario | 0 |
| `failed` | SSv2 failed the scenario: its `exitFailure` fired, a condition was not met in time, the ego collided where the scenario says it must not | 1 |
| `error` | the run cannot be judged: the stack failed (a world server or bridge error, a pilot that never became ready, a map switch that failed, SSv2 ending on an infrastructure error), or a pilot's process crashed during the run | 2 |

The difference between `failed` and `error` is the one a test table needs: a `failed` run
says something about the autopilot, an `error` says something about the setup. A suite's
`retry: {on: error}` runs an `error` again; it never retries a `failed` run. `result.json`
holds the first cause in `reason`, and `ros2 run scensim runs why` the first warning or error
in any process's log.

A pilot whose process crashes while the scenario runs (an Autoware component, a node)
ends the run `error`, with the crash in `reason` ("pilot ego: component euclidean_cluster
crashed (killed by signal 11) during the run"): part of the autopilot was not running, so
the run did not test it as configured. A crash is a finding about the autopilot or its
environment, which scensim cannot tell apart: it is not counted as `failed`, and it is never
hidden: `reason` names the component and the signal, `runs list` shows the run as
`error`, and a suite's `summary.csv` carries the reason, so its crashes can be picked out
("during the run"). A process that only displays (RViz and the marker publishers it shows)
is not part of the autopilot: its crash is a warning in the run's output and timeline, and
the run stands. The verdict is the same under either launcher, and
so is what follows: scensim then starts the stack's pilots again before any other run (`scensim
run` says `pilots   restarting after the crash of ego`). The launchers differ underneath.
play_launch runs each component in a process of its own and restarts it alone; under `ros2
launch` a component's crash ends its whole container, and the pilot's launch runs on
without it, which is why the pilots start afresh.

## The verdict is SSv2's

SSv2 judges the scenario in its own world: the declared bounding boxes, in 2D, and nothing
static. That is what OpenSCENARIO describes and what the scenario's author wrote, so it is
the run's verdict. Its limits are known:

- the ego can clip a pole or a curb, which SSv2 does not model;
- a car whose simulator body is larger than its declared box can touch the ego without
  SSv2's boxes meeting.

## CARLA's contacts beside it

The bridge records what CARLA's contact sensor saw for every ego run. The
`physical_verdict` judge reads that record and adds a `carla_contact` verdict beside
SSv2's ([Tutorial 5](../tutorials/05-suites.md#3-add-a-judge)):

| `carla_contact` | When |
|---|---|
| passed | no contact, or only contacts the scenario declares |
| failed | CARLA saw the ego touch something else; the message names it, the frame and the impulse |
| error | no contact report for the run |

A scenario that is *meant* to make contact declares with whom (SSv2 ignores the
parameter); `static` allows contact with the static world:

```xml
<ParameterDeclaration name="csbExpectedContacts" parameterType="string"
                      value="npc_rear" />
```

When the two disagree (SSv2 passed, `carla_contact` failed), the scenario passed only
because SSv2 did not see what CARLA did.

The bridge gives every NPC the CARLA body closest in size to its declared box. When CARLA
has nothing close (a 12 m truck: the longest CARLA truck is 8.5 m), the judge's output
lists it without failing anything:

```
body outside tolerance: 'Npc1' declared 4.00 x 1.80 x 2.50 m, spawned as
'vehicle.jeep.wrangler_rubicon' 3.87 x 1.91 x 1.88 m
(difference -0.13 / +0.11 / -0.62 m)
```

It tells you where the two worlds' geometry differ; `model3d` on the entity chooses the
body yourself.

## Judges beside the verdict, never in its place

A judge is a program over a finished run: a comfort score, a contact check, a course's
grading rule. Its verdict is recorded beside the run's (the run line, `summary.csv`'s
`judges` column, `judges/<name>.json`), never in place of it. So the run's verdict always
means one thing, SSv2's, and a scoring rule can change, or be versioned (`version:`), or
be applied again to old runs, without rewriting what happened. A gate that needs both
reads both.
