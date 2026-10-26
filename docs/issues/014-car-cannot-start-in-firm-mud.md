# 014: a rear-drive car cannot start from rest in mud_wet, which the measured tire pulls in

**Severity**: medium (a car stopped in firm mud stays stopped: at full throttle it reaches
0.1 m/s in 3 s). **Status**: not a defect: what a rear-drive road car with traction control
does there (below); one model error found on the way and fixed (the clay's tire curve). **Found**: 2026-10-27, roadmap 053, after issue 013 took
the third of extra grip away.

## What is wrong

`mud_wet` is fitted to Wismer 1966's clay at cone index 46: the measured tire's drawbar
pull is +0.209 of its load at 13.5 % slip and +0.202 at 20 %, towed -0.085. A rear-drive car
with that tire starts: its driven wheels pull 0.2 of their load net while the free ones
resist 0.085 of theirs. On the car (`spikes/soil_drive.py`, `SOIL_SPIKE_ONLY=mud`, full
throttle from rest, `play_log/soil_drive/2026-10-27_013-mud_wet-peak015.log`) it reaches
0.1 m/s in 3 s and stays there:

- the soil's resistance at that crawl is 2.0-2.6 kN on the four wheels, against 1.1-1.2 kN
  rolling at 10 m/s (the measured towed resistance, 0.085 of 17.3 kN, is 1.47 kN): the
  driven wheels have dug in to 2.4 cm (1.5 cm rolling), by slip sinkage at their slip;
- traction control cycles the rear wheels' slip between 0.06 and 0.41 (it cuts at 0.25
  and reapplies at 0.15, `sim_vehicle_ecu`), so their mean traction is below the peak;
- the rear wheels' friction is the class's measured gross traction, 0.29 (issue 012), at
  most about 2.5 kN: the resistance takes all of it.

Before issue 013 the same run reached 1.07 m/s: the third of extra grip hid this.

## Not yet known

Which of the three is wrong against the measurement: the dig-in at a crawl (slip sinkage
fitted at 1 m/s steady, applied to a wheel spinning at near-zero speed, where PhysX's slip
is read with its 4 m/s denominator, issue 010), traction control's cycle on soft ground (a
real one holds a lower slip on low-friction ground), or the resistance's return below 0.1
m/s (`RESIST_V0`). A steady-slip run of the car at 13.5 % on mud_wet, against the
measurement's net pull, separates them.

## What it turned out to be (2026-10-27)

**The soil is right.** The grid alone (no PhysX), a Model 3 wheel at 4.3 kN on mud_wet:
rolling at 0.05, 0.1, 0.3, 1 or 5 m/s with the soil's slip at 13.5 %, it resists 0.091-0.097
of its load and grips 0.29 (the class's measured gross traction), net +0.19, as measured. No
speed effect in the soil itself.

**The tire curve was wrong, and is fixed.** mud_wet's (054 T1, engineered) fell to 0.64 of
its peak past 25 % slip, where traction control swings the slip. On clay the measured pull
"does not reach a definite maximum except possibly at a very high slip", 20 % slip giving
80-90 % of the pull at 100 % (Wismer 1966, Fig. 3 and paragraph 36; measured 0.209 at 13.5 %,
0.202 at 20 %). Both mud classes now peak at 20 % and keep it (`sliding: 1.0`). The launch
went from 0.10 to 0.17 m/s in 3 s (`play_log/soil_drive/2026-10-27_014-mud_wet-flat.log`):
not the cause.

**What stops the car is the start itself.** From rest the driven wheels spin before the car
moves and dig their own pits, 2.4 cm against 1.5 cm rolling; out of them the four wheels meet
2.0-2.6 kN, against 1.2 kN rolling at 10 m/s, while traction control holds the rear slip at
0.02-0.23, mostly below what this clay needs. Rolling into the same mud, each wheel meets
fresh soil at a small slip and the car holds 10 m/s asking 0.68 m/s^2.

**That is what a real rear-drive car does.** The evidence (no measurement of a passenger
car starting in clay exists; this is inference and practice):

- Wismer and Luth's clay equations for this soil: the car starts only if its rear wheels may
  slip more than about 15-20 %; at 10 % or less the net force is negative. A traction
  control holding slip near break-even gives a crawl like the 0.1 m/s seen.
- WES's mobility index puts this car's vehicle cone index at about 46 (315 kPa), the clay's
  own 320 kPa: at the limit. WES TM 3-240 (18th supplement) recorded a 1.65 t 4x4 station
  wagon that "could not travel as 4x2" at cone index 50 until its front drive was engaged.
- Tesla's Model 3 manual: traction control "minimizes wheel spin by controlling brake
  pressure and motor power"; Stuck Assist (formerly Slip Start) exists "to dislodge Model 3
  when stuck in mud", for "starting on a loose surface". Owners report a Model 3 RWD stuck in
  about 10 cm of mud, Slip Start not enough, towed out.
- Traction control patents (ZF/Kelsey-Hayes US6882921B2: on deformable ground "the vehicle
  slows to a stop" at a pavement slip target; Rivian US12227188B2: on sand, dirt or mud the
  car "may lose acceleration if the slip target ... is too low"). Pavement targets are about
  5-15 % slip; soft clay wants 15-30 %.
- Breakdown services: "the key is to keep moving"; too much power makes the wheels "spin and
  the car dig in" (Green Flag).

So `town01_mud_stop`'s car, stopped in mud_wet with Autoware asking 0.51 m/s^2 and traction
control on, staying stopped is the expected outcome, and driving through at speed is too.
What the simulated car lacks is a Stuck Assist mode (traction control allowing more slip at
a crawl), which Autoware would not use anyway.

Sources (gathered 2026-10-27): Tesla Model 3 owner's manual, traction control and Stuck
Assist (https://www.tesla.cn/ownersmanual/model3/en_cn/GUID-CC0F27E7-FD78-448D-B739-209EBBEDF2C3.html);
US6882921B2 (https://patents.google.com/patent/US6882921B2/en); US12227188B2
(https://patents.google.com/patent/US12227188B2/en); WES TM 3-240, 18th supplement
(https://digital.library.unt.edu/ark:/67531/metadc303870/m2/1/high_res_d/metadc303870.pdf);
Tesla Motors Club threads on Slip Start and on a Model 3 stuck in mud
(https://teslamotorsclub.com/tmc/threads/what-are-the-conditions-to-use-slip-start.141597/,
https://teslamotorsclub.com/tmc/threads/stuck-in-the-mud.175452/); Green Flag
(https://blog.greenflag.com/?p=7460). No measured one-pass cone index of a 4x2 car, no
passenger-tire drawbar data on clay and no Tesla slip setpoint were found.
