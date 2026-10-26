# Phase 053: Soil Calibration Against Published Measurements

Calibrate the soil classes against measured ground rather than against other simulators.
047 S5 fitted sand and dirt to Chrono SCM, and Chrono CRM disagreed with SCM on traction;
only measurements decide between them. Mud (Wong's clay row, `unvalidated:`) and snow have
not been checked against anything. This phase takes the measurements from published
datasets, so it does not wait for the NTU field (040), whose cone index and drawbar test
become the local check at the end.

**Status**: in progress, 2026-10-24. Step 1 (the survey) done. 2026-10-26: steps 2 and 3
done for sand (two classes now, `sand` and `sand_loose`, fitted to He 2020's measured tire);
dirt, mud and snow wait for data a script cannot fetch (step 2).

**Source**: user direction 2026-10-24: "Does soil calibration require real data? If yes, I
prefer to find real dataset online. Same for mud and snow cases. We can do dataset survey and
keep them noted."

**Depends on**: [047](047-off-road-surfaces.md) S5's rig (`spikes/crm_calibrate.py`),
[052](052-isaac-physics-completeness.md) (snow reachable from a scenario). **Feeds**:
[040](040-ntu-offroad-field.md), [041](041-competition-director.md).

## Steps

- [x] **1. Survey.** [design/terramechanics-datasets.md](../design/terramechanics-datasets.md):
  for each class (dry sand, sandy loam, wet clay as mud, snow), the published measurements
  (bevameter, shear, cone index, single-wheel and vehicle tests of pneumatic tires: pull,
  resistance and sinkage against slip), with soil state, tire, load, format and access.
- [ ] **2. One reference per class.** Sand done (He 2020, both compactions, read off the
  plots); dirt, mud and snow need the data fetched by hand (below). A primary and a fallback dataset per class, preferring
  a pneumatic tire near the Model 3's 4.5 kN wheel load; obtain the data (digitise a plot
  when only a plot is published, and say so).
- [ ] **3. Fit.** Sand done (`spikes/fit_reference.py`, below), with a new soil term,
  `rolling_resistance`. The single-wheel rig on our grid at the reference's tire and load, against
  the measured curves; fit what 047 S5 fitted (Janosi K, slip sinkage) and, where the
  reference measures it, the pressure-sinkage row. A table per class: measured, ours before,
  ours after.
- [ ] **4. Drop the flags.** A class within tolerance (pull within ±20 % over slip 0.1-0.5,
  sinkage within ±25 %) loses `unvalidated:`; one that is not keeps it, with the reason.
- [x] **5. Re-run.** `town01_soil.xosc`, `town01_mud.xosc` and `town01_snow.xosc` on Isaac
  with Autoware after the fit, against 052's runs.
- [ ] **6. The field (040).** Cone index and a drawbar test of a road tire on the field's
  sand and dirt, held to the same tolerance, when the field is available.

## Acceptance

- [x] Every soil class names its reference in `surface_classes.yaml` (`calibration:`, required
  of a soil class: the measurements, or where the row comes from and what is missing).
- [ ] The per-class table in this doc; each class calibrated or flagged with the reason.
- [x] The three soil scenarios PASS after the fit (2026-10-26, below; `town01_mixed_ground`
  too).

## Progress

### Step 1: the survey (2026-10-24)

[design/terramechanics-datasets.md](../design/terramechanics-datasets.md). Almost no open data
measures a passenger tire on soft soil. The exception, and the primary reference for sand, is
Virginia Tech's single-wheel rig with the Standard Reference Test Tire (P225/60R16) at 5 kN on
dry loamy sand (He 2020, open on VTechWorks): a peak drawbar-pull coefficient of 0.12 at about
12 % slip at 290 kPa, between treaded SCM's (about 0.3) and CRM's (below zero), which is the
disagreement 047 S5 left open. The picks for step 2, as the survey makes them:

| class | primary | access | what it lacks |
|---|---|---|---|
| sand | He 2020, SRTT on dry loamy sand | open | walking pace (6 cm/s), 1 % moisture |
| dirt | NG-NRMM 2018 Keweenaw bevameter, cone index, moisture | public (Distribution A) | a 12 t truck's tire: soil parameters only |
| mud | WES TR 3-666 (Wismer 1966), a 9.00-14 smooth tire on near-saturated fat clay | public domain | a tractor-size tire; stays `unvalidated:` |
| snow (fresh) | Lee et al. 2012, J. Terramechanics 49(5) | paywalled | the tire unverified |
| snow (packed), ice | CRREL 93-9 (Shoop 1993); Virginia Tech's ice data (Zenodo 1009017, CC BY 4.0) | public domain, open | |

`snow_loose` is exactly Wong's US snow row in SI units; its Janosi K is not from the table.
The survey's own unverified items (Wong's values from the literature, not the book; DTIC and
ERDC reports from catalogue records; some papers' tires) are marked there.

### Steps 2 and 3 for sand (2026-10-26)

**The reference.** He (2020), the dissertation the survey read: the SRTT, a 225/60R16
passenger tire, at 5 kN and 42 psi (290 kPa, a Model 3's pressure) on a dry loamy sand at two
compactions the dissertation tests at 42 psi, its single-wheel rig at 6 cm/s with the slip
pinned. The curves are published only as plots, so they were read off by eye at 220-250 dpi
(about +-0.005 of the pull coefficient and +-1 mm) into `spikes/references/`:

- `he2020_srtt_low.yaml`: freshly loosened (cone index 245 kPa), the drawbar pull
  coefficient and the rut depth from towed to 72 % slip (Figs. 4.42a, 4.44a). He: the rut
  stands for the sinkage up to 20 % slip; above 30 % the soil the tire throws back fills it,
  so there it is a lower bound.
- `he2020_srtt_medium.yaml`: two passes of the roller (cone index 601 kPa), the pull and the
  sinkage measured inside the tire, to 30 % slip (Figs. 4.42b, 4.45a).

Both curves peak at 12 % slip, as He says of every case. Loose: a peak pull coefficient of
0.11, a towed resistance of 0.135 and 29-38 mm of rut. Medium: 0.22, 0.045 and 17 mm of
sinkage tripling to 51 mm at 30 % slip.

**The fit** (`spikes/fit_reference.py`: our grid's single rigid wheel at the reference's
radius, width and load, the carrier at 1 m/s, the steady half of a 6 m run; Nelder-Mead on
the logarithms of the soil keys from several starts; tolerance as step 4 states it, pull from
the peak's slip to 50 %). What it found:

- **Wong's dry sand row (our `sand` until now) is far softer and grips far more than the
  measured tire.** On the medium sand it sinks the wheel 100-118 mm against 17-51 mm, pulls
  up to +83 % more past the peak and resists 4.4 times as much towed. On the loose sand: 101
  against 29-38 mm, pull +80 to +265 %.
- **The tire grips like a friction angle of 15-16 deg.** Past the peak the pull coefficient
  is 0.09 (loose) and 0.17 (medium); with the resistance it carries, the gross traction is
  about a quarter to a third of the load, where a dry sand's own friction angle (about
  33 deg) would give two thirds. Every start converged there (11-18 deg, cohesion fitted to
  nearly 0 when free). A rigid Bekker-Janosi wheel takes the soil's full shear; a road
  tire's tread on loose sand does not. Chrono's CRM said as much (047 S5: pull near 0, a
  rigid wheel's markers gripping far less than the full shear); the truth lay between, as S5
  suspected, nearer CRM. So `friction_angle` here is the tire's on that sand, not the sand's.
- **The grid's rigid wheel lacks a resistance.** Towed on the loose sand the tire resists
  0.135 of its load; with the sinkage held to the measured, a rigid wheel's compaction gives
  0.06-0.09, whatever the other keys. The missing part is the tire flexing in soft ground and
  the soil pushed ahead of it (Wong, Theory of Ground Vehicles, ch. 2). A new soil key,
  `rolling_resistance`: that share of the load the soil carries, added to the wheel's
  resistance (default 0, so dirt, mud and snow are unchanged); fitted with the rest.
- **Slip sinkage is strong.** The medium sand's sinkage triples from 2 % to 30 % slip; the
  free fit put Lyasko's factor to the power 3.9. Beyond 30 % that would be extrapolation the
  data does not back (the loose sand, measured to 72 %, doubles from 10 % to 50 %: power
  1.15), so `sand` takes 2.5, the least that holds the tolerance (at 30 % slip its sinkage is
  then 22 % short of the measured), and a second new key holds the factor past the slip the
  measurement reached: `slip_sinkage_until`, 0.3 for `sand` (0.72 for `sand_loose`; default
  0.95, as before). Without it the car could not start: in `spikes/soil_drive.py`'s
  full-throttle launch the rear wheels' first moment of spin, at the factor carried to slip 1
  (26 times the static sinkage), dug them 6.75 cm in and the car sat in its hole against
  5 kN of resistance (0.09 m/s after 3 s). Held at 0.3 it reaches 2.44 m/s in 3 s
  (Wong's row: 3.29; dirt: 7.72), and the rig's numbers to 30 % are unchanged.

| `sand` against He 2020, medium | towed | 4 % | 8 % | 12 % (peak) | 16 % | 20 % | 30 % |
|---|---|---|---|---|---|---|---|
| pull coefficient, measured | -0.045 | 0.108 | 0.210 | 0.220 | 0.190 | 0.173 | 0.170 |
| Wong's row (before) | -0.198 | 0.008 | 0.133 | 0.207 | 0.250 | 0.289 | 0.311 |
| fitted | -0.046 | 0.116 | 0.186 | 0.200 | 0.203 | 0.201 | 0.181 |
| sinkage mm, measured | | 17.0 | 19.5 | 23.0 | 31.0 | 36.5 | 51.0 |
| Wong's row | 100.5 | 102.7 | 105.0 | 107.3 | 109.7 | 112.0 | 118.0 |
| fitted | 16.9 | 19.2 | 21.7 | 24.2 | 27.1 | 30.3 | 40.0 |

Within tolerance: the pull -9 to +16 % past the peak and -3 % towed, the sinkage -22 to +13 %.

| `sand_loose` against He 2020, low | towed | 10 % | 11.5 % (peak) | 16 % | 20.5 % | 27 % | 36 % | 45 % | 54 % |
|---|---|---|---|---|---|---|---|---|---|
| pull coefficient, measured | -0.135 | 0.060 | 0.110 | 0.110 | 0.097 | 0.095 | 0.087 | 0.090 | 0.085 |
| Wong's row | -0.200 | 0.173 | 0.199 | 0.250 | 0.291 | 0.307 | 0.311 | 0.311 | 0.310 |
| fitted | -0.122 | 0.078 | 0.085 | 0.097 | 0.107 | 0.104 | 0.097 | 0.087 | 0.075 |
| rut mm, measured | 38.0 | 29.0 | | | 34.5 | 41.0 | 51.5 | 59.5 | 67.0 |
| Wong's row (sinkage) | 101.0 | 106.7 | | | 112.9 | 116.8 | 122.2 | 128.0 | 134.0 |
| fitted (sinkage) | 28.2 | 32.3 | 33.0 | 35.0 | 37.0 | 39.9 | 44.6 | 49.5 | 55.2 |

Within tolerance (pull -12 to +12 % past the peak, +10 % towed; sinkage -4 to +24 % from
10 to 27 % slip, above the rut) but at two points: the peak at 11.5 % slip (0.085 against 0.110, -22 %;
its error bar in Fig. 4.42a spans 0.06-0.16, and the next three points are within -16 %),
and the towed rut (-26 %: the rut is deeper towed than at 10 % slip, which He puts down to
soil carried back into the rut; a Bekker wheel's sinkage cannot fall with slip).

**Two classes.** On the loose sand a rear-drive car cannot start: its driven wheels pull at
most 0.11 of their load while its towed front wheels resist 0.135 of theirs. That is a real
result (cars do bog in freshly loosened sand) but not what a sand track a car is meant to
cross should do, so `sand` is the medium sand (a track that has been driven on) and
`sand_loose` the loose one, for scenarios that mean to stop a car. Each class's `mu` (the
friction where it is a label, not a soil zone) is now its soil's limit (0.3, 0.26; was 0.5),
and `peak_slip` the measured 0.12 (was 0.3). The lateral values (`peak_slip_angle`, T2's
sidewall) stay engineered: nothing measured sideways (054's open questions).

**What the fit cannot say.** The tire ran at 6 cm/s; the soil layer is driven at 10-20 m/s,
and no open data shows how pull or sinkage change with speed. The soil is dry (1 %
moisture); a moist sand is cohesive and grips more. The wheel in the grid is rigid, so the
fitted row is a rigid wheel equivalent to this tire at this load, not the sand's plate row
(He's Table 5.8, also from the tire's ruts, is recorded in the reference files, not used).

**Dirt, mud and snow: the data a script cannot fetch.** The NG-NRMM CDT bevameter set
(dirt's primary, Distribution A) is on KRC's FTP server, whose login is printed in the
public access sheet (`mtu.edu/cdt`, `data-set-access-info.pdf`); WES TR 3-666 Report 3
(mud's) and the CRREL snow reports are public domain on DTIC and the ERDC library, which
answer a script with 403 and a captcha. Each needs a person to fetch it once; then steps 2
and 3 run as for sand. Until then the three keep Wong's rows and say so in `calibration:`.

On the car (`spikes/soil_drive.py`, `play_log/soil_drive/2026-10-26_053-sand-until.log`):
coasting into `sand` at 15 m/s slows 0.33 m/s^2 (Wong's row 1.22), holding 10 m/s asks
0.35 m/s^2 with 1.4 cm of sinkage, the launch above, and a stop from 15 m/s takes 24.7 m
(dry asphalt 14.0).

### Step 5: the scenarios after the fit (2026-10-26)

On Isaac with Autoware, one at a time, each with its own surface map and every stack stopped
between them, the host at a load of 20-90 on 24 cores from other work:

| scenario | result | recording (`play_log/record/`) |
|---|---|---|
| `town01_soil` | PASS | `2026-10-26_21-45-06-ego` |
| `town01_mixed_ground` | PASS | `2026-10-26_21-50-09-ego` |
| `town01_mud` | PASS | `2026-10-26_21-55-18-ego` |
| `town01_snow` | PASS | `2026-10-26_21-58-53-ego` |

Mud and snow keep Wong's rows, so their runs only show that the new code paths
(`rolling_resistance` and `slip_sinkage_until` at their defaults, the `calibration:` check)
change nothing for them. On the fitted sand the ego crossed at 3.9-4.2 m/s with the wheels'
longitudinal slip at most 0.02 (`town01_mixed_ground`, 123 frames on sand); `town01_soil`'s
route clips its sand zone for only 0.3 s, so the mixed-ground run is the one that exercises it.

