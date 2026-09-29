# GM torque road-load estimate

`gm-torque` starts from `lead-confidence-poc` (`25d60004559ad68563daa5864b8484c5a585cbcf`).
It keeps the acceleration-to-torque controller and makes road-load parameters part
of each GM platform's fingerprint configuration. The Sierra/Silverado parameters
are an experimental ACC calibration, not a measured coastdown fit.

## Acceleration-feedback refinement

The Sierra/Silverado openpilot-longitudinal tune now uses the existing integral
gain table to reduce small city-speed corrections and track changing highway
acceleration bias faster. No controller algorithm or planner policy is added.

| Speed | Previous Ki | Updated Ki |
| --- | ---: | ---: |
| 0–7.2 km/h | 0.20 | 0.20 |
| 18 km/h | 0.05 | 0.05 |
| 36–54 km/h | 0.05 | 0.025 |
| 72 km/h | 0.05 | 0.0875 |
| 90 km/h and above | 0.05 | 0.15 |

The existing interpolation blends continuously between breakpoints. Feedback
through 5 m/s (18 km/h) is identical to the previous tune. The parameter change
is confined to the Sierra/Silverado with openpilot longitudinal control. Stock
ACC and every other GM fingerprint keep their prior gains. Mass, geometry,
road-load scale 1.0, 0.3 s delay, torque conversion, brake tables, braking-mode
request, one-unit entry rule and final-stop path are unchanged. Pitch remains
disabled in the torque equation.

Route 285 segments 1–2 and 9–14 record the earlier 0.75 road-load build. The
response analysis separated positive propulsion, friction-brake units and a
braking-mode offset, removing slow local trends before fitting the dynamic
response. The response fits are sensitivities, not measurements of delivered
torque. They did not establish a reliable new physical torque or brake scale,
so this update changes only the existing feedback gains.

The tune was compared with the published 1.0 baseline in route-conditioned
differential projections using the source-identical native planner/MPC. The
planner receives projected ego speed/acceleration and lead range adjusted for
projected ego displacement. Recorded lead motion, neural outputs and unmodeled
disturbances are reused. Three response models vary lag, time constant and
propulsion/brake sensitivity. A uniform increase in moving-speed Ki was rejected
because it added city-speed brake pulses.

| Projection metric | Published 1.0 baseline | Updated gain table |
| --- | ---: | ---: |
| 40 km/h holding: contiguous brake pulses | 9 | 8 |
| 40 km/h holding: brake-active seconds | 2.64–2.84 | 2.40–2.68 |
| 40 km/h holding: speed RMSE | 0.228–0.234 km/h | 0.227–0.236 km/h |
| Highway, at least 80 km/h: contiguous brake pulses | 19 | 17 |
| Highway acceleration RMSE | 0.159 m/s² | 0.148–0.150 m/s² |
| Highway brake-active seconds | 69.95–70.11 | 69.27–69.55 |

Pulse counts are contiguous nonzero command intervals, not hydraulic brake
measurements. The 40 km/h speed-error change is at most 0.003 km/h in these
projections; one model changes crawl-exit brake duration by +0.04 s. The minimum
projected following range through the slow approach increases, while the minimum
highway range changes by about -0.07 to +0.13 m across models. All native solver
updates complete without failure and no additional stopping-state entries are
predicted. These results support a modest trial tune, not a claim that real-world
phantom braking or stopping distance has been validated. Reusing recorded
disturbances makes the recorded-build closure exact by construction; it is not
independent validation of the response model.

The new level reference is approximately +0.2 degrees in route 284's calibrated
frame. Inserting the corresponding +0.13 to +0.20 degree range directly into the
highway force predictor worsens its RMSE. The earlier roughly 1.1 degree fitted
offset must not be treated as an established neutral pitch, so grade compensation
is not part of this refinement.

## Vehicle parameters

`opendbc/car/gm/values.py`, `GMCarSpecs`, defines:

| Parameter | Sierra/Silverado | Meaning |
| --- | ---: | --- |
| Base mass | 2,450 kg | The interface adds the existing 136 kg payload allowance. |
| Operating mass | 2,586 kg | Confirmed by the owner; unchanged. |
| Wheel radius | 0.419 m | Nominal radius for the confirmed 275/60R20 tire size. |
| `dragArea` | `0.30 * 3.61` = 1.083 m² | Existing effective Cd multiplied by the revised frontal-area estimate. |
| `rollingResistanceCoefficient` | 0.004 | Effective rolling feedforward; reduced from the initial 0.008. |
| `roadLoadScale` | 1.0 | Full aero + rolling feedforward; the 0.75 experiment has been reverted. |
| Nominal air density | 1.225 kg/m³ | Fixed reference assumption, not a weather estimate. |

Drag area and the original rolling-resistance estimate came from the experimental Silverado values in
[opendbc PR #951](https://github.com/commaai/opendbc/pull/951#issuecomment-2409101515).
The current calibration halves rolling compensation after route-response checks
showed that the initial model underpredicted acceleration, especially at low
speeds. This is a conservative parameter adjustment, not a measurement of the
truck's tire coefficient. Mass remains unchanged; wheel radius and frontal area are updated from the confirmed tire size and revised geometry estimate.
Other GM fingerprints retain their prior torque mapping: `dragArea=None` selects
the legacy 0.3 force multiplier, and rolling resistance defaults to zero. A future
vehicle tune can set its own parameters in its `GMCarSpecs` entry. Fingerprinting
selects a configuration; it does not measure payload, tires, wind or road load.

## Force balance and allocation

The moving controller uses:

```
torque = radius * (mass * accel_command
                  + mass * gravity * sin(pitch)
                  + road_load_scale * (0.5 * air_density * drag_area * speed**2
                                       + rolling_coefficient * mass * gravity * rolling_blend))
```

`rolling_blend` rises continuously from zero at rest to one at 1 m/s. This is a
low-speed regularization, not a measured tire model, and prevents a new positive
torque offset at rest. Pitch is still passed as zero; no raw IMU pitch is treated
as measured road grade. Acceleration-command gain, mass and wheel radius remain
unchanged.

The same torque estimate drives propulsion and moving friction-brake allocation.
Using a new propulsion model with the old brake crossover can request brakes while
the model still requires positive propulsion. A delayed-plant check exposed this
failure, so that implementation was rejected.

In the existing `stopping` state, LongControl resets its integral feedback. The
controller therefore retains the previous final-stop force calculation and brake
lookup calibration in that state. Gas remains at the existing braking request.
Stopping-state entry, stopping acceleration, low-speed Ki and brake tables are
unchanged.

## Expected change

At zero acceleration command, the road-load feedforward is:

| Speed | Legacy mapping | Initial gm-torque (0.008) | Current gm-torque (0.004) |
| --- | ---: | ---: | ---: |
| 0 km/h | 0.0 Nm | 0.0 Nm | 0.0 Nm |
| 30 km/h | 8.9 Nm | 107.8 Nm | 61.8 Nm |
| 60 km/h | 35.4 Nm | 172.4 Nm | 119.7 Nm |
| 90 km/h | 79.7 Nm | 280.0 Nm | 216.2 Nm |
| 120 km/h | 141.7 Nm | 430.7 Nm | 351.3 Nm |

These are calculated requests, not measured delivered wheel torque. The original
Crr-only refinement, before the geometry update, removed 43.13 Nm of rolling
feedforward at speeds of at least 1 m/s, equivalent to 0.03924 m/s². The current
column also includes the revised geometry. Below 1 m/s, the existing blend
proportionally reduces that change to zero at rest. The adjustment applies across
speeds without another speed table. It can reduce the negative feedback correction required when the previous
feedforward was excessive, but does not directly filter planner fluctuations.
Recorded integral corrections belong to the recorded tune; frozen-input replay
cannot predict how feedback or the vehicle trajectory will adapt.

While moving, friction braking begins at a more negative acceleration command
because the model includes more natural road-load deceleration. For example, at
60 km/h the unrounded zero-torque point is about -0.1105 m/s², versus -0.1568 for
initial gm-torque and -0.0322 for the legacy mapping. Thus the refinement can
request slightly more moving friction braking at the same frozen acceleration
input, while requesting less propulsion. Gas and brakes retain the shared force
calculation; no separate crossover or deadband is introduced.
Rounding and the existing one-unit brake hysteresis also affect the actual onset.
This is a material actuation change; replay cannot establish real stopping
distance or comfort. The late-braking 27c approach still needs particular attention
in subsequent vehicle validation.

## Reverted road-load scaling experiment

The previous build applied a Sierra/Silverado-specific `roadLoadScale=0.75` to the
aerodynamic and rolling-resistance feedforward only. The planner/PID acceleration
term `mass * accel_command` remains full strength, and the gravity term is not
scaled so future grade compensation can remain physically independent. Other GM
platforms default to a scale of 1.0. The Sierra/Silverado now uses that default as
well after the driver reported no speed-holding improvement with 0.75.

With the revised 0.419 m wheel radius and 3.61 m² frontal-area basis, the scale
reduces zero-acceleration road-load torque from about 119.7 to 89.8 Nm at 60 km/h,
216.2 to 162.2 Nm at 90 km/h, and 351.3 to 263.5 Nm at 120 km/h. At 60 km/h the
moving zero-torque crossover shifts from about -0.1105 to -0.0829 m/s².

This was a feedforward experiment rather than a throttle multiplier.
It does not reduce requested acceleration gain and does not change the dedicated
`stopping` force calculation or final-stop brake calibration. Because lowering
road-load feedforward also moves the gas/brake crossover toward zero, it can cause
friction braking to begin at a milder negative acceleration request; vehicle logs
must determine whether earlier coasting reduces the later corrective brake events.

## Scope and validation

Stock planner targets, following distances, lead filtering, cruise policy, PID
gains, actuator delay, stop-state logic, CAN encoding and safety limits are
unchanged. The inherited confidence and lane-control behavior stays as it was on
the parent branch. There is no new temporal filter or acceleration gain reduction.

The initial release passed 19 GM tests and 7,776 platform/configuration comparisons, with
no changes to other GM fingerprints. Replay covered 55,493 actuation ticks across
37 uploaded route segments, including 2,072 unchanged final-stop ticks. Moving
brake commands changed on 11,643 ticks, with a maximum reduction of 26 command
units under the frozen old controller inputs. Only longitudinal gas/brake CAN
messages changed. Independent force-balance, encoding, limit and synthetic
delayed-plant crossover checks also passed. The route replay holds
the recorded vehicle state and acceleration command fixed; it compares mappings,
not the future vehicle trajectory. The synthetic plant is a regression check for
allocation consistency, not evidence that the chosen coefficients are accurate
for the real truck.

## Vehicle-geometry refinement

The current calibration also updates the Sierra/Silverado nominal wheel radius from
0.425 m to 0.419 m for the confirmed 275/60R20 tire size and revises frontal area
from 3.97 m² to 3.61 m². The existing effective Cd of 0.30 and the route-checked
rolling-resistance coefficient of 0.004 are unchanged. These geometry changes lower
zero-acceleration road-load torque from about 129.2 to 119.7 Nm at 60 km/h and from
about 387.6 to 351.3 Nm at 120 km/h. They do not change planner targets, PID gains,
brake tables, stopping-state calibration, or the mass-based acceleration gain.

## Rolling-compensation refinement

The refinement changes one production calibration value in `GMCarSpecs` for the
Sierra/Silverado fingerprint, from 0.008 to 0.004. Controller code, planner code,
lead processing, gains, actuator delay, stopping policy and brake tables are
byte-for-byte unchanged from the first gm-torque release.

Both calibrations were scored against the same recorded positive torque requests
and observed one-second speed changes, with the configured 0.30 s delay. Require
active PID, no driver pedals, positive torque, zero friction brake and speed at
least 1 m/s throughout the delay history and response interval. These samples
exclude braking and standstill. Parameters were selected after inspecting the
routes; these are calibration checks, not independent held-out validation.

| Data | One-second windows | Initial RMSE | Refined RMSE | Change |
| --- | ---: | ---: | ---: | ---: |
| Eight routes, pooled | 1,306 | 0.1353 | 0.1238 | 8.5% lower |
| Below 30 km/h, pooled | 44 | 0.2595 | 0.2310 | 11.0% lower |
| New route 281 | 77 | 0.1598 | 0.1455 | 9.0% lower |
| Route 281 below 30 km/h | 8 | 0.3020 | 0.2762 | 8.6% lower |

Errors are acceleration RMSE in m/s². Six of eight route aggregates improve;
26f worsens by 13.3% and 27c by 1.1%. On common samples across 0–0.60 s delay,
pooled error improves by 7.5–8.7%. The route-281 low-speed sample is particularly
small and still fits worse than the original legacy model (0.2541 m/s²).
Lower coefficients were also screened; 0.004 retains half the provisional
compensation instead of chasing the lowest pooled error or removing it entirely.
Unknown grade, drivetrain creep, torque delivery and delay prevent interpreting
this value as an independently identified physical tire coefficient.

The 19 GM tests and 7,776 platform/configuration comparisons pass. Fixed-input
replay covers 64,493 sampled actuation ticks across 43 segments and eight routes.
All 3,053 stopping-state ticks are unchanged; other GM platforms are unchanged.
Only longitudinal gas/brake CAN messages differ. Moving brake requests change on
11,442 ticks, with a maximum increase of eight command units. Positive propulsion
requests drop by up to 44 Nm after rounding; switching from small positive torque
to the existing -540 Nm braking request is not a measured wheel-torque step.
Synthetic delayed-plant checks at 10 and 60 km/h, with 0.30 and 0.60 s delays and
multiple assumed loads, settle without sustained brake cycling. These check
allocation behavior, not the real truck's plant response.

The segment-6 crawl acceleration mismatch remains larger than this parameter
change. No claim of eliminated phantom braking, improved real stopping distance
or validated launch response follows from these offline results. Further vehicle
logs should check low-speed acceleration, integral recovery and the moving
gas/brake transition before expanding the tune.

Before treating this as a validated tune, compare delivered acceleration and
steady integral correction over several speeds, plus lead-braking and final-stop
behavior. If bias varies approximately with speed squared, adjust drag area; if
the bias is approximately constant above the low-speed blend, investigate rolling
resistance and other steady loads. Grade and wind can mimic both effects.

## Route 285: rollback and delay check

The uploaded segments 1 and 2 of `615b11d4c01d81ec/00000285--dd310aa5eb`
record `gm-torque` root commit `04df252e568d6df38c7df42b8427190b2b5670bb`,
0.3 s longitudinal actuator delay, 2,586 kg and 0.419 m wheel radius. The log
reports a dirty build, but replay of the 0.75 controller matches every gas and
brake command on all 944 engaged actuation ticks used in the comparison.

Restore only the Sierra road-load scale to 1.0. Retain geometry, Crr=0.004,
controller structure, PID gains, 0.3 s delay, brake tables, stop behavior and
planner policy. At 40 km/h this adds about 19.2 Nm of road-load feedforward and
moves the unrounded zero-torque crossover from -0.0532 to -0.0709 m/s². The
0.75 experiment made friction braking start at a milder negative command.

With recorded motion, acceleration requests and engagement held fixed:

| Sample | Actuation ticks | Brake-active at 0.75 | Brake-active at 1.0 |
| --- | ---: | ---: | ---: |
| Both segments, engaged | 944 | 121 | 87 |
| 39–41 km/h, throttle permitted | 768 | 70 | 41 |
| Segment 2, engaged | 502 | 56 | 27 |

These are command-mapping projections, not closed-loop speed predictions.
Feedback, grade and subsequent speed would change on the next drive.

An independent source-identical full-planner replay ran 2,400 updates per delay
setting with recorded model, radar and vehicle inputs. At 0.3 and 0.5 s, all 758
engaged planner frames selected cruise and had identical acceleration targets;
all solver calls succeeded. The 0.3 s reconstruction had 0.00144 m/s² target RMSE
against the log (0.00360 at the 95th absolute-error percentile), reflecting
asynchronous input reconstruction. Experimental mode was off.

The cruise candidate uses current measured speed directly. The actuator delay
changes the MPC trajectory sampling horizon, so it does not anticipate reaching
set speed in this winning cruise path. Changing delay also changes a neural-model
input; inference was not rerun here, so this is conditional on recorded model
outputs. These logs do not validate 0.5 s for active lead following.

An exploratory two-channel command/acceleration fit over 28 s of speed holding
favored 0.24 s alignment with logged acceleration; separate propulsion-only checks
favored 0.24 and 0.28 s. Profiling first-order response time constants gives
0.1627 m/s² fit RMSE at 0.3 s versus 0.1834 at 0.5 s. This short, in-sample fit
cannot independently identify physical transport delay because grade, drivetrain
response, feedback and acceleration filtering are confounded. It supplies no
positive evidence for increasing the configured delay.

Segment 1 also contains a distinct model throttle-permission intervention near
47–50 s: the existing coast cap requests deceleration even below 40 km/h.
That policy remains intact. The marked segment-2 event is dominated by small
speed errors and gas/brake crossover pulses; no lead candidate wins while engaged.
The rollback passes all 18 torque-model/brake-map tests, including other GM
fingerprints, stock ACC tuning, command encoding, limits and final-stop behavior.
