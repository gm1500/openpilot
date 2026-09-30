# GM torque road-load estimate

`gm-torque` starts from `lead-confidence-poc` (`25d60004559ad68563daa5864b8484c5a585cbcf`).
It keeps the acceleration-to-torque controller and makes road-load parameters part
of each GM platform's fingerprint configuration. The Sierra/Silverado parameters
are an experimental ACC calibration, not a measured coastdown fit.

## Acceleration-feedback refinement

The Sierra/Silverado openpilot-longitudinal tune uses a strong integral correction
only near the final approach, then rapidly reduces integral authority with speed.
The current experimental table is:

| Speed | Previous Ki | Current Ki |
| --- | ---: | ---: |
| 0–7.2 km/h | 0.20 | 0.20 |
| 18 km/h | 0.05 | 0.02 |
| 36 km/h | 0.025 | 0.005 |
| 54 km/h | 0.025 | 0.005 |
| 72 km/h | 0.0875 | 0.00375 |
| 90 km/h and above | 0.15 | 0.0025 |

The exact breakpoints are `[2, 5, 10, 15, 25] m/s` with Ki values
`[0.2, 0.02, 0.005, 0.005, 0.0025]`; interpolation between them is continuous.
The high low-speed gain is retained for the final approach, while city and highway
feedback is intentionally much weaker to reduce stored acceleration correction
carried through the cruise setpoint.

The Sierra/Silverado `longitudinalActuatorDelay` is now 0.5 s, up from 0.3 s.
In route 286 segment 1, a command-to-measured-acceleration timing fit around the
40 km/h approach/hold aligned best near 0.58 s effective lag; 0.5 s actuator
delay corresponds to a 0.55 s MPC action point after the model timestep is added.
That fit includes drivetrain response and filtered `aEgo`, so it is an effective
control delay rather than a direct ECU dead-time measurement.

The delay parameter changes where the lead MPC trajectory is sampled; it does
not delay or freeze the LongControl integrator, and it does not directly change
the no-lead cruise acceleration candidate. The reduced Ki table therefore targets
the observed integral carry-through independently of the longer MPC timing.
Stopping-state logic still resets the integral and retains the dedicated final-stop
calibration. Stock ACC and other GM fingerprints keep their existing tuning.

Mass, geometry, road-load scale 1.0, torque conversion, brake tables, braking-mode
request, one-unit entry rule and pitch handling are unchanged by this refinement.

## Adaptive vision lead range velocity experiment

The `lead-range-adaptive-v1` branch extends the private vision range tracker to
low speed and close range after the existing maturity gate. Raw model velocity
remains the fallback while the range track is immature, and the existing hard
braking/TTC guard still overrides a lagging range estimate immediately.

Uploaded route data shows raw `dRel` high-frequency residuals increasing strongly
with range: about 0.06 m inside 5 m, 0.14 m at 5-10 m, 0.28 m at 15-20 m,
0.55-0.65 m at 30-50 m, and about 0.92 m at 50-70 m. The quiet steady-following
process-noise schedule is therefore still distance dependent:

| dRel breakpoint | Process noise |
| ---: | ---: |
| 5 m | 2.5 |
| 10 m | 1.0 |
| 20 m | 0.2 |
| 35 m | 0.05 |
| 50 m | 0.03 |
| 70 m | 0.025 |
| 100 m | 0.02 |

The original 1.0 m minimum measurement-standard-deviation floor remains unchanged.

A fixed-motion native planner replay of the earlier `82a811d` candidate found a
mixed result: highway planner fluctuation improved, but low/mid-speed fluctuation
increased and several sustained-closing events reached stronger deceleration
later than `gm-torque`. Synthetic probes reproduced the same issue for mild
lead deceleration: the quiet Kalman schedule could lag a real sustained speed
change even though the hard-braking guard remained intact.

The refinement keeps the quiet distance schedule for normal following, but adds
a separate sustained-closing response path. It activates only when both of these
causal cues agree that the distance Kalman state is too fast:

1. model lead velocity is at least 0.35 m/s below the range state; and
2. a 0.6 s high-confidence raw-range trend, including a +2 sigma uncertainty
   bound, also implies a lead speed at least 0.35 m/s below the range state.

When corroborated, the published range-derived speed is pulled down toward the
less aggressive of the model and range-trend estimates at no more than
2.5 m/s per second and no more than 3.0 m/s total offset. The Kalman process
noise is also temporarily boosted, but only up to the previous nominal 0.1
base responsiveness; the far-range steady-following filter therefore does not
become globally noisier.

Near standstill, the distance state continues learning privately, but when ego
speed is below 5 m/s, range is below 10 m, and the model lead speed is below
1 m/s, the published speed is held to the model baseline. This preserves the
previous stop/launch semantics that the earlier all-speed extension changed.
When the model indicates the lead is moving again, release begins no lower than
that moving model speed before blending back toward the range state.

The regression test suite now also checks that isolated model-speed drops or
isolated range trends cannot trigger the sustained-closing response, that the
dynamic far-range boost never exceeds the previous nominal process-noise level,
and that stop-hold release does not begin below the model speed. The stale 20 m
process-noise assertion from the earlier candidate is corrected from 1.2 to the
intended 0.2.

Fresh route 288 data exposed two additional stop-approach effects. First, a
standstill-to-PID launch accumulated about +0.39 m/s² of integral while the
vehicle was still waiting for its longitudinal response, and roughly
+0.16-0.17 m/s² of that bias was still opposing braking at the later stopped-lead
approach. LongControl now freezes integral accumulation, while preserving the
planner feedforward command, for the configured longitudinal actuator delay
when leaving the stopping state below 1 m/s with a positive acceleration target.
With the Sierra's 0.5 s delay this targets launch windup without reducing the
normal low-speed Ki used during braking.

Second, a confirmed stopped-lead mode handles the model's small residual creep
estimate. A mature vision lead is treated as stopped only after 0.30 s with all
of the following true: ego at least 2 m/s, model lead speed at or below
0.75 m/s, absolute model lead acceleration at or below 0.25 m/s², high model
probability, and range inside a 6 s closing-time envelope. The published speed
is then clamped to zero while the private distance state continues learning.
The mode releases immediately to no lower than the moving model speed once the
stopped conditions no longer hold. This supplements, rather than replaces, the
existing hard-braking/TTC guard and near-standstill stop-hold path.

These latest changes have not yet been rerun through the full native planner
regression or validated on-road. Unit tests were updated for launch integral
hold, stopped-lead confirmation, rejection of creeping/unsettled cases, and
clean release, but have not been executed in this editing session.

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
