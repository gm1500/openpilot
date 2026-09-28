# GM torque road-load estimate

`gm-torque` starts from `lead-confidence-poc` (`25d60004559ad68563daa5864b8484c5a585cbcf`).
It keeps the acceleration-to-torque controller and makes road-load parameters part
of each GM platform's fingerprint configuration. The Sierra/Silverado parameters
are an experimental ACC calibration, not a measured coastdown fit.

## Vehicle parameters

`opendbc/car/gm/values.py`, `GMCarSpecs`, defines:

| Parameter | Sierra/Silverado | Meaning |
| --- | ---: | --- |
| Base mass | 2,450 kg | The interface adds the existing 136 kg payload allowance. |
| Operating mass | 2,586 kg | Confirmed by the owner; unchanged. |
| Wheel radius | 0.425 m | Confirmed by the owner; unchanged. |
| `dragArea` | `0.30 * 3.97` = 1.191 m² | Drag coefficient times frontal area. |
| `rollingResistanceCoefficient` | 0.004 | Effective rolling feedforward; reduced from the initial 0.008. |
| Nominal air density | 1.225 kg/m³ | Fixed reference assumption, not a weather estimate. |

Drag area and the original rolling-resistance estimate came from the experimental Silverado values in
[opendbc PR #951](https://github.com/commaai/opendbc/pull/951#issuecomment-2409101515).
The current calibration halves rolling compensation after route-response checks
showed that the initial model underpredicted acceleration, especially at low
speeds. This is a conservative parameter adjustment, not a measurement of the
truck's tire coefficient. Mass, radius and drag area remain unchanged.
Other GM fingerprints retain their prior torque mapping: `dragArea=None` selects
the legacy 0.3 force multiplier, and rolling resistance defaults to zero. A future
vehicle tune can set its own parameters in its `GMCarSpecs` entry. Fingerprinting
selects a configuration; it does not measure payload, tires, wind or road load.

## Force balance and allocation

The moving controller uses:

```
torque = radius * (mass * accel_command
                  + mass * gravity * sin(pitch)
                  + 0.5 * air_density * drag_area * speed**2
                  + rolling_coefficient * mass * gravity * rolling_blend)
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
| 30 km/h | 8.9 Nm | 107.8 Nm | 64.7 Nm |
| 60 km/h | 35.4 Nm | 172.4 Nm | 129.2 Nm |
| 90 km/h | 79.7 Nm | 280.0 Nm | 236.9 Nm |
| 120 km/h | 141.7 Nm | 430.7 Nm | 387.6 Nm |

These are calculated requests, not measured delivered wheel torque. The refinement
removes 43.13 Nm of rolling feedforward at speeds of at least 1 m/s, equivalent to
0.03924 m/s². Below 1 m/s, the existing blend proportionally reduces that change
to zero at rest. The adjustment applies across speeds without another speed
table. It can reduce the negative feedback correction required when the previous
feedforward was excessive, but does not directly filter planner fluctuations.
Recorded integral corrections belong to the recorded tune; frozen-input replay
cannot predict how feedback or the vehicle trajectory will adapt.

While moving, friction braking begins at a more negative acceleration command
because the model includes more natural road-load deceleration. For example, at
60 km/h the unrounded zero-torque point is about -0.1176 m/s², versus -0.1568 for
initial gm-torque and -0.0322 for the legacy mapping. Thus the refinement can
request slightly more moving friction braking at the same frozen acceleration
input, while requesting less propulsion. Gas and brakes retain the shared force
calculation; no separate crossover or deadband is introduced.
Rounding and the existing one-unit brake hysteresis also affect the actual onset.
This is a material actuation change; replay cannot establish real stopping
distance or comfort. The late-braking 27c approach still needs particular attention
in subsequent vehicle validation.

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
