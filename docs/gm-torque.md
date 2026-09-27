# GM torque road-load estimate

`gm-torque` starts from `lead-confidence-poc` (`25d60004559ad68563daa5864b8484c5a585cbcf`).
It keeps the acceleration-to-torque controller and makes road-load parameters part
of each GM platform's fingerprint configuration. The initial Sierra/Silverado
parameters are an experimental calibration, not a measured coastdown fit.

## Vehicle parameters

`opendbc/car/gm/values.py`, `GMCarSpecs`, defines:

| Parameter | Sierra/Silverado | Meaning |
| --- | ---: | --- |
| Base mass | 2,450 kg | The interface adds the existing 136 kg payload allowance. |
| Operating mass | 2,586 kg | Confirmed by the owner; unchanged. |
| Wheel radius | 0.425 m | Confirmed by the owner; unchanged. |
| `dragArea` | `0.30 * 3.97` = 1.191 m² | Drag coefficient times frontal area. |
| `rollingResistanceCoefficient` | 0.008 | Rolling force divided by vehicle weight. |
| Nominal air density | 1.225 kg/m³ | Fixed reference assumption, not a weather estimate. |

Drag area and rolling resistance start from the experimental Silverado values in
[opendbc PR #951](https://github.com/commaai/opendbc/pull/951#issuecomment-2409101515).
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

| Speed | Previous torque | Revised torque |
| --- | ---: | ---: |
| 0 km/h | 0.0 Nm | 0.0 Nm |
| 30 km/h | 8.9 Nm | 107.8 Nm |
| 60 km/h | 35.4 Nm | 172.4 Nm |
| 90 km/h | 79.7 Nm | 280.0 Nm |
| 120 km/h | 141.7 Nm | 430.7 Nm |

These are calculated requests, not measured delivered wheel torque. The change
provides more holding-load feedforward and can reduce the integral correction
needed to hold speed if the road-load assumptions are accurate. It does not
attenuate fast fluctuations in the planner acceleration input. Recorded integral
corrections belong to the previous tune, so fixed-input replay overstates how much
additional torque would remain after feedback adapts.

While moving, friction braking begins at a more negative acceleration command
because the model includes more natural road-load deceleration. For example, at
60 km/h the unrounded zero-torque point moves from about -0.0322 to -0.1568 m/s².
Rounding and the existing one-unit brake hysteresis also affect the actual onset.
This is a material actuation change; replay cannot establish real stopping
distance or comfort. The late-braking 27c approach still needs particular attention
in subsequent vehicle validation.

## Scope and validation

Stock planner targets, following distances, lead filtering, cruise policy, PID
gains, actuator delay, stop-state logic, CAN encoding and safety limits are
unchanged. The inherited confidence and lane-control behavior stays as it was on
the parent branch. There is no new temporal filter or acceleration gain reduction.

Validation passed 19 GM tests and 7,776 platform/configuration comparisons, with
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

Before treating this as a validated tune, compare delivered acceleration and
steady integral correction over several speeds, plus lead-braking and final-stop
behavior. If bias varies approximately with speed squared, adjust drag area; if
the bias is approximately constant above the low-speed blend, investigate rolling
resistance and other steady loads. Grade and wind can mimic both effects.
