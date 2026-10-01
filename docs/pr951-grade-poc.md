# PR 951 grade compensation POC

## Withdrawn after the stopped-lead incident

The pitch-compensation experiment has been reverted. The current branch pins
OpenDBC `e0a5a21aadd48195ca8d3afc9b776e4177b27d3c`; its source tree is identical to
the `boat-anchor` dependency at `6151ef3de18e0813ede7a83b9c5a3dc91edccc71`.
Planner, lead estimation, Ki, lane centering and nudgeless retain the base behavior.

Replay of the reported incident exactly reproduced 2,795 analyzed gas/brake
transmission pairs. Positive measured pitch reduced approach braking by up to
31 command units (approximately 0.31 m/s² in the lookup). Removing that input
restores the prior brake mapping; it does not prove a different stopping outcome.

A separate cruise fault appeared in the vehicle's raw status while stationary
and was only surfaced by the existing car-state logic after movement began.
Grade compensation was zero in that interval and both controller versions
produce identical outputs there. That standstill fault remains unresolved.
The earlier offline plant did not model ECU acceptance or validate on-road stops.
This branch is withdrawn as a driving experiment, not promoted as a verified fix.

## Historical experiment and results

The original experimental commit added the missing direct grade input to the Sierra /
Silverado torque conversion. It is based on `boat-anchor` at
`b1fc9dbabad49c38e0aef012e21e8995316f5380` and pins OpenDBC
`17dfd16fd469e6b4995dd50ecd0a5540d25f55d9`, based on
`6151ef3de18e0813ede7a83b9c5a3dc91edccc71`.

Stock planner policy, lead estimation, lane centering, nudgeless, Ki and the
0.5 s actuator-delay setting are retained. No experimental test files are
included in the vehicle branch.

## Implementation

Mass, wheel radius, aerodynamic drag, rolling resistance and CAN torque units
were already implemented. The old live torque conversion supplied zero pitch.
The POC enables the existing `mass * gravity * sin(pitch)` term using calibrated
vehicle-frame pitch from `carControl.orientationNED`.

The pitch-derived acceleration is bounded to ±0.75 m/s² and filtered with a
1 s time constant at 100 Hz. Its contribution fades in between 5 and 8 m/s
(18–28.8 km/h). Missing/nonfinite pitch, inactive control, stopping and speeds
at or below 5 m/s retain the previous conversion. Existing brake-entry
suppression, stopping calibration, torque/brake limits and CAN protocol
requests are preserved. Gas and friction braking use the same force balance.
Only the Sierra / Silverado fingerprint enables this input.

The bound is an acceleration limit, not the previously reverted 0.75 road-load
multiplier. Vehicle parameters remain 2,586 kg including standard cargo,
0.419 m nominal radius, `Cd * area = 0.30 * 3.61 m²`, rolling coefficient 0.004,
and nominal air density 1.225 kg/m³. The PR's example area/rolling values were
tested separately and are not substituted for this vehicle's profile.

## Offline results

All 18 supplied logs from 28b segments 3–19 and 28c segment 5 were read.
Seventeen sustained active windows provide 820.85 s of scored motion after a
5 s warmup per window. Simulated speed and acceleration feed back into the
unchanged production planner and longitudinal controller. Published leads
remain in world coordinates and their gaps change with simulated ego position.
Recorded model throttle cues remain exogenous; cameras/model inference are
not rerun.

The nominal surrogate treats filtered recorded pitch as road grade, with the
existing drag/rolling profile, a 0.4 s force delay, 0.1 s response time constant
and 0.1 s acceleration measurement filter. This is not a vehicle-validated
drivetrain model or surveyed road-grade replay.

| Nominal metric | boat-anchor | Grade POC |
| --- | ---: | ---: |
| Friction-brake entries | 31 | 22 |
| Completed brake bouts ≤0.8 s | 10 | 2 |
| Brake-active time | 86.46 s | 85.94 s |
| Fast brake-command fluctuation RMS | 1.059 units | 1.005 units |
| Acceleration jerk RMS | 0.192 m/s³ | 0.207 m/s³ |
| Native solver failures | 0 | 0 |

Fewer brief on/off bouts do not establish an 80% improvement in overall brake
pulsation or comfort. Total brake time is almost unchanged; acceleration jerk
increases approximately 7.8%. Some simulated following gaps become smaller.
The lower-speed following window adds a short bout, and the diagnostic 28c
window increases brake-active time from 7.80 to 11.04 s.

Twenty-six controlled scenarios cover 40, 75 and 100 km/h, flat/uphill/downhill
terrain, pitch noise and changing-grade cruise/following. Correct grade
feedforward improves idealized speed holding, while necessary downhill braking
remains. Noisy pitch adds modulation during continuous braking. A 3 s filter
probe softens this ripple but does not consistently improve recorded-window
brake entries; this POC retains 1 s filtering.

Seven plant sensitivities vary neutral offset, torque response, brake response
and delay in three diagnostic windows. Fewer brief bouts coexist with following
and brake-duration regressions. No fitted neutral offset is installed.

## Validation and limitations

The published implementation was restored from the saved POC evidence after a
temporary-workspace reset. Its controller/simulator equivalence checks were
rerun: 12,000 varied cycles match actuator outputs and CAN packets exactly.
Transmitting cases for inactive control, stopping, low speed and missing/
nonfinite pitch match the prior controller. Another 2,040 cycles across the
other 17 GM fingerprints produce identical outputs and CAN packets. Other
vehicle controllers, parameters, schemas and safety code remain unchanged.

Calibrated vehicle-frame pitch still includes body attitude and may have
neutral bias. A 0.5° pitch error contributes approximately 93 Nm. Arbitrary
stationary pitch is not automatically learned as level ground. Stock model
throttle-permission fluctuations remain. This is an isolated experimental POC;
the simulations do not establish a general braking fix or validated road
performance. Vehicle boot and driving are not tested here.

Sources:

- [PR 951 force formula](https://github.com/commaai/opendbc/pull/951#issuecomment-2409101515)
- [PR 951 grade follow-up](https://github.com/commaai/opendbc/pull/951#issuecomment-2409257448)
