# Distance-based vision lead-speed experiment

Base: `lp-e2e-blend` at `79762bcaf8555795e309e630329c7b313d92b48b`.
This is an estimator experiment, not a validated improvement in vehicle control.

## Purpose and scope

Recorded vision distance and vision relative speed can disagree for several
seconds. During two steady stock-ACC intervals, the gap stayed near 34 m while
openpilot's speed estimate placed the lead about 2.5 km/h below ego speed.

Each vision lead slot now has its own Kalman filter. The state is distance and
absolute lead speed. Wheel-derived ego speed supplies ego displacement between
model messages. After initialization, distance is the only measurement updating
the filter; model speed supplies the initial prior, fallback, discontinuity
checks, and a bound on the published correction. It is not a recurring Kalman
velocity measurement.

For sample interval `dt`, prediction is:

```
gap += lead_speed * dt - (previous_ego_speed + ego_speed) * dt / 2
lead_speed = lead_speed
```

The filter uses a constant-velocity transition, continuous white acceleration
process noise `Q = 0.1 * [[dt^3/3, dt^2/2], [dt^2/2, dt]]`, and distance
measurement variance `R = max(model_distance_std, 1 m)^2`. A Joseph covariance
update preserves symmetry and positive semidefiniteness under roundoff.
The process-noise setting favors steady following over rapid speed tracking.
Model uncertainty is a weighting heuristic; correlated vision errors mean the
filter covariance is not a calibrated guarantee of accuracy.

`vRel`, `vLead`, and `vLeadK` are updated together. The MPC actually consumes
`vLead`, so this affects planning inputs rather than just logging/display.
Measured distance remains immediate and unfiltered. Model acceleration and its
existing decay remain unchanged, isolating the experiment to lead speed.

## Activation and fallback

- Vision-only lead, raw probability at least 0.95, finite positive distance
  uncertainty no larger than 8 m, and valid input messages.
- Ego speed above 15 m/s (54 km/h), with full weight at 20 m/s (72 km/h).
- Distance above 10 m, with full weight at 20 m; upper distance bound 150 m.
- Two seconds of continuous history, followed by a one-second blend-in.
- Published speed differs from the original model estimate by at most 1.5 m/s
  (5.4 km/h) in either direction. A large disagreement is capped, not trusted
  without limit.
- Immediate original-model fallback for predicted lead deceleration stronger
  than 0.2 m/s², model closing speed over 2.5 m/s, low speed, short gap, lost
  confidence, invalid data, or radar-associated leads.
- Reset on stale/nonmonotonic timestamps, large distance/lateral/speed jumps,
  or a distance innovation exceeding four predicted standard deviations.
  Both slots reset when model leads disappear. One slot's reset does not clear
  the other. Vision has no reliable persistent object ID, so these checks cannot
  identify every possible lead switch.

The brake fallback preserves the baseline response when its condition fires;
it does not prove zero added delay in all braking situations. Range-only motion
estimation inherently trades noise against delay and can mistake visual range
drift for motion. This version intentionally falls back for stops and close
following; it is not a change to standstill stopping distance.

The following-time settings, MPC costs, longitudinal PID, GM torque mapping,
lateral policy, and safety limits come directly from the base. The opendbc
gitlink and `.gitmodules` are unchanged. In particular, Silverado/Sierra retains
the base's **0.3 s** actuator-delay setting and Ki 0.05; this branch does not
include the later 0.5 s or propulsion-blend experiments.

Telemetry is logged once per second per slot under `lead-kf:` with activation,
fallback reason, distance, published speed, speed correction, and history age.
The original `modelV2` outputs remain available for comparison.

## Checks performed

26 focused tests cover steady-gap convergence, ego-motion compensation, opening
gaps, noisy distance, bounded corrections, initialization, input preservation,
braking/low-speed/short-gap fallback, radar bypass, confidence loss, lead changes,
timestamps, finite positive covariance, and independent lead slots. Integration
tests execute the real `RadarD` and real Cap'n Proto schemas; the local runner
stubs unavailable OS/IPC services. Ruff and Python compilation pass. A full
native process/vehicle simulation was not run.

In an initialized openpilot environment:

```bash
python -m unittest \
  openpilot.selfdrive.controls.tests.test_vision_lead_kalman \
  openpilot.selfdrive.controls.tests.test_radard_vision_kalman
```

Causal replay examined 44,767 lead-one model frames from routes 261, 262, and
266. Metrics select confident highway tracking without pedal override, blinkers,
experimental mode, large range/lateral jumps, or timestamp gaps. The filter was
active on about 65–69% of eligible frames; fallback frames are included below.

| Route | Five-second median absolute speed/range disagreement, model → KF | Two-second range-prediction MAE, model → KF |
| --- | --- | --- |
| 261, earlier openpilot ACC | 0.947 → 0.389 m/s | 2.827 → 2.416 m |
| 262, factory ACC | 0.698 → 0.322 m/s | 2.355 → 2.131 m |
| 266, latest openpilot ACC | 0.983 → 0.496 m/s | 3.178 → 2.690 m |

The range-prediction check keeps the same model acceleration and decay and uses
recorded future ego travel to isolate the speed estimate's consistency with
subsequent vision distance. Future samples are used only for evaluation, never
for the filter. The two-second mean error improved about 9–15%; this is not a
closed-loop following-distance or safety result. Errors do not improve in every
sample: for example, stock ACC's one-second 90th-percentile absolute range error
rose slightly, from 3.25 to 3.31 m.

In the steady stock intervals, model relative speeds of -0.727 and -0.697 m/s
became -0.081 and -0.022 m/s; distance slopes were +0.086 and -0.044 m/s.
Frame-to-frame speed-change RMS fell about 21–26% across the three drives.
Both range and model speed are vision estimates, so these are consistency and
signal-variation checks, not independent lead-speed ground truth. Different
drives, controller feedback, correlated range noise, curves, and lead switches
limit what can be concluded about road behavior.
