# Future lead position estimator POC

Branch: `batmobile-future-lead-prediction-poc`.
The branch now also includes the guarded [responsive lead stage](lead-responsive-poc.md)
and derived-speed E2E assist gate. The original bounded stage described below remains.
Built on `39e6e244945d0f1198f70db8c8f3a5772a7fbb23`, retaining the
learned lane-width retention changes and all preceding batmobile fixes.

The model already publishes lead positions at 0, 2, 4, 6, 8 and 10 seconds.
This POC fits a quadratic to the first three positions to estimate current
lead velocity and acceleration. These positions use the stationary ego-start
frame: differentiating them gives lead speed, without adding ego speed.
The constant camera-to-radar position offset cancels in the derivative.

`LeadFutureMotion` runs after the existing range tracker, confidence filter,
and transition recovery in radard. It adds a limited velocity correction when
the observed range trend supports the same direction. The prior update's
predicted short forward travel, minus measured ego travel, can also reduce
small fluctuations in the current published range. Actual current observations
remain the anchor. Future positions are never inserted directly as current dRel.

This is an active estimator experiment on this branch; no toggle is required.
It does not replace the MPC's acceleration-decay trajectory or modify lead
acceleration, lead acceptance, track association, or Kalman state/covariance.
The separate lead-reacquisition observer remains read-only.

## Eligibility and bounds

| Item | POC behavior |
| --- | --- |
| Supported motion | Existing continuous single-target qualification, ego 15–50 m/s (54–180 km/h), current gap 10–150 m |
| Initial qualification | Existing continuity history of at least 0.8 s, then 0.20 s of consistent predictions |
| Future positions | Finite, nondecreasing 0/2/4 s positions, probability at least 0.9, plausible fitted speed and acceleration |
| Uncertainty | Conservative sum of absolute derivative-weighted position uncertainties; no independence assumption across horizons or slots |
| Velocity blend | At most 35%, reduced by predicted speed uncertainty |
| Velocity correction | At most -2 / +1 m/s (-7.2 / +3.6 km/h) relative to the existing estimator |
| Correction entry rate | At most -2 / +1 m/s per second; corrections are revoked immediately when unsupported |
| Range correction | At most 0.30 m relative to the existing estimator, only for a mature distance track |
| Closing or mild braking | Do not increase published distance; mild braking/pre-closing cannot receive an upward speed correction |
| Strong braking, short TTC, stopped/low-speed lead | Existing output passes through unchanged |
| Gaps, cut-ins, ambiguous motion, in-place track resets | Discard prediction correction and require fresh qualification |
| Duplicate lead hypotheses | One shared track correction without increased confidence |
| Slot swap | State follows the existing vehicle track, not the slot number |

The existing continuity qualifier intentionally excludes distinct simultaneous
targets and substantial lateral motion. This first POC therefore does not claim
to improve every cut-in or supply a new vehicle-identity detector. Predicted
position and direct model velocity share the same perception model; neither
is independent confirmation of the other. Thresholds are initial POC choices,
not calibration established from recorded future-position errors.

## Diagnostics

The existing `lead-pre` log line adds `future_motion` (qualification reason),
`future_v`, `future_std`, `future_w`, `future_dv`, and `future_dr`.
Corrections are relative to the previous batmobile estimator output. The
original model trajectory remains available in `modelV2.leadsV3` for replay.
Logs use the existing once-per-second diagnostic cadence.

## Validation and limits

New tests cover serialized model messages, coordinate frames, decelerating trajectories, uncertainty,
correlated/contradictory slots, speed/range bounds, constant-gap behavior,
range jitter, cut-ins, gradual lateral motion, slot swaps, stopped leads,
braking, invalid inputs, dropped observations, and tracker resets.
The synthetic closing-approach integration test uses the actual range tracker,
confidence filter, and transition recovery before applying the POC.
All 121 selected longitudinal tests and all 259 road-edge tests pass. Ruff and
Python compilation checks pass. The lane/road-edge files match the base commit.

In the deterministic synthetic example (ego 33 m/s, actual lead 22 m/s,
direct model velocity biased to 32 m/s), the correction begins at 1.05 s.
It reduces early speed error; this is a constructed scenario, not evidence
of an improvement on a recorded drive or closed-loop driving validation.
Future-position accuracy, activation rate, and regression behavior still need
comparison on the user's modelV2/radarState recordings before widening the
gates or trying full future lead trajectories in the MPC.

Run the longitudinal regressions with:

```sh
PYTHONPATH=opendbc_repo python -m unittest openpilot.selfdrive.controls.tests.test_lead_future_motion openpilot.selfdrive.controls.tests.test_lead_control openpilot.selfdrive.controls.tests.test_lead_continuity openpilot.selfdrive.controls.tests.test_lead_transition openpilot.selfdrive.controls.tests.test_lead_reacquisition
```
