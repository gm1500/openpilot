# lead-confidence-poc

Experimental branch based on `lead-cont-v3` commit
`a18e3663b8f3868ed7d165ef1453bc325c2ca4d6`. It integrates the bounded
confidence-adaptive filter evaluated offline on 28 uploaded segments from five
routes. The filter is enabled on this branch. It has not demonstrated a
meaningful reduction in planner fluctuations or phantom braking.

The branch now also includes a Silverado/Sierra actuator follow-up from route
27c. See [lead-confidence-stop.md](lead-confidence-stop.md) for the low-speed
feedback tune, one-unit brake-onset hysteresis, validation and limitations.
The sections below describe the original confidence-filter integration at
`102ac92`; its lead estimator and planner policy remain unchanged.

## Implementation

`VisionLeadConfidenceFilter` runs in radard after the unchanged V3 tracker and
before publishing the leads. It alters only `dRel` and mild `aLeadK` values.
The offline fixed-gain and range-only comparison modes are not runtime options.

The causal range filter predicts from its previous output using V3 relative
speed, then corrects toward the current V3 range. Its nominal 20 Hz gain is
`clip(1 / (1 + (max(xStd, 1) / probability / 5)^2), 0.30, 1)`.
The lead acceleration filter uses
`clip(1 / (1 + (aStd / 0.35)^2), 0.25, 1)`.
Both gains are adjusted to elapsed time. Greater uncertainty causes stronger
smoothing. `vStd` is checked for validity but does not weight V3's
distance-derived speed.

Additional changes are limited to ±0.75 m range and ±0.08 m/s² acceleration
relative to the current V3 outputs. Activation requires a mature track (at
least 2 s), V3 readiness, completed distance-mode transition, no uncertain
handoff, and the existing supported ego speed/range limits. Filtering bypasses
immediately for:

- Acceleration below −0.15 m/s² in either accepted hypothesis of a shared track.
- Headway below 1.5 s or conservative closing time below 12 s, using raw, V3
  and Kalman speed with a one-standard-deviation margin.
- New/reassigned tracks, fallback, pre-closing, braking and handoffs.
- Invalid/missing observations or uncertainty, radar leads, missing leads,
  abrupt raw range changes above 3 m or acceleration changes above 0.20 m/s².

Bypasses clear filter history. The first supported sample re-anchors to V3,
as does a new track or an update gap outside 0.01–0.10 s. Missing model arrays
and invalid messages reset both slots. A missing individual lead does not
discard the other slot. Raw lead dictionaries are preserved for braking checks;
filtered output is never fed back into tracker state or acceptance.

Existing 1 Hz `lead-pre` logs now include `confidence` (active or bypass reason),
range/acceleration `gains`, `conf_dr` and `conf_da` offsets. These diagnostics
help identify when the experiment contributes to an observed event.

## Preserved policy

No changes to V3 association, Kalman state, lead speed, fallback/pre-closing,
acceptance, confidence or acceleration decay. Stock planner/MPC source, costs,
following policy, actuator limits/tuning, lateral behavior, CAN/safety code and
opendbc revision are unchanged. The published `lead-cont-v3` branch is unchanged.

## Evidence and limitations

The integrated implementation matches the offline adaptive POC exactly in all
33,540 recorded frames (67,080 lead-slot comparisons, maximum difference 0).
The replay comparison used route 272 for development and routes 26c, 26f, 270
and 271 for validation. On validation data:

| Measure | Change from V3 |
| --- | ---: |
| Eligible stable-lead range fluctuation RMS | 6.69% lower |
| Eligible stable-lead acceleration fluctuation RMS | 23.81% lower |
| Lead-controlled planner target fluctuation RMS | 0.019% higher |
| Lead-controlled target jerk RMS | 0.020% lower |

These results are effectively unchanged at planner output. Fluctuations were
measured relative to a centered one-second linear trend for analysis only;
the filter itself uses no future samples. The replay retains recorded ego
motion, so it does not validate actuator response or closed-loop road behavior.
The route-272 segment-5 fast-closing event crosses the −0.5, −1 and −2 m/s²
braking thresholds on the same samples with V3 and the POC at 20 Hz. This single
event does not establish general responsiveness or safety.

Validation for this integration:

- 94 tracker/filter unit tests pass (72 existing V3 tests and 22 confidence tests).
- 13 radard integration tests pass using real Cap'n Proto messages and radard
  calculations with offline import adapters for unavailable native messaging.
  The ordinary radard test import is blocked here by unbuilt `msgq`.
- Ruff, Python compilation and whitespace checks pass for changed Python files.
- No native device build or road test was performed.

On a fully built checkout, run:

```sh
python3 -m unittest \
  openpilot.selfdrive.controls.tests.test_vision_lead_tracker \
  openpilot.selfdrive.controls.tests.test_vision_lead_adaptive \
  openpilot.selfdrive.controls.tests.test_vision_lead_continuity \
  openpilot.selfdrive.controls.tests.test_vision_lead_v3 \
  openpilot.selfdrive.controls.tests.test_vision_lead_confidence \
  openpilot.selfdrive.controls.tests.test_radard_prelead
```

This branch exists for the requested confidence experiment. The replay result
does not justify calling it a phantom-braking fix or increasing smoothing.
