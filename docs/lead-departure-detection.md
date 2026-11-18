# Lead departure detection POC

This branch starts from the seven-feature `batmobile` consolidation at
`cebc3f0b11f90fa4d2f5113e3df44762a56ff5df`. It adds an observational detector
for a previously coherent pair of model lead hypotheses separating during an
ego lane change. It does not change accepted lead data, following distance,
planner costs, acceleration, friction-brake commands or E2E mode selection.

The investigation found that the primary lead's unexpectedly short range and
low speed were already present in the raw model output at the four braking
events. Removing inherited speed corrections did not remove those pulses.
A range discontinuity or two diverging triangles does not establish that a
vehicle is false: a real closer cut-in can produce the same signals.

## Runtime observation

`LeadDepartureShadow` acquires a coherent pair for at least 0.5 seconds before
the maneuver. It retains a fixed pre-maneuver hypothesis, transports it with
measured ego speed and yaw, and reports the range residual. The observation
expires after 2.5 seconds; a split cannot renew it or acquire a new anchor
mid-maneuver. A warning additionally requires a 4 m range separation,
1.4 m/s speed separation, 0.55 m directional lateral separation, measured yaw
in the lane-change direction, and 0.15 seconds of persistence.

These thresholds were developed on the four discovery events. They are not
an independently validated false-lead classifier. Output is deliberately
`split_ambiguous`; `can_ignore_lead` is always false. Neither head is removed,
renamed as a different physical vehicle, or assigned the other's history.
The constant target-motion assumption used for the residual is diagnostic.

Radard copies only scalar observations into the detector. Measured yaw comes
directly from `deviceMotion.angularVelocityDevice`, rotated into calibrated
FRD with `rot_from_euler(rpyCalib).T`, matching `PoseCalibrator`. A freshly
published `carControl` message can contain cached yaw, so it is not used as
the motion source. Separate four-sample buffers select causal measurements.
The motion's own filter timestamp must be at or before model time and no older
than 100 ms. An already received motion message may have been published up to
100 ms after model time; this does not make an earlier sensor measurement a
future observation. Ego samples must be published at or before model time and
no older than 100 ms. Valid calibrated extrinsics must already
be available at the motion measurement time and be no older than two seconds.
Motion input, sensor, visual-odometry and measurement validity are required.
The optional motion/calibration subscriptions are excluded from radar health
checks. Missing, invalid, stale or discontinuous inputs revoke the observation;
unexpected exceptions disable only this observer. Logs use the
`lead-departure-shadow` prefix when the reason changes and periodically while
active, rate limited to 5 Hz. No camera process or OpenCV dependency is added
to driving runtime.

## Regression cases

The committed scalar fixture contains 2,073 observations across eleven
windows. It preserves 20 Hz timing, measurement offsets, availability delay,
source hashes and recorded code revisions, while excluding camera images,
GPS, route identifiers and original timestamps.

This scalar fixture is derived from all received source messages. It tests the
observer's logic and is distinct from replaying SubMaster's latest-message
sampling and its four-sample buffers.

| Cases | Expected diagnostic behavior |
| --- | --- |
| Four ego departures around a slower lead | Report an ambiguous split before the recorded friction pulse |
| Slower lead exits and reveals another lead | No ego-departure warning |
| Slower lead exits and disappears | No ego-departure warning |
| Clear-lane right return | No warning |
| Steady slower following and no-departure bookmark | No warning |
| Real semi cut-in before driver intervention | No ego-departure warning; preserve lead output |
| Approaching a much slower lead before driver intervention | No warning; preserve lead output |

The semi and older slower-lead recordings use older software; their replay is
a counterfactual input test, not evidence that this code ran on those drives.
The actual semi cut-in occurs without an ego lane change. Synthetic true
cut-ins during ego lane changes are therefore also required: they can trigger
the same ambiguous warning, and must never authorize discarding a lead.

Tests also cover mirrored turns and head order, braking/stationary/closer
targets, aborted/reversed maneuvers, insufficient acquisition, missing heads,
nonfinite values, duplicate/backwards/gapped time, bounded expiry, input
immutability, causal prefix invariance, postdated measured messages, fresh
envelopes with stale motion data, missing or invalid calibration, observer
failure and unchanged serialized RadarD output and validity.

## Reproduction

In the normal built openpilot development environment:

```sh
python -m unittest \
  openpilot.selfdrive.controls.tests.test_lead_departure_shadow \
  openpilot.selfdrive.controls.tests.test_lead_departure_replay \
  openpilot.selfdrive.controls.tests.test_radard_lead_motion
```

The optional offline camera tool and its separate tests are described in
[lead-departure-camera.md](lead-departure-camera.md). It requires manual
pre-split target acquisition and optional OpenCV installed outside driving
dependencies. Image continuity is additional evidence, not proof that a lane
is clear or that an independently appearing nearer object can be ignored.

The recorded ego trajectory is held fixed in planner replay. Identical lead
inputs should give identical planner output; this is a noninterference check,
not a claim of reduced real-world braking. Road validation and automatic
physical-target association remain outside the demonstrated results.

## Validation on November 18, 2026

- 237 unittest cases passed, including the new detector, serialized RadarD
  integration, 22 optional camera tests, existing lead/future/responsive motion,
  highway closing and signal activation. One optional private approach fixture
  was unavailable and skipped.
- 259 road-edge/paint-memory/width-confidence/inner-line/width-refinement pytest
  cases passed. Total: 496 passed, one skipped. Ruff and whitespace checks pass.
- The host used real schemas, serialization, estimators and RadarD code, with
  adapters for unavailable IPC, logging and persistent-parameter imports.
  Native IPC, full device builds and road behavior were not validated.
- The full-source scalar fixture detects all four ambiguous splits; the seven
  comparison windows remain quiet. Synthetic real cut-ins can still trigger
  the warning, as expected for an ambiguity detector.
- A separate replay reconstructing latest inputs available by the logged radar
  publication catches **three of four** departure cases. It compares the actual
  RadarD caller with the observer enabled and disabled over all 2,073 frames.
  Accepted leads are identical, paired native-MPC acceleration differences are
  exactly zero, and both arms have zero solver failures. Actual SubMaster dequeue
  times are not logged, so this is a causal scheduling approximation.

Case D is deliberately not counted as a runtime detection: immediately before
lane-change entry, the newest received motion measurement is 5.18 ms after the
model timestamp, while the prior buffered measurement is 101.89 ms old. The
100 ms causal freshness guard revokes the anchor. The complete source log has
an intervening sample that the reconstructed latest-message subscription
misses; feeding that sample directly explains the scalar fixture's success.
The guard was not loosened to make this known example pass.

The three runtime warnings precede the recorded friction pulses by approximately
0.45, 0.43 and 0.48 seconds. In case C the warning is already about 2 ms after
the planner target first falls below -0.3 m/s². This reinforces the need for
physical-feature acquisition before a split, rather than starting at a warning.
Manual camera tracking supports further association work in all four examples,
but the dark semi camera trial fails to retain a usable track. These results
do not establish a safe false-lead rejection policy or a reduction in braking.
