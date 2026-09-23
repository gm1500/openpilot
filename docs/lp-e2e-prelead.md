# lp-e2e-prelead: early distance-derived lead speed experiment

Built on `lp-e2e-kf` (`9c116b1e605b91bf5b5a531468bc3dfc706cb25b`),
which is based on `lp-e2e-blend` (`79762bcaf8555795e309e630329c7b313d92b48b`).
This replaces the independent, model-initialized lead-speed filters with private
distance histories that can begin before a lead is accepted for control.

## Data flow

1. `radard.get_lead` still decides whether to publish a lead. Its existing
   asymmetric probability hold and **held probability > 0.5** rule are unchanged.
2. Independently, each valid camera observation can start a private history at
   **raw probability >= 0.2**. This never creates a control lead.
3. A two-state Kalman filter estimates gap and absolute lead speed. It subtracts
   measured wheel-speed displacement between observations. Initialization uses
   wheel speed with a deliberately broad speed variance; model speed and model
   acceleration never initialize or update the Kalman state.
4. Once history and uncertainty qualify, distance-derived speed replaces
   `vLead`, `vLeadK`, and `vRel` on an already accepted vision lead. Raw distance,
   lateral position, confidence, presence, acceleration, and radar fields stay
   unchanged. Both `leadOne` and `leadTwo` pass through this stage before
   `RadarState` is published.

Established distance mode has **no permanent model-speed blend or +/-1.5 m/s
correction cap**. A mode change has a one-second finite offset handover. A
prewarmed track that has not published an accepted lead can supply speed on its
first accepted frame, provided it is already ready. Early observation cannot
guarantee that sufficient history exists at acquisition.

## Tracking and boundaries

| Item | Setting |
| --- | --- |
| Private observation range | 2 < gap < 200 m; 0 < distance standard deviation <= 50 m |
| Measurement variance | `max(distance_std, 1)^2 / raw_probability^2` |
| Initial speed variance | 1000 (m/s)^2 |
| Continuous acceleration noise density | 0.1 (m/s^2)^2 s |
| Minimum history | 1.5 s |
| Maximum internal speed variance for readiness | 2.25 (m/s)^2 |
| Distance speed publication | 15 < ego speed < 60 m/s; 10 < gap < 150 m; raw group confidence >= 0.5 |
| Same-camera hypothesis grouping | Longitudinal separation <= 1.5 m and lateral separation <= 0.35 m |
| Temporal association | <= 5 m motion-compensated distance difference and <= 0.75 m lateral difference |
| Time step / private loss allowance | 0.01 to 0.2 s |
| Innovation rejection | Squared residual > 16 times innovation variance |

The uncertainty bound is an internal filter criterion, not a calibrated guarantee
of true lead-speed error. The loose private observation bounds only permit
learning; they do not lower control acceptance.

Nearby matching hypotheses share one update and speed history. Their positions
are confidence/uncertainty weighted, but their uncertainties are not reduced as
if they were independent sensors. At most two histories are assigned across the
two slots, allowing slot swaps. Separation creates a fresh history for the new
object; merging removes redundant history. Association is a position heuristic,
not a persistent model object ID, so ambiguous or closely spaced vehicles remain
a limitation.

Short missing observations retain private history for up to 0.2 s without
publishing a stale lead. Invalid messages, stale timestamps, rejected motion, or
outliers discard the relevant history. Radar leads, low speed, and close range
continue to use the original estimates.

## Remaining model dependencies

The camera model still supplies distance, lateral position, confidence, and
distance uncertainty. Correct measurement timing and wheel-speed alignment
remain necessary. The model's ego/lead speed alignment no longer sets settled
distance-mode speed, but remains in the original fallback calculation.

The original model `aLeadK` and its downstream acceleration decay remain intact.
When model acceleration is below -0.5 m/s^2 or model-estimated closing time is
under five seconds, output immediately uses the lower of the candidate speed
and original speed. The distance filter continues learning. A recovery hold of
up to two seconds prevents immediate return to an optimistic, lagging distance
estimate when the braking indication ends; the normal handover then applies.
This is still model-dependent braking protection, not a distance-only emergency
braking estimator.

Hard rejection of an object history returns to the original estimate immediately.
That can produce a large speed-estimate step. Deliberately smoothing every new
object or braking-related drop could delay reaction to a slower car.

## Scope and verification

No longitudinal MPC policy, following-time setting, lateral path logic, steering
tune, gas/brake torque mapping, or actuator delay changes are included.
`opendbc_repo` stays at `f4bb1db17f0f38bbeab8f1cefd7492a8154ebd5c` and
`.gitmodules` stays unchanged. This inherits the `lp-e2e-blend` baseline, including
its 0.3 s longitudinal delay; it is not based on the delay05 or long-v1 branch.

35 focused tests cover private acquisition, constant and changing gaps, wheel
acceleration, independence from model-speed bias, correlated hypotheses, slot
swaps, cut-ins, confidence gaps, invalid input, radar bypass, covariance, actual
RadarD output wiring, and braking/recovery. The RadarD tests use real capnp
schemas; the local harness substitutes OS/IPC imports. Tests can also be run in
a built checkout with:

```bash
python -m unittest openpilot.selfdrive.controls.tests.test_vision_lead_tracker openpilot.selfdrive.controls.tests.test_radard_prelead
```

A synthetic lead slowing from 25 to 19 m/s at -3 m/s^2 verifies immediate
braking fallback and less than 0.6 m/s overestimation during recovery. That test
supplies accurate model braking signals: it validates the guard, not independent
distance-only braking performance.

Recorded replay results are reported below. These are fixed-input comparisons
using recorded vehicle motion, not a full native process replay, MPC solve,
vehicle simulation, or road validation.

## Recorded input replay

38 segments / 44,767 model frames, replaying both slots causally through the
actual RadarD implementation. Baselines are original model-derived speed and
the published `lp-e2e-kf` implementation. All other lead fields were checked
against the original on every frame. These routes contain no radar points.

Metrics select engaged following above 20 m/s with raw and held confidence
above 0.95, no pedals/blinkers/experimental mode, adjacent valid timestamps, and
no abrupt position discontinuity. Filter state still evolves through the entire
recording, including frames outside the reporting mask.

| Route / control recorded | Segments | Slot | Speed-step RMS: model / old KF / prelead (m/s) | Two-second gap-change MAE: model / old KF / prelead (m) |
| --- | ---: | ---: | --- | --- |
| `261--f0b430254e` / openpilot | 10-20 | 0 | 0.358 / 0.272 / **0.112** | 2.825 / 2.419 / **2.359** |
| Same | | 1 | 0.362 / 0.274 / **0.113** | 2.777 / 2.411 / **2.342** |
| `262--93c81f5e48` / stock ACC | 0-8 | 0 | 0.358 / 0.266 / **0.128** | 2.330 / **2.098** / 2.193 |
| Same | | 1 | 0.364 / 0.270 / **0.136** | 2.285 / **2.087** / 2.185 |
| `266--fa900d4a53` / openpilot | 1-18 | 0 | 0.370 / 0.294 / **0.188** | 3.188 / 2.693 / **2.602** |
| Same | | 1 | 0.373 / 0.297 / **0.189** | 3.163 / 2.704 / **2.601** |

Speed-step RMS improves approximately 59%, 49-52%, and 36% versus the old KF on
the three routes respectively. Gap-change consistency improves about 2-4% on
the openpilot routes, but **worsens 4.5-4.7% on the stock-ACC route** versus the
old KF. Smoother output is therefore not uniformly more accurate.

The prediction metric integrates current speed and the retained model
acceleration with its 0.3 decay over approximately two seconds, subtracts actual
recorded ego travel, then compares with the future camera gap. Complete windows
must satisfy the reporting mask. This checks consistency against the same
camera's later output, not independent true lead speed. It does not reveal what
the vehicle would have done under the new controller input. The recorded routes
used a 0.5 s configured delay; replay holds their motion fixed and does not test
the effect of changing to this branch's inherited 0.3 s tune.

Matching hypotheses have speed-disagreement RMS of **0.011 / 0.012 / 0.028 m/s**
on the three routes, versus **0.216 / 0.034 / 0.177 m/s** with the independent old
filters. Matching, distance-active outputs share the same speed exactly;
remaining disagreement comes from original fallbacks or unavailable tracks.

### Tail events remain

On route 266, the lead-0 99th-percentile absolute speed step improves from
1.069 to 0.795 m/s, but its **maximum step worsens from 2.546 to 4.093 m/s**.
The latter is a downward estimate jump on segment 1 at approximately 39.699 s
after its first model frame. A motion-compensated distance/identity rejection
creates an unready track and immediately returns to original model speed.

On route 261, segment 10 at approximately 16.199 s, the model braking guard
produces a 2.854 m/s downward speed-estimate step. Model acceleration crosses
the -0.5 m/s^2 guard while the distance estimate is higher. These are estimator
steps, not measured instantaneous changes in vehicle speed. Neither tail event
is resolved by the lower average volatility.

### Early acquisition remains conditional

Across both slots there are 72 presence-rising edges above 15 m/s (including
confidence flicker, not 72 unique cars). 56 already have some private distance
history, but **none are ready on the first accepted frame** in these recordings.
The synthetic low-confidence prewarm test demonstrates that immediate use is
possible with sufficient stable history. These logs demonstrate history
retention and smoother established tracking, not guaranteed immediate readiness.

This branch is an experiment with mixed replay results. Road comfort, braking
performance, lead identity, and closed-loop following stability remain unvalidated.
