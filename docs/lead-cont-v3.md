# lead-cont-v3: remove artificial fallback dips and bound early closing cues

Based on `lead-cont-v2` at `a0dd2631feafd5e7b13a9a94a71b6127d008fbe9`.
The priority is to remove braking triggers introduced by our estimator while
preserving the stock longitudinal policy. Changes are confined to
`vision_lead_tracker.py` and its tests.

## Phantom-braking cleanup

In v2, a cold shared track records the minimum model speed of its accepted
hypotheses internally. While that track is unready and in braking mode, radard
publishes the original per-slot model values. When the guard clears, the old
transition can blend from that unpublished minimum, artificially lowering the
primary lead speed even though distance mode was never active.

V3 explicitly records whether the custom output was published. Pure model
fallback after cold braking stays at each original model value. The existing
bounded transitions from established distance mode and uncertain handoffs remain.

Output reset is centralized. Missing observations and unsupported inputs clear
the range-filter activation and output transition. The raw-range reset now runs
before early fallback/braking returns. The first distance-mode sample after a
bypass re-anchors to the current raw range, preventing reuse of stale filtered
range. Established range filtering retains its 0.60 correction gain and updates
once per shared track per frame.

## Fallback → pre-closing → distance

Full distance mode retains the existing minimum 1.5 s history, speed variance
limit of 2.25 (m/s)², and all existing operating and association gates. Actual
readiness can take longer when distance uncertainty is high.

Before readiness, a limited downward speed cue can qualify only when:

- The existing radard lead is accepted, vision-based, and within the existing
  ego-speed and range operating limits.
- The same private track has at least 0.7 s history. Its latest 0.8 s window
  contains at least seven observations spanning at least 0.6 s, all with raw
  probability above 0.90. An observation gap above 0.1 s clears this window.
- The distance-derived speed is below the lowest accepted model speed, is
  nonnegative, and has variance at most 36 (m/s)².
- Both the raw-range regression and the immature Kalman estimate indicate
  closing time below 13 s after conservative margins: two regression standard
  errors and one Kalman speed standard deviation, respectively.
- No existing uncertain-handoff or fallback transition is still active.

The cue is at most 35% of the speed disagreement and never exceeds 3 m/s.
Its onset grows by at most 2 m/s per second. It releases when corroboration
disappears, and immediately shrinks if needed to stay within the current speed
disagreement. These statistical margins are engineering gates, not calibrated
probabilities for correlated camera observations.

Each accepted hypothesis retains its own model-speed baseline during
pre-closing. Sharing range history must not copy a lower secondary model speed
into the primary hypothesis. Range and acceleration stay raw in this phase.
Once ready, the existing distance-mode transition starts from the bounded cue.
Pre-closing cannot slow the response to a newly lower model speed.

## Preserved behavior

No changes to the stock planner/MPC, desired following distance, lead acceptance,
acceleration decay, gas/brake tuning, actuator delay, lateral control, CAN or
safety code. The opendbc revision remains
`621a736c526eabf6b6ae51d08029cd708e7413bc`.

Kalman prediction, update, covariance, association, grouping, and readiness are
unchanged. Cold/new/cut-in tracks start with fresh state. Private history cannot
create an accepted lead. Low-speed, close-range, radar, and invalid inputs keep
their original outputs.

The existing acceleration guard below -0.5 m/s², five-second closing guard,
eight-second uncertain-handoff guard, and bounded braking recovery remain
immediate and unchanged. The earlier proposal to debounce the ordinary TTC
guard was not implemented: this route has no TTC-only trips within the custom
estimator's supported operating range, so it does not support weakening that
response as a phantom-braking fix.

## Validation

All **72 tracker tests** pass, including 16 new v3 cases. They cover the early
cue's bounds, per-slot fallback speeds, confidence/history requirements, isolated
and alternating range noise, changing targets, cut-ins, missing leads, protected
paths, immediate guards, range re-anchoring, and single shared-range updates.
Ruff and Python compilation checks pass for the changed Python files.

```sh
python3 -m unittest \
  openpilot.selfdrive.controls.tests.test_vision_lead_tracker \
  openpilot.selfdrive.controls.tests.test_vision_lead_adaptive \
  openpilot.selfdrive.controls.tests.test_vision_lead_continuity \
  openpilot.selfdrive.controls.tests.test_vision_lead_v3
```

Recorded-input comparison uses all **7,200 frames** from segments 3–8 of
`615b11d4c01d81ec/00000272--f0d01ec577`. The logs identify the clean v2 baseline
above. Radar messages are paired to model messages by exact `mdMonoTime`;
recorded lead presence/probability and ego motion are retained. Ego speed is
reconstructed from published `vLead - vRel` when available; original model
velocity is restored using radard's model-ego correction. Replayed v2 matches
logged lead speed within 0.00000191 m/s and range within 0.00000381 m.

| Check | Result |
| --- | --- |
| Private state, covariance, age, and readiness | Exactly equal in 11,482 available slot checks |
| Presence, acceleration, confidence, radar identity, and other non-estimator fields | Unchanged in all 14,400 slot checks |
| Missing/radar/low-speed/out-of-range output | Original in all 6,950 protected checks |
| V2 braking-mode speed and range | Unchanged in all 621 slot checks |
| Segment 3, 24.884 s: artificial fallback speed deficit | 1.488 m/s (5.36 km/h) → 0 |
| Segment 7, 28.795 s: artificial fallback speed deficit | 1.166 m/s (4.20 km/h) → 0 |
| Segment 5: first pre-closing cue | 6.043 s, 0.600 s before full distance mode |
| Segment 5: full distance readiness | 6.643 s in both versions; track age 1.951 s |
| Segment 5: largest earlier speed reduction relative to v2 | 1.199 m/s (4.32 km/h), bounded handoff |

Times are relative to the first car/model/radar event in each supplied segment.
The segment-5 bookmark is at 17.163 s. Primary speed changes occur in 27, 22,
33, 0, 3, and 0 frames in segments 3 through 8 respectively. Range re-anchoring
and the changed handoff speed affect 223 primary-range outputs; the steady
range correction gain is unchanged.

This is an estimator replay with recorded ego motion, not a closed-loop MPC or
vehicle simulation. It establishes removal of the extra fallback speed dips
and earlier bounded closing information, not reduced brake pressure, improved
stopping distance, or elimination of all phantom braking. The radard integration
test could not import the unbuilt native `msgq` dependency in this environment.
No device build or road test was performed.
