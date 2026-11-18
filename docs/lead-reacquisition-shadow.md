# Uncertain lead reacquisition: shadow experiment

A range jump during a pass can break the existing 5 m association gate and
replace a mature distance-derived speed with a slower raw-model speed. Drive
`615b11d4c01d81ec/0000003e--d787e1bc63` shows this in segments 16, 40 and 50.
The shorter reported range can itself add braking demand; velocity alone is
not enough to explain or correct every event.

This commit adds **diagnostics only**. `LeadReacquisitionShadow.update` returns
no leads, has no enable parameter, and never changes production association,
Kalman state, probability, range, speed, acceleration, planner input or controls.
It runs after the existing tracker, confidence filter and transition recovery.
If it raises, radard disables the observer for the process lifetime and continues
publishing the existing lead output. CPU/logging overhead still needs on-device
validation.

## Candidate policy

- Collect up to 20 uncertainty samples from an accepted, mature distance track.
  Qualification needs at least eight samples spanning 0.45 s. Duplicate slots
  sharing one track count as one observation, not independent confirmation.
- Freeze the stable uncertainty history when the new standard deviation is
  greater than 1.25 times its median. Use the last stable baseline for at most
  one second; do not require an adjacent-frame doubling.
- When association breaks, retain a copied snapshot of the previous published
  range/speed as a private hypothesis. The new production track stays separate.
  The stored object reference is used only for identity, never state transfer.
- A candidate requires raw probability at least 0.9, range uncertainty at least
  10 m and twice the stable baseline, yaw-compensated lateral residual at most
  0.30 m, range residual at most 15 m and within the combined reported standard
  deviation, and a speed disagreement no greater than 5 m/s.
- Predict range and lateral position using the anchor speed and observed ego
  motion. Compute hypothetical range/speed differences against the published
  lead. Do not independently clip these into an inconsistent trajectory.
- Limit the diagnostic prediction to **0.20 s total**, with an immutable episode
  start. Repeated resets cannot renew it. There is no prediction for an absent
  or unaccepted lead. Ambiguity, observation gaps, invalid data or missing yaw
  clear the episode.
- A track already assigned to the other accepted hypothesis cannot be copied
  into a new candidate. Two distinct candidates cannot share one anchor.
- Revoke immediately for modeled braking below -0.15 m/s², a gap below the larger
  of 20 m and one second of ego travel, or time to collision below eight seconds
  using either raw or published lead speed. A replacement consistent across
  three frames also ends prediction if it is more than 3 m nearer or 2 m/s slower.
  Consistency is a conservative veto, not proof of object identity.

These are prototype screening thresholds, not validated settings for vehicle
control. The source exposes immutable diagnostic statuses, not corrected control
messages. Presence, emergency handling and normal following retain their owners.

## Logs

`lead-pre` adds `reacquire_shadow` and cumulative per-slot `shadow_frames` at the
existing one-second cadence. `lead-reacquire-shadow` records active episodes at
up to 10 Hz plus immediate start/end transitions:

- `reason`, `active`, `age`: decision and seconds since the fixed episode start;
- `dr`, `dv`: hypothetical minus published range (m) and speed (m/s);
- `std_ratio`: current versus last stable range standard deviation;
- `frames`: cumulative candidate frames, counted separately for each slot.

No negative probability or triangle-count heuristic is introduced. Blinkers and
lane-change state are not inputs. There is no planner or vehicle-control tuning.

## Validation

Focused tracker, confidence/control, transition, continuity, highway-closing and
new observer tests: **97 tests and 42 subtests passed**. New cases cover a frozen
baseline, the fixed deadline, duplicate hypotheses, single accepted hypotheses,
slot swaps, shared-anchor rejection, yaw, precise replacements, braking, short
TTC/gaps, invalid inputs, stale history, and identical production tracker output
and covariance with the observer running.

Recorded Python pipeline replay used original ego motion and accepted lead flags:

| Recording | Matched model frames | Shadow candidate frames in reviewed maneuver |
| --- | ---: | ---: |
| Drive 3e, segment 4 (clean pass) | 1,199 | 0 |
| Drive 3e, segment 16 | 1,199 | 0 |
| Drive 3e, segment 40 | 1,200 | 4 in slot 0 during the second change |
| Drive 3e, segment 41 (clean change) | 1,199 | 0 |
| Drive 3e, segment 50 | 1,199 | 0 |
| Drive 4b5, segment 53 (closing approach) | 1,200 | 0 |

Across **7,196 model updates**, the observer left production lead dictionaries,
Kalman states, covariances, ages, filtered ranges and tracker outputs unchanged.
Segment 40's candidate lasted through 0.150 s and expired at the 0.20 s deadline.
Segments 16 and 50 do not demonstrate relief: the conservative matching/closing
checks prevent a candidate in 16; 50 also encounters an anchor still assigned to
the other slot and later a short-gap veto. The observer deliberately does not
relax those checks to fit these recordings.

Host-only observer update p95 was below 0.1 ms in each recording, excluding log
serialization. This is not on-device timing, a native radard/MPC replay, or a
closed-loop vehicle test. No reduction in braking or unchanged hazard-response
latency for an enabled correction is claimed. The next step is to inspect these
signals and validate a candidate through native planner replay before enabling
any control correction.

```sh
python -m pytest -q openpilot/selfdrive/controls/tests/test_lead_reacquisition.py \
  openpilot/selfdrive/controls/tests/test_lead_control.py \
  openpilot/selfdrive/controls/tests/test_lead_transition.py \
  openpilot/selfdrive/controls/tests/test_lead_continuity.py \
  openpilot/selfdrive/controls/tests/test_highway_closing.py
```
