# Continuous single-lead closing-speed acquisition

A distant, continuously observed slower lead can disagree with the model's
speed estimate without being a new vehicle. The previous disagreement reset
could repeatedly discard its distance history when the closing time exceeded
the existing 13-second protection window.

This change qualifies observed motion separately from model speed. When the
accepted observations form one target group, it keeps up to 1.2 seconds of
position and ego-motion samples. At least 0.8 seconds and 12 samples are needed.
Yaw compensation removes the first-order apparent lateral motion caused by ego
rotation. An ego-motion-compensated range regression supplies a speed estimate
and an upper uncertainty bound. It uses observations across frames, not future
predictions as additional independent measurements.

Qualification requires probability at least 0.9, range uncertainty at most
20 m, ego speed between 15 and 50 m/s, absolute yaw rate at most 0.1 rad/s,
compensated lateral excursion at most 0.35 m, range residuals at most 4 m,
and estimated speed standard error at most 1.5 m/s. The speed margin is at least
0.75 m/s or twice the standard error, whichever is larger. Missing yaw,
observation gaps, discontinuities, or distinct target groups revoke the cue.
Two agreeing model slots remain one correlated observation, not two cars or
two independent confirmations. Triangle rendering is not an input.

Qualified continuity prevents the model-speed-disagreement reset. It does not
relax the existing per-frame association gates or transfer Kalman state across
a target switch. Separate targets retain the existing association, slot-swap,
handoff and recovery behavior. A genuinely ambiguous centered replacement can
still look like one continuous car; this is motion evidence, not visual identity.

When both the measured speed upper bound and the private distance state indicate
closing by more than 2 m/s, a bounded cue can lower the published lead speed
before the ordinary filter matures. The cue grows at most 2.5 m/s per second and
is capped at 6 m/s below model speed. It cannot increase speed, and is removed
when current evidence no longer supports it. The covariance/readiness criteria,
model braking guards, lead acceptance and planner settings are unchanged.
Existing filtered-range behavior can follow the changed published speed.

The existing `lead-pre` diagnostic adds `continuity` and `single_dv`. The latter
is the allowed speed correction, not necessarily an additional change in the
published estimate: another existing correction may already be stronger.
`continuous` is not a claim of a permanent vehicle ID, and `new_track` in the
confidence filter still means an immature estimate rather than a confirmed cut-in.

## Validation

The consolidation commit has exactly the tree of
`9ce60e229efd963e82534b02af71285778d1de65`. It retains the six original feature
commits and folds the road-edge follow-ups into feature commit seven. This
algorithm is the separate eighth feature commit on that base.

327 tests and 29 subtests passed, covering the road-edge suite and lead control,
transition, highway-closing and new continuity suites. New cases include one
accepted hypothesis, duplicate hypotheses, distinct targets, slot swaps, cut-ins,
cut-outs, lateral drift, yaw compensation, missing yaw, noisy steady following,
confidence loss, observation gaps and immediate model braking. An inactive cue
must not cap an established distance-derived speed to the model estimate.

Synthetic replay through the tracker, confidence filter and transition recovery:

| Case | Before | After |
| --- | --- | --- |
| 145 m initial gap, ego 108 km/h, lead 79.2 km/h, model 104.4 km/h | Six history resets in five seconds; estimate 104.4 km/h at 3 s | No resets; estimate 79.22 km/h at 3 s |
| Ego 120 km/h approaching 80 km/h, model 112 km/h, 12 m reported range uncertainty and small range jitter | Estimate first below 95 km/h at 3.75 s | First below 95 km/h at 2.65 s |

Three recorded segments supplied 3,598 model updates. Replaying the Python
tracker, confidence filter and recovery in their production order produced no
changed published lead speeds on these recordings. The earlier recorded highway
approach still first fell below 90 km/h at approximately 56.45 s; the additional
cue did not exceed its existing closing correction. Thus there is no demonstrated
additional real-drive pickup improvement from these recordings. Recorded ego
motion and acceptance/validity flags were used; this was not a native radard/MPC
or closed-loop vehicle replay. On-device timing and physical braking improvement
remain unvalidated.

Run the focused checks from the repository root with dependencies available:

```sh
python -m pytest -q openpilot/selfdrive/modeld/tests \
  openpilot/selfdrive/controls/tests/test_lead_continuity.py \
  openpilot/selfdrive/controls/tests/test_lead_control.py \
  openpilot/selfdrive/controls/tests/test_lead_transition.py \
  openpilot/selfdrive/controls/tests/test_highway_closing.py
```
