# Lane-change lead recovery trial

This fixture captures a vision-only highway lane change on batmobile
`6d2fd5868`. It contains only motion and lead estimates from seconds 28–37 of
the clip; location, absolute timestamps, identifiers and video are excluded.
The model speeds and distances are estimates, not independently measured truth.

Run from the repository root:

```sh
python -m unittest openpilot.selfdrive.controls.tests.test_lane_change_lead
```

The observer runs during an active lane change and up to two seconds afterward.
It keeps its own short acquisition history, compensates association for the
current model yaw estimate, rejects discontinuities and conflicting hypotheses,
and quarantines brief outliers without publishing an optimistic correction.
Both hypotheses must agree on the current target before range observations can
support a bounded upward speed correction. The existing distance tracker and
confidence filter continue to run unchanged.

The correction requires a sustained opening gap with a conservative regression
margin, at least 0.7 seconds of history, and another 0.15 seconds of confirmation.
It is capped at 2 m/s above the raw model, additionally capped by model speed
uncertainty, and ramps upward at 2 m/s per second. Braking cues, short gaps,
urgent closing, missing observations and ambiguous targets revoke it immediately.
Presence, distance, acceleration, cruise speed and planner limits are unchanged.
Diagnostics are appended to the existing `lead-pre` log as `lane_recovery` and
`dv` (the observer's correction allowance, which may exceed its actual additional
contribution if the ordinary distance tracker already supplies a higher speed).

For the supplied segment, a host replay of all 1,199 model updates through the
radard Python update logic changed 17 lead-speed samples, starting at 33.44 s
and ending at 35.74 s. The largest additional correction was 1.00 m/s (3.59 km/h).
The first ambiguous slowdown was unchanged. This is an acquisition/recovery
trial, not evidence that the original speed dip is eliminated. The replay used
host stand-ins for IPC infrastructure; no native MPC, closed-loop vehicle, or
on-device validation was performed.

Unit coverage includes noisy ranges, genuine target switches, gradual switches,
left/right yaw, false opening, braking, stopped/close leads, hypothesis disagreement,
time discontinuities, missing inputs, expiry, and the recorded acquisition.
The test branch is based on the road source-handoff fix `d651ab5e`, which remains
unchanged by this trial.
