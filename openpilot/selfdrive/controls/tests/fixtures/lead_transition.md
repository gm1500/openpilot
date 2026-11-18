# Lead-transition recovery trial

This fixture captures a vision-only highway target handoff on batmobile
`6d2fd5868`, from seconds 28–37 of the supplied clip. It contains only motion and
lead estimates. Lane-change metadata, location, absolute timestamps, identifiers
and video are excluded. The speeds and distances are estimates, not independently
measured truth.

Run from the repository root:

```sh
python -m unittest openpilot.selfdrive.controls.tests.test_lead_transition
```

The observer watches lead observations continuously. It does not consume lane
change state, turn signals, or a maneuver timer. Initial acquisition, a detected
range/lateral target change, or reacquisition after loss starts a three-second
recovery period. A settled track continues to be monitored without automatically
rearming when that period expires. A normal expiry fades out its correction;
new targets, braking and other rejection conditions revoke it immediately.

The observer keeps its own short history, compensates association using the
current model yaw estimate, and requires both current hypotheses to agree on the
target. Large discontinuities start a new history immediately. Intermediate
range jumps revoke output while briefly retaining private history: one noisy
observation can recover, while a persistent displacement starts a fresh track.
No previous lead's velocity correction is intentionally bridged into a new one.
The ordinary distance tracker and confidence filter continue to run unchanged.

An upward correction needs at least 0.7 seconds of history, a conservative
regression margin showing an opening gap, and 0.15 seconds of confirmation.
It is capped at 2 m/s above raw model speed, further limited by model speed
uncertainty, and ramps upward at 2 m/s per second. Normal handover also releases
at 2 m/s per second. Existing eligibility checks remain: vision-only leads,
ego speed between 15 and 50 m/s, high confidence, and sufficient headway and
closing-time margins. Close, stationary, braking or urgently closing leads
retain the ordinary controller's immediate response. Presence, distance,
acceleration, cruise speed and planner limits are unchanged.

Diagnostics appear in the existing `lead-pre` log as `lead_transition` and `dv`.
The latter is the observer's correction allowance; its actual contribution may
be smaller when the ordinary tracker already supplies a higher lead speed.

Validation: 28 unit tests and 21 pytest subtests cover straight-driving cut-ins,
cut-outs, acquisition, small and large jumps, missing detections, noisy ranges,
left/right yaw, braking, close/stopped leads, hypothesis disagreement, expiry,
normal handover, and the recorded sequence. The small cut-in regression verifies
that a seven-metre target change cannot retain the previous opening correction.

A host replay of 3,599 model updates across three available segments exercised
the actual radard Python update logic and Capnp output. The new code ran with
lane metadata inaccessible. Relative to batmobile without the trial:

| Segment | Updates with a correction | Largest additional speed correction |
| --- | ---: | ---: |
| Supplied lane-change clip | 14 | 2.52 km/h |
| Mainline/map-handoff segment | 2 | 1.17 km/h |
| SPI investigation segment | 0 | 0 |

All corrected samples had lane-change state `off`; invalid samples received no
correction. In the supplied clip the first correction was at 35.29 seconds.
The initial ambiguous slowdown remains unchanged; this is not evidence that its
speed dip is eliminated. Replay used host stand-ins for IPC infrastructure, with
recorded validity flags. Native MPC, closed-loop vehicle and on-device behavior
have not been validated.

The test branch retains road source-handoff patch `d651ab5e` unchanged.
