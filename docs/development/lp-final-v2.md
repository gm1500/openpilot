# lp-final-v2: bounded approach damping

Base: `lp-final` commit `8535a3fa027e5362c4b1fd59604343f8d087a744`.

This candidate adds a small, stateless prediction to the lane-offset correction.
When fitted lane offset and heading have opposite signs, speed times heading
approximates the rate at which the offset is closing. A 0.20-second prediction
reduces the offset contribution by at most 35%. It cannot reverse that term.
The existing heading term can still request countersteering.

Damping is fully available within 0.25 m and fades to zero at 0.50 m. It fades
between the existing confidence thresholds, between 8 and 12 m/s, and with the
existing heading authority in curves. One-line holds use the original target.
There is no new temporal filter, derivative state, or waiting timer.

Parallel offsets and motion away from center retain the original target. The
existing 0.00045 1/m correction cap, engage/release steps, confidence arming,
width memory, turn/reversal guards, and exact E2E fallbacks are retained.

## Validation

- 21 policy tests pass, including the 15 inherited tests and six new tests.
  These were executed with the actual policy definitions isolated from modeld's
  hardware/runtime imports. Full modeld startup and device integration were not
  tested in this environment. The tests remain in the normal test module:
  `python -m unittest openpilot.selfdrive.modeld.tests.test_lane_policy`.
- Python compilation and `git diff --check` pass.
- A simple straight-lane kinematic simulation with steering delay and lag reduced
  overshoot in 12 cases: speeds 15, 20, 28, 33 m/s; delay/lag pairs 0.10/0.15,
  0.20/0.25, 0.30/0.35 s. For example, at 28 m/s with 0.20/0.25 s delay/lag,
  overshoot after a 0.25 m initial offset was 13.56 cm in the baseline and
  11.03 cm in the candidate. This is not a calibrated vehicle model.
- Recorded-input replay used route 23f segments 3, 6, 7, 8, 12, 17. The first
  201 frames of each segment were excluded from summary comparisons, leaving
  5,994 frames. Segment 13 was excluded because it contains a selector-off
  interval without a per-frame selector signal. Replay assumes selector-on in
  the remaining segments. Raw E2E was reconstructed with the baseline policy;
  it was not separately logged. Numerical agreement is not independent proof
  of that reconstruction. No active-state differences were observed, outputs
  stayed finite and within the existing correction cap, and blinker fallback
  returned exact E2E.
- The replay has a tradeoff: per-segment 95th-percentile correction frame steps
  increased approximately 5–15%. The change targets delayed closed-loop
  overshoot, not lower command jitter on a fixed recording. This is why the
  prediction was limited to 0.20 seconds rather than the initially evaluated
  0.35 seconds.

Actual improvement in lane centering and comfort remains unverified. Recorded
input replay cannot reproduce the changed vehicle motion and subsequent model
predictions. Keep the original lp-final branch for comparison.
