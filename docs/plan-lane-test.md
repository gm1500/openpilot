# plan-long lane-centering test

Base: `gm1500/openpilot` batmobile, `3f09538824e57ca3b56a13d6d4c6ce1fafc7bd8d`.
Steering policy source: `commaai/openpilot` plan-long,
[`1c2df3b`](https://github.com/commaai/openpilot/commit/1c2df3b3fbdb2b5aed93217953515ccee0ab7cee).

The existing on-road lane toggle selects this policy. With the toggle off,
the unmodified model steering action is used. There is no new setting.

## Changed steering calculation

- Compare the model path with the accepted lane centre at `max(2 * speed, 10)` metres.
- Apply half the geometric curvature correction, weighted by lane confidence.
- Fade the correction target in from 4 to 8 m/s.
- Limit added lateral acceleration to 0.5 m/s² and filter with a 0.5 s time constant.
- Reapply the same limit to the filter state when speed changes.

This replaces the previous multi-point path fit, heading damping, learned centering
bias, curve-reversal release logic and correction slew limiter. Those corrections
are not combined with the new calculation.

## Retained qualification and fallback

- Require both lane probabilities at least 0.92 for 0.75 s before activation.
- Retain accepted lines down to probability 0.70.
- Keep the existing width/width-variation checks and learned lane width.
  At highway speeds, apply those checks through the longer lookahead point.
- Keep up to one second of one-line reconstruction, including left-line priority
  when visible lines disagree. Its confidence weight comes from the retained line;
  with two accepted lines, use the lower of their probabilities.
- Release immediately on toggle off, blinkers, lane-change intent, invalid inputs,
  or loss of usable lane geometry. Clear the filter and require requalification.
- Require the model and lane geometry to cover the lookahead; do not clamp a
  too-short path to its endpoint.
- Preserve the HUD active/arming/one-line-hold status interface.

Longitudinal control, lead tracking, CEM, SLC, Sentry and vehicle actuator tuning
are unchanged. This does not import plan-long's longitudinal changes.

## Validation

Run `python -m unittest openpilot.selfdrive.modeld.tests.test_lane_policy`.
Coverage includes confidence qualification/retention, lane width, both one-line
fallbacks, lane-change/toggle reset, curve symmetry, numerical correction strength,
speed fade, smoothing, bounds and malformed/short paths.

Host validation: 24 unit tests passed. The curvature calculation matched the
upstream function on 2,000 generated cases (maximum absolute difference
1.73e-18 1/m). Existing detector/hold states matched batmobile over 3,000 sequential
synthetic frames within its original detection horizon. Python compilation and
`git diff --check` also passed.

Host checks establish implementation behaviour, not closed-loop driving quality.
No on-device or road validation has been performed for this branch.
