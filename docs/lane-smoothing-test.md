# Original lane policy with plan-long smoothing

Test branch: `batmobile-lane-smoothing-test`.
Base: `batmobile` at `3f09538824e57ca3b56a13d6d4c6ce1fafc7bd8d`.

This experiment keeps batmobile's lane midpoint calculation, E2E-shaped target,
32–45 m lookahead, centering gains, heading/approach damping, and learned bias.
Lane confidence, width validation, arming, one-line hold, and fallback rules
also stay intact. It does not use the weaker correction formula from the
separate `batmobile-plan-lane-test` branch.

The change replaces the 0.30 s lane-correction reversal pause with the
`FirstOrderFilter(0.0, 0.5, DT)` smoothing used by commaai
[`plan-long` at 1c2df3b](https://github.com/commaai/openpilot/blob/1c2df3b3fbdb2b5aed93217953515ccee0ab7cee/openpilot/selfdrive/modeld/modeld.py).
At the 20 Hz model rate, each frame closes 1/11 of the remaining difference
between the filtered correction and the requested correction. The 0.5 s value
is a time constant, not a waiting period. Steady correction strength is unchanged;
transient correction responds more slowly.

The filter acts on the geometric centering correction after approach damping
and the small-correction deadband, before the existing learned bias and final
curvature/slew limits. It runs on straights and curves. Correction sign changes
alone no longer trigger a timed zero correction or reset motion/bias feedback.
The separate 0.35 s guard for a reversal of the base model's turn direction is
retained; it clears opposing filtered correction. Fallback clears all filter
state immediately. The base model's curvature is never filtered.

## Validation

Run from the repository root:

```sh
python -m unittest openpilot.selfdrive.modeld.tests.test_lane_smoothing -v
```

The 13 host tests cover the filter recurrence, correction reversals, retained
turn guard, original steady correction at 50/80/100/120 km/h in both curve
directions, confidence/width rules, one-line expiry, fallback/rearming, malformed
paths, motion feedback, learned bias, and correction limits.

These are synthetic, open-loop tests, not a closed-loop vehicle evaluation or
replay of the reported oscillation. The filter adds phase lag and can retain
the previous correction direction briefly after the target changes. This
branch does not establish that curve oscillation is fixed or guarantee exact
vehicle centering.
