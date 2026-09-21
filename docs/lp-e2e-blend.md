# E2E path fit experiment

Based on `lp-anchor-v3` commit `88d594ec33be3f0dd6775497ca29a2d0e79e1a63`.
This branch changes the geometry of the lane correction. It is an experiment;
recorded-input replay does not establish improved vehicle motion.

## Target and steering

The previous policy fitted affine lane/E2E disagreement over 8–40 m and evaluated
that fit at one lookahead distance. The new policy uses the shared path samples
over 8–55 m, limited to actual E2E coverage. It does not use the entire distant
model horizon. The existing minimum short-plan coverage remains required.

1. Fit a lateral shift and heading adjustment between the E2E path and the lane
   midpoint across all usable samples.
2. Apply that alignment to E2E. Blend the remaining shape difference toward the
   lane midpoint with the existing smooth 10–45 m spatial transition. Near the
   start, the target retains E2E's shape plus its fitted alignment. At 45 m and
   beyond, the virtual target reaches lane midpoint geometry.
3. Fit the added steering correction across the target samples, weighted around
   the existing speed-dependent lookahead. Keep the existing heading damping.
   Normalize the response so a constant parallel lane/E2E displacement produces
   exactly the previous base centering correction, including on shorter plans.
4. Apply the existing approach easing, centering learner, turn guards, correction
   limits and entry/release rates. Add this correction to the original E2E action.

This is a virtual tracking target: translating a path alone does not change its
curvature or make the vehicle center. Its displacement must generate a steering
correction. The E2E action remains the immediate feedforward command, and the
near-lane centering learner still corrects persistent vehicle offset when the
two predicted paths happen to match. Large disagreements remain bounded by the
existing correction limit; convergence against arbitrary E2E commands is not
guaranteed.

The original position, orientation and model action outputs are not rewritten.
Only the final scalar action receives the lane correction. The raw model path
therefore remains available for diagnosis. Model delay compensation is unchanged;
the new averaging is spatial and adds no temporal steering filter.

## Confidence and diagnostics

Temporary one-line holds retain v3's 8–40 m affine correction because their far
geometry depends on reconstructed lane width. Confidence arming, hold expiry,
lane-change intent, blinker and invalid-input fallback follow v3. Fallback returns
the exact E2E command and resets policy state.

Telemetry uses `lp-e2e-blend:` with the existing command, lane-motion and bias
fields, plus `path_blend=1` for the new path fit or `0` for a one-line hold.

## Validation

Run the 52 policy tests from the repository root:

```bash
python -m unittest openpilot.selfdrive.modeld.tests.test_lane_policy
```

The tests cover affine alignment, near/far target shape, unchanged parallel
centering gain, left/right symmetry, large path disagreement, shared-horizon
coverage, one-line behavior, exact fallback, motion estimation and correction
limits. Both recorded drives (31,200 model frames) also passed replay checks for
unchanged engagement/hold states, unmodified model inputs, exact fallback, and
bounded correction, slew and learned bias.

Across six steady windows in the latest drive, faster command variation
(0.15–0.8 Hz) decreased 3.0–3.8%; slower variation (0.04–0.15 Hz) decreased
0.1–2.5%. Four windows from the earlier drive showed 2.1–3.1% less faster command
variation, while slower variation was mixed (−0.8% to +1.4%). These are command
statistics on fixed inputs, not predicted reductions in vehicle oscillation.
Turn-guard timing can change because the fitted geometry changes. Road comparison
is still needed for curve tracking, settling time and persistent lane offset.
