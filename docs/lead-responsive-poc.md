# Responsive lead fusion and derived-speed E2E assistance

Branch: `batmobile-future-lead-prediction-poc`, 2026-11-17. This POC retains the
lane-width/road-edge changes and the existing future-position range correction.

## Runtime behavior

`LeadResponsiveMotion` runs after the existing lead tracker, confidence filter,
transition recovery and bounded future-position estimator in radard.

- Retain the private range Kalman history through brief outliers. A robust
  quadratic fit to up to 2.4 s of ego-motion-corrected range checks for a genuine
  change in motion. At least 1.2 s/24 samples, a negative two-sigma acceleration
  bound, a lower current-speed bound and 0.15 s of consistent evidence are
  required before increasing process noise. State and association history are
  preserved; the same observations are not assimilated a second time.
- Keep the separate future-velocity Kalman (velocity/acceleration states,
  measurement updates at most 4 Hz), covariance-intersection fusion and small
  direct-model buffer. The buffer is at most 5% at 140 m, 2.5% at 100 m, zero
  at/below 80 m, reduced by model uncertainty and bounded to +/-0.5 m/s.
- Existing accepted lead speed is a ceiling: a slow private estimate cannot
  overwrite a lower speed already accepted by the current pipeline. Additional
  slowing requires current-model closing of at least 0.5 m/s and is limited by
  the range estimate's upper two-sigma speed bound. Forecast agreement alone
  cannot justify an added slowdown.
- Additional correction enters at at most 2.5 m/s per second. The old total
  correction limit of 3 m/s is removed. Invalid, absent, ambiguous, braking,
  short-gap and unsupported inputs retain the current pipeline. Losing target
  support discards extra correction instead of carrying it into a new car.
- Current presence, dRel, yRel, aLeadK and aLeadTau are unchanged by the new
  stage. MPC still uses its existing future trajectory; the full predicted
  distance trajectory is not substituted. A runtime exception disables the
  added stage for that process and retains the already accepted output.

The final output is deliberately more guarded than the earlier unrestricted
offline combination. Nominal forecast fusion weights do not describe the
effective output share after these bounds and source-priority checks.

## E2E slowing gate

The selective assist may enter when **model lead speed <55 km/h OR accepted
radard lead speed <55 km/h**. The latter includes qualified derived velocity.
Exactly 55 km/h does not qualify. Either valid source may qualify; ego speed
does not determine this threshold. No private or unaccepted track can enter.

The accepted-lead, fresh-data, engagement, pedal-override, lead-relevance and
0.15 s sustained E2E braking checks remain. Stronger regular braking always wins.
Once latched, the existing go/pull-away release applies even when speed rises
above 55 km/h. This remains an E2E braking constraint, not a switch into full
experimental mode.

## Completed comparisons

Native branch MPC/ordinary VOACC host harness, 20 Hz point-mass plant, 0.5 s
command queue and 0.2 s acceleration lag. Aggressive personality, cruise
120 km/h. Recorded-drive replay holds ego motion fixed; maps/E2E/lateral
arbitration are excluded from these estimator comparisons.

| Comparison | Previous unrestricted combination | Guarded responsive POC |
| --- | ---: | ---: |
| Morning replay: earlier crossing of -0.5 m/s² | 2.55 s | 0.25 s |
| Older difficult replay: earlier crossing | 0.50 s | 0.15 s |
| Braking-lead synthetic onset, five matched seeds | Up to 1.0 s later | Same time in all five |
| Braking-lead minimum-gap change versus current POC | As much as -8.62 m | 0 to +0.76 m |
| Standard noise/bias cases: worst extra deceleration | 1.23 m/s² | 0.033 m/s² |

The safeguards give back much of the morning timing gain to remove the large
braking-lead regression. The adaptive covariance mechanism contributes in two
noisy braking seeds; preserving the existing lower speed is the main fix.
The older difficult event still reaches -3.5 m/s²; it remains unresolved.

The updated estimator is evaluated against the earlier matched baselines with
120 new closed-loop runs (eight scenarios x five seeds x three variants),
480 fixed-ego noise/bias runs and 90 additional biased-forecast runs. Controls
disable either adaptive noise or both future/buffer components. No native
solver failures occur. Steady/opening and overly fast future scenarios match
the current POC; noisy approaches have the same or earlier braking onset.

The additional challenge holds true lead/ego speed at 33 m/s, biases current
model speed to 32.3 m/s, predicts future speed at 22 m/s, and adds correlated
distance noise. One of 30 candidate cases still adds over 0.1 m/s² of braking,
with a maximum of 0.115 m/s². Range-only produces the same worst result, so
this remaining case is not cured by removing future prediction.

Full morning estimator replay covers 21 segments/24,694 paired updates.
The final stage adds a correction on 92 updates, with a maximum of 2.226 m/s
below baseline. Presence, distance, lateral position and acceleration fields
are preserved on every paired update.

Separately, the E2E helper replay covers 24,686 frames, of which 17,737 meet the
available recorded eligibility checks. Derived speed alone qualifies 38 frames;
22 also have an E2E braking request. The changed gate affects 40 commands,
including its release, with maximum additional deceleration 1.283 m/s². This
uses recorded regular commands, already containing the old assist; it is an
isolated helper replay, not a counterfactual full planner/vehicle validation.

## Validation and limits

170 lead-related tests pass in the analysis checkout, including serialized
Cap'n Proto model inputs; 24 E2E-stop tests pass. The optional private recorded
fixture is not distributed with the public source: its one unit test skips
when the fixture is absent, while synthetic regression tests still run.
Changed Python files pass Ruff, compilation and whitespace checks.

Full radard process replay is unavailable on this host because native msgq is
not installed. The broader conditional-mode/UI suite also requires pyray.
These results do not replace on-device validation or establish performance
across all roads, weather, target changes or perception errors.

Diagnostics add `responsive`, `responsive_dv` and `responsive_fast` to the
existing `lead-pre` log. `responsive_fast` describes private adaptive filtering;
the actual output correction is `responsive_dv`. The existing E2E assist HUD
continues to show actual E2E braking contribution.
