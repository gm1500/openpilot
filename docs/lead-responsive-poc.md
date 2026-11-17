# Responsive lead fusion and derived-speed E2E assistance

Branch: `batmobile-future-lead-prediction-poc`, 2026-11-17. This POC retains the
lane-width/road-edge changes, one-lane-change-per-signal fix and the existing
future-position range correction. The intermediate guard prioritizes earlier
pickup of a large closing-speed difference; following distance and planner
gains are unchanged.

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
  slowing requires current-model closing of at least 0.5 m/s and qualified
  retained-motion history. The range estimate's upper two-sigma speed bound
  no longer floors the final speed correction. Range uncertainty checks and
  covariance floors remain; model agreement is not independent evidence.
  Correlated range/model bias can therefore add unwanted slowing, including
  when the actual lead is maintaining its distance.
- Additional correction enters at at most 2.5 m/s per second. The old total
  correction limit of 3 m/s is removed. Invalid, absent, ambiguous, braking,
  short-gap and unsupported inputs retain the current pipeline. Losing target
  support discards extra correction instead of carrying it into a new car.
- Current presence, dRel, yRel, aLeadK and aLeadTau are unchanged by the new
  stage. MPC still uses its existing future trajectory; the full predicted
  distance trajectory is not substituted. A runtime exception disables the
  added stage for that process and retains the already accepted output.

This is the intermediate policy, not the unrestricted offline combination:
existing lower speeds still have immediate priority, and additional correction
still enters gradually. The earlier strict guard remains available in commit
`2aa46656b414710dbe4a2c0d4a3d97a264d64ac2`. Nominal forecast fusion weights do not
describe the effective output share after source-priority checks.

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

All changes below are relative to the original future-lead POC baseline.

| Comparison | Previous strict guard | Intermediate guard |
| --- | ---: | ---: |
| Morning replay: earlier crossing of -0.5 m/s² | 0.25 s | 1.30 s |
| Older difficult replay: earlier crossing | 0.15 s | 0.15 s |
| Braking-lead synthetic onset, five matched seeds | Same time in all five | 0.05 s later to 0.15 s earlier |
| Braking-lead minimum-gap change | 0 to +0.76 m | +0.005 to +1.82 m |
| Standard noise/bias cases: worst extra deceleration | 0.033 m/s² | 0.132 m/s² |
| Standard cases exceeding 0.1 m/s² extra | 0/120 | 1/120 |
| Additional biased challenge: worst extra deceleration | 0.115 m/s² | 0.560 m/s² |
| Biased challenge cases exceeding 0.1 m/s² extra | 1/30 | 12/30 |

The intermediate policy recovers another 1.05 s on the morning replay while
preserving existing braking priority. The older difficult event still reaches
-3.5 m/s²; it remains unresolved. The earlier unrestricted candidates gained
2.55–2.70 s on the morning event but delayed braking up to 1 s in the braking-lead
stress test and lost up to about 8.8 m of minimum gap. They are not deployed.

The production candidate and strict parent are rerun against the same baseline:
120 closed-loop runs (eight scenarios x five seeds x three variants),
360 fixed-ego noise/bias runs and 90 additional biased-forecast runs. Steady and
opening scenarios match the baseline. Approaches have the same or earlier
onset; overly fast future predictions do not delay onset. Lead removal and
faster handoff scenarios retain the existing target handling. No native solver
failures occur.

The additional challenge holds true lead/ego speed at 33 m/s, biases current
model speed to 32.3 m/s, predicts future speed at 22 m/s, and adds correlated
distance noise. Twelve of 30 intermediate cases add over 0.1 m/s² of braking.
The worst peak, 0.560 m/s², stays over 0.1 m/s² for 1.05 s; another case peaks at
0.416 m/s² and stays over 0.1 for 2.30 s. These are fixed-ego command differences,
not measured speed losses or expected real-road event frequencies. This is the
known tradeoff accepted for earlier closing-speed response.

## Semi cut-in replay

Two full-rate segments cover 2,400 paired model/radar updates. During the event
in segment 23, all three native-MPC replays first cross -0.5 m/s² at 18.352 s.
The recorded manual brake begins at 19.411 s; automatic comparisons stop before
that intervention. The intermediate guard makes no output change in either
segment, and does not carry an added correction across a target discontinuity.

In the 15–19.41 s event window, 89 updates are rejected before reaching the final
guard: 48 acquiring, 14 range quarantine, 11 distinct/missing, 8 not accepted
and 8 uncertain observation. The lead is late and range changes repeatedly
interrupt qualification. Relaxing only the final speed bound cannot fix that.
The semi case still needs a separate acquisition/continuity investigation.

Presence, distance, lateral position and acceleration fields are identical on
every paired semi update. Recorded lead acceptance is held fixed; this is not
a rerun of visual detection. The host MPC excludes full map/E2E/lateral
arbitration and cannot establish whether intervention would have been avoided.

## Earlier E2E gate validation

Separately, the E2E helper replay covers 24,686 frames, of which 17,737 meet the
available recorded eligibility checks. Derived speed alone qualifies 38 frames;
22 also have an E2E braking request. The changed gate affects 40 commands,
including its release, with maximum additional deceleration 1.283 m/s². This
uses recorded regular commands, already containing the old assist; it is an
isolated helper replay, not a counterfactual full planner/vehicle validation.

## Validation and limits

215 tests pass across lead, E2E-stop and lane-change suites, with one optional
private recorded-fixture test skipped in the public checkout. Tests cover a
large closing difference, immediate priority for an existing slower estimate,
rate limits, correlated-bias tradeoffs, and range/lateral/distinct-lead changes.
Serialized Cap'n Proto inputs remain covered. Changed Python files pass Ruff
and whitespace checks. Replays include the morning, older difficult approach
and semi cut-in recordings.

Full radard process replay is unavailable on this host because native msgq is
not installed. The broader conditional-mode/UI suite also requires pyray.
These results do not replace on-device validation or establish performance
across all roads, weather, target changes or perception errors.

Diagnostics add `responsive`, `responsive_dv` and `responsive_fast` to the
existing `lead-pre` log. `responsive_fast` describes private adaptive filtering;
the actual output correction is `responsive_dv`. The existing E2E assist HUD
continues to show actual E2E braking contribution.
