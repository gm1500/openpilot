# lead-cont-v2: bounded handoff after an uncertain range jump

Based on `lead-cont` at `a38e0aa5829c0ed2aa8c8b46e9010c629c926cb9`.

## Problem and implementation

Route 26f, segment 9, at 36.787 s contains a primary-lead range jump from
45.16 m to 34.55 m while reported range uncertainty rises from 8.81 m to
19.28 m. The strict association correctly starts new filter history, but the
immediate switch to model speed drops the published lead speed by 11.15 km/h
in one frame. The secondary hypothesis also moves laterally away from the
previous shared track.

This revision permits a narrowly gated handoff of the old published speed to
the new track's existing one-second transition to model fallback:

- The old track must be ready, publishing distance-derived speed, and at least
  five seconds old. The next observation must arrive within 0.01–0.10 s.
- New lead probability must be at least 0.90. Range standard deviation must be
  at least 10 m and at least twice the previous observation's standard deviation.
- The observation must be within 15 m longitudinally and 0.15 m laterally of
  the previous measurement prediction, and within one combined standard
  deviation of the predicted filtered distance.
- The old track must be unmatched under the original association rules. Only
  one unmatched group in that track's previous slots may qualify. An ordinary
  match elsewhere or two plausible recipients prevents the handoff.
- Every receiving lead must be accepted vision input, with finite distance,
  speed and acceleration, ego speed between 15 and 60 m/s, and gap between
  10 and 150 m. Model acceleration below -0.5 m/s² or a model closing time
  below eight seconds prevents entry and immediately cancels an active handoff.

The new track starts with fresh state, covariance and age. The original strict
association and grouping gates, Kalman updates, and readiness rules are retained.
Only `vLead`, `vLeadK` and `vRel` can change. Raw distance, acceleration, lead
acceptance and radar behavior remain as supplied by radard. The existing
five-second closing guard applies outside the uncertain handoff.

The blend uses a fixed starting speed and the current fallback target, so a
changing target cannot create interpolation overshoot. It completes on the
first sample at or after one second. A new unready track cannot renew it.

An earlier prototype retained the old filter state through the range jump.
It was rejected after an ambiguous slower-cut-in scenario retained an excessive
speed estimate for multiple seconds. Range uncertainty alone cannot establish
that a new observation belongs to the same physical car.

## Validation

All **53 tracker tests** pass, including 14 new continuity tests:

```sh
python3 -m unittest \
  openpilot.selfdrive.controls.tests.test_vision_lead_tracker \
  openpilot.selfdrive.controls.tests.test_vision_lead_adaptive \
  openpilot.selfdrive.controls.tests.test_vision_lead_continuity
```

New cases cover uncertain splits, a slower same-lane cut-in, bounded expiry,
changing targets, repeated resets, urgent braking and closing, stopped leads,
confident cut-ins, competing matches, slot ownership, immature or invalid state,
and unsupported inputs. In the ambiguous slower-cut-in case, published speed
reaches the model target by the first sample at one second; stronger braking
or closing evidence takes effect immediately.

Recorded-input replay covers **34,798 frames across 29 segments and four routes**.
Both lead slots are evaluated against the parent implementation using identical
observations and recorded ego motion. Filter state, covariance, age and readiness
are exactly equal in all 64,640 available lead-slot checks. All 776 urgent-guard
checks and 10,996 protected-path checks pass; nonvelocity output fields match
their inputs.

| Route / window | Parent | lead-cont-v2 |
| --- | ---: | ---: |
| 26f/9, initial downward speed step at 36.787 s | 11.154 km/h | 0.000 km/h |
| 26f/9, largest downward step during 36.7–38.0 s | 11.154 km/h | 1.713 km/h |
| 26f/9, minimum lead speed during 36.7–38.0 s | 99.165 km/h | 106.491 km/h |
| 26c/12, largest downward step during 56.39–56.70 s | 0.011 km/h | 0.011 km/h |
| 26c/12, largest downward step during 57.09–57.50 s | 0.269 km/h | 0.269 km/h |
| 26d/8, peak lead speed during 55–59 s | 112.536 km/h | 112.536 km/h |

Only **21 primary-lead outputs** change, from 36.787 to 37.786 s in 26f/9.
The secondary lead is identical throughout. All 29,998 frames from the earlier
26b, 26c and 26d routes match the parent outputs exactly. The handoff completes
at the next sample, 37.836 s, and all subsequent outputs match the parent.

The 26f logs report parent commit `a38e0aa` with a dirty working tree. The parent
replay reproduces their published speeds within 0.000004 m/s. Extracted input
SHA-256 values used for the comparison are:

| Input | SHA-256 |
| --- | --- |
| Earlier 29,998-frame replay inputs | `6eca1652deeecfee99d2f0249e27cefd098cbd42cf2dd4d44112d0528fb68eef` |
| 26f, segments 6–9, aligned inputs | `5f96fe4581a246ce25c567b52d41db6c94af5e814fd25f2cc82efc5cea180063` |

## Scope and remaining limitation

This is an estimator change, validated by unit tests and recorded-input replay.
No native MPC replay, alternative vehicle trajectory, device build or road test
has been performed. The raw range still drops by about 10.6 m; these results do
not establish that the braking event disappears.

A real slower cut-in with the same uncertain observations can receive the
bounded speed handoff when neither guard fires. Its model-speed response can
therefore be delayed by up to one second. The thresholds are conservative
experimental gates tested on these logs, not calibrated target-identity
probabilities. Closed-loop validation remains necessary before road use.


## Mild kinematic planner-range filter

The branch now also applies a deliberately mild causal correction to the camera
range only while an established vision track is publishing distance-derived
speed. Each frame predicts the next range from the previous filtered range and
the published relative speed, then accepts 60% of the new camera-range
innovation:

`predicted = previous + (vLead - vEgo) * dt`

`filtered = predicted + 0.60 * (raw - predicted)`

This is not a conventional long time-constant low-pass. It is intended to
reduce frame-to-frame range inconsistency while retaining most new measurement
information. New/cold tracks, radar leads, unsupported leads, active uncertain
handoffs, and braking/rapid-closing mode continue to publish raw range. Entering
braking mode resets the private filtered range to the raw measurement so a real
closing/braking event is not delayed by stale range history.

When two model hypotheses share one private track, the range state updates once
per frame from the merged observation; each slot retains its small raw offset.

The 0.60 correction gain is the conservative candidate from the prior offline
kinematic comparison. The newly supplied route-270 raw logs still require a
capnp-capable replay environment for quantitative retuning; this commit does
not claim a fresh full-stack replay of those files.
