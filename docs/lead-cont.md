# lead-cont: bounded speed handoff across a brief lead split

This revision replaces the wider history-retention gates introduced in
`0668c426cd378e43d49b004ba1160aeaac33f8aa`. The underlying estimator and vehicle
configuration remain based on `brake-map` at `43e9ef63912723510fd7dae20538817f3979393f`.

## Why the history change needed correcting

The wider gates removed short model-speed fallback dips, but a subsequent
recorded drive exposed a range-reversal regression. Keeping the older filter
state through a hypothesis split produced a lead-speed estimate up to 20.2 km/h
above the previous estimator on the same inputs. The planner continued to
request acceleration as the reported gap shrank before segment 9.

## Updated behavior

- Grouping again always uses the original **1.5 m longitudinal / 0.35 m lateral**
  limits. A new track learns its own state and covariance.
- When a ready, shared distance track briefly splits, its last published speed
  may seed the new track's existing **one-second output transition**. Both
  observations must be valid, match the previous track independently, and stay
  within **3 m / 0.5 m** of each other. These wider limits permit an output
  handoff only; they never retain or copy the Kalman history.
- A persistent split fades to model fallback. A new, unready track cannot
  repeatedly renew the handoff. Rejected filter states cancel the bridge for
  both slots before either output is published.
- Mode transitions now interpolate between a fixed starting output and the
  current target. The previous additive-offset approach could overshoot both
  when the target changed during the transition.
- Current model-braking and closing-time guards override the transition
  immediately. Lead acceptance and unsupported/low-speed fallback are preserved.

The Kalman gains, raw published distance and acceleration, planner policy,
actuator delay, Ki, brake mapping, and opendbc revision are unchanged.

## Verification

39 focused tracker/adaptive tests pass. They cover cold/prewarmed acquisition,
independent tracks, slot swaps, loss and reacquisition, low-speed/radar fallback,
cut-ins, immediate braking guards, covariance behavior, bounded handoff expiry,
changing-target interpolation, rejection of stale states, and regrouping onto
new history.

```sh
python3 -m unittest \
  openpilot.selfdrive.controls.tests.test_vision_lead_tracker \
  openpilot.selfdrive.controls.tests.test_vision_lead_adaptive
```

Recorded-input replay covers **29,998 frames across 25 supplied segments** from
three routes, including city stops and the two highway regression cases. Both
lead slots are replayed. The new private filter states match the original
strict-grouping estimator; only the published handoff differs.

| Earlier dip window | Original maximum downward speed step | Updated maximum downward step |
| --- | ---: | ---: |
| Route 26c, segment 12, 56.39–56.70 s | 6.290 km/h | 0.011 km/h |
| Route 26c, segment 12, 57.09–57.50 s | 6.366 km/h | 0.269 km/h |

For route 26d, segment 8, 55–59 s, the current estimate's 2 s gap-consistency
error falls from **11.76 m to 5.86 m**. The original strict-grouping version is
5.58 m in this window. Peak estimated lead speed falls from **123.6 to
112.5 km/h**. The update preserves short-dip suppression while addressing the
large retention regression; it does not eliminate the underlying filter lag.

The gap-consistency calculation uses recorded ego travel and the same model
lead acceleration for all speed sources. It compares predictions against the
later camera gap, not independent ground truth. These are estimator replays,
not a native MPC solve or a simulated alternative vehicle trajectory. They do
not establish a new minimum driving gap or road performance.
