# lp-prelead-adapt: adaptive distance-derived lead speed

Based directly on `lp-e2e-prelead` at
`0e27d7ba9768d90b0f8c6f322c8356ee0d7c2113`.

This experiment varies the distance track's acceleration process-noise density
with measured time headway and estimated closing time. The intent is to reduce
small speed-estimate fluctuations at longer gaps while preserving the existing
response nearby and allowing faster changes when closing quickly. It is an
incremental estimator adjustment, not a demonstrated fix for phantom braking.

## Schedule

Time headway is `observed_distance / max(ego_speed, 5 m/s)`. This is a measured
input to the filter; it does not change the planner's desired following time.

| Measured headway | Normal acceleration-noise density q | Expected effect |
| --- | ---: | --- |
| At or below 1.0 s | 0.100 | Original prelead response |
| 1.0 to 2.5 s | Linearly interpolated from 0.100 to 0.050 | Gradually stronger smoothing |
| At or above 2.5 s | 0.050 | Strongest normal smoothing |

At 90 km/h, these headway boundaries correspond to approximately 25 m and
62.5 m. Using headway makes the schedule scale with ego speed instead of
applying the same physical-distance thresholds at every speed.

When ego speed exceeds the track's estimated lead speed, estimated closing time
is `observed_distance / (ego_speed - distance_track_speed)`. A second linear
schedule sets a floor under q: 0.200 at or below 6 s, declining to 0.050 at
12 s. The larger of the normal q and this floor is used. Thus a distant but
rapidly closing lead does not receive the strongest smoothing simply because
its distance is large. This estimate can itself lag; it is not an independent
braking guarantee.

The resulting q is bounded between 0.050 and 0.200, in units of
`(m/s^2)^2 s`. The existing constant-velocity process covariance remains:

```text
Q = q * [[dt^3 / 3, dt^2 / 2],
         [dt^2 / 2, dt]]
```

The schedule is continuous at its boundaries. It does not reset the filter or
scale the published velocity directly. Measurement variance already adapts to
camera distance uncertainty and confidence, and remains
`max(distance_std, 1)^2 / raw_probability^2`.

Both lead slots use this schedule. Matching camera hypotheses still share one
track/update. Distance and wheel motion still estimate velocity without model
speed updating the Kalman state. The original model braking guard, recovery
hold, fallback, acquisition history, and output handover are retained.

## Replay results

Compared with the published prelead baseline at fixed q = 0.100, using the same
recorded inputs for each candidate. Two predefined candidates were compared:
physical-distance scaling and the selected time-headway scaling. Neither wins
every metric. Headway scaling gives less smoothing at high speed for the same
physical gap and had a smaller gap-prediction regression on route 261.

The comparison covers 46 segments and approximately 54,000 model frames:

- `615b11d4c01d81ec/00000261--f0b430254e`, segments 10-20.
- `615b11d4c01d81ec/00000262--93c81f5e48`, segments 0-8; recorded stock ACC motion.
- `615b11d4c01d81ec/00000266--fa900d4a53`, segments 1-18.
- `615b11d4c01d81ec/00000267--9ffd5a270b`, segments 5-6; recorded old KF.
- `615b11d4c01d81ec/00000268--32e32f7999`, segments 0, 4, 9, 12, 13, 14; recorded prelead.

| Route | Slot | Speed-step RMS: prelead / adaptive (m/s) | Two-second gap-change MAE: prelead / adaptive (m) |
| --- | ---: | --- | --- |
| 261 | 0 | 0.11249 / 0.11328 | 2.3593 / 2.3809 |
| 261 | 1 | 0.11290 / 0.11369 | 2.3418 / 2.3629 |
| 262 | 0 | 0.12819 / 0.12785 | 2.1928 / 2.1885 |
| 262 | 1 | 0.13649 / 0.13617 | 2.1849 / 2.1806 |
| 266 | 0 | 0.18823 / 0.18725 | 2.6024 / 2.5832 |
| 266 | 1 | 0.18942 / 0.18843 | 2.6013 / 2.5819 |
| 267 | 0 | 0.07064 / 0.06942 | 1.9839 / 1.9799 |
| 267 | 1 | 0.07139 / 0.07019 | 1.9692 / 1.9653 |
| 268 | 0 | 0.08343 / 0.08248 | 2.0334 / 2.0252 |
| 268 | 1 | 0.06929 / 0.06811 | 1.5900 / 1.5816 |

The incremental reduction in speed-step RMS is approximately **1-2% on the new
267/268 clips**. On route 261, RMS instead increases about **0.7%**, and
gap-prediction MAE increases about **0.9%**. On 262/266 the improvements are
small. These mixed results justify keeping a separate experimental branch.

Large reset/guard steps remain. For example, route 266's maximum lead-0 step
falls from 4.093 to 3.901 m/s, but route 268's increases slightly from 1.3749 to
1.3763 m/s. Lower average variation does not mean every event improves.

The reporting mask selects engaged following above 20 m/s with raw and held
confidence above 0.95, no pedals/blinkers/experimental mode, adjacent valid
timing, and no abrupt position jump. All frames still update the causal
estimator. For the new noncontiguous clips, the first 10 s after each replay gap
are excluded from metrics. Speed-step RMS measures estimator smoothness, not
vehicle jerk. Gap-change MAE uses current speed, retained model acceleration
decay, recorded ego travel, and the camera's later gap: it checks consistency
against the same sensor, not independently measured lead velocity.

## Scope and validation

41 focused tests pass: the 35 prelead estimator/RadarD tests plus six adaptive
tests covering speed-scaled headway, bounds, closing-time override, continuity,
noise suppression, and changing-speed response. Tests use real RadarD logic
and capnp schemas with unavailable OS/IPC imports substituted by the local
harness. Run in a built checkout with:

```bash
python -m unittest \
  openpilot.selfdrive.controls.tests.test_vision_lead_tracker \
  openpilot.selfdrive.controls.tests.test_radard_prelead \
  openpilot.selfdrive.controls.tests.test_vision_lead_adaptive
```

The final source also reproduces the selected candidate on all 9,315 aligned
frames from routes 267/268, with a maximum speed-field difference below
1.1e-14 m/s. Ruff lint and formatting checks pass.

Raw published lead distance and acceleration are unchanged. Detection changes,
noisy raw gaps, and the city experimental-mode planner can still cause
slowdowns. Distance-speed output remains inactive below 15 m/s (54 km/h).
The existing model braking guard is retained and tested; these checks do not
establish road braking performance or eliminate the filter's response lag.

There is no following-time, longitudinal MPC policy, actuator torque-map,
lateral, or delay adjustment. The inherited longitudinal delay remains 0.3 s.
Both new recorded branches also used 0.3 s. A fixed-plan projection at 0.3 and
0.5 s showed only small request differences in the examined highway events;
it was not a closed-loop delay experiment.

`opendbc_repo` remains at `f4bb1db17f0f38bbeab8f1cefd7492a8154ebd5c` and
`.gitmodules` is unchanged. This work includes no native MPC solve, full
vehicle simulation, or road validation of the adaptive branch.
