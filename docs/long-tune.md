# long-tune: low-speed brake-map experiment

Base: lp-prelead-adapt, e8fd4529a2d88b70bbebf3f048fe0e78e75dfd73.

This first experiment targets the repeatable shortfall between requested and measured gentle deceleration in routes 26a/26b. It applies only to the Chevrolet Silverado / GMC Sierra platform with openpilot longitudinal control.

## Change

For an existing negative friction-brake acceleration request, add up to 25% of its magnitude, capped at 0.10 m/s² equivalent. Multiply this correction by a speed weight interpolated through:

| Speed (m/s) | Weight |
|---:|---:|
| 0.0 | 0.0 |
| 0.3 | 1.0 |
| 1.0 | 1.0 |
| 2.0 | 0.0 |

Skip compensation when carState reports standstill. Existing brake-output saturation remains in place. The map is continuous in request and speed; no new filter or integral state is introduced.

This is a conservative calibration candidate, not a fitted physical brake model. The added command is smaller than the approximately 0.13 m/s² low-speed tracking deficit seen in the close-stop windows.

Delay remains 0.3 s, Ki remains 0.05, and the planner, lead tracker, lateral control, high-speed brake map and standstill hold calibration retain the baseline implementation. Integral-transition handling is reserved for a separate experiment after evaluating this change.

## Validation

Six tests pass: five targeted map/controller tests plus the existing GM fingerprint test. Coverage includes bounded and monotonic correction, continuity, no braking from nonnegative requests, unchanged other vehicle outputs, unchanged high-speed/standstill/disengaged outputs, and existing output limits.

Fixed-input command replay over 19,200 model-frame snapshots:

| Route | Frames | Frames with a changed brake command | Maximum extra brake units |
|---|---:|---:|---:|
| 26a | 12,000 | 186 | 10 |
| 26b | 7,200 | 286 | 10 |

Mean extra brake commands in the reviewed slow-stop windows:
- 26b / 23 / 36–41.1 s: 6.58 raw units.
- 26b / 25 / 12.8–15.3 s: 5.98 raw units.
- 26b / 26 / 42.35–46.4 s: 7.57 raw units.

No commands change at or above 2 m/s, while disengaged, or when standstill is reported. Replay holds the recorded controller requests and vehicle state fixed: it does not predict a new stopping distance or validate closed-loop road behavior.

The distant-lead intervention spanning segments 25–26 remains a separate perception issue. This tune cannot supply braking before the planner requests it.

The opendbc submodule URL points to gm1500/opendbc so clean installs can fetch the pinned custom revision.
