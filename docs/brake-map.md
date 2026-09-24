# Silverado/Sierra stock-sampled brake mapping trial

This branch starts at `lp-prelead-adapt` (`e8fd452`), with opendbc based on
`f4bb1db`. It replaces the abandoned `long-tune` experiment rather than building
on its percentage correction. The change is confined to GM actuator mapping.

## Evidence

Fourteen recorded segments have factory ACC enabled and openpilot longitudinal
disabled: route 262 segments 0-8, route 156 segments 0/3/5, and route 163 segments
1/2. Read factory commands from incoming camera CAN bus 2; none of these segments
contains openpilot `sendcan` gas/brake commands. Exclude driver pedals, AEB,
inactive/nonadaptive cruise, stale CAN, and standstill hold from moving-response
calibration. Ten other older candidates (164 and 16e) were openpilot controlled
and were excluded from factory calibration.

The final moving portions of two complete stops in 262/5 and 262/6 have roughly
0.12-0.13 m/s² less deceleration than the old 100-command-units-per-m/s² scale
predicts. At higher speeds the old scale is substantially closer. Factory ACC
uses a -540 Nm encoded torque request during braking, versus this fork's -500
Nm request. This is a command-value observation, not a measurement of wheel torque.

## Mapping

The Silverado/Sierra lookup uses these points, after the existing torque/drag
conversion to brake acceleration:

| Brake acceleration (m/s²) | Existing/high-speed command | Low-speed command |
| --- | ---: | ---: |
| -4.0 | 400 | 400 |
| -1.0 | 100 | 100 |
| -0.5 | 50 | 62 |
| -0.2 | 20 | 32 |
| -0.1 | 10 | 10 |
| 0.0 | 0 | 0 |

Use the low-speed curve through 2 m/s (7.2 km/h); blend continuously to the
existing curve at 4 m/s (14.4 km/h). Near-zero braking remains gradual, without
a minimum-command floor. The mapping continues into standstill, so the ordinary
-0.37 m/s² stopping request produces 49 units instead of 37 without a fade-out.
Stock hold observations are contextual; stationary acceleration was not used
to fit the moving brake response. Existing brake modes and the 400-unit cap stay
in place. Braking/stopping torque uses -540; inactive output stays -500.

Other GM platforms retain their tables and torque commands. Silverado delay
remains 0.3 s and Ki remains 0.05. Planner, lead estimation, longitudinal state
machine, and lateral path behavior are unchanged. The opendbc URL points to the
matching fork so the pinned custom commit can be fetched on the device.

## Validation and limits

- Eight GM controller/fingerprint tests pass. Camera-longitudinal safety suite:
  29 pass, four existing skips. The safety test's stale 1346 Nm maximum was
  corrected to the fork's already-existing 2450 Nm limit; production safety
  code and limits were not changed.
- Actual baseline and candidate controllers were each exercised on 19,200
  fixed recorded inputs from routes 26a/26b. Each passed 38,398 gas/brake packet
  checks against compiled safety. One segment-start snapshot lacked coherent
  engagement metadata; its two packet checks were excluded, its output retained.
- Brake increase is bounded at 12 units; brake values at/above 4 m/s, inactive
  outputs, and propulsion commands with no braking/stopping are preserved.
- Command-to-deceleration mean absolute error on the separate final stop falls
  from 0.121 to 0.033 m/s². This is an inverse-map comparison on recorded stock
  commands, not a prediction of improved stopping distance or closed-loop stability.
- A short crawl/release window becomes less accurate (0.104 to 0.149 m/s² at
  zero alignment shift). Release dynamics, engine creep, gear changes, grade,
  and load cannot all be identified by this static map. Only two complete stock
  stops support the low-speed calibration; further road logs are required.
- This mapping acts after a braking request. It cannot fix an event in which
  the planner has not yet requested deceleration.
