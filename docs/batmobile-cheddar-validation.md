# Batmobile validation

## E2E, AOL and map-gap update — 2026-11-20

The update is based on upstream `d0800d005e8ef89a468c24d358984da83f84cbe0`,
preserving the radard/plannerd CPU-core change and the retained GM tuning.
The previous complete branch is preserved as
`archive/batmobile-pre-aol-20261120` at
`bd579d480fa0d1517f46063510acbb007b6089b8`.

Current checks include:

- SLC, preview, settings and stock cruise: 161 tests and 33 subtests passed,
  with six expected base/unsupported skips.
- Planner mode selection, cruise recovery, stopping and immediate SLC handoff:
  111 tests passed, including the native MPC cases.
- AOL host state, readiness, driver monitoring, control separation and existing
  signal/lane-change behavior: 114 tests and 21 subtests passed. Coverage includes
  temporary brake/gear pauses, faults occurring during those pauses, and blocking
  automatic lane changes during AOL-only steering.
- Sentry and stock system-power monitoring: 135 tests passed.
- Native Params rebuilt with both new persistent settings; default AOL ON,
  saved OFF, and default E2E OFF verified. Cereal schemas were regenerated.
- Native x86 pandad compiled and linked with the current schemas and heartbeat
  protocol, with warnings treated as errors and no missing dynamic dependencies.
- UI gestures, colors, fresh-state/steering-speed gates and saved defaults:
  66 tests passed, including the unavailable outline and pause precedence.
- All-brand native safety: 3,138 tests and 10,953 subtests passed, with 1,451
  expected abstract/base skips. The GM subset, MISRA checks, Panda protocol,
  and actual ARM DEBUG/RELEASE application and bootstub builds passed. A separate
  reviewer reproduced and checked the pause-fault, stale-host, capability-reset
  and SET/RESUME rearming cases.

A final combined host/SLC/planner/UI/power run passed 478 tests and 64 subtests,
with six expected skips. These counts overlap the focused suites above.

The route-83 speed-selector replay used 49,700 recorded input frames across the
requested clip. The previous helper matched 49,699 recorded SET values; the
remaining frame differed by one 10 ms map-recovery ordering interval. The old
helper raised SET from 100 to 105 km/h during four map gaps. The fixed helper
retained 100 km/h through all five gaps (approximately 32.6, 55.8, 4.4, 29.6 and
0.4 seconds), including two RESUME re-engagements, while preserving later
explicit +1/-1 km/h inputs. Three unwanted SET increases happened while stopped
and one near 8 km/h. This validates speed selection, not actual vehicle
acceleration or closed-loop braking.

Tests that instantiate logging disable only the blocked Unix logging handler in
the test launcher. UI tests set `SCALE=1` for the headless host instead of querying
a physical monitor. Production code, native Params, model/lead inputs and control
calculations are not replaced. This update has not been installed or driven on
the vehicle from this workspace.

## Original Cheddar rebuild — historical validation

Validated on Linux x86_64 against upstream openpilot
`1b09aabb6bb6e04820559955ac26f63eebddbee5`, with the checked-in lockfile's
production dependency versions and opendbc
`66e8086083e8b09783fffbf044350a2417bcfd83`.

## Builds and regression coverage

Native builds passed for Params, msgq, VisionIPC, cereal, socketmaster,
the regenerated longitudinal MPC, loggerd, encoderd, sentry_loggerd and the native
Sentry recorder test. The MPC was regenerated with CasADi 3.6.7 and acados
0.2.2.post103; NumPy was 2.5.3 and raylib was 6.0.0.1.post111.

| Suite | Result |
| --- | --- |
| GM calibration, safety and all 255 platform interfaces | 497 passed; 740 subtests passed; 141 base/abstract skips |
| Maps, SLC, painted-lane policy and signal-triggered lane changes | 446 passed; 108 subtests passed |
| Planner modes, model-only lead assist, virtual stops, stock longcontrol/following and longitudinal maneuvers | 119 passed; 27 subtests passed; 1 upstream skip |
| UI and Sentry UI behavior | 218 passed; 25 subtests passed |
| Sentry, native registry/state, upload/deletion/power and WebRTC compatibility | 417 passed; 1 optional camera-sample skip |
| Real encoded-video Sentry recorder integration | 13 passed; 2 private VisionIPC transport skips |

These suite counts can overlap and should not be summed. All 23 upstream
longitudinal maneuver scenarios passed, including stopped/slowing leads and
cut-ins. Ruff, Python compilation, shell syntax, whitespace and lockfile checks
passed. Service, Event and parameter registration were audited. Native
`radard.py` and `longcontrol.py` remain byte-for-byte upstream; model assets and
the upstream runtime submodule pins are unchanged except the documented GM fork.

## Inactive-overlay comparison

A separate untouched upstream six-parameter MPC/planner was compared with the
rebuilt seven-parameter MPC/planner over 12,000 synthetic frames. Both ran in
separate processes with locked dependencies, the same input sequences, actual
reset/engagement handling, cruise arbitration and final acceleration clipping.
The comparison covered three personalities, five ego speeds and eight lead
scenarios; custom overlays were asserted inactive.

- No solver failures or differences in `shouldStop`.
- No-lead and standstill-launch acceleration commands matched exactly.
- Absolute acceleration-command difference: 95th percentile
  **0.0000805 m/s²**, 99th percentile **0.000580 m/s²**.
- Largest difference: **0.03397 m/s²** on the first abrupt cut-in frame at
  5 m/s ego speed (upstream 0.75350, rebuilt 0.78748 m/s²). The next-frame
  difference was -0.00242 m/s² and the transient decayed.
- Maximum integrated five-second command difference: **0.001633 m/s**.

The added virtual-stop inequality changes solver numerics even while inactive;
this is not a claim of bitwise solver equivalence. This synthetic comparison
found no material regression in the scenarios exercised.

## Limits

The workspace denies the Unix datagram logging transport, so test launchers
removed only that logging handler. They did not replace control calculations,
native Params, schema serialization or the rebuilt MPC. Recorder tests used a
workspace temporary directory with adequate free space, preserving the real
storage-reserve guard. Private encoder-buffer IPC tests and full card-process
replay could not be completed here.

No device installation, full driving-model replay, live camera/screen check or
road drive was performed. Passing unit and synthetic tests does not establish
braking improvement or on-device timing. See [the behavior specification](batmobile-cheddar.md)
for the retained features and removed experiments.
