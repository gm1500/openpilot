# Batmobile validation

## AOL audio and indicator refinement — 2026-11-20

Cancel and brake/regen/parking-brake pauses during AOL-only steering suppress
only their new disengagement chimes. Normal cruise disengagement, queued alerts
and simultaneous fault warnings retain their sounds. AOL-origin warning faults
keep their original duration without being renewed while suspended, including
single-frame faults that soundd could otherwise miss. OFF, successful rearm and
offroad transitions clear only those retained warnings. Shared event alerts and
steering authority are unchanged.

Unavailable hands now have a grey body and bright light-grey outline, with both
forearms fading out. Active, ready, paused and off indications are unchanged.

The combined final-source regression run passed 518 tests and 45 subtests,
covering AOL, alert lifetimes, the normal cruise state machine, speed/Panda
readiness, both hold controls, stop presentation and existing stop/slow-assist
logic. Ruff and whitespace checks passed. This refinement is Python-only and
introduces no settings or parameter migration. Device audio is not yet verified.

## Lane-wide stop-bar presentation — 2026-11-20

Reviewed upstream `45e522b` (lane-wide stop bar, PR #39429) and `ccbd8c0`
(planned stops without a lead, PR #39352). Both were already in Batmobile's
upstream history; the new drawing was confined to mici. The C3X presentation
now uses the lane-wide flat quad, road projection, edge easing and fades while
retaining its existing colours, ModelStop marker, stop distance and labels.

The upstream cue detects the first predicted forward speed below 1 m/s. It adds
no red-light classification. Batmobile already uses a qualified slow terminal
trajectory before near-standstill `shouldStop`. Comparing 10,799 recorded model
frames found 650 raw mici-only candidates, all below 9.21 km/h: 610 were within
the upstream 8 m appearance floor and the other 40 were hidden by its lead rule.
Several described acceleration/creeping. No additional independent appearance
candidate justified changing control qualification in this sample. This is not
a traffic-light ground-truth evaluation or a claim about every driving scenario.

The port preserves existing stop-distance filters instead of stacking mici's
30% per-model-frame correction on top. Only lateral edges, visibility and
engagement brightness are animated. Fresh ordinary withdrawal may fade briefly;
stale data, invalid calibration, a new lead, drive/timestamp resets and released
held/green markers cannot leave a cached target. Flat-quad inset tests cover
camera roll and skew so the red fill remains inside its yellow outline.

Stop geometry, animation, actual renderer integration, existing ModelStop and
stop/slow-assist regression checks passed 194 tests and 45 subtests. The actual
CPU renderer was also inspected with a simulated-road animation and both AOL
indicator sizes. No model, planner, firmware, schema or parameter changes are
required by this presentation update. On-device appearance remains unverified.

## AOL startup and speed-floor correction — 2026-11-20

The installed AOL build correctly advertised GM lateral-only support, but never
requested it. All 1,752 GM-mode Panda samples in the diagnostic logs carried a
latched `interruptRateCan2` fault. The fault first appeared during ELM327 startup;
the diagnostic controller then stopped receiving/forwarding traffic, its error
counter remained constant, and the driving controllers stayed clean. An earlier
pre-AOL drive showed the same historical diagnostic bit.

Reconstruction from three full log segments covered 17,713 host samples. Of the
9,958 samples above 10 km/h, 9,524 satisfied the other readiness conditions; the
remaining samples were brake/gear pauses or Cancel events. Message ages stayed
within the configured freshness limits. This identified the blanket Panda-fault
gate as the startup blocker, rather than a missing parameter or UI color bug.

The correction separately qualifies this exact historical diagnostic after two
seconds of fresh quiet telemetry, retaining all other fault gates and explicit
rearming after a new failure. A shared AOL speed helper also enforces the
Silverado's 10 km/h steering floor independently of the warning-suppression
override. Low speed withdraws AOL without latching a fault; the UI outlines the
hands until steering is available and confirmed active.

Validation from this source tree:

- AOL health, speed boundaries, host/control separation, E2E and lane-button UI,
  and lane-change behavior: 296 tests and 21 subtests passed.
- SLC fallback/preview/settings, Sentry, and stock power monitoring: 238 tests
  and seven subtests passed, including the configured parked voltage cutoff.
- Ruff and whitespace checks passed. Native Params used the existing build
  after its source inputs were compared byte-for-byte with this tree.

The corrected host replay qualifies the historical fault while parked, then
first requests AOL at 10.003 km/h. An onroad replay with the actual GM27 native
safety library, recorded CAN traffic and reconstructed heartbeat requests grants
lateral permission and the host observes its acknowledgment. Cancel, SET rearm
and brake release retain their intended behavior. The replay submits recorded
commands; it does not generate a new closed-loop AOL steering trajectory.

This correction changes Python code only; it adds no parameters, schema changes,
or Panda firmware changes relative to the original AOL build. The log analysis
uses recorded publication times to reconstruct consumer timing. It does not
establish real vehicle actuation, UI timing, or road behavior after the change.

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
