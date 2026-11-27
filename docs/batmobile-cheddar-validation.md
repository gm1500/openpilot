# Batmobile validation

## Five-feature consolidation — 2026-11-27

The ten Batmobile commits above upstream `40aa55877` were rebuilt as the five
feature commits listed in `batmobile-cheddar.md`. The previous tip remains at
`archive/batmobile-before-five-commits-20261127` (`475393c6`). No upstream update
or model/submodule version change is part of this consolidation.

The cleanup removes the unconsumed CEM junction traversal/publication and its
obsolete tests, redundant cruise way-ID arguments, an unread lane-release timer,
and an unused polygon renderer. Historical wire ordinals are retained. Shared
parked-position validation and vehicle identity move from Sentry to `common`;
the map downloader no longer imports Sentry for them.

Validation on the cleaned final tree:

- **2,230 tests and 276 subtests passed; 10 skipped.** Coverage includes map
  matching/preview, cruise/Resume, model stops/native MPC, lane changes, AOL,
  UI behavior, Sentry, parked-location handling and upload priority.
- All five intermediate trees pass Python syntax, newly introduced import,
  Cap'n Proto schema and logged-service/Event checks. Focused intermediate
  tests pass: GM helpers **82 + 38 subtests**, lane controls **99 + 21 subtests**,
  and SLC integration **45 + 44 subtests**. Counts overlap the final suite.
- Regenerated native schemas and the rebuilt native messaging registry pass
  **7 service tests** and the Sentry state test (included above). Params/msgq
  builds were reused after checking matching sources; the MPC generation
  functions are unchanged and use the existing compiled solver.
- Ruff, whitespace, Python syntax and shell syntax checks pass. The final
  tree differs from the archived branch only by this cleanup and documentation.
- Replay of the two C2 Resume windows still restores **115 and 110 km/h**;
  the pre-fix implementation selects **100 km/h** in both windows. This checks
  logged button/speed selection, not vehicle acceleration.

The launcher disables only the host-blocked Unix logging handler and sets
`SCALE=1` to avoid querying a physical monitor. Two full-window UI startup
suites are excluded because native window initialization is unavailable on
this host; behavior tests use the real UI modules. Optional recorder fixture
coverage remains skipped where its build/media dependency is unavailable.
The earlier GitHub static-analysis baseline has unrelated type/codespell
failures; this refactor does not claim those repository-wide checks are clean.
No device installation or road test was performed.


## Upstream consolidation — 2026-11-25

The eight retained feature groups are reapplied above `commaai/master`
`40aa55877cca9d4b1f8fac51686d7997e1dcd3aa`, including all 16 incoming commits.
The pre-integration head `57cadbf99d8a97b4fbbd254450633ded37651a01` is retained
as `archive/batmobile-before-20261125-upstream-eight`.
The latest retained-CAN-history correction belongs to the AOL feature commit;
it is not a ninth custom commit. This consolidation updates `batmobile` only.

The upstream Crumbly Cheddar small-model pointer matches the already prepared
CH3 model exactly: SHA-256
`962396e7909556af6a9c7bfb7ad6db01a8d2c8da0760379eb71efda004cb3548`,
69,699,272 bytes. No model parser/planner interface change accompanies this
upstream update. The model-assets tree matches upstream byte-for-byte.

opendbc `9ac0d3c0f795cc50c65cdea291ffd5a41d13ea76` merges the GM/AOL fork with
upstream `66b4002800e30b0ca28f91f73e7dd872da147c76`. The GM conflict keeps the
independent AOL rearm and Cancel behavior while adopting upstream's
`enter_controls()` and `exit_controls()` helpers. GM tuning, vehicle controller,
and lateral limits retain their prior custom contents. Panda
`5dbe0da1bf547bb1009cecd29e835c7f153130df` changes only its standalone opendbc
dependency pin to this same revision; its existing AOL firmware source remains.

Three UI conflicts were resolved by combining the retained camera zoom, model
stop bar, map sign, lane control, and E2E/AOL widgets with upstream's per-widget
status fades, glow helper, driver-warning routing and variable path width.
Removed upstream personality-overlay methods are no longer called. The current
SLC/CEM, lane-policy, Sentry, schema and host-AOL files outside these upstream
overlaps retain the previous branch's contents. Documentation records the new
lineage and validation separately from the historical results below.

Validation on the integrated source:

- Full opendbc safety suite: 3,727 tests and 12,138 subtests passed; 1,598
  abstract/platform skips. The focused GM/tuning/CAN suite separately passed
  377 tests and 1,054 subtests.
- Panda H7 and body H7 firmware builds, signed test binaries and host library
  builds succeeded against the merged opendbc headers, with warnings as errors.
- AOL health, state and steering-speed tests: 471 passed using merged opendbc.
  The retained-history implementation and regression tests match the previously
  reviewed fix byte-for-byte.
- UI tests: 213 tests and 25 subtests passed. A targeted Mici check constructed
  the real HUD, camera view and model renderer, verified custom child controls,
  and exercised dynamic path projection. Unavailable visual assets/fonts were
  substituted for that check; it does not establish graphical rendering.
- Maps, car-SLC, lane policy and upload priority: 525 tests and 104 subtests
  passed. A separate vehicle/cruise/longitudinal/MPC/desire suite using merged
  opendbc passed 303 tests and 156 subtests, with seven base/platform skips.
  These groups overlap; their counts are not an aggregate unique-test total.
- Sentry control, power, parking, upload and UI-coupled cases: 484 unique tests
  passed. Optional native video/state fixture suites were not rerun; their
  source is unchanged. The existing MPC binary was reused only after the solver
  generator and model constants matched; the differing following-time helper
  is Python runtime logic and was exercised in the merged-opendbc suite.
- Python compilation, Ruff and whitespace checks passed for the integration
  edits. The final eight-commit tree must match the reviewed integration tree.

The test launcher removes only this host's denied Unix-socket logging sink;
stderr logging remains enabled. These checks do not establish on-device model inference, graphical startup,
firmware flashing or road behavior. No vehicle update was performed.

## Upstream consolidation — 2026-11-24

The branch is consolidated into the original eight feature commits above
`commaai/master` at `e170f08049b4d91f595a0a11378b0cf030f3afb1`.
The previous `batmobile` head `cab06d156ebefad9d7027831434344b801d9fccb`
is preserved as `archive/batmobile-before-20261124-eight-commits`.
The prior upstream-test branch through `519f804b37efc4359beca6e96a5a5c8d01b2eb18`
is included. Consolidation checks require the eight-commit head's tree to match
exactly the reviewed integration tree, with all upstream commits as ancestors.

Incoming upstream changes include the Qualcomm priority cleanup, tracked-lead
trajectory revert, Fairy Tale large model, Smoked Cheddar small C3X model,
personality tuning and mici display updates. Model pointers match upstream;
GM/AOL opendbc and Panda pins remain `f089655ec` and `77ae0d198`.
The upstream version-header change to 0.11.3 is retained.

Personality integration keeps following times at Relaxed/Standard 1.45 s and
Aggressive 1.25 s. Full-E2E acceleration boosts are respectively 0.0, 0.05 and
0.15 m/s², using upstream speed/braking scaling. Batmobile applies these only
to the full-E2E candidate, after raw stop/slowing qualification. Chill limits,
stronger real-lead/cruise constraints, the 2.0 m/s² full-E2E ceiling and the
6 m stopped gap remain. Relaxed's previous 1.75 s following time is intentionally
replaced by upstream's 1.45 s. Both UI descriptions reflect Batmobile's speed
caps rather than promising an unlimited model-selected speed.

The retained fixes include AOL startup, Cancel and parked-door handling,
steering-wheel speed selection, Initial Max Speed, stop bars, and parked-area
map refresh. Mici's new personality overlay no longer hides the separate E2E/AOL
control. Two additional map continuity regressions are covered by recorded GPS:

- AA: the 100 km/h mainline remains matched through the cosmetic road-name
  change from Anthony Henday Drive NW to Anthony Henday Drive, both ref 216.
  The prior missing-match interval at 719.593–721.602 s is removed.
- A7: the established 80 km/h road remains selected at 777.002 s instead of
  blanking. The 100 km/h limit still applies only at entry, 777.998 s. The
  exception requires exactly two candidates and a unique unentered continuation;
  true forks, third candidates and invalid position data retain their guards.

The A4 replay confirmed the existing cruise handoff: 100 was initially an
upcoming preview while the current limit was 80; the current 100 limit applied
SET immediately. No additional override of higher-limit preview rules was added.

A7 logs contain missing-timezonefinder import failures. The explicit parked SLC
updater now verifies/repairs timezonefinder 8.2.0 and tzdata 2026.6 using the
active interpreter, locked missing dependencies and frozen existing package
versions. It verifies geographic lookup and system/package civil-time handling
before build/reboot. This source change does not repair a vehicle until the
parked helper runs; an ordinary branch update does not install Python packages.

Validation on the integrated source:

- Maps, car SLC, AOL health/state and UI regression run: 1,064 tests and 129
  subtests passed before the final A7/updater additions. The affected map suite
  was rerun separately after those additions: 301 tests and 68 subtests passed,
  including 16 parked-updater checks. The existing Sentry updater suite also
  passed all 28 tests alongside the new updater suite.
- Planner modes, cruise approach, immediate SLC and following distance: 44
  tests and 72 subtests passed; one expected abstract/base skip.
- Stop/slowing, native stop-target MPC, lane policy and desire helper: 163 tests
  and 48 subtests passed. All 23 upstream longitudinal maneuvers passed.
- Sentry power, parking, upload, controller and stock power monitoring: 203
  tests passed. These results retain existing firmware and safety behavior.
- Python syntax, Ruff, shell syntax and whitespace checks passed.

Tests use the actual Cap'n Proto schemas and native MPC. The existing Params
library was reused only after its source inputs matched byte-for-byte. The MPC
solver generator and constants match the existing build; only the runtime
following-time helper changed. Exact-pin msgq/VisionIPC was rebuilt. Core
numerical dependency versions match the lockfile. The test launcher removes
only the Unix socket logging handler prohibited by this host.

The graphical UI startup smoke test cannot pass on this host: native window
initialization crashes identically on the unchanged previous branch and the
integration, with no display server or Xvfb available. UI control/geometry tests
pass, but graphical startup is not established here. No model inference,
on-device package installation, firmware flashing or road validation was done.
No locationd health tolerances or lane-change retry behavior were added.

## Upstream test branch — 2026-11-23

`batmobile-upstream-test` branches from `cab06d156ebefad9d7027831434344b801d9fccb`
and merges upstream master through `af90646a4a02a93ab48d34a3fc017a9414ac79b9`.
The three incoming commits remove the obsolete Qualcomm GPU priority override,
revert model-predicted future lead trajectories to tracked-lead extrapolation,
and update the large driving model to Fairy Tale. The small driving model used
by the current C3X without Chestnut/eGPU is unchanged.

The longitudinal merge retains the virtual stop target, its separate MPC
constraint, hold/release behavior, and planner handoff. Lead processing and
extrapolation match upstream exactly. Both tracked leads still constrain MPC;
the published `hasLead` flag follows upstream's primary-slot-only semantics.
The updated regression test verifies second-slot braking and that future model
lead arrays cannot replace the tracked lead trajectory. GM tuning, submodule
pins, AOL, SLC, Sentry, lane policy, and desire logic retain their Batmobile
versions.

Validation on the merged source:

- Native MPC, msgq and Params built through repository SCons.
- 197 targeted regression tests and 109 subtests passed, covering stop-target
  constraints and release, planner modes, SLC handoff, cruise recovery, slowing,
  stop horizon, lane policy, and lane-change/AOL gating.
- All 23 upstream longitudinal maneuver tests passed, for 220 unique tests in
  total. The test runner disabled only the Unix-socket logging handler blocked
  by the host environment; stderr logging, planner logic and the native solver
  were unchanged.
- Ruff, shell syntax and whitespace checks passed.

No vehicle installation or driving validation was performed. These checks do
not establish that the Qualcomm priority change resolves the recorded timing
stalls or red takeover alerts, or validate the large model on eGPU hardware.

## AOL historical receive-loss qualification — 2026-11-21

The next drive ran clean `9017c364` with AOL enabled, but the host never requested
lateral-only permission. Four recorded segments contain 10,465 selfdrive samples
above 10 km/h and 2,352 GM Panda samples. CAN0 retained `totalErrorCnt = 1` and
`totalRxLostCnt = 1` throughout; its live error flags and the driving CAN2 bus
were clear. This pair was already present in the first no-output Panda sample,
before diagnostics and GM control started. The existing historical-diagnostic
qualification required every driving-bus counter to be zero forever, so the
unchanging receive-loss history prevented qualification for the entire sample.

The correction permits only this one matching CAN0 error/receive-loss pair in
the already-restricted Silverado/tres/GM27 diagnostic-fault qualification. CAN2
and all other driving-bus error counters remain zero-only. Qualification needs
two seconds of fresh, distinct samples with unchanged diagnostic and driving
error counters, clean live bus status and the existing integrity checks. Any
counter change resets qualification; the separate armed-AOL fault latch still
requires explicit driver rearming after a new failure. The helper does not
require a no-output packet that SelfdriveD might not have observed at startup.

Validation from this source tree:

- AOL health/state, stock state and alert management, lane-change behavior,
  car lateral-only handling and E2E-button UI: 368 tests and 21 subtests passed.
- Replaying all four recorded segments with the original helper rejects Panda
  health in all 10,465 samples above 10 km/h. The corrected helper rejects none;
  the same 1,157 samples retain their normal brake/gear pauses, and 9,308 pass
  all explicitly reconstructed readiness gates. Qualification completes while
  parked, before the first sample above the steering-speed floor.
- A separate replay using the GM27 native safety library and reconstructed
  heartbeats grants lateral permission after the host first requests it at
  10.004 km/h; the host observes the acknowledgment about 130 ms later. It has
  no rejected steering messages, RX rejections or new native faults. One
  recorded gas/regen message is rejected identically with both original and
  corrected host helpers; it is not a regression from this change.
- Ruff and whitespace checks passed.

This correction is Python-only relative to `9017c364`; it adds no parameters,
schema changes or firmware changes. The readiness replay does not recreate
SubMaster service-frequency checks. The native replay uses recorded commands,
not newly generated closed-loop steering commands. Neither reconstruction
proves physical steering or on-device behavior after the correction.

## Cancel preserves AOL state — 2026-11-21

Cancel now disables cruise only. The host excludes `buttonCancel` from AOL
readiness gates and GM safety preserves its independent lateral permission and
fault latch. AOL remains active, off, temporarily paused or fault-suspended as
appropriate. Pressing, holding and releasing Cancel cannot change that intent.
A SET-release edge emitted while switching to Cancel cannot rearm a fault on
either side. A subsequent SET/RESUME or AOL OFF-to-ON still rearms when ready.

Brake/regen/gear pauses, the 10 km/h floor, driver monitoring and fault guards
remain in effect. Cancel during a temporary pause does not prevent recovery
when the pause ends. Normal cruise cancellation retains its disengagement
sound; AOL-only Cancel remains quiet and other warnings remain audible.

Host, cruise state-machine, alert, lane-change, speed/health and indicator
regressions passed 342 tests and 21 subtests. Native GM safety passed 297 tests
and 601 subtests, with 145 expected abstract/unsupported skips. The safety
matrix includes cruise-to-AOL handoff without another heartbeat, independent
steering permission, denied gas/brake commands after Cancel, existing fault
latches and SET-to-Cancel transitions. Ruff and whitespace checks passed.

This update changes GM safety in the opendbc submodule. The existing Panda
source pin is unchanged because its build imports that safety source. The H7
firmware built successfully with warnings treated as errors. Updating only
Python is insufficient: the normal device build/signature-check process must
install the matching firmware; a prebuilt installation needs a newly built
matching image. No settings or parameters need initialization.
No firmware was flashed to the vehicle during development.

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
