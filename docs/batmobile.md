# Batmobile

`batmobile` combines `boat-anchor` and Sentry on comma master
`6f4c0faf1f85132734e94aa39c67143f823b9324`. It retains tinygrad
`d3f09c9bbd542fbfbe68c8569a1173550c408969` and the Sierra opendbc revision
`63c1fe02b20f9df99367671ae060652d16bbd00d`.

The base was grouped into eight feature commits: Sierra tuning, Nudgeless, Lane policy,
[VOACC fixes](voacc.md), SLC, [CEM](conditional-experimental.md),
[C3x UI](c3x-ui.md), and [Sentry mode](sentry.md).
See [boat-anchor](boat-anchor.md) for the driving features and defaults.

Only the `batmobile` branch shows **Batmobile Activated** at startup, with the usual driving reminder.
Construction and school sign headers fill to the rounded edges with padding
below the label. Sentry includes the latest three-camera sensitivity calibration,
recording clock/upload fixes, and motion eye positioning and reflections.

The sidebar shows device temperature and live vehicle voltage. Cleanup removes
the duplicate recording timer assignment and reuses each camera's sampling grid
until its dimensions change. Retired telemetry ordinals remain reserved for existing logs.

This branch preserves the published driving policy, mode defaults, submodule
pins and stop distances. History cleanup does not add approach-mode hysteresis
or an early-stop offset.

Validation: the native Sentry recorder, encoder, params and messaging targets
build. Sentry/modem tests pass 141 checks; control/map/UI tests pass 338 checks.
One VisionIPC test is skipped by this environment, and the full UI process test
fails during font loading without a working display/assets setup. Zone-header
drawing calls pass 36 size/state checks; no rendered-device or road validation
is implied. Each feature commit's Python syntax and Cap'n Proto schema is checked.

## Parked maintenance update

Sentry now applies a delayed Wi-Fi skip until first arming completes, unless a mode
was explicitly selected. SLC adds optional Daily or weekday-based Weekly refreshes
for installed maps, using offroad unmetered Wi-Fi with catch-up and resumable jobs.
Manual updates retain the existing network-metering policy. A conflicting request
no longer discards download progress.

Cleanup removes the one-shot recording-cleanup/update wrapper, the retired
`slc-offline-maps` update target, and the GPS helper's obsolete caller-supplied
startup-frame argument. Freshness checks, active recovery fixes and driving policy
remain. The standalone explicit recording-cleanup utility is retained.

Validation: 625 tests and 94 subtests passed across the relevant map, cruise,
Sentry, upload-priority and settings suites. The parameter library was rebuilt
with the new keys; branch-added Python files pass lint and the updater passes
shell syntax checks. On-device UI, Wi-Fi timing and real provider downloads were
not exercised. Use `tools/slc_update.sh` for the required native parameter rebuild.
