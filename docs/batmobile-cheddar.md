# Batmobile on Cheddar

This rebuild starts from upstream openpilot and reapplies the retained Batmobile
features. It uses upstream Cheddar model assets and the current model/runtime
architecture, with native lead processing restored.

## Source lineage and dependencies

| Reference | Revision |
| --- | --- |
| Upstream openpilot base | `d0800d005e8ef89a468c24d358984da83f84cbe0` |
| Previous Batmobile, archived as `archive/batmobile-pre-cheddar-20261118` | `cebc3f0b11f90fa4d2f5113e3df44762a56ff5df` |
| Upstream model-assets tree, unchanged | `903fd28aef90d787adceb88071fd2ca73ba88924` |
| tinygrad, unchanged | `ec91b74b5b6b0fc924b72d3f27689679ca2438b9` |
| msgq, unchanged | `42554d0a6f1cb2e87238c994760078be36cfdb27` |
| Panda base before separate AOL permission | `5e8131ec962f6c2d88adda4ec7166e5d308f095e` |
| Panda with independent AOL heartbeat and health | `77ae0d198be68ef20f60ecbef6a58a201871522b` |
| rednose, unchanged | `e60428b2e52f139d5fe5466d4d9fa3c6adeadad8` |
| teleoprtc, unchanged | `f19ceb4553d1d9ccd46cc86f47483564814423ec` |
| Upstream opendbc base | `5cb4ca34c7abb2a976fb835ba9fca6bf4386ed4c` |
| opendbc GM calibration base before AOL | `66e8086083e8b09783fffbf044350a2417bcfd83` |
| opendbc with independent GM AOL safety | `196d1b12da7ebf32fbd0081b16d3ff95817fb610` |

The upstream version is **0.11.4**, with **AGNOS 20.6**. The lockfile retains
tinygrad **0.14.0**, NumPy **2.5.3**, pycapnp **2.1.0**, and the CPU raylib package
**6.0.0.1.post111**. Map civil-time handling adds timezonefinder **8.2.0** and
tzdata; use the checked-in lockfile and submodule revisions when reproducing the
build. The GM fork carries the retained Sierra/Silverado torque, mass, gas and
safety calibration on the upstream opendbc base. The checked-in Panda and
opendbc gitlinks add a separate GM lateral-only permission; their branches are
`batmobile-aol-20261120` in the gm1500 forks. The upstream move of radard and
plannerd off the UI core remains included.

## Driving behavior

Ordinary mapped-speed operation uses native real-lead MPC with the restored
pre-Cheddar Chill cruise acceleration curve and direct speed-error response.
The stock personalities remain **Aggressive: 1.25 s**, **Standard: 1.45 s**, and
**Relaxed: 1.75 s**, with their upstream jerk settings. `radard.py` and
`longcontrol.py` remain byte-for-byte upstream. Vision leads use upstream model
velocity with its ego-speed correction; native radar association and filtering
remain available where the vehicle supplies radar.

Chill positive acceleration is limited to **1.6, 1.2, 0.8 and 0.6 m/s²** at
**0, 36, 90 and 144 km/h**, interpolated between those points: approximately
**1.02 m/s² at 60 km/h** and **0.69 m/s² at 120 km/h**. Manual and automatic E2E
retains the **2.0 m/s²** ceiling. The selected positive output also obeys its
mode's ceiling. The measured-acceleration taper and newer one-second cruise
filter have been removed; the original speed-dependent cruise jerk limits remain.

When a lead or model candidate is controlling acceleration, positive cruise
history is bounded by the selected positive output. Releasing that constraint
therefore cannot uncover an accumulated high cruise request. Negative lead
braking is not copied into positive cruise history. Stronger lead/model braking
still wins the existing minimum-candidate selection. A lower mode ceiling applies
immediately; an already-high acceleration request can drop when that ceiling
changes. SET is applied before map recovery clears E2E in the same card update.
The vehicle's actuator delay and existing acceleration can still cause transient
overshoot after a late lower SET. Upstream's fixed Aggressive-personality
acceleration boost remains limited to full E2E and does not affect Chill or the
raw-model braking-assist gates; this policy has no learned acceleration boost.

The narrow slowing assist can enter only when the model's current lead-speed
estimate is **below 55 km/h**, with a present accepted lead and sustained model
braking. Distance and native lead data still provide proximity and release
checks. The assist does not replace lead velocity or relax the regular MPC's
braking. Once entered, a brief speed estimate above 55 km/h does not itself
release the assist. Qualified model stopping intent can also activate the
retained stop target, with real-lead overlap checks and existing release rules.

When enabled SLC has no usable mapped limit, its request automatically admits
the native model acceleration and curve-speed result into planning. The initial
no-map ceiling defaults to **105 km/h**, configurable from **8–145 km/h**.
A fresh SET without a map selects the higher of current speed and that ceiling,
bounded by the normal cruise limits. A map outage while already engaged keeps
the previous SET, and RESUME retains the previous selection. A later explicit
SET during a gap waits for the planner's fresh fallback acknowledgment before
applying its new ceiling; a gap alone cannot request that change.

The top-right button restores a persistent manual E2E selection. E2E ON admits
the model action even with a valid mapped speed, capped by the offset-adjusted
map target and any lower manual SET. That maximum applies even if automatic
SLC is disabled. E2E OFF retains automatic no-map fallback and the existing
qualified stop/slow-lead assists. Mapped recovery applies SET before clearing
automatic fallback in the same card update. Manual E2E stays on across recovery.

## Always-on lateral (AOL)

AOL defaults ON when the setting has never been saved; later choices persist.
Once booted, calibrated and ready in Drive, it can request steering without
engaging cruise. It is supported only on the camera-long GM Sierra/Silverado
configuration carrying the new safety flag and matching Panda firmware.
Older firmware or another safety configuration cannot grant AOL permission.

Holding the brake, regen braking, parking brake, and any gear other than Drive
temporarily pause AOL. Brake release or return to Drive can resume it once the
readiness checks pass. Cancel suspends it until SET/RESUME or a saved OFF-to-ON
toggle. Fault suspension also requires explicit rearming. AOL-only steering
cannot initiate automatic lane changes. Normal lane-change assistance remains
available with cruise engaged, and engaging cruise with an already-held signal
still requires a fresh signal activation before a maneuver can begin.

AOL has separate host intent, Panda acknowledgment and actual steering states.
It never grants longitudinal control. Driver-monitoring, calibration, fresh
model/CAN messages, gear, driver torque, torque/rate limits, heartbeat and fault
checks remain required. Driver alerts continue while AOL steers without cruise.

Painted-lane centering requires two sufficiently confident, plausible lane lines.
Entry requires both line probabilities to remain at least **0.92** for **0.75 s**,
with the fitted width inside **2.70–5.10 m**. An active policy can retain ordinary
widths with both probabilities at least **0.70**. A narrower **2.60–2.70 m** fit
can retain an already-active policy only while both probabilities remain at
least **0.92**; it cannot start centering. A further model-estimate tolerance
can retain already-active centering with median width at least **2.50 m** and
10th-percentile width at least **2.45 m**, but only when both probabilities are
at least **0.92**, both near-field line standard deviations are at most
**0.10 m**, and the robust width spread is at most **0.20 m**. These bounds are
qualification tolerances, not a measurement of the physical road width. Ordinary
widths retain their existing variation check. Every decision uses the current
two painted lines; no old width or boundary is substituted. Missing paint or
geometry outside the applicable bounds immediately returns to E2E.
It retains the bounded curvature correction, fresh-confidence arming, visual
status and curve-reversal pause, which decreases from **0.30 s** at low speed to
**0.05 s** at 100 km/h. Missing paint returns to the model path. Nudgeless lane
changes require a fresh signal activation after engagement; an existing or held
signal cannot initiate another maneuver. Blindspot, minimum-speed and timeout
guards remain.

## Maps, display and parked recording

- SLC retains offsets, acceptance/resume preferences, live maps, optional offline
  packages, scheduled updates and conditional school/construction restrictions.
  Advanced preview is optional. Fresh valid limits apply without an extra
  two-second qualification or warning timer. Green sign pulsing follows the
  controller's actual adjustment. Lower-limit anticipation and the narrow
  highway-merge exception retain their geometry, freshness and distance checks;
  ordinary higher limits still wait for road entry.
- Engaged driving always requests the narrow, zoomed camera view, including
  low-speed and automatic E2E operation. Disengaged driving retains low-speed
  wide-view hysteresis. Camera and overlay crops share calibrated alignment on
  the current upstream CPU renderer.
- The gap button's three bars represent stock personalities: tap to cycle, hold
  to toggle painted-lane centering. The separate top-right button taps between
  Chill and E2E, while a 0.6-second hold toggles AOL. Its green hands require
  fresh, confirmed active steering; ready, off and paused states remain distinct.
  The background stays dark in every state.
  The original comma icons have solid hands with fading forearms and no text
  above or below the control. Brake and blindspot indicators,
  SLC/Sentry controls and the Batmobile appearance are retained.
- Sentry retains parked recording, sensor triggers, parking choices, power
  limits, eye display and upload coordination. Its separate encoder sockets and
  native service registrations are included in the build. While Sentry is
  enabled, its selected voltage cutoff also overrides the system cutoff during
  cooling, skipped parking, expired capture and a missing controller lease.
  Disabling Sentry restores the stock threshold. Forced shutdown and ordinary
  ignition, time and energy-budget protections are preserved.

See [SLC and offline maps](slc-offline-maps.md),
[dynamic speed zones](dynamic-speed-zones.md), and [C3x UI](c3x-ui.md) for details.

## Removed experiments and validation limits

This rebuild excludes custom lead IDs, differentiated-distance lead velocity,
custom future-lead velocity integration and transition-history blending. It also
excludes road-edge centering assistance, learned lane-width carryover, inferred
one-line boundaries and their virtual UI geometry. Raw upstream road-edge model
outputs and standard display remain. Retired schema ordinals are reserved for
recorded-log compatibility; they do not enable the old policies.

Validation includes lane/nudgeless behavior, automatic SLC and slowing-assist
gates, stop-target behavior, native MPC cases, map matching and preview,
UI geometry/state, GM calibration and Sentry regression tests. The integration
audit checks real schema construction, service/parameter registration and the
unchanged upstream lead-processing files. Preserve these checks when updating
upstream; schema and service additions require rebuilding native consumers.

Subsequent recorded drives have been reviewed to diagnose UI and SLC behavior.
The restored cruise limits and handoff change have offline planner replay and
synthetic closed-loop validation; this revision has not yet been validated in a
vehicle. The narrow-paint exception has qualification replay and unit coverage,
not closed-loop lateral validation. These checks do not
establish road performance, model-inference timing, camera transition appearance
or braking improvement. No new road-validation result is implied by retaining
an archived feature.
