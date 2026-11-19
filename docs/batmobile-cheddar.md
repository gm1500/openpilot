# Batmobile on Cheddar

This rebuild starts from upstream openpilot and reapplies the retained Batmobile
features. It uses upstream Cheddar model assets and the current model/runtime
architecture, with native lead processing restored.

## Source lineage and dependencies

| Reference | Revision |
| --- | --- |
| Upstream openpilot base | `1b09aabb6bb6e04820559955ac26f63eebddbee5` |
| Previous Batmobile, archived as `archive/batmobile-pre-cheddar-20261118` | `cebc3f0b11f90fa4d2f5113e3df44762a56ff5df` |
| Upstream model-assets tree, unchanged | `903fd28aef90d787adceb88071fd2ca73ba88924` |
| tinygrad, unchanged | `ec91b74b5b6b0fc924b72d3f27689679ca2438b9` |
| msgq, unchanged | `42554d0a6f1cb2e87238c994760078be36cfdb27` |
| panda, unchanged | `5e8131ec962f6c2d88adda4ec7166e5d308f095e` |
| rednose, unchanged | `e60428b2e52f139d5fe5466d4d9fa3c6adeadad8` |
| teleoprtc, unchanged | `f19ceb4553d1d9ccd46cc86f47483564814423ec` |
| Upstream opendbc base | `5cb4ca34c7abb2a976fb835ba9fca6bf4386ed4c` |
| opendbc with retained GM calibration and tests | `66e8086083e8b09783fffbf044350a2417bcfd83` |

The upstream version remains **0.11.3**, with **AGNOS 20.6**. The lockfile retains
tinygrad **0.14.0**, NumPy **2.5.3**, pycapnp **2.1.0**, and the CPU raylib package
**6.0.0.1.post111**. Map civil-time handling adds timezonefinder **8.2.0** and
tzdata; use the checked-in lockfile and submodule revisions when reproducing the
build. The GM fork carries the retained Sierra/Silverado torque, mass, gas and
safety calibration on the upstream opendbc base.

## Driving behavior

Ordinary mapped-speed operation uses native real-lead MPC and cruise planning.
The stock personalities remain **Aggressive: 1.25 s**, **Standard: 1.45 s**, and
**Relaxed: 1.75 s**, with their upstream jerk settings. `radard.py` and
`longcontrol.py` remain byte-for-byte upstream. Vision leads use upstream model
velocity with its ego-speed correction; native radar association and filtering
remain available where the vehicle supplies radar.

The narrow slowing assist can enter only when the model's current lead-speed
estimate is **below 55 km/h**, with a present accepted lead and sustained model
braking. Distance and native lead data still provide proximity and release
checks. The assist does not replace lead velocity or relax the regular MPC's
braking. Once entered, a brief speed estimate above 55 km/h does not itself
release the assist. Qualified model stopping intent can also activate the
retained stop target, with real-lead overlap checks and existing release rules.

When enabled SLC has no usable mapped limit, its request automatically admits
the native model acceleration and curve-speed result into planning. The initial
no-map cruise setting defaults to **105 km/h**, configurable from **8–145 km/h**.
A later map outage waits for the planner's fallback acknowledgment before raising
SET; mapped-speed recovery retains the coordinated handoff. There is no manual
Chill/E2E/CEM mode cycle.

Painted-lane centering requires two sufficiently confident, plausible lane lines.
It retains the bounded curvature correction, fresh-confidence arming, visual
status and curve-reversal pause, which decreases from **0.30 s** at low speed to
**0.05 s** at 100 km/h. Missing paint returns to the model path. Nudgeless lane
changes require a fresh signal activation after engagement; an existing or held
signal cannot initiate another maneuver. Blindspot, minimum-speed and timeout
guards remain.

## Maps, display and parked recording

- SLC retains offsets, acceptance/resume preferences, live maps, optional offline
  packages, scheduled updates and conditional school/construction restrictions.
  Advanced preview is optional. A fresh candidate can flash its posted limit
  green before qualification completes; displaying it does not change SET.
  Lower-limit anticipation and the narrow highway-merge exception retain their
  geometry, freshness and qualification checks.
- Engaged driving always requests the narrow, zoomed camera view, including
  low-speed and automatic E2E operation. Disengaged driving retains low-speed
  wide-view hysteresis. Camera and overlay crops share calibrated alignment on
  the current upstream CPU renderer.
- The gap button's three bars represent stock personalities: tap to cycle, hold
  to toggle painted-lane centering. Automatic no-map operation shows the E2E
  icon and OVERRIDE. Brake and blindspot indicators, SLC/Sentry controls and the
  branch-specific Batmobile appearance are retained.
- Sentry retains parked recording, sensor triggers, parking choices, power
  limits, eye display and upload coordination. Its separate encoder sockets and
  native service registrations are included in the build.

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

The rebuilt branch has **not** completed an on-device drive or a full driving-model
replay. Unit tests, synthetic planner cases and geometry checks do not establish
road performance, model-inference timing, camera transition appearance or braking
improvement. No new road-validation result is implied by retaining an archived
feature.
