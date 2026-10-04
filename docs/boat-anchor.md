# boat-anchor-slc

Based on upstream openpilot master `c8fb906815530460ed156f14e09e1f312bb0f851`,
the same base as the working boat-anchor map-cruise build. Custom history is
grouped into five commits: **Sierra Tune**, **Nudgeless**, **Lead velocity fix**,
**Lane policy**, and **Map speed limit**.

## Vehicle tuning

The working coast100 tune is preserved exactly, including opendbc commit
`9fedf640ae1a9d807e902d3a79bfa59672a58617`. The Sierra/Silverado profile retains
the wheel-torque controller, 2,586 kg including standard cargo, 0.419 m tire
radius, drag area 0.30 × 3.61 m², and rolling coefficient 0.004. Pitch compensation
is not enabled. Longitudinal actuator delay remains 0.5 s, with the existing
launch-integrator hold and final-stop brake calibration.

Current Ki is 0.050 at 40–60 km/h, blending from the existing lower-speed values
and back to 0.001875 at 90 km/h. The coast allowance is up to 100 Nm, phased in
over 18–40 km/h and removed as modeled friction demand rises from 0.1 to
0.3 m/s². Stopping bypasses that allowance. Brake-active torque remains -540 Nm.
This branch does not retune propulsion, braking, gains or delays.

Vehicle parameters remain fingerprint-specific. Shared non-GM wheel-radius
defaults and the prior initialization fixes are retained.

## Nudgeless

The existing nudgeless lane-change behavior is carried over unchanged.

## Lead velocity fix

For accepted vision leads, relative-range history and ego motion estimate lead
speed through a private Kalman tracker. Model velocity remains the fallback
while the range-derived estimate matures. Output transitions through a bounded
pre-closing cue before the range-derived estimate takes over. Normal transitions
blend; urgent braking bypasses the blend. Low-speed closing and stopped-lead
handling retain their model-based constraints.

The mild range correction and mature-track confidence shaping remain. Physical
radar output and stock lead acceptance are unchanged. Planner costs, constraints,
following-distance policy and stopping policy are unchanged.

## Lane policy

The existing policy fits the inner-lane midpoint against the E2E path and adds a
bounded curvature correction over their shared forward horizon. Two confident
inner lines arm it; a learned lane width supports a brief one-line hold.
Invalid or missing geometry, lane-change intent and blinkers return E2E steering.
The per-drive lane toggle, blending, correction limits and learner are preserved.

Both comma 3/3X and comma 4 use the shared policy and compatible HUD controls.

## Map speed limit

The default-on sign toggle enables map cruise on non-PCM platforms with openpilot
longitudinal control. Stock PCM, lateral-only, passive and non-car platforms keep
a display-only sign.

| Action or event | Behavior |
| --- | --- |
| Short SET or SET engagement | Immediately select the current fresh, heading-matched numeric limit. Explicit SET does not wait for the automatic two-second debounce. |
| SET with no usable map match | Arm tracking and retain ordinary engagement initialization as the fallback. A held advisory sign cannot supply a target. |
| RES engagement | Restore the previous cruise speed. |
| RES/+ while engaged | Adjust normally and hold the manual target until the next confirmed different limit or SET. |
| Long +/- | Retain repeated 5 km/h or 5 mph steps. Manual hold/release keeps priority over a concurrent map change. |
| New limit while map mode is on | Automatically update after two seconds of stable, fresh data with advancing GPS timestamps. |
| Missing or ambiguous map data | Keep the existing cruise target. The sign may briefly hold its last confirmed value, then show a dash. |
| Toggle off | Hold the existing cruise target and restore ordinary SET/RES behavior. |
| Accelerator pressed | Defer automatic target updates; SET retains the existing current-speed floor. |

The larger units, numeric cruise target, thicker green active/ready border and
pronounced 1 Hz green pulse are shared by both screen sizes. MAP ON/READY indicates
tracking/readiness, MAP WAIT means no qualified automatic target, MAP HOLD means
a manual target is being held, and MAP OFF means manual cruise.

The pulse starts for an automatic change and continues after the set speed
updates until measured vehicle speed (`carState.vEgo`) stays within ±1 km/h of
the selected target for one second. Speed-button presses, accelerator or brake,
disengagement and map-off cancel it. It stays cancelled/completed for that target;
ordinary speed drift cannot restart it. Explicit SET is a driver action and
cancels an existing pulse. A later automatic limit change can start a new pulse.
The feedback state is observational and does not change planner or actuator output.

### Persistent road tracking

The matcher retains the accepted way, direction and progress along its geometry.
Fresh position changes are checked against the mapped curve. At connected ways,
bounded forward traversal follows shared OSM node IDs, including junctions inside
a way and short intervening segments. Coordinate coincidence alone cannot connect
a bridge to a road beneath it. The obsolete compass-only continuity exception is
removed. Model-path curvature is not needed for this implementation.

A nearby different-limit road still requires geometric separation. The tracked
road cannot win merely because it was selected previously: a closer competing road,
an unresolved fork, a wrong-way heading, implausible progress, changed geometry or
expired history can clear the match. Unrecognized or missing limits remain unknown.
Ambiguous and repeated GPS fixes do not refresh accepted-history age.

Overpass supplies way IDs, node references, tags and geometry using `out body geom`.
The cache remains in memory: 1.5 km query radius, prefetch after 700 m, ten-minute
expiry, at most one request every 30 seconds, bounded response size and error
backoff. Cache replacement and process exit release old data; no growing map
database or route archive is written. Requests contain coordinates but no device
identifier or route history. Matching and network access remain outside the
control loop.

Control samples and advisory display holds have separate validity and timestamps.
The existing 0.8 s publisher/receiver freshness, 3 s GPS freshness and bounded
2 s/60 m advisory hold remain. A short processing handoff retains only the
preceding sample's original timestamp; an ambiguous result cannot become fresh
control data. Missing map data cannot engage cruise or launch the vehicle.

This is road matching, not advance planning for future speed zones. A lower
limit is applied after matching the new zone; the branch does not guarantee
reaching it before the roadside sign. OSM tags can differ from posted restrictions.

Map data: © [OpenStreetMap contributors](https://www.openstreetmap.org/copyright),
under the [Open Database License](https://opendatacommons.org/licenses/odbl/).
Device attribution is under Settings > Software > Map Data.

## Validation

- 60 map/cruise regressions cover immediate SET, unchanged RES and long holds,
  speed-convergence feedback, intervention cancellation, message serialization,
  curved-road continuity, connected/internal junctions, ramp departure, unknown
  roads, one-way/directional limits, cache replacement and stale/frozen GPS.
- In a synthetic curved-road case with lagging GPS bearing and a nearby different
  limit, confirmed matches improve from 4/17 to 17/17 and visible-sign samples
  from 6/17 to 17/17. Neither version selects the wrong limit in that fixture.
  These figures describe the synthetic test, not a measured driving improvement.
- A real mapd/messaging test with synthetic 1 Hz GPS and a local road fixture
  selects 50 km/h. Stopping mapd holds that target and reports MAP WAIT.
- Isolated production UI tests check pulse lifecycle, stale status and touch/alert
  cancellation. Software layouts, schema generation, lint and whitespace pass.
- Five upstream cruise-helper cases pass with hardware/manager bootstrap isolated.
  The full upstream maneuver suite was not run; its environment dependencies are
  unavailable here.

No new route coordinates were sent to a map service during validation. Native
device appearance and real-road performance still require verification.
