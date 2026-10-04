# boat-anchor-slc

Based on upstream openpilot master `c8fb906815530460ed156f14e09e1f312bb0f851`,
the same base as the working boat-anchor map-cruise build. Custom history is
grouped into five feature commits: **Sierra Tune**, **Nudgeless**, **Lead velocity fix**,
**Lane policy**, and **Map speed limit**, with follow-ups for advisory speeds and
ramp reacquisition.

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
| Short SET or SET engagement | Immediately select the current fresh, heading-matched legal limit, or advisory if no usable legal limit exists. Explicit SET does not wait for the automatic two-second debounce. |
| SET with neither legal nor advisory data | Arm tracking and retain ordinary engagement initialization as the fallback. A held sign cannot supply a target. |
| RES engagement | Restore the previous cruise speed. |
| RES/+ while engaged | Adjust normally and hold the manual target until the next confirmed different limit or SET. |
| Long +/- | Retain repeated 5 km/h or 5 mph steps. Manual hold/release keeps priority over a concurrent map change. |
| New limit while map mode is on | Automatically update after two seconds of stable, fresh data with advancing GPS timestamps. |
| Missing or ambiguous map data | Keep the existing cruise target. The sign may briefly hold its last confirmed value, then show a dash. |
| Advisory speed on the matched road | Display the recommendation on yellow with no MAXIMUM/SPEED LIMIT heading. Cruise uses it only when no usable legal limit is available on that road. |
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

Numeric `maxspeed:advisory` tags, including forward/backward variants and mph,
are read separately from legal `maxspeed` tags. An advisory no longer invalidates
an otherwise usable legal limit. A lower advisory takes display priority; if
the legal limit is equal or lower, the legal sign remains. For cruise targets,
a usable legal limit always wins, even when the displayed advisory is lower.
Advisory-only roads use the fresh matched recommendation as a fallback target,
with the same two-second automatic qualification, immediate explicit SET,
manual override and RES behavior as legal limits. The sign stays yellow.
The fallback never reads a held display value or borrows a neighbouring road's limit.
Unsupported conditional,
variable or lane-specific qualifiers remain unavailable within their own tag family.
The yellow background stays steady while an advisory is shown, even if a separate
legal-limit adjustment is in progress. The actual set speed remains below the sign.

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

When a shallow ramp fork outlives confirmed history, two fresh, moving fixes can
reacquire the same or a connected road. Both must favour it by at least 6 m over
conflicting alternatives, be within 8 m of its geometry and within 25 degrees of
its direction. Confirmation requires at least 5 m of movement over 0.5–3 seconds;
faster GPS feeds accumulate evidence from the initial tentative fix. Equal forks,
stationary/frozen fixes and disconnected jumps do not qualify. Advisory differences
also count as conflicts, even when the two roads have the same legal limit. This
prevents a ramp recommendation from appearing merely because the adjacent motorway
shares its legal maximum. Ramp recommendations end when the matched road changes;
they are not inherited onto untagged ramps or the mainline after merging.

Overpass supplies way IDs, node references, tags and geometry using `out body geom`.
The cache remains in memory: 1.5 km query radius, prefetch after 700 m, ten-minute
expiry, at most one request every 30 seconds, bounded response size and error
backoff. Cache replacement and process exit release old data; no growing map
database or route archive is written. Requests contain coordinates but no device
identifier or route history. Matching and network access remain outside the
control loop.

Control samples and display holds have separate validity and timestamps.
The existing 0.8 s publisher/receiver freshness, 3 s GPS freshness and bounded
2 s/60 m display hold remain. A short processing handoff retains only the
preceding sample's original timestamp; an ambiguous result cannot become fresh
control data. Missing map data cannot engage cruise or launch the vehicle.

This is road matching, not advance planning for future speed zones. A lower
limit is applied after matching the new zone; the branch does not guarantee
reaching it before the roadside sign. OSM tags can differ from posted restrictions.

Map data: © [OpenStreetMap contributors](https://www.openstreetmap.org/copyright),
under the [Open Database License](https://opendatacommons.org/licenses/odbl/).
Device attribution is under Settings > Software > Map Data.

## Validation

- 70 map/cruise regressions cover immediate SET, unchanged RES and long holds,
  speed-convergence feedback, intervention cancellation, message serialization,
  curved-road continuity, connected/internal junctions, ramp departure, unknown
  roads, one-way/directional limits, cache replacement and stale/frozen GPS,
  independent legal/advisory parsing, fork reacquisition, on-ramp merges,
  fast GPS feeds, legal-priority advisory fallback and held-display/control separation.
- In a synthetic curved-road case with lagging GPS bearing and a nearby different
  limit, confirmed matches improve from 4/17 to 17/17 and visible-sign samples
  from 6/17 to 17/17. Neither version selects the wrong limit in that fixture.
  These figures describe the synthetic test, not a measured driving improvement.
- Real mapd/messaging tests with synthetic 1 Hz GPS and local road fixtures select
  a legal 50 km/h target over a 30 km/h advisory, then separately select 30 km/h
  when only the advisory exists. Stopping mapd holds the target and reports MAP WAIT.
- Isolated production UI tests check pulse lifecycle, stale status and touch/alert
  cancellation, yellow advisory signs without a heading, metric/imperial display
  and both screen sizes. Software layouts, schema generation, lint and whitespace pass.
- Five upstream cruise-helper cases pass with hardware/manager bootstrap isolated.
  The full upstream maneuver suite was not run; its environment dependencies are
  unavailable here.

Ramp validation uses synthetic geometry and tags. The raw OSM tags at the
bookmarked offramp in route 2ad remain unverified because the live lookup failed.
Native device appearance and real-road performance still require verification.
