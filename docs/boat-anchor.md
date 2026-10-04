# boat-anchor-slc

Based on upstream openpilot master `c8fb906815530460ed156f14e09e1f312bb0f851`,
the same base as the working boat-anchor map-cruise build. Custom history is
grouped into five feature commits: **Sierra Tune**, **Nudgeless**, **Lead velocity fix**,
**Lane policy**, and **Map speed limit**, with follow-ups for advisory speeds,
road tracking and SET behavior.

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
| Short SET or SET engagement | Immediately select the higher of the fresh, heading-matched map limit and rounded current driving speed, within cruise bounds. A legal limit takes priority; advisory is the fallback when no legal limit exists. Like the ordinary 105 km/h experimental-mode default, the map provides the initialization floor. Explicit SET does not wait for the automatic two-second debounce. |
| SET while driving above the map limit | Hold the selected driving speed until the next confirmed different map limit or another SET. Qualification of the same limit cannot immediately undo that selection. |
| SET with neither legal nor advisory data | Arm tracking and retain ordinary engagement initialization as the fallback. A held sign cannot supply a target. |
| RES engagement | Restore the previous cruise speed. |
| RES/+ while engaged | Adjust normally and hold the manual target until the next confirmed different limit or SET. |
| Long +/- | Retain repeated 5 km/h or 5 mph steps. Manual hold/release keeps priority over a concurrent map change. |
| New limit while map mode is on | Automatically update after two seconds of stable, fresh data with advancing GPS timestamps. |
| Missing or ambiguous map data | Keep the existing cruise target. Hide the sign's number when it is not usable by explicit SET. |
| Advisory speed on the matched road | If no legal limit exists, display the fresh recommendation on yellow with no MAXIMUM/SPEED LIMIT heading and use it as the map input to SET. |
| Toggle off | Hold the existing cruise target and restore ordinary SET/RES behavior. |
| Accelerator pressed | Defer automatic target updates. Explicit SET still selects the higher of the map limit and current driving speed. |

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
an otherwise usable legal limit. The sign and SET share the same selection:
a usable legal limit always wins. An advisory is shown only when the legal limit
is absent, so the sign does not advertise a different map input from SET.
Advisory-only roads use the fresh matched recommendation as a fallback target,
with the same two-second automatic qualification, immediate explicit SET,
manual override and RES behavior as legal limits. The sign stays yellow.
The fallback never reads a legacy display value or borrows a neighbouring road's limit.
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
removed. Model-path curvature is not used.

Fresh CAN standstill retains the last approach heading and confirmed road through
a stop. GPS must continue to supply fresh positions; drift is bounded to 12 m from
a fixed stop anchor, with the accepted road within 15 m. Turning the wheel or noisy
GPS course at rest cannot choose a new road. Movement resumes normal ambiguity checks.

Measured `controlsState.curvature` supplies steering evidence using the calibrated
vehicle model, including steering ratio, offset, stiffness, wheelbase and roll.
It is not commanded steering and does not require a wheel-radius parameter.
Integrated motion supplements fresh GPS course; disagreement above 25 degrees
discards the steering estimate. A low-speed heading fallback is bounded to 20 m,
45 degrees of turn and ten accumulated moving seconds. Time stopped does not consume
the departure allowance. Standstill retention can continue
with fresh GPS and bounded drift. Invalid CAN, stale inputs, reverse gear or missing
GPS clear the fallback; missing calibration while moving leaves ordinary GPS matching.
GPS positions and timestamps are never extrapolated or refreshed by steering.

A nearby different-limit road still requires geometric separation. The tracked
road cannot win merely because it was selected previously: a closer competing road,
an unresolved fork, a wrong-way heading, implausible progress, changed geometry or
expired history can clear the match. Unrecognized or missing limits remain unknown.
Ambiguous and repeated GPS fixes do not refresh accepted-history age.

When a shallow ramp fork outlives confirmed history, two fresh, moving fixes can
reacquire the same or a connected road. Both must favour it by at least 6 m over
conflicting alternatives, be within 8 m of its geometry and within 25 degrees of
its direction. Confirmation requires at least 5 m of movement over 0.5–3 seconds;
faster GPS feeds accumulate evidence from the initial tentative fix. When steering
agrees within 12 degrees and separates every conflicting direction by at least
another 15 degrees, the reacquisition margin can fall from 6 to 3 m. During confirmed
tracking it can fall from 3 to 1.5 m after at least 5 m of GPS movement. The closest
road must still win, with directed motion and shared-node connectivity. Equal forks,
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

The sign and SET share control validity, heading checks, 8–145 km/h bounds,
0.8 s publisher/receiver freshness and 3 s GPS freshness. There is no display-only
hold after control eligibility is lost. A bounded 0.3 s/60 m processing handoff
retains only the preceding sample's original timestamp; an ambiguous result clears
both the sign and the available map target. Explicit SET can use a fresh match
before the two-second automatic qualification finishes. Missing map data cannot
engage cruise or launch the vehicle.

This is road matching, not advance planning for future speed zones. A lower
limit is applied after matching the new zone; the branch does not guarantee
reaching it before the roadside sign. OSM tags can differ from posted restrictions.

Map data: © [OpenStreetMap contributors](https://www.openstreetmap.org/copyright),
under the [Open Database License](https://opendatacommons.org/licenses/odbl/).
Device attribution is under Settings > Software > Map Data.

## Validation

- 78 map/cruise regressions cover immediate SET, unchanged RES and long holds,
  speed-convergence feedback, intervention cancellation, message serialization,
  curved-road continuity, connected/internal junctions, ramp departure, unknown
  roads, one-way/directional limits, cache replacement and stale/frozen GPS,
  independent legal/advisory parsing, fork reacquisition, on-ramp merges,
  fast GPS feeds, legal-priority advisory fallback, ready-only display, accelerator
  override, the current-speed floor, stopped-road retention and steering evidence.
- Recorded-input replay of route `000002b0--29d880bc44`, segment 1, reproduces all
  5,977 recorded cruise values with the preceding helper. The first SET changes
  from 40 to 60 km/h with this patch; the next two remain 60. The map was already
  ready: accelerator override at engagement caused the old 40 km/h initialization.
  This replay holds recorded vehicle inputs fixed; it does not predict acceleration.
- At 5 Hz, heading replay retains direction for all 225 stopped samples in route
  `000002af--2171aedb58`, segment 3, and all 85 in `000002b0--29d880bc44`, segment 1.
  The preceding GPS-only input has no heading for those samples. This establishes
  heading availability, not route-level map matching or correct ramp selection.
- Production UI and mapd functions pass isolated checks with real serialized
  messages, synthetic road fixtures and mocked transport. They cover legal/advisory
  agreement between display and SET, stale/invalid inputs, and moving-to-stopped
  retention with small negative filtered ego speeds. Lint and whitespace pass.
- Five upstream cruise-helper cases pass with hardware/manager bootstrap isolated.
  The full upstream maneuver suite was not run; its environment dependencies are
  unavailable here.

Ramp validation uses synthetic geometry and tags. Raw OSM geometry for the latest
route locations was unavailable because live lookups failed, so end-to-end road
matching there remains unverified. Native device appearance, transport and
real-road performance were not revalidated for this follow-up.
