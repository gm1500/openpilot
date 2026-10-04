# boat-anchor-e2e-assist-poc

POC based on `boat-anchor-slc` at `bc08097535715ec1981ee1deafb575a04ad96d96`.
The inherited features below remain, with the E2E slowing-assist POC and focused
follow-ups for cut-in velocity history and assistance/release behavior.

Based on upstream openpilot master `c8fb906815530460ed156f14e09e1f312bb0f851`,
the same base as the working boat-anchor map-cruise build. Custom history is
grouped into five feature commits: **Sierra Tune**, **Nudgeless**, **Lead velocity fix**,
**Lane policy**, and **Map speed limit**, with follow-ups for advisory speeds,
road tracking and SET behavior.

## E2E slowing assist POC

The assist can enter in this branch when openpilot longitudinal control is active,
Experimental Mode is off, a valid accepted lead is present, and planner input
health checks pass. Gas/brake intervention, disengagement or invalid input clear
its state immediately. A healthy message reporting lead loss instead releases
the previous constraint gradually. It does not enable Experimental Mode or change
the cruise target, SLC selection, SET/RES, torque tune, lane policy or nudgeless.
Stock ACC and lateral-only operation do not activate it.

| Condition | POC behavior |
| --- | --- |
| Entry below 65 km/h | E2E requests at most -0.2 m/s² for three model cycles (0.15 s), with a present lead and nonzero relevance. |
| Slowing assistance | Scale E2E deceleration from zero to full influence according to gap and closing motion. Select the more restrictive of this constraint and regular acceleration. Stronger regular braking always wins. |
| E2E wants to go | Above +0.1 m/s² with `shouldStop` false for four cycles (0.2 s) releases the latch. |
| Lead pulls away | At speed above 2 m/s, require 0.5 s of continuous positive lead relative speed above 0.5 m/s, range growth above 0.25 m, and gap above 6 m + 1.5 s. E2E above -0.1 m/s², regular acceleration above +0.1 m/s² and no stop request allow the same 0.2 s release confirmation. Range jumps clear this evidence. |
| Distant, no longer closing | Zero relevance plus E2E above -0.1 m/s² and no stop request permits confirmed release. Low relevance alone cannot restore propulsion during a latched strong E2E slowdown. |
| Handoff | Increase the constraint by at most 1.0 m/s³ toward regular acceleration. Stronger braking is applied immediately after entry. |
| Lead loss | Do not enter on absent leads or consume new E2E braking commands. Release an existing constraint through the same handoff; pedal intervention and disengagement still clear immediately. |
| Crossing 65 km/h | Release through the same handoff; re-arm below 63 km/h to avoid boundary chatter. |
| Standstill | Preserve all regular stop requests; add E2E `shouldStop` while latched. A fresh model stop request below 0.3 m/s retains an already latched stop through lead loss. Existing stop-hold and resume logic remain in control. |

Proximity influence falls continuously from full at `6 + 2.0*vEgo` metres to
zero at `6 + 3.5*vEgo`. Closing influence rises from zero at 0.2 m/s² to full at
0.6 m/s² of `(vEgo² - vLead²) / (2 * max(dRel - 6, 1))`, clipped nonnegative.
Use the larger influence; a fresh E2E stop request retains full influence.
Increasing urgency takes effect immediately after entry, while influence falls
by at most one per second. These weights do not modify MPC following distances.
While latched, the weighted constraint remains nonpositive until release.

During a latched episode, weak/brief positive model requests cannot immediately
restore propulsion: the constraint approaches zero until go intent is confirmed.
When released, regular acceleration resumes through the handoff. Positive E2E
requests never limit ordinary catch-up when the assist has not been entered.
These thresholds are provisional POC values, not calibrated stopping guarantees.
The lead gate does not prove why E2E requests braking; it can also react to other
scene features. No separate E2E lead detector or learned stop-intent flag is added.

`longitudinalPlan.e2eAssistActive` reports when the constraint actually lowers
`aTarget`; the selected source is then `e2e`. On both comma 3/3X and comma 4,
the path is yellow/orange/red according to that selected negative acceleration,
using the experimental hue scale. The path geometry and lane colours are unchanged.
Regular control, positive handoff acceleration, stale data or override restore the
ordinary path colours. This reports a deceleration request, not measured braking
or friction-brake engagement. Full Experimental Mode retains its original behavior.

### Offline validation

Recorded-input selector replay of route `615b11d4c01d81ec/000002b0--29d880bc44`,
segments 1 and 5 (2,400 planner frames), preserved every regular stop flag and
never requested more acceleration than the regular recorded request.

| Approach | Regular request below -0.5 m/s² | POC request below -0.5 m/s² | Earlier |
| --- | --- | --- | --- |
| Segment 1, completed stop | 32.437 s | 30.537 s | 1.900 s |
| Segment 5, driver intervention | 27.743 s | 25.193 s | 2.550 s |

Times are relative to each segment's first carState sample; crossings must persist
for three samples. The segment 1 search starts at 28 s to exclude a separate
earlier deceleration. Segment 5's recorded launch from 52–59 s was unchanged.
The assist also added more than 0.2 m/s² of slowing for 7.9 s in segment 5's first
25 s of following. Scene review and driving feedback are needed to judge whether
that extra slowing is helpful; this is not evidence of reduced phantom braking.

The refinement replay also covered all 14,400 planner frames from route
`000002b1--e670ceec8d` segments 1–12, comparing selectors on identical recorded
regular-planner, model and lead inputs:

| Case | Original POC → refined POC |
| --- | --- |
| Segment 10, first 11 s, far lead | Integrated added negative acceleration request falls 19.1%. This is not a measured reduction in friction-brake events. |
| Segment 2, lead pulls away | Selected acceleration exceeds +0.1 m/s² at 37.039 s instead of 41.040 s: 4.001 s earlier. |
| Segment 7, lead disappears at 15.607 s | Old selector jumps from -1.011 to +1.449 m/s². Refined selector changes from 0.000 to +0.050 m/s² and continues its bounded release until driver braking clears it. Earlier requests also differ because of relevance weighting. |
| Completed stops, segments 1 and 8 | -0.5 m/s² onset unchanged. Segment 1's -1.0 m/s² crossing is one 50 ms cycle later; segment 8's is unchanged. Final five seconds of each tested stop window are identical. No assist stop flags are lost anywhere in the 12 segments. |

The largest reductions in requested braking within those two stop windows were
0.201 m/s² (one frame above 0.2) and 0.063 m/s². Their integrated added negative
requests differ by less than 1%. A lead-loss handoff can temporarily retain more
slowing than the original POC, which immediately returned to regular acceleration.
All replayed selections remain at or below the regular request. The estimator
replay described below confirms unchanged lead inputs throughout this 12-segment
route. Both replays hold recorded vehicle/model feedback fixed.

Validation also passed 29 focused assist/UI/lead-handoff tests, 38 existing map-cruise tests,
and 330 isolated production-planner comparisons across 11 exclusion conditions.
Production publication/schema round-trip and both renderer draw paths were checked.
The production planner also passed a lead-loss release and immediate-override check.
The native MPC solver, messaging transport and graphics backend were isolated for
the integration checks. A full native build, rendered-device check and closed-loop
vehicle/model simulation were not performed. Replaying recorded inputs does not
predict a changed stopping distance or establish that the intervened stop is fixed.

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

A gradual cut-in can survive per-frame association and mix the old and new car's
range history. A sustained mismatch now re-anchors that history at the current
distance and immediately uses the current model velocity, without an old-speed
bridge. This guard requires a gap above both 30 m and 1.5 s, confidence at least
0.9, model closing below 3 m/s, and model acceleration at least -0.15 m/s².
Both the private velocity and the range slope's upper two-sigma speed must be
more than 5 m/s below the model for 0.25 s, using at least 0.6 s of range history.
Close, stopped and model-braking leads retain their existing constraints.
Fresh history follows the existing bounded pre-closing cue, 1.5 s minimum maturity,
uncertainty check and one-second transition; these are not a delay in accepting
the new distance or responding to model braking.

An 18,000-frame estimator replay covered route `000002a9--f6fcae3435` segment 85,
route `000002b1--e670ceec8d` segments 1–12 and route `000002b0--29d880bc44`
segments 1 and 5. The cut-in at 14.390 s publishes 99.2 km/h instead of 62.6 km/h,
at 47.4 m instead of 47.6 m; the original model estimate there is 100.3 km/h.
The other 14 segments' published lead velocities are unchanged. This verifies
the estimator change on recorded inputs, not a changed braking or vehicle-speed
trajectory. Shared range-slope calculations were consolidated without changing
their existing results.

The mild range correction and mature-track confidence shaping remain. Physical
radar output and stock lead acceptance are unchanged. MPC costs, constraints and
following-distance policy are retained. The POC assist above can further lower
the final acceleration request and add a stop request alongside the MPC.

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
