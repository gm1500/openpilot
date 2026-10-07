# Conditional experimental junction POC

Branch: `boat-anchor-conditional-e2e-poc`, based on `boat-anchor` at
`ec919f529c63cf324517602199513689441357af`. The opendbc pin and vehicle tune are unchanged.

The on-road button cycles **VOACC → conditional → full experimental → VOACC**.
VOACC retains the existing lead slowing assistance. Conditional adds a junction
attention window in which sustained model slowing can contribute. Full
experimental retains its existing planner behavior. The selection persists;
experimental confirmation and longitudinal capability checks still apply.

## HUD

The approved icon retains its existing size, colours and thick rings, without text.

| Appearance | Meaning |
| --- | --- |
| White glyph | Conditional selected; no qualified junction in range |
| Cyan glyph and ring | A connected intersection, split or merge is in range |
| Amber glyph and ring | The junction helper contributes to the selected slowing/stop command |

Cyan does not require model slowing, engagement, or an unpressed brake pedal.
Driver intervention or invalid model data immediately clears assistance while
preserving qualified map awareness. Lead-only E2E assistance does not turn this
icon amber. Stale planner/state messages clear the indication.

## Range from current SET speed

Use the current valid `carState.vCruise` in km/h, converted to m/s. The range is
calculated at that speed with **zero assumed acceleration**, using the selected
personality and vehicle actuator delay:

1. One second of qualification/response allowance plus actuator delay.
2. A gradual acceleration ramp into comfortable braking.
3. Constant comfortable braking to zero speed.
4. A 12 m allowance, with a minimum activation distance of 20 m.

| Profile | Comfortable deceleration | Transition jerk | SET 50 km/h | SET 60 km/h | SET 100 km/h |
| --- | ---: | ---: | ---: | ---: | ---: |
| Aggressive | 2.2 m/s² | 1.2 m/s³ | 89 m | 115 m | 254 m |
| Standard | 1.8 m/s² | 1.0 m/s³ | 99 m | 129 m | 293 m |
| Relaxed | 1.4 m/s² | 0.8 m/s³ | 114 m | 151 m | 353 m |

Examples use a 0.5 s actuator delay. Slowing, braking or stopping does not shrink
this entry range while SET is unchanged. Changing SET or personality recalculates
it. An already armed approach remains latched through a reduction in SET until
its normal release conditions occur. The planner's temporary `forceDecel` speed
request does not replace the driver's SET value for map awareness.

Unset, zero, negative, nonfinite or out-of-range SET values do not arm the helper.
The 255 km/h unset sentinel is rejected before conversion, not clamped into a
valid SET. Current vehicle speed still informs model-intent and standstill checks;
SET sizes the attention window only. Neither this calculation nor map proximity
changes actuator limits or commands a stop.

## Targets from road connectivity

Stop/light tags are no longer queried or interpreted by this policy. The OSM
request fetches drivable ways, node IDs and geometry. Road type and one-way tags
still establish drivable roads and travel direction; speed-limit tags still serve
the separate speed-limit feature.

A node connected to **at least three distinct neighbouring road nodes** is a
junction target. This includes T junctions, crossroads, shallow splits, slip roads,
exit ramps and merges, including an incoming merge with only one forward exit.
Service-road connections count. No road-name, angular-continuation or traffic
control classification is required to identify the junction.

Two-neighbour way boundaries, bends and speed-tag changes are not intersections.
Duplicate edges do not add arms. A shared coordinate without a shared node ID
does not connect an overpass, underpass or nearby road. A standalone stop/light
or pedestrian crossing with no road branch is not independently targeted.

Measure distance along the matched directed road to the first junction, following
unique continuations across ordinary way boundaries, up to 1,000 m and 16 ways.
The junction itself is the target: the lookup does not choose a turn beyond a
fork. Missing connectivity and conflicting plausible road matches remain rejected.

Map-message and real-GPS freshness checks remain. A brief projected position is
allowed only while its actual GPS anchor is at most three seconds old. Two
advancing GPS observations of the same target are required before arming;
projection cannot manufacture confirmations. A restored parking checkpoint
without fresh GPS cannot arm the helper.

Release on a missing/changed target, lost map validity, or passage more than 12 m
beyond the target. A passed-target latch resists rearming from position jitter.
Closely spaced junctions can briefly clear cyan while the new target confirms.
Road-match dropouts still clear it. This change does not modify the existing
position estimator or its output selection between GPS updates.

## Model confirmation and arbitration

The shortened rendered path alone is not stopping evidence. Use the model's
actual desired acceleration, stop request, predicted velocity and position.

Entry requires a sustained trajectory speed reduction, model acceleration at or
below −0.2 m/s² and at least 0.15 m/s² below the ordinary boat-anchor proposal,
confirmed for 0.25 s. Speed magnitude avoids interpreting a turn's reduced forward
component as slowing. Predicted stop distance remains logged, but is no longer
required to end before the map target: a split/merge is an attention landmark,
not a stop line, and the light or queue may lie beyond it.

Choose the lower of the ordinary and accepted junction E2E acceleration. Stronger
ordinary braking wins. A sustained model go request releases assistance; the
return toward acceleration is softened at 1 m/s³. Conditional never sets the
full `ExperimentalMode` flag.

Driver override, disengagement, invalid model data and leaving conditional mode
clear assistance. Map loss while moving releases the extra constraint. A stop
already established may remain held through map loss while a fresh, valid model
continues requesting a stop at rest; map loss cannot create a stop request.

## Logging and validation

The existing `mapTrafficControl` service carries `kind=junction`, target node/way,
signed distance, matched road, actual GPS age, projection status and lookup reason.
Legacy stop/light enum values remain readable in old logs but cannot arm this
new policy. `longitudinalPlan.conditionalExperimental` additionally logs the valid
SET speed used as `activationSpeed` in m/s, alongside activation distance, target
distance, model intent, both acceleration proposals, arming and contribution.

Validation: **192 tests and 51 subtests passed**, plus lint and an isolated test
of actual planner update/publish and Cap'n Proto serialization. The map tests use
the Python reference transformations in place of the unavailable native extension.
Tests cover untagged intersections, shallow forks, merges, ordinary way boundaries,
bridges, duplicate edges, parallel-road ambiguity, freshness, SET/profile changes,
standstill, invalid SET sentinels, model confirmation and stronger ordinary braking.

Recorded-input replay covers 11 segments and 13,178 planner frames using a
city-wide OSM download, locally cropped for replay. All three newest bookmarks
are cyan. The formerly late first approach arms about 12.2 seconds earlier and
permits 82 frames (4.1 s) of additional model-requested slowing before driver
brake override, up to about 1.07 m/s² below the ordinary proposal. Other replayed
segments have no selected-acceleration changes. The target is now the nearest
road junction, so distances are not directly comparable with previous distances
to tagged signals. Brief map/target-confirmation gaps remain.

Replay supplies the map snapshot directly rather than emulating original cache
or network timing, and applies logged vehicle/model inputs rather than simulating
the vehicle's response to changed braking. These are offline results, not a full
device build or a demonstrated stop. Map proximity does not establish right of
way or guarantee that the model sees a light, stop sign or conflicting traffic.

Map data © [OpenStreetMap contributors](https://www.openstreetmap.org/copyright),
under the [ODbL](https://opendatacommons.org/licenses/odbl/).
