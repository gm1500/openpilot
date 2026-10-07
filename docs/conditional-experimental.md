# Conditional experimental stopping POC

Branch: `boat-anchor-conditional-e2e-poc`, based on `boat-anchor` at
`ec919f529c63cf324517602199513689441357af`. The opendbc pin and vehicle tune are unchanged.

The on-road mode button cycles **VOACC → conditional → full experimental → VOACC**.
VOACC retains boat-anchor's existing lead slowing assistance. Conditional adds the
map-gated junction helper; full experimental keeps the existing experimental
planner behavior. The selection persists. The existing experimental confirmation
and openpilot longitudinal capability checks still apply. Settings that explicitly
choose full experimental or ordinary mode clear conditional selection.

## HUD

The approved stop-sign and traffic-light icon has no text. On comma 3/3X it uses
the existing 192 px button and 144 px artwork footprint. The shared control is
also available at the smaller comma 4 HUD scale.

| Appearance | Meaning |
| --- | --- |
| White glyph | Conditional mode selected; no armed approach |
| Cyan glyph and thick ring | Qualified mapped approach is within the activation distance, independently of E2E slowing intent |
| Amber glyph and thick ring | Junction assistance contributes to the selected slowing or stop command |

Map proximity, a model proposal, and the existing lead-only assist cannot by
themselves produce amber. A positive-acceleration handoff is not shown as active
slowing. Cyan remains available with conditional mode selected during driver
braking, disengagement, or invalid model trajectory data. Those conditions clear
assistance and amber immediately; re-entry requires fresh model confirmation.
Stale planner/state messages clear the indicator. Both coloured rings use 4% of
button width (7.7 px on the 192 px button), with matching coloured artwork.

## Activation distance

Use `carState.vEgo`, `carState.aEgo`, the current longitudinal personality, and the
vehicle's longitudinal actuator delay. Calculate distance through:

1. One second of qualification/response allowance plus actuator delay, at current acceleration.
2. A gradual acceleration ramp, using the profile's comfortable jerk, into comfortable braking.
3. Constant comfortable braking to zero speed.
4. A 12 m margin, with a minimum activation distance of 20 m.

| Profile | Comfortable deceleration | Transition jerk | At 50 km/h | At 100 km/h |
| --- | ---: | ---: | ---: | ---: |
| Aggressive | 2.2 m/s² | 1.2 m/s³ | 89 m | 254 m |
| Standard | 1.8 m/s² | 1.0 m/s³ | 99 m | 293 m |
| Relaxed | 1.4 m/s² | 0.8 m/s³ | 114 m | 353 m |

Examples assume zero current acceleration and a 0.5 s actuator delay. Positive
acceleration increases the distance. Existing deceleration reduces it, but the
calculation never assumes braking stronger than the selected comfort level will
persist. Tiny signed-speed filter noise down to −0.1 m/s is treated as standstill;
the caller still requires a forward gear. These constants size the attention window; they do not change actuator
limits or force the model to deliver a comfortable stop.

## OSM targeting

Fetch `highway=stop` and `highway=traffic_signals` nodes belonging to the cached
drivable ways. Measure distance along the matched directed road and connected
continuations, up to 1,000 m and 16 ways. A side street does not block a clear
straight continuation: its heading change must be at most 25°, and at least 30°
better than every alternative. Shallow splits and ambiguous forks remain blocked;
the lookup does not select a turn at a branching junction. A control at the
junction itself can qualify. Nearby parallel roads,
cross-street signs, and grade-separated crossings are not radial targets.

Respect `direction`, `stop:direction`, and `traffic_signals:direction`. Accept
undirected controls on one-way roads, central junction signals, and explicit
all-way stops. Ambiguous undirected two-way approach signs are skipped. Separate
roadside sign objects, pedestrian-only crossing tags, yields, ramps, and railway
crossings are outside this POC.

Require two advancing GPS observations of the same target before arming. Speed
limit availability is independent. Both message and GPS freshness are checked;
brief motion projection is allowed only while the real GPS anchor is at most
three seconds old. Projected positions do not count as new GPS confirmations.
A restored parking checkpoint with no fresh GPS cannot arm the helper.

Once armed, keep that approach armed as its computed activation distance shrinks.
Release for a changed/missing target, lost map validity, or passage more than 12 m
beyond the node. A passed-node latch prevents near-junction position jitter from
immediately rearming the same control.

## Model confirmation and arbitration

The rendered path endpoint is not a stop detector: the UI also clips the path
around leads. Inspect the model's actual desired acceleration, stop request,
predicted velocity, and predicted position instead.

Entry requires a sustained trajectory speed reduction plus model acceleration
at or below −0.2 m/s² and at least 0.15 m/s² below the existing boat-anchor
request, confirmed for 0.25 s. Speed magnitude avoids treating a turn's smaller
forward component as a stop. A stop forecast is derived only when predicted
speed remains below 0.5 m/s for at least 0.75 s. If visible, its path distance
must not lie substantially beyond the mapped target. Earlier stops, including
queues, are allowed. A complete stop need not already be visible in the model's
finite horizon to accept sustained slowing.

Select the lower of the existing boat-anchor command and accepted junction E2E
command. Stronger regular braking always wins. Conditional mode never sets the
full `ExperimentalMode` flag. A sustained model go request releases assistance;
only the return toward acceleration is softened, at 1 m/s³. Driver override,
disengagement, invalid model data, and leaving conditional mode clear assistance immediately.
Map loss while moving releases the additional constraint. An already established
stop can remain held through map loss while a fresh, valid model continues to
request stopping at rest; map loss cannot create a new stop request.

## Logging and validation

`mapTrafficControl` records target type/IDs, matched way, signed path distance,
real GPS age, and projection status independently of `mapSpeedLimit`. Its `reason`
field distinguishes no control, direction rejection, an ambiguous fork or road,
poor alignment, stale GPS, missing position, and an unavailable map cache.
`longitudinalPlan.conditionalExperimental` records activation distance, target
distance, predicted stop distance, both acceleration proposals, stop intent,
arming, actual contribution, state, and transition reason.

Focused tests cover the stopping-distance calculation against numerical
integration, profile ordering, map direction/topology, stale/estimated positions,
mode cycling, model confirmation, holding/release, target passage, overrides,
and HUD contribution semantics. Existing map/SLC/lead regression suites also run.
Local result: **189 tests and 33 subtests passed**, plus lint and Python syntax
checks. The map regressions used the Python reference transformations in place
of the unavailable native extension.
An isolated harness exercises the real planner update/publish with real Cap'n
Proto messages and substitutes the native IPC/MPC dependencies. Another checks
the actual button's rapid taps and captures its drawing primitives at both sizes.

Recorded-input replay of five first-drive segments covers 5,987 planner frames.
With the recorded map targets held fixed, the updated policy restores 104 frames
of map-only cyan during driver intervention, with no change to the selected
acceleration. This replay cannot validate newly discoverable map targets: the
logs contain the selected target, not the downloaded OSM road/node snapshot, and
live OSM retrieval was unavailable during this investigation. Straight-road
lookahead is covered by topology tests, including continued rejection of shallow
forks, cross-street controls, disconnected geometry, and competing parallel roads.

These are offline checks, not a full device build or closed-loop road validation.
OSM completeness and model perception limit coverage. OSM supplies no live signal
phase here, and this helper cannot guarantee a stop or determine right of way.

Tag references: [stop nodes](https://wiki.openstreetmap.org/wiki/Tag:highway%3Dstop)
and [traffic signals](https://wiki.openstreetmap.org/wiki/Tag:highway%3Dtraffic_signals).
Map data © [OpenStreetMap contributors](https://www.openstreetmap.org/copyright),
under the [ODbL](https://opendatacommons.org/licenses/odbl/).
