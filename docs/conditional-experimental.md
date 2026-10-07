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

Fetch and normalize controls belonging to the cached drivable ways:

| Encoding | Interpretation |
| --- | --- |
| `highway=stop`, with `stop=all/minor` or no subtype | Stop sign; approach direction still has to be established |
| `highway=traffic_signals` | Traffic light, subject to the subtype exclusions below |
| `highway=crossing` with `crossing=traffic_signals` or `crossing:signals=yes` | Signal-controlled crossing, including marked and unmarked variants |
| `highway=crossing` with `crossing_ref=pelican/puffin/toucan/pegasus` | Named signal-controlled crossing, unless explicit tags say unsignalized |
| `traffic_sign`, `traffic_sign:id`, `traffic_sign:forward/backward` | Exact stop-sign names/codes, including comma/semicolon lists and omitted repeated country prefixes |
| `highway=stop` with compass lists such as `stop=E;W` | Stop only on the named incoming arms; the letters describe the side of the junction, not the vehicle's travel heading |
| `stop=yes/-1/both` on a drivable way | Legacy stop at the last/first/both endpoints, applying only to that way |

The sign parser recognizes `stop`, `US:R1-1`, `DE:206`, Canadian `CA:RA-1`
and provincial `CA:<province>:RA-1`, `CA:BC:R-001`, and Quebec
`CA:QC:P-010/P-010A`. These are an explicit supported code set, not a claim to
recognize every country's sign catalogue. `US:R1-3P` and `CA:AB:RA-1-T` identify
all-way stops only when combined with a stop sign. Tabs alone, stop-ahead signs,
unknown IDs and qualified IDs such as `US:R1-1[100 m]` are not stop targets.

Signal-controlled crossing nodes use the traffic-light target type; they describe
the crossing rather than one signal head, so both road approaches qualify unless
explicit direction tags restrict them. `crossing:signals=no` rejects a crossing
even if an older crossing tag disagrees. Zebra markings or a `pxo` reference alone
do not establish traffic signals. Standard, pedestrian/cyclist, HAWK, secondary,
and normally blinking full signals are included through `highway=traffic_signals`.
`traffic_signals=stop` is an all-way stop beacon. A generic `blinker` has ambiguous
red/yellow approach semantics and is excluded unless `stop=all` establishes a
stop requirement. Continuous-green, ramp-meter, railway and bridge signal
subtypes, and explicitly inactive controls are outside this junction POC.

Measure distance along the matched directed road and connected
continuations, up to 1,000 m and 16 ways. A side street does not block a clear
straight continuation: its heading change must be at most 25°, and at least 30°
better than every alternative. Shallow splits and ambiguous forks remain blocked;
the lookup does not select a turn at a branching junction. A control at the
junction itself can qualify. Nearby parallel roads,
cross-street signs, and grade-separated crossings are not radial targets.

Respect `direction`, `stop:direction`, `traffic_sign:direction`, directional
sign keys, and `traffic_signals:direction`; conflicting explicit restrictions
reject the approach. Compass stops support all 16 compass points. A named
incoming arm must be within 35° and at least 15° closer than every competing arm.
Legacy way-end stops retain real endpoint node IDs and never spread to a crossing
road. Explicit node controls take precedence over an implied way-end stop. Accept
undirected controls on one-way roads, central junction signals, and explicit
all-way stops. Ambiguous undirected two-way approach signs are skipped. Separate
roadside sign objects, footway-only signals, unsignalized crossings, yields,
ramps, and railway crossings are outside this POC.

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
Local result: **200 tests and 122 subtests passed**, plus lint and Python syntax
checks. The map regressions used the Python reference transformations in place
of the unavailable native extension.
An isolated harness exercises the real planner update/publish with real Cap'n
Proto messages and substitutes the native IPC/MPC dependencies. Another checks
the actual button's rapid taps and captures its drawing primitives at both sizes.

Recorded-input replay of eight drive segments covers 9,581 planner frames using
a freshly retrieved OSM snapshot. The old lookup reproduces the three latest
bookmarked misses; the updated lookup finds signal-controlled crossing nodes on
the matched road and reaches `inRange` at all three (approximately 32 m, 35 m,
and 47 m from the respective crossings). Model go requests keep these cases cyan
without extra braking. The replay changes no selected acceleration in these
recorded segments. The live production query also returns all three target nodes
within the existing response-size limit. Older bookmarks outside the calculated
range, at ambiguous forks, or without a current road match remain unarmed.

The replay supplies the public map snapshot directly rather than reproducing
the device's original download timing/cache contents; the original OSM response
is not in the logs. Road matches agree with the logs on 98–100% of map frames
(rounded), including all three latest bookmarked locations. Topology tests also
cover continued rejection of shallow forks, cross-street controls, disconnected
geometry, and competing parallel roads.

The encoding expansion was separately replayed against the preceding crossing
fix across the same 9,581 frames. The three latest light approaches retain the
same targets and states. A previously ignored `stop=E;W` node now adds 46 cyan
planner frames (about 2.3 s) in the older route's segment 5; its east/west arms
qualify while the north/south arms remain rejected. No selected acceleration
changes in these eight recorded-input segments. Added tests cover sign-code
lists, directional overrides, named crossings, explicit exclusions, all-way
beacons, all 16 compass points, ambiguous arms, and way-end stop isolation.

These are offline checks, not a full device build or closed-loop road validation.
OSM completeness and model perception limit coverage. OSM supplies no live signal
phase here, and this helper cannot guarantee a stop or determine right of way.

Tag references: [stop nodes](https://wiki.openstreetmap.org/wiki/Tag:highway%3Dstop),
[traffic signals](https://wiki.openstreetmap.org/wiki/Tag:highway%3Dtraffic_signals),
and [signal-controlled crossings](https://wiki.openstreetmap.org/wiki/Tag:crossing%3Dtraffic_signals).
Additional encoding references: [crossing signals](https://wiki.openstreetmap.org/wiki/Key:crossing:signals),
[named crossings](https://wiki.openstreetmap.org/wiki/Key:crossing_ref),
[stop variants](https://wiki.openstreetmap.org/wiki/Key:stop),
[traffic signs and directions](https://wiki.openstreetmap.org/wiki/Key:traffic_sign),
[signal subtypes](https://wiki.openstreetmap.org/wiki/Key:traffic_signals),
[Canadian signs](https://wiki.openstreetmap.org/wiki/Canada/Road_signs/Regulatory),
[Alberta signs](https://wiki.openstreetmap.org/wiki/Canada/Road_signs/Alberta/Regulatory),
[US stop signs](https://wiki.openstreetmap.org/wiki/Tag:traffic_sign%3DUS:R1-1),
and [German stop signs](https://wiki.openstreetmap.org/wiki/Tag:traffic_sign%3DDE:206).
Map data © [OpenStreetMap contributors](https://www.openstreetmap.org/copyright),
under the [ODbL](https://opendatacommons.org/licenses/odbl/).
