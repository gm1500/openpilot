# Conditional experimental junction POC

Branch: `boat-anchor-conditional-e2e-poc`, based on `boat-anchor` at
`ec919f529c63cf324517602199513689441357af`. The opendbc pin and vehicle tune are unchanged.

The on-road button cycles **VOACC → conditional → full experimental → VOACC**.
VOACC retains the existing lead slowing assistance. Conditional directly enables
the E2E acceleration/stop candidate inside a qualified junction window. Full
experimental retains its existing planner behavior. The selection persists;
experimental confirmation and longitudinal capability checks still apply.

## HUD

The native vector icon combines the approved perspective intersection, a stop
sign on the near-left roadside corner and a signal on the opposite corner.
The lower-left road edge is continuous; the signal retains clear separation
from the road. All three signal lamps are neutral symbols, not observed phases.
The button has exactly two conditional appearances:

| Appearance | Meaning |
| --- | --- |
| White glyph | Conditional selected, E2E currently inactive |
| Orange glyph and ring | Conditional E2E enabled, even without extra braking |

The model may request acceleration, cruising, slowing or stopping while orange.
The stronger ordinary constraint can still win without changing the icon.
Disengagement, a pedal override, invalid model data, or stale HUD messages make
it white immediately. Qualified map awareness can remain latched internally
while the driver overrides, allowing direct re-entry when control resumes.
Lead-only E2E assistance cannot turn this icon orange.

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

## Junction continuity

Initial entry still requires two advancing real-GPS observations of the same
junction. Once qualified, retain the old attention window while a nearby new
junction confirms, or through a fresh explicit `noRoadMatch`, `roadAlignment`,
`ambiguousRoad` or `ambiguousFork` interruption. A hold has all three bounds:

- At most **2.25 seconds** since the last qualified match, measured by monotonic
  elapsed time rather than a count of planner updates.
- At most **40 metres** of integrated vehicle travel during the interruption.
- The latest validated actual GPS observation must remain **at most 3 seconds
  old**. Repeated messages and projected positions cannot renew that timestamp.

The new candidate must itself be in range. Two advancing observations qualify
it before it replaces the held target. Changing candidate IDs does not renew
the hold budget. A recovered previously qualified target need not start its
initial confirmation again while the hold remains valid.

A definite clear road, a new out-of-range target, stale/missing messages,
expired map cache, missing position, backwards timestamps, or an expired hold
releases the window. An interruption cannot create a new approach. Both mapd
look-behind and policy passage now use **20 m**, removing the previous 12/20 m
mismatch. The passed-target latch still prevents position jitter from rearming
an already passed node. The position estimator and speed-limit matcher are
unchanged.

## Direct E2E candidate and arbitration

A qualified junction directly enables the valid model acceleration and stop
proposal. The old 0.25-second model-slowing confirmation, speed-drop threshold
and acceleration-difference threshold no longer gate entry. The model's speed
reduction and predicted stop distance remain logged as diagnostics.

Choose the lower of the ordinary boat-anchor acceleration and the model
candidate. A positive model acceleration may participate too, while the normal
cruise/turn limits and stronger ordinary braking still win. An enabled model
stop request joins the ordinary stop request immediately. The map supplies
eligibility, never a stop command or right-of-way classification. Conditional
does not set the full `ExperimentalMode` flag or change its cruise-acceleration
limits. Leaving the window removes the candidate; the return toward ordinary
acceleration retains the 1 m/s³ upward slew limit.

Driver override, disengagement and invalid model data clear assistance
immediately. A stop already requested by E2E may remain held at rest through
map loss while a fresh valid model continues to request it. Map loss cannot
create a stop request, and a model go request ends that stationary hold.

## Logging and validation

The existing `mapTrafficControl` service carries `kind=junction`, target node/way,
signed distance, matched road, actual GPS age, projection status and lookup reason.
`longitudinalPlan.conditionalExperimental` records SET-based activation speed and
distance, target, model intent, both acceleration proposals, arming and actual
contribution. New `e2eEnabled` and `state=active` distinguish model participation
from additional braking; legacy state enum values remain readable. Reasons
`junctionHandoff` and `mapHold` identify continuity holds while control is eligible.

Validation: **153 tests and 19 subtests passed**, covering policy, map topology/
matching, lead assistance and HUD state; actual planner update/publish with Cap'n Proto serialization; and
native HUD drawing at 192 px and 82 px button sizes. The integration harness
substitutes unavailable native IPC and the MPC solver. Map tests use the Python
reference transformations in place of the unavailable native extension.

Recorded-input replay covers **17 segments and 19,743 planner frames**. The six
new `2c3` segments use their actual recorded junction messages; the baseline
reproduces 99.97% of recorded armed states (two frame-boundary differences).
All **eight short detection gaps during engaged driving** disappear in this
replay. Four were target-ID confirmation restarts, two were road-match losses,
and two included the mismatched passage threshold. Longer matching outages
still release. Three short internal qualification gaps remain while control
is ineligible; the new HUD would already be white in those periods.

In `2c3`, conditional E2E becomes enabled for about **74 seconds** and changes
the selected acceleration for **29.1 seconds**, including 10.5 seconds with
negative selected acceleration. Maximum additional deceleration compared with
the previous policy is about **0.93 m/s²**. The model does not request a stop
in eligible windows of this route, so this replay is not a demonstrated stop.
Stronger ordinary braking wins in every replayed frame, and no conditional
contribution occurs while control is ineligible.

The other 11 segments use the previously validated topology replay from a
public city-wide OSM snapshot, because their original logs predate junction
lookup. Both policies receive identical map/vehicle/model inputs. These replays
hold recorded trajectories fixed: they do not simulate how the vehicle or
model would respond to changed acceleration, and are not a full device build
or on-road validation.

Map data © [OpenStreetMap contributors](https://www.openstreetmap.org/copyright),
under the [ODbL](https://opendatacommons.org/licenses/odbl/).
