# SLC with conditional experimental mode POC

Branch: `boat-anchor-conditional-e2e-poc`, based on `boat-anchor` at
`ec919f529c63cf324517602199513689441357af`. The opendbc pin and vehicle tune are unchanged.

**SLC and conditional E2E have separate controls.** With SLC enabled and
conditional E2E off, the branch uses normal ACC-style control with mapped SET.
Junctions and missing speed data do not activate conditional E2E or set 105.
Missing speed retains the current SET, as ordinary SLC did previously.

Turning **conditional E2E on**, while SLC is enabled, enables the model
acceleration/stop candidate when **either** condition applies:

| Condition | Mode and SET behavior |
| --- | --- |
| Qualified junction in range | Temporarily enable E2E; retain normal SLC SET behavior |
| No usable map speed | Enable E2E, then automatically set 105 km/h |
| Usable map speed and no qualified junction | Regular mode with mapped SET |

The on-road mode button cycles **VOACC → conditional → full experimental →
VOACC**. Changing mode preserves the separate SLC toggle, including when returning
to normal VOACC. Full experimental remains an explicit override with its existing
planner behavior. VOACC retains the existing lead slowing assistance.
Automatic activation requires both SLC and conditional mode, experimental
confirmation and supported non-PCM openpilot longitudinal control. Turning SLC
on alone cannot opt into conditional E2E. SLC keeps its existing default-on reset
at manager start and each on-road transition. Conditional selection defaults off
and persists independently, as does full experimental selection.

## Missing-speed fallback and recovery

Use the existing SLC speed selector: a fresh valid legal limit, or its accepted
advisory fallback when the legal limit is absent. Missing, invalid, stale,
misaligned or unsupported speed data is unavailable. An ordinary speed-zone
change with a valid new candidate does not itself request fallback.

- **Entry:** require one continuous second without usable speed data. An explicit
  SET while speed is unavailable requests fallback immediately.
- **SET 105:** first publish the E2E request. Raise SET only after a valid planner
  message, at most 0.3 seconds old, acknowledges active `noSpeedLimit` E2E.
  Junction activation alone is not this acknowledgement. Engagement, healthy
  model inputs, forward gear and no pedal override are still required.
- **Recovery:** require two seconds of stable usable speed with advancing map
  position observations, retaining SLC's existing GPS/projection freshness checks.
  Apply the mapped SET, then clear the no-speed request. The
  planner checks the applied SET across the separate status and carState messages
  so reordered delivery cannot expose the old 105 SET in regular mode.

Recovered speed resumes SLC even if it is the same limit as before the outage.
A qualified nearby junction independently keeps E2E enabled after recovery.
Without one, regular mode resumes. A fresh explicit SET can accept the recovered
map limit before the automatic recovery timer completes.

The automatic 105 change happens once per outage, including when a previous
manual selection paused SLC before speed data disappeared. Manual +/- or RES
after the fallback request cancels any pending raise; the helper does not
repeatedly undo a driver's adjustment during the outage. A simultaneous manual selection can
override automatic recovery. Pedals and disengagement prevent an automatic
raise; disabling SLC or conditional mode clears both automatic conditions.
Turning conditional mode off preserves the SLC setting. A previously established
no-speed request survives a stale status message until fresh recovery or an
explicit mode change, subject to the planner's ordinary health checks.

105 km/h is the cruise SET ceiling used for fallback, not a promised driving
speed or demonstrated ramp/parking-lot behavior.

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

During acknowledged no-speed E2E, the MAX sign replaces the map number with
the existing experimental-mode icon, scaled to the number area on both screens.
An orange border, `OVERRIDE` caption and `NO MAP LIMIT` status distinguish it
from a mapped speed. The actual SET remains underneath (normally 105 km/h,
or its mph conversion, unless manually adjusted). The override stays through
map recovery qualification and returns to the speed number when SLC takes over.
Junction-only E2E does not replace a usable speed number. Disengagement, pedal
override, disabled conditional/SLC mode, or stale feedback clears the override.

Green pulsing belongs to a mapped-speed adjustment, never the 105 E2E ceiling.
Entering no-speed fallback cancels any old pending map pulse. A missing map
number and the experimental override both suppress the pulse and reset its
animation clock. Recovery can start a fresh pulse when the mapped SET applies.

## Experimental camera transition

Both raylib layouts adapt the wide-camera animation from upstream
[`33ddd8eb`](https://github.com/commaai/openpilot/commit/33ddd8eb444ca4767d141e46e3b7e0abed6815da).
Full Experimental mode retains its original camera-selection behavior.
Selecting conditional mode permits the same speed-based camera behavior,
including between actual E2E activations while the conditional icon is white.
The camera follows the selected mode independently of the longitudinal planner;
E2E entering or leaving a junction/no-speed condition does not reset the camera
request. Regular cruise and SLC alone do not request wide. Current-drive, fresh
mode and vehicle-speed messages are required to start a request. Brief invalid
or stale inputs preserve an existing request for up to one second after the
last valid input check, so a single dropped update cannot erase the speed
hysteresis latch. Longer gaps return to narrow. A fresh mode-off cancels the
request immediately, even when speed is invalid; offroad clears the grace.

The existing speed thresholds remain: request wide below **18 km/h**, narrow
above **36 km/h**, and retain the requested direction between them. On entry,
the wide image starts cropped to the narrow field of view, then zooms out.
On exit, wide zooms back in before switching to live narrow frames. The original
easing takes about **0.85 seconds** and is normalized for elapsed time, so
20 Hz and 60 Hz rendering have the same timing. Missing camera streams cannot
stall a return to narrow; reversing the request cancels a pending stale switch.
If wide was missing from the initial stream list, discovery retries once per
second while the primary camera is connected. Empty-buffer connections are
retried at 200 ms intervals for both primary and pending streams, rather than
being treated as usable merely because VisionIPC reports connected.

Camera projection and the road overlay use the same animated zoom. The
projection cache updates on every zoom step. Each layout retains its normal
framing, including mici's existing narrow-camera speed-dependent crop; the
added wide/narrow animation is confined to Experimental entry and exit.
The handoff maps the centre ray of the actual, edge-clamped narrow viewport
through the calibrated lens rotation (`wideFromDeviceEuler`). Its corresponding
position in the wide image anchors the start/end of the zoom. The crop then
eases toward normal wide framing, including the portrait layout's vertical
offset. This prevents independently centred crops from jumping at the switch.
At both lens handoffs, a **200 ms crossfade** overlaps two live VisionIPC
streams. The incoming image fills the background while the outgoing image's
opacity falls with smooth easing; their effective weights sum to one. The
outgoing crop follows the incoming crop's calibrated centre and magnification,
including while wide begins zooming out. Model/UI overlays render only once.
Each stream owns its textures and EGL images independently, even when their
buffer indices coincide. The outgoing client/resources are released after the
fade, on offroad, or if its frames stop arriving for 150 ms. Reversing during
the fade reuses the same two clients and preserves their current blend weights.
These are live frames, not a frozen screenshot. Lens distortion and parallax
from nearby objects can still leave a small residual mismatch during overlap.

## Recovery after a missed shallow exit

A committed highway path can discard an exit before the lanes separate. The
matcher may now reconsider a branch behind its current projection when both
roads share a directed outgoing fork within 12 seconds of travel (40–400 m).
The retained road must be over 20 m away; the alternative must be within 8 m,
align within 5 degrees of both GPS and vehicle-motion headings, and improve
both heading errors by at least 1 degree. Three fresh, non-projected fixes,
at least one second and 30 m of forward motion on that branch are required.
An isolated GPS jump, a closer unconnected road, or a parallel branch with no
measured departure cannot reopen the old fork. SLC's two-second speed
qualification remains in place after the corrected road match.

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
it. An already armed approach remains latched through an ordinary reduction in
SET until its normal release conditions occur. On no-speed recovery, recheck
the junction against the restored SET so it does not inherit the larger 105
entry window. The planner's temporary `forceDecel` speed
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
releases the junction window. The separate no-speed condition can still enable
E2E. An interruption cannot create a new approach. Both mapd
look-behind and policy passage now use **20 m**, removing the previous 12/20 m
mismatch. The passed-target latch still prevents position jitter from rearming
an already passed node. The position estimator and speed-limit matcher are
unchanged.

## Direct E2E candidate and arbitration

A qualified junction or the no-speed fallback directly enables the valid model
acceleration and stop proposal. The old 0.25-second model-slowing confirmation, speed-drop threshold
and acceleration-difference threshold no longer gate entry. The model's speed
reduction and predicted stop distance remain logged as diagnostics.

Choose the lower of the ordinary boat-anchor acceleration and the model
candidate. A positive model acceleration may participate too, while the normal
cruise/turn limits and stronger ordinary braking still win. An enabled model
stop request joins the ordinary stop request immediately. The map supplies
eligibility, never a stop command or right-of-way classification. Conditional
does not set the full `ExperimentalMode` flag or change its cruise-acceleration
limits. Clearing both conditions removes the candidate; the return toward ordinary
acceleration retains the 1 m/s³ upward slew limit.

Driver override, disengagement and invalid model data clear assistance
immediately. A stop already requested at a qualified junction may remain held at
rest through map loss while a fresh valid model continues to request it; a model
go request ends that stationary hold. A no-speed-only stop is not retained after
speed recovery unless the junction condition independently applies. Neither map
condition can create a stop request without the model requesting it.

## Logging and validation

The existing `mapTrafficControl` service carries `kind=junction`, target node/way,
signed distance, matched road, actual GPS age, projection status and lookup reason.
`longitudinalPlan.conditionalExperimental` records SET-based activation speed and
distance, target, model intent, both acceleration proposals, arming and actual
contribution. New `e2eEnabled` and `state=active` distinguish model participation
from additional braking; legacy state enum values remain readable. Reasons
`junctionHandoff` and `mapHold` identify continuity holds while control is eligible;
`noSpeedLimit` identifies an enabled no-speed candidate. `mapCruiseState` adds
`automaticE2e`, `e2eFallback` and applied `setSpeed` for the cross-service handshake.
Card publishes transitions immediately in addition to its regular 5 Hz status.

Initial validation: **246 tests and 101 subtests passed**, covering both OR conditions,
missing/recovered speed, SET buttons, map matching/topology, stronger ordinary
braking, driver override, mode selection and HUD state. The actual cruise helper
and longitudinal planner were also coupled through Cap'n Proto request/ack
messages. Checks include SET-before-release recovery, deliberately reordered
messages, recovery with and without a junction, rechecking the restored SET
range, and invalid model/plan, stale acknowledgement and pedal blocking.

Native messaging, parameters and the longitudinal MPC solver were built locally.
The ordinary/full-experimental cruise and longitudinal maneuver suites pass:
21 tests and 56 maneuver subtests. The initial nine integration tests couple the
actual cruise helper, native MPC, SubMaster health checks and native planner
publications. They cover both triggers, 105 entry, recovery with and without a
junction, the smaller recovered SET window, reordered recovery messages, invalid
model/plan blocking and SLC off. They also verify that SLC alone stays in regular
mode at junctions and through missing/recovered speed, and that conditional
selection controls both triggers without disabling SLC. Seven abstract
parameterized test templates are skipped; their generated cases run. The earlier substitute-solver checks remain
supplemental. Native HUD drawing and mode cycling pass at 192 px and 82 px sizes.

The native suite exposed and verified a fix for dictionary-based simulation
inputs: optional map readers now treat absent service metadata as unavailable
data, rather than raising an exception or trusting an unqualified map message.

The earlier SET/mode replay covers six `2c3` segments and **6,556
planner frames** using the recorded map-speed, junction, button and model inputs.
It raises SET to 105 once, after the planner acknowledgement, then returns to
40 when map speed qualifies. A second recovery occurs while disengaged and
does not raise SET. Junction-only E2E remains **73.95 seconds**, matching the
previous junction smoothing replay; missing-speed E2E adds **2.75 seconds**.
Stronger ordinary braking wins in every replayed frame, and no conditional
contribution occurs while control is ineligible.

The `2c4` follow-up reproduces the missed 105 SET in segments 18 and 19:
SLC was already paused when the logs began. No-speed E2E activated at segment
18 +43.450 s, 14.33 seconds before the bookmark, but the old tracking gate
left SET at 100. Replaying the corrected selector over **2,400 planner frames**
raises SET to 105 at +43.452 s after acknowledgement. The restored 70 limit
applies at segment 19 +30.610 s before releasing fallback; junction activation
remains independent. This replay seeds the observed initial paused/100 state.

Follow-up validation: **129 tests and 63 subtests passed**, including the real
MPC/messaging integration with previously paused SLC, manual adjustment after
the fallback request, and inactive/stale/previous-drive HUD rejection. Six
abstract test templates are skipped. Native sign drawing calls were checked at
both production widths (180 px and 96 px), in metric and imperial modes: the
experimental icon stays inside the number area and the actual SET stays visible.

Camera/pulse follow-up: **81 tests and 38 subtests passed**, including both
native raylib camera classes' stream-switch and projection code with synthetic
frame sources and GPU uploads replaced. Cases cover delayed target frames,
mid-animation reversal, pending-switch cancellation, stream loss, offroad reset,
mode eligibility, matching fields of view, animated overlay/cache updates,
and 20/60 Hz timing. Cruise and native MPC integration tests verify the 105
handoff and mapped-speed recovery still work, while only the mapped target
starts green feedback. This does not include device GPU or on-road validation.

The previous junction-only validation covered 17 segments and 19,743 frames.
It removed all eight short detection gaps during engaged driving in `2c3`:
four target-ID confirmation restarts, two road-match losses and two involving
the passage threshold mismatch. That junction continuity policy is retained.
The older 11 segments used a public city-wide OSM topology replay because their
logs predate junction lookup.

Replays hold recorded vehicle/model trajectories and ordinary acceleration
proposals fixed. The current replay verifies SET/mode sequencing; it does not
predict acceleration or vehicle response after changing SET. These checks are
not a full device build or on-road validation, and this route does not demonstrate
an eligible model stop.

Map data © [OpenStreetMap contributors](https://www.openstreetmap.org/copyright),
under the [ODbL](https://opendatacommons.org/licenses/odbl/).
