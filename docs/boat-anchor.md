# boat-anchor

Based on commaai/openpilot master `c8fb906815530460ed156f14e09e1f312bb0f851`.
This includes upstream's **Super Leicht** driver-monitoring model and its
current model-loading, hardware and vehicle-support changes.

The custom history is grouped into five commits: **Nudgeless**, **Lane centering
policy**, **Lead distance velocity**, **Sierra Tune**, and a separate moving-Ki
experiment. Earlier experimental documentation and custom test files are not
carried into this branch. Upstream tests remain available.

## Lane centering

The policy adds a bounded curvature correction to the original E2E steering
action. It fits the inner lane midpoint against the E2E path over their shared
8–55 m horizon, estimates their offset and heading difference, and blends the
remaining target shape toward lane geometry over 10–45 m. The fit is spatial;
there is no new temporal filter on the steering command. Original model paths,
orientations and acceleration outputs remain available unchanged.

Strong confidence in both inner lines arms the policy after 0.75 s. Lower
confidence can retain an already valid lane, and a learned lane width supports
a one-line hold for at most one second. Invalid geometry, missing inputs,
nonfinite policy inputs, blinkers or lane-change intent return the exact E2E
command and clear the correction state. Correction limits, faster release,
turn-reversal guards, motion reconciliation and the bounded centering learner
are retained from the current lane policy.

The per-drive selector defaults on. Both comma 3/3X and comma 4 HUDs use the same
selector and status: blue inner lines indicate readiness/hold, and mint lines
indicate an active policy while engaged. The comma 4 selector has its own touch
guard so a tap does not also trigger the camera-view action.

## Lead velocity from distance

For an accepted vision lead, range evolution provides the kinematic relation:

`lead speed = ego speed + rate of change of relative distance`.

A private Kalman tracker estimates distance and absolute lead speed from range
observations, ego motion and observation uncertainty. Model lead velocity does
not update that private velocity state. Both model hypotheses share one track
when their distance/lateral observations agree; brief hypothesis changes can
retain compatible history. Private observations never bypass stock lead
acceptance or radar matching.

Output progresses from model fallback through a bounded pre-closing cue to
range-derived speed once history and velocity uncertainty qualify. The quiet
process-noise schedule depends on range. Sustained closing pulls down an
optimistic published speed only when the model and a causal range trend agree.
Normal mode changes blend continuously; urgent braking bypasses that blend.
Low-speed closing retains the model ceiling, and a repeatedly near-zero model
lead can enter confirmed-stopped mode while the private tracker keeps learning.
Release from a stopped mode respects the newly moving model speed.

Mild kinematic range correction and bounded confidence shaping are retained.
Confidence shaping applies only to mature, safely spaced highway following;
its corrections are bounded to 0.75 m in distance and 0.08 m/s² in acceleration,
and it bypasses deceleration, closing, track transitions and invalid inputs.
Physical radar leads retain their original output. Planner costs, constraints,
cruise policy, following-distance policy and lead acceptance are unchanged.

## Sierra/Silverado tune

The existing torque-controller structure, fingerprint-specific road-load
estimate, low-speed brake calibration and lateral tune are retained. The Sierra
profile uses 2,586 kg including standard cargo, a 0.419 m wheel radius,
`Cd × area = 0.30 × 3.61 m²`, and rolling compensation coefficient 0.004.
The reverted 0.75 road-load scale path is removed; full road-load compensation
remains. The dedicated stopping calibration and 0.5 s actuator delay remain.
Launch integral suppression during that delay applies only to Sierra/Silverado.

Shared vehicle specs define a neutral wheel-radius default of zero. Platforms
using wheel torque override it. This fixes the EV6/non-GM initialization crash
without inventing a tire radius or applying the Sierra torque model to them.

### Separate moving-Ki experiment

Sierra/Silverado integral gain is unchanged through 36 km/h, blends down over
36–40 km/h, and is 25% lower at 40 km/h and above. This changes only the existing
gain table; planner outputs, stopping gain, actuator delay and torque/brake
mapping retain the preceding Sierra Tune behavior.

| Speed | Previous Ki | New Ki |
| --- | ---: | ---: |
| 36 km/h | 0.005 | 0.005 |
| 40–54 km/h | 0.005 | 0.00375 |
| 90 km/h and above | 0.0025 | 0.001875 |

Offline replay of route `615b11d4c01d81ec/0000028a--c8284506d8`, segments 7–19,
holds recorded motion and planner targets fixed. Across 69,214 moving frames
at or above 40 km/h, mean positive integral correction falls 24.1%; the largest
acceleration-command change is 0.0038 m/s². Projected brake-active actuator
ticks change from 1,854 to 1,855. This supports a smaller integral contribution,
but does not demonstrate fewer brakes or reduced closed-loop oscillation.

## Validation scope

The feature port passes 194 lane/lead/controller/monitoring tests and 177 GM
safety tests. All 254 platforms initialize their interface/controller in both
longitudinal configurations (508 cases). Non-GM parameters match upstream
except the added neutral radius field. Torque cleanup matches the prior
controller's actuator outputs and CAN messages on 6,048 GM grid cases.
Native parameter registration, selector toggles, shared status colors and
invalid lane-input fallback are checked separately.

These are offline checks. A full device boot and closed-loop road behavior
have not been validated here.

## OSM speed-limit display and map cruise POC

The coast100 branch shows an OpenStreetMap speed-limit sign in the cruise HUD,
with the actual `SET` cruise speed directly below the sign. Both comma 3/3X and
comma 4 layouts support km/h and mph. On comma 4 the driver-monitoring icon
sits beside the sign, and alerts take priority over it.

On `boat-anchor-map-cruise-poc`, tap the sign to toggle map cruise. It defaults
on each drive, like the lane selector. The coast100 torque tune, Ki, lead handling,
lane policy and longitudinal planner algorithms are unchanged. This feature
changes the cruise set-speed input to the existing planner.

| Action or state | Behavior |
| --- | --- |
| Map on / SET | Follow the current heading-matched map limit after it is stable for two seconds across fresh GPS fixes. This replaces the ordinary 40 km/h or experimental 105 km/h initialization when a qualified limit exists. Subsequent limit changes update SET automatically. |
| RES re-engagement | Restore the previous set speed and hold it until the next confirmed map-limit change or SET. |
| RES/+ while already engaged | Normal speed increment, then hold that manual target until the next confirmed map-limit change or SET. Standstill RES keeps its existing launch behavior without a speed increment. |
| Long +/- press | Keep the existing repeated 5 km/h steps (5 mph in imperial mode); hold the manual target until the next confirmed map-limit change or SET. |
| Tap off | Hold the current target and restore normal SET/RES button behavior. |
| Tap back on | Resume automatic map selection when a qualified limit is available. |
| Missing/stale GPS, no reliable heading or ambiguous map | Hold the existing set speed; show MAP WAIT. SET with no qualified limit arms tracking; ordinary engagement initialization remains the fallback. |
| Accelerator override | Hold automatic target updates while pressed. SET with the accelerator pressed keeps the original current-speed floor and pauses map tracking. |

Green MAP ON means tracking while engaged; MAP READY means a qualified limit is
available before engagement. Amber MAP WAIT means no qualified target; MAP HOLD
means a manual target is held until the next confirmed limit change or SET. MAP OFF is manual cruise. The sign
remains visible when off. Stock PCM cruise, lateral-only, passive and non-car
platforms keep a display-only sign; this feature cannot engage cruise or launch
the vehicle by itself.

Heading compensation selects the applicable road direction and directional
`maxspeed` tag. It is not road-grade compensation or anticipation of an upcoming
speed zone. Limit changes are applied after matching the new zone, so this POC
does not guarantee slowing to a lower limit before its sign. Map errors remain
possible; the driver must monitor the actual set speed and posted restrictions.

An internet connection is needed to fetch nearby road geometry and speed tags
from `https://overpass-api.de/api/interpreter`. Queries send the current GPS
coordinates, without device identifiers or route history. Requests run in the
background, at most once every 30 seconds, with backoff on failure. A small
in-memory map cache covers 1.5 km around each query and expires after 10 minutes.
There is no offline map download or route upload.

The sign shows a dash when GPS is stale or inaccurate, the cached area is no
longer usable, the road match is ambiguous, or the road has no supported numeric
limit. Directional limits are supported. Conditional, variable, lane-specific
and vehicle-specific limits are left unknown rather than guessing. Road signs
and temporary restrictions can differ from the map. Unsupported limits are not
used as cruise targets.

The internal Qualcomm GPS publisher leaves horizontal accuracy unset (zero).
For those otherwise valid fixes only, the matcher uses a conservative 15 m
matching allowance, not a claimed accuracy measurement. Other receivers still
require a positive reported accuracy. GPS publishing is unchanged. Active map
selection additionally requires a valid heading and a stable limit within the existing 8–145 km/h cruise range.

Map data: (c) [OpenStreetMap contributors](https://www.openstreetmap.org/copyright),
available under the [Open Database License](https://opendatacommons.org/licenses/odbl/).

A separate optional `mapd` process handles matching and network access, publishing
`mapSpeedLimit` at 5 Hz. The control loop performs no network operations. It checks
both the packet age (0.8 s) and original matched GPS age (3 s). A frozen fix cannot
qualify a new target. `mapCruiseState` records selection status at 5 Hz. If `mapd`
stops, map selection becomes unavailable while ordinary manual cruise remains
available; existing required-process checks are unchanged for every other process.

Validation: 39 offline regression tests cover parsing, road matching, GPS validity,
cache expiry, message freshness, qualification, SET/RES, long presses, gas and
standstill behavior, mph conversion, and stock/lateral-only platform isolation.
A further 60,000 synthetic frames matched the preceding cruise helper exactly
with map mode off or unsupported (PCM/lateral-only), in both unit systems.
Native messaging, C++ schema generation and parameter default/reset checks pass.
A real mapd/IPC test with synthetic GPS and a local map fixture selects 50 km/h;
stopping mapd holds that target and reports waiting. The production raylib widget
was rendered at both device sizes and its tap, drag-out, unsupported-platform and
alert-interruption behavior checked. Lint and whitespace checks pass.

These checks do not validate a full device boot, closed-loop vehicle behavior or
map accuracy. No new route coordinates were sent to OSM during this validation.

### Coast100 display-stability port

The map-cruise branch includes the coast100 matcher wakeup, bounded road
continuity near ramps, and advisory sign hold (at most two seconds / 60 m).
Confirmed speed changes replace the sign immediately; turns, position jumps,
invalid GPS and off-road transitions clear the hold.

Display values and control samples have separate validity and original GPS
timestamps in `mapSpeedLimit`. A held sign never becomes a fresh cruise target.
During the short asynchronous handoff to a new GPS fix, the preceding confirmed
sample may remain available for up to 0.3 seconds with its original timestamp,
only within 60 m and 20 degrees. A current ambiguous result invalidates control
data immediately. The existing two-second map qualification and SET/RES/manual
button behavior remain unchanged. The UI also checks held-display expiry itself.

The port passes 39 regressions, including display/control separation, expiry,
ramp departure and button behavior, plus an isolated real-IPC check using 1 Hz
synthetic GPS at highway speed. This is offline validation, not a driving test.

With map cruise enabled, a newly qualified speed-limit change automatically
updates the current SET speed, including after RES or manual +/- adjustment.
RES still restores the previous set speed immediately. A missing-data interval
followed by the same limit does not cancel that manual hold. Changes retain the
two-second qualification; a long press in progress keeps priority, and releasing
it preserves the manual target until a subsequent limit change. Turning map
cruise off prevents automatic speed changes.

### Speed-sign UI polish

The shared comma 3/3X and comma 4 sign uses larger units and shows only the
numeric cruise target underneath (without the SET prefix). OpenStreetMap credit
and licence information are under Settings > Software > Map Data, replacing the
tiny line below the onroad MAP status.

A pronounced 1 Hz green background pulse previews an automatic target change during
the existing two-second qualification. The sign returns to white when the cruise
target changes or the pending change is cancelled. It does not wait for vehicle
speed to reach the target. The UI-only `mapCruiseState.pendingSpeed` field comes
from the actual selector; a held advisory sign or same-limit recovery during a
manual hold cannot start the pulse. Disengagement, map-off, unsupported platforms,
gas override, held +/- buttons and stale status suppress the preview. This is not
look-ahead to a future road sign and adds no delay to speed selection.

Validation: all 45 map/cruise regressions pass, including pending-state clearing,
manual priority, read-only preview behavior and message serialization. Isolated
production UI checks cover stale status, immediate target-change clearing, pulse
reset and touch/alert cancellation. A software drawing backend was inspected at
both screen sizes; native raylib preview was unavailable due to display-socket
permissions, so on-device appearance still needs confirmation. C++ schema
generation, lint and whitespace checks pass. Planner and torque tuning are unchanged.

The visibility refinement doubles the pulse's peak green tint, thickens the green
active/ready border from 4 to 7 design pixels, and enlarges the cruise-target font
from 35 to 46. The target strip grows to fit, with MAP status moved below it on
both display sizes. This UI-only refinement does not change road matching; the
reported sign dropout through bends remains a separate matcher issue.
