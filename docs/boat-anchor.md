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

These are offline checks. A full device boot, comma 4 display rendering and
closed-loop road behavior have not been validated here.
