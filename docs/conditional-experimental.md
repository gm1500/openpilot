# Conditional experimental mode

SLC and conditional E2E have separate controls. The on-road mode button cycles
VOACC → conditional → full experimental. Conditional selection persists and
defaults off; SLC alone cannot enable E2E.

With SLC and conditional mode enabled, a qualified junction, unavailable map
speed, or a model-requested stop independently enables the model candidate. Supported openpilot longitudinal control,
experimental confirmation, fresh inputs, forward gear and no pedal override are
required. Ordinary braking constraints retain precedence.

## Junctions

Mapd finds intersections, splits and merges from connected OSM node geometry.
Stop-sign and signal tags do not gate activation. Entry distance uses current
cruise speed, acceleration, personality and actuator delay. Bounded continuity
holds cover brief matching gaps and qualified handoffs; stale or ambiguous data
cannot establish a new approach.

## Missing map speed

One continuous second without usable speed requests E2E; an explicit SET can
request it immediately. SET rises to 105 km/h only after fresh planner feedback
acknowledges the no-speed candidate. This is a cruise ceiling, not a target
vehicle response. Manual adjustments cancel pending automatic increases.

Recovery requires two seconds of stable map speed with advancing observations.
Card applies mapped SET before clearing fallback; the planner checks the applied
SET to handle reordered messages. Junction E2E can remain active independently.
Pedals, disengagement and disabled SLC/conditional mode prevent automatic raises.

## Model-requested stops

A sustained model stop prediction (the same near-zero-speed trajectory used by
the MODEL STOP flag), or an explicit E2E `shouldStop` request, immediately enables
the E2E candidate without an OSM junction match. The trajectory must predict at
most 0.5 m/s for at least 0.75 seconds. The flag's bumper offset and display
smoothing do not affect activation.

The published reason is `modelStop`; CEM turns orange while control is eligible.
A future stop enables E2E acceleration planning, but only the E2E action's actual
stop request adds a standstill command. Stronger ordinary braking still wins.
The existing eligibility, mode, gear, pedal and input-health checks remain in
force. A valid stop request can persist through map loss or speed-limit recovery.
When neither model stop nor another CEM condition remains, the existing release
slew softens the return to acceleration. This condition does not change cruise SET
or acknowledge the 105 km/h missing-speed handshake.

## Regular ACC stop target

On `boat-anchor-model-stop-poc`, a sustained model stop prediction creates a
stationary obstacle alongside the two real leads in ordinary/conditional ACC.
It requires fresh, valid model data, enabled longitudinal control, forward gear
and no pedal override. SLC, CEM and a real lead are not required. Full experimental
mode retains its existing model control.

The MPC calculates braking from vehicle speed and the tracked remaining travel.
The virtual obstacle sits 6 m beyond the target to account for the MPC's existing
standstill gap; this adds no earlier-stop margin. Real leads retain their normal
gap policy. Only the virtual target removes the ordinary 25% gap-compression slack.
Stronger model or real-lead braking still wins. The brake calibration is unchanged.

Remaining travel is reduced by measured vehicle motion. New nearer predictions
are filtered over 0.2 seconds; the active target cannot recede ahead of the truck.
A fresh prediction with neither a sustained stop nor `shouldStop` immediately
removes the target. At rest, a positive model action plus a sustained departure
trajectory also releases it, even if the trajectory begins with a near-zero prefix.
A positive action alone cannot cancel a still-predicted future stop. Stale/invalid
inputs, reverse, disengagement and pedal overrides clear all target history.

A close target holds only below 0.3 m/s; an explicit model `shouldStop` can also
request hold. An explicit stop without any known position never invents an
obstacle. The ordinary planner handles the acceleration transition after release.
Telemetry includes `stopTarget.active`, `stopTarget.distance` (-1 when absent),
and `stopTarget.holding`; source `stopTarget` distinguishes this from a real lead.
`e2eStopDistance` retains the raw model prediction for comparison. Legacy POC
schema fields remain readable but have no control effect.

This remains a POC: the model position is a prediction, not a detected stop line.
The sustained-stop qualification can occur later than initial model slowing.

## Display

A white conditional icon means selected but inactive. Orange means the E2E
candidate is enabled, even when ordinary braking wins. Lead-only assistance
cannot activate the icon. Stale feedback and driver overrides return it to white.

Acknowledged no-speed E2E replaces the map number with the experimental icon,
orange border and OVERRIDE caption, retaining actual SET underneath. Recovery
restores the map number. Green pulses indicate mapped-speed adjustments only.

Map data © [OpenStreetMap contributors](https://www.openstreetmap.org/copyright),
under the [ODbL](https://opendatacommons.org/licenses/odbl/).
