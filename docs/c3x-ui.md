# C3x UI

Only the exact `batmobile` branch greets the driver with **Batmobile Activated**
and the existing hands-on-wheel, eyes-on-road reminder. Other development branches
use the standard untested-branch alert. Dashcam, unsupported-car and security-key
alerts retain their own messages.

On `batmobile`, the C3x driver-monitor face has rounded Batman ears and angled
eyes. They share the existing head-pose, depth and fade calculations, white
opacity, steering-wheel icon and tracking arcs. Other branches retain the
standard face. Driver-monitoring alerts also remain active during AOL steering
without cruise engagement.

The sidebar TEMP value shows the device's reported maximum temperature in °C,
with the existing thermal warning colours. VEHICLE shows live supply voltage
in volts. Missing or stale readings show dashes; a disconnected panda retains
the NO PANDA status.

Construction and school headers fill the sign's full width, following its rounded
top corners. The original label row retains a small lower padding; sign size and
speed-number placement are unchanged.

Camera framing stays zoomed in whenever openpilot is engaged, including Chill
and automatic end-to-end control at low speeds. Disengaged low-speed driving
uses the wide view. Calibrated crop alignment is shared by camera video and model
overlays; a zoom animation reaches the matching narrow crop before switching.
The current upstream CPU camera renderer is retained, without the former EGL
texture crossfade. Brief invalid inputs retain the current request; offroad resets
it. The same framing policy applies to comma 4.

The speed-limit sign displays the new posted limit without a separate warning
countdown. Green pulsing follows the controller's actual speed adjustment and
stops when that adjustment is cancelled or completed. A future preview stays
visible but does not flash for a different adjustment; matching includes the
configured speed offset. A fresh valid limit can
apply on its first update; preview geometry, range and manual-override rules
still apply. When no mapped limit is available, the sign shows the automatic end-to-end icon and
OVERRIDE without a MAXIMUM or km/h label. The configurable initial no-map ceiling
is 105 km/h by default. Map gaps while engaged retain the previous SET.

The top-right control restores a manual E2E toggle. Tap to switch between Chill
and E2E; a 0.6-second hold toggles AOL and never also changes E2E. The fill ring
shows hold progress. The control uses comma's original wheel or experimental
icon at its original scale, without text above, below or inside the button.
Solid hands grip the icon, with forearms fading to transparent. E2E respects the
current map target including its configured offset, or a lower manual SET.

| AOL state | Appearance | Meaning |
| --- | --- | --- |
| OFF | Original icon without hands, dark background | AOL disabled; ordinary cruise-linked steering remains available |
| READY | White hands, dark background | Enabled, but not currently steering |
| ON | Green hands, dark background | Fresh controller acknowledgment and actual lateral actuation |
| PAUSED | Solid orange hands, dark background | Brake held, Reverse/other non-Drive gear or fault suspension |
| Waiting / unavailable | Grey hands with a bright light-grey outline, dark background | Below steering speed, waiting for fresh state, or unsupported safety configuration |

Brake, regen, parking brake and non-Drive gear pause AOL temporarily. Cancel
only cancels cruise; AOL stays in its current state. A fault suspension requires
SET/RESUME or an OFF-to-ON toggle to re-enable. The wheel/experimental symbol
remains independent of the AOL status. The same button is included in both HUD
layouts, and sleep/offroad transitions cancel pending touches.
AOL-only steering does not initiate automatic lane changes.
Driver Cancel and brake pauses during AOL-only steering are quiet. Normal
cruise disengagement and fault warnings retain their sounds. The unavailable
hands have a dark-grey body and brighter contour; both forearms fade to clear.

Three following-gap bars represent stock Cheddar Aggressive, Standard and Relaxed
personalities. Tap to cycle; hold to toggle painted-lane centering. A green ring
fills clockwise over the 0.6-second hold, whether centering is currently on or off,
and clears on release or cancellation. A hold never also changes the following gap.
No virtual road edge, retained-width or memory boundary is drawn.

The engagement border shows a brighter red center quarter along the top while
driver, vehicle-hold or applied openpilot braking is active. GM does not currently
decode physical brake-lamp status; coasting alone does not light this strip.
Orange blind-spot brackets follow the border's thickness and corner profile,
wrapping slightly onto top and bottom with short faded ends. Signaling toward an
occupied side flashes its bracket red at 2 Hz. Feedback also works in manual drive
and is drawn over the on-road content.

Real-lead triangles retain their top profile with broader rounding at both side
corners. Virtual stops adapt mici's lane-wide, flat ground-plane bar, including
perspective, curved-road geometry, 0.2-second lane-edge easing, 0.1-second
visibility fade, and 0.25-second engagement brightness transition. Confident
inner lane lines define the width; otherwise the display uses a 3.6 m fallback.
The green approach cue and yellow-to-red confirmed/held cue retain Batmobile's
colours, distance readout, near-stop dock and brake-hold label.

The display consumes the existing ModelStop marker without another longitudinal
filter or a replacement stop-distance estimate. The current model/target
filters and bumper offset remain authoritative. Ordinary confirmed-target
withdrawal may fade only with fresh, valid inputs and no real lead. Stale data,
calibration loss, a new lead, timing/drive resets, released brake holds and green
withdrawal clear the cached presentation immediately. Animation never changes
the planner or retains a control target.
