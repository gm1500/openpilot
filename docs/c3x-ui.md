# C3x UI

Only the exact `batmobile` branch greets the driver with **Batmobile Activated**
and the existing hands-on-wheel, eyes-on-road reminder. Other development branches
use the standard untested-branch alert. Dashcam, unsupported-car and security-key
alerts retain their own messages.

On `batmobile`, the C3x driver-monitor face has rounded Batman ears and angled
eyes. They share the existing head-pose, depth and fade calculations, white
opacity, steering-wheel icon and tracking arcs. Other branches retain the
standard face. Driver-monitoring policy and alert visibility are unchanged.

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
is 105 km/h by default. There is no manual CEM/EXP mode selector.

Three following-gap bars represent stock Cheddar Aggressive, Standard and Relaxed
personalities. Tap to cycle; hold to toggle painted-lane centering. No virtual road
edge, retained-width or memory boundary is drawn.

The engagement border shows a brighter red center quarter along the top while
driver, vehicle-hold or applied openpilot braking is active. GM does not currently
decode physical brake-lamp status; coasting alone does not light this strip.
Orange blind-spot brackets follow the border's thickness and corner profile,
wrapping slightly onto top and bottom with short faded ends. Signaling toward an
occupied side flashes its bracket red at 2 Hz. Feedback also works in manual drive
and is drawn over the on-road content.

Real-lead triangles retain their top profile with broader rounding at both side
corners. Virtual stops use the rectangular [virtual stop bar](batmobile-cheddar.md), which also
remains visible while braking at standstill.
