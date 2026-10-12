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

Speed-based camera zoom is available in every driving mode, including manual
and regular VOACC. Calibrated alignment and a 200 ms live two-stream crossfade
work in both directions. Brief invalid input retains the request; missing streams
are retried, and offroad resets the animation. Shared camera behavior also applies
to comma 4.

The engagement border shows a brighter red center quarter along the top while
driver, vehicle-hold or applied openpilot braking is active. GM does not currently
decode physical brake-lamp status; coasting alone does not light this strip.
Orange blind-spot brackets follow the border's thickness and corner profile,
wrapping slightly onto top and bottom with short faded ends. Signaling toward an
occupied side flashes its bracket red at 2 Hz. Feedback also works in manual drive
and is drawn over the on-road content.

Real-lead triangles retain their top profile with broader rounding at both side
corners. Virtual stops use the rectangular [VOACC stop bar](voacc.md), which also
remains visible while braking at standstill.
