# VOACC fixes

## Lead velocity fixes and E2E assistance

Use bounded early closing cues before mature range-derived lead velocity takes
over. Inconsistent cut-ins reset tracking history without hiding current range.
Urgent braking and stopped-lead constraints retain priority. Selective E2E lead
slowing qualifies below a model-estimated lead speed of 55 km/h, releases on
confirmed departure or override, and cannot weaken ordinary braking.

## Model-requested stops

Regular VOACC and CEM can use a sustained E2E stop prediction as a stationary
virtual lead alongside both accepted real-lead slots. It requires
fresh model data, enabled longitudinal control, forward gear and no pedal override;
SLC is not required. Full experimental
mode retains its existing model control.

Before adding a virtual target, compare the raw model stop against the nearest
real lead's equivalent ego stop (lead distance plus its stopping-equivalent travel,
minus the ordinary 6 m gap). A target must be over 2 m earlier to enter; an active
target releases when it is within 1 m of the real-lead stop. This hysteresis gives
VOACC priority for overlapping stops without switching on small prediction jitter.
The comparison precedes the 1.5 m margin, so the margin cannot create a duplicate
stop behind a car. A positionless stop request with a real lead creates no new
virtual assistance. Existing selective lead assistance and CEM remain independent.

For a separate earlier target, MPC considers it alongside both real leads over
the prediction horizon. A car moving through that stop cannot disable it. If a
previously covered stop becomes earlier than the real-lead stop, it is reacquired
from the current model prediction. Real-lead gap tuning is unchanged; a separate
model stop can require stopping earlier. Position matching is a POC heuristic,
not a classifier: the model does not label lights, signs, or stopped traffic, and
genuinely separate stops within the matching band cannot be distinguished.

Each model observation is shifted 1.5 m earlier before filtering and tracking
vehicle travel. The separate 6 m MPC standstill gap is accounted for by placing
the virtual obstacle 6 m beyond that adjusted target. It is not an additional
6 m early-stop margin. Real-lead gaps and brake calibration are unchanged.
Stronger model or real-lead braking still wins.

Nearer observations are filtered over 0.2 seconds; the committed target cannot
recede. A fresh withdrawal releases the obstacle immediately. At rest, a positive
model action plus a sustained departure trajectory also releases it. Stale or
invalid inputs, reverse, disengagement and pedal overrides clear the target.
A close target holds below 0.3 m/s; an explicit model shouldStop can request hold.
A stop request without a usable position does not invent an obstacle.

Telemetry exposes stopTarget.active, distance and holding, with source stopTarget.
e2eStopDistance remains the raw, unshifted prediction for comparison. Legacy
schema fields remain readable. Model stop positions are predictions, not detected
painted stop lines; the new margin still needs closed-loop driving validation.

## Stop bar

Comma 3/3X draws a solid road-aligned rectangle with subtly rounded corners,
following the upstream lead-bar profile. Its fixed 450 px width is about 2.5 times
the close lead triangle's outer width, bounded to fit the viewport. Thickness is
twice the original bar, including its docked display. The yellow base and dynamic red fill
share the regular lead's distance/closing-speed calculation. Real vehicles retain
their usual triangles. STOP TARGET shows adjusted remaining travel; manual/full
experimental MODEL STOP displays the approximate raw model prediction.

The bar replaces the earlier cyan flag and post. Its projection includes the
nominal camera/model-to-front offset (1.52 m by default, adjustable after measuring
ModelStopFrontOffset). This display offset never changes vehicle travel or the
planner's target. Close/off-screen targets remain visible as the same bar at the
screen edge; unavailable geometry is identified instead of inventing a road point.

If a target is withdrawn while the driver pedal, vehicle brake hold, or fresh
applied openpilot braking remains active, only its visual is retained. Measured
travel continues to reduce its displayed distance. It clears when all brake
sources release, or immediately on stale inputs, reverse, a new drive, or gas.
The stop bar can remain visible alongside real-lead triangles.
This display hold never retains or restores a planner obstacle.
