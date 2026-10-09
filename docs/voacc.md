# VOACC fixes

## Lead velocity fixes and E2E assistance

Use bounded early closing cues before mature range-derived lead velocity takes
over. Inconsistent cut-ins reset tracking history without hiding current range.
Urgent braking and stopped-lead constraints retain priority. Selective E2E lead
slowing qualifies below a model-estimated lead speed of 55 km/h, releases on
confirmed departure or override, and cannot weaken ordinary braking.

## Model-requested stops

Regular VOACC and CEM can use a sustained E2E stop prediction as a stationary
virtual lead only when neither accepted real-lead slot is present. It requires
fresh model data, enabled longitudinal control, forward gear and no pedal override;
SLC is not required. Full experimental
mode retains its existing model control.

Either real lead immediately clears the virtual target, its hold, and its extra
E2E stop candidate. MPC also ignores a supplied virtual-stop distance whenever a
real lead is present. This leaves the existing real-lead following/stopping gap
and selective lead-slowing assistance unchanged. After lead loss, a new stop target
must come from the current model prediction; no old target is restored.

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

Comma 3/3X draws a solid 1.8 m road-plane rectangle with subtly rounded corners,
following the upstream lead-bar profile. The yellow base and dynamic red fill
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
An accepted real lead also clears the stop bar, including a brake-held visual.
This display hold never retains or restores a planner obstacle.
