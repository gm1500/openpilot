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
minus the ordinary 6 m gap). A target must be over 6 m earlier to enter; an active
target releases within a 5 m car-length overlap band of the real-lead stop. This hysteresis gives
VOACC priority for overlapping stops without switching on small prediction jitter.
The comparison uses the model stopping position, with no artificial early-stop
offset. A positionless stop request with a real lead creates no new
virtual assistance. Existing selective lead assistance and CEM remain independent.

For a separate earlier target, MPC considers it alongside both real leads over
the prediction horizon. A car moving through that stop cannot disable it. If a
previously covered stop becomes earlier than the real-lead stop, it is reacquired
from the current model prediction. Real-lead gap tuning is unchanged; a separate
model stop can require stopping earlier. Position matching is a POC heuristic,
not a classifier: the model does not label lights, signs, or stopped traffic, and
genuinely separate stops within the matching band cannot be distinguished.

Each model observation is compensated for travel since its timestamp, then
filtered and tracked without an artificial early-stop offset. The separate 6 m
MPC standstill gap is accounted for by placing the virtual obstacle 6 m beyond
the tracked target. It is not an additional
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
painted stop lines; stopping accuracy still needs closed-loop driving validation.

## Stable trajectory endpoints

A stable trajectory endpoint is always an additional virtual stopped-lead
trigger in regular VOACC and CEM. The existing sustained near-zero trajectory
trigger still activates immediately; neither waits for `shouldStop`. Full
Experimental Mode retains its model longitudinal control.

For an endpoint-only trigger, the model must predict a converging, slow tail
with at least 1.5 s left in its 10 s forecast: at most 4 m of path travel and
speed magnitude at most 2.5 m/s. Tail speed must fall to at most 35% of the
initial prediction, with a drop of at least the larger of 1 m/s and 8% of ego speed. An already
stationary vehicle instead requires the entire path to stay within 0.75 m and
all predicted speeds at or below 0.5 m/s. A short path, turn, slow constant
creep, or merely reaching the finite forecast boundary is insufficient.

Across distinct fresh frames, the endpoint must remain within a bounded range
for 0.3 s after compensating for measured vehicle travel and model age. The
allowed range is 5% of endpoint distance, bounded between 1 m and 3 m, retaining
the tighter tolerance near the vehicle. This is a stability tolerance, not a
stopping offset. Use the existing near-zero stopping location when present,
otherwise the actual forecast endpoint. The relaxed tail limit does not move
the target backward to the tail's start. No artificial early-stop offset is added.

After acquisition, either continuing stop trajectory can maintain the target.
The explicit model stop action still requests braking/hold immediately, without
inventing a position. Fresh withdrawal releases the target; pedal, engagement,
freshness, and real-lead overlap checks are unchanged. This does not freeze the
final stop position or change its existing filter.

`longitudinalPlan.stopTarget.horizon*` records the candidate, stability,
candidate distance, remaining forecast time, observed stability duration, and
motion-compensated endpoint spread. The bar uses the selected planner target.
The manual/full-experimental display continues to show the original model cue.

Replay of four segments covering three approaches (2d3/19, 2d4/4, and
2de/10-11) now produces an endpoint-only green display on each approach.
On 2de, the first endpoint qualifies at 47.65 s: about 5 s earlier than the
previous endpoint check and 0.8 s before the first near-zero stop cue. Its
initial planner request lasts one frame; existing display-only brake retention
keeps green visible from about 47.66 to 48.41 s before yellow confirmation.
Qualification can still be intermittent. These samples do not establish
improved stopping accuracy or broad false-positive performance. Replay uses
recorded vehicle motion and brake outputs, not a closed-loop policy rollout.

## Stop bar

Trajectory-only virtual targets use a solid green bar at half width (225 px).
A sustained near-zero stop trajectory, immediate model stop request, or planner
stop hold uses the full-width yellow/dynamic-distance-colour bar. The stop/hold style remains while braking;
display-only brake retention preserves the last style until release. Both
stages share the same thickness, rounded trapezoid profile and target position.
This is visual only; both stages are active virtual stopped leads.

Comma 3/3X draws a solid ground-perspective trapezoid with subtly rounded corners.
Both edges stay horizontal; the far edge is 80% of the near edge's width, including
when docked. Its fixed 450 px near-edge width is about 2.5 times
the close lead triangle's outer width, bounded to fit the viewport. Thickness is
twice the original bar, including its docked display. The stopping point anchors
the bar, but road curvature and camera roll never rotate it. The yellow base and dynamic red fill
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
Real-lead overlap suppression takes priority over every display hold. The same
5 m band and 6 m re-entry threshold apply to manual/full experimental MODEL STOP
displays, so an overlapping bar disappears even while any brake source holds.
A clearly separate earlier stop can still appear alongside real-lead triangles.
This display hold never retains or restores a planner obstacle.
