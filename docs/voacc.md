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
offset. A positionless stop request creates no virtual obstacle. Lead overlap
suppresses the duplicate obstacle without suppressing a fresh E2E action or
discarding the current endpoint observation. Existing selective lead assistance and
CEM remain independent.

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

## E2E approach and confirmed stop

A converging trajectory immediately enables the E2E longitudinal action in
regular VOACC and CEM. This green approach stage previews the endpoint without
creating a stationary MPC obstacle or committing to a stop position. The model
action competes with ordinary cruise and real-lead braking; stronger braking
wins. Full Experimental Mode retains its existing model longitudinal control.

For an endpoint-only trigger, the model must predict a converging, slow tail
with at least 1.5 s left in its 10 s forecast: at most 4 m of path travel and
speed magnitude at most 2.5 m/s. Tail speed must fall to at most 35% of the
initial prediction, with a drop of at least the larger of 1 m/s and 8% of ego
speed. An already stationary vehicle requires the entire path to stay within 0.75 m and
all predicted speeds at or below 0.5 m/s. A short path, turn, slow constant
creep, or merely reaching the finite forecast boundary is insufficient.

There is no endpoint-stability delay for enabling E2E. The obsolete 0.3 s
stability window and spread calculation have been removed.
The green preview follows the current endpoint and can refine in either
direction; it never seeds the committed stop filter.

A sustained near-zero trajectory acquires a stationary virtual lead at its
current stopping position. An explicit `shouldStop` also confirms a current
usable endpoint; without a position it requests hold without inventing an
obstacle. The existing real-lead overlap band still gates virtual obstacles.
No artificial early-stop offset is added.

After confirmation, continuing stop geometry can maintain the target through
near-zero speed jitter. Fresh withdrawal releases assistance and the obstacle;
pedal overrides, disengagement and unhealthy data clear them immediately.
The confirmed target retains its existing filter and cannot recede.

`longitudinalPlan.stopTarget.horizonCandidate`, `horizonDistance` and
`horizonRemaining` record the candidate, distance and remaining forecast time.
Old stability fields retain their schema ordinals for existing logs but are no
longer computed or published. `stopTarget.approaching` and
`approachDistance` identify E2E approach and its age-compensated preview;
`active` and `distance` identify an actual MPC virtual obstacle.
`e2eStopActive` includes both approach and confirmed-stop E2E requests.
The manual/full-experimental display continues to show the original model cue.

On 2df/3, replay enables approach E2E at 16.86 s, approximately 77 m ahead at
55 km/h, 2.25 s before the previously logged virtual-target activation.
The virtual obstacle still waits for confirmation; first entry is 19.11 s.
Real-lead overlap no longer discards the model action during the handoff.
Replay also covers 2d3/19, 2d4/4, 2de/10-11 and the 2df/1 resume sample.
Raw geometry can still withdraw briefly. Recorded motion and brake outputs
are reused; these are not closed-loop stopping-accuracy or false-positive trials.

## Sierra brake release

With Sierra/Silverado openpilot longitudinal control, the planner and brake
pedal govern release from stopping. Stock `cruiseState.standstill` cannot keep
our own brake hold applied after the planner permits departure. Other vehicles
and stock longitudinal behavior retain the existing standstill gate. Resume
does not override a fresh planner stop request or the brake pedal.

In 2df/1, Resume was received at 19.73 s. The planner subsequently cleared its
stop and requested acceleration while stock standstill stayed true, leaving
the old controller in stopping. The narrow control change removes that wait;
later stop requests in the recorded trace still reapply hold.

## Stop bar

The E2E approach preview uses a solid green bar at half width (225 px), labeled
E2E APPROACH. Its displayed distance is smoothed over 0.5 s with ego-motion
compensation and clipped to the current available path. Adjacent planner/model
frames must not send a distant green preview to the near-stop dock; an
unprojectable green cue is hidden instead. This filtering is display-only and
does not alter E2E activation, braking, or the confirmed virtual stop.

Brief approach dropouts retain the preview for 0.25 s, then fade it over 0.15 s,
only while fresh model data still indicates slowing. Departure, pedal takeover,
disengagement, invalid data, or real-lead overlap clears it immediately. Stop
confirmation immediately replaces the preview. Applied brakes alone do not
retain green indefinitely. In 2e1/14-15 replay, the green phase changes from
three flashes with 22/30 unprojectable frames to one continuous interval with
35/35 projectable frames, approximately 45-65 m down the road.

A confirmed virtual stop uses the full-width yellow/dynamic-distance-colour bar,
labeled STOP TARGET. Its display-only brake retention remains until release.
Both stages share the same thickness and rounded trapezoid profile. Green
previews the smoothed endpoint; yellow shows the committed stopping position.

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
