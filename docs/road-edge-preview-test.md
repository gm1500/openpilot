# Road-edge and speed-preview test branch

Based on `batmobile` at `3f09538824e57ca3b56a13d6d4c6ce1fafc7bd8d`.
This branch retains the original lane-centering gains, geometric midpoint target,
correction limits and entry/release rate limits. It does not add the previous
experimental 0.5-second curvature filter.

## Painted lanes and curves

The lane-target reversal pause in a curve is exactly three 20 Hz model frames
(0.15 s), replacing the original 0.30 s timer. Small sign changes inside twice
the correction deadband do not trigger it. Reversals during the pause update
direction history without extending that pause. The lane-motion estimator keeps
its history through this short release. The separate 0.35 s E2E turn-direction
guard remains. These changes address repeated correction release/reapplication;
they do not establish that every observed oscillation came from that mechanism.

## One road edge plus one painted line

A road edge can replace a weak/missing inner lane line only when the opposite
painted line supplies a reliable anchor. Two road edges alone cannot arm it.
An unmarked range road therefore does not qualify, and loss of the painted-line
anchor immediately clears the edge estimate, including any temporary hold.
Qualified two-line paint remains preferred; when recovering from edge centering,
both painted lines must be strong for 0.75 s before switching back.

| Check | Current test setting |
| --- | --- |
| Opposite paint probability | At least 0.92 to earn qualification; brief dips to 0.85 may pause it; 0.70 to retain an established boundary |
| Edge model standard deviation | At most 0.50 for normal qualification; up to 0.65 with longer, stricter stability qualification; 0.65 to retain |
| Geometric lane width | 3.0–4.6 m over 5–55 m; narrow near-field space rejected |
| Width spread over horizon | At most 0.75 m between the 10th/90th percentiles |
| Normal qualification | 15 strong observed frames within a 1.25 s window; confidence-only pauses at most 0.25 s each; width 10th–90th percentile span at most 0.30 m and latest width within 0.30 m of the window median |
| Moderate-uncertainty qualification | 30 strong observed frames within 2.0 s; temporal width span/latest deviation at most 0.20 m, spatial width span at most 0.50 m |
| Median-width smoothing | 1 s time constant; change at most 0.20 m/s; immediate inward geometry cap kept separate |
| Ordinary short hold | At most 0.30 s or 5 m, whichever comes first |
| Established curb, outward opening | At most 2.0 s or 30 m, with stronger continuous paint support |
| Width reference after active hold | At most 5.0 s or 100 m since full observed qualification; never steering permission by itself |
| Established curb, uncertainty increase | At most 0.50 s or 8 m; edge standard deviation must stay at or below 1.0 |
| Immediate rejection | Narrowing below minimum, loss of anchor, invalid geometry or lane-change intent |

These are experimental geometric gates derived from the supplied samples, not
certification of a road-design standard. Speed limits how far a held edge can
persist; speed alone does not manufacture a lane width. Adjacent-lane extrapolation
is not used to excuse a wide shoulder or multiple-lane corridor.

The extended bridge starts only with paint probability at least 0.92 and an
already qualified curb. Paint must then remain at least 0.85, with no near-field
paint jump above 0.20 m per model frame. A large inward change, weak paint or
unstable anchor cancels the extended budget; loss below the ordinary 0.70 paint
threshold or a corridor below 3.0 m still invalidates immediately. Three
consecutive agreeing edge observations are required to replenish a gap budget.
One or two isolated good frames cannot keep renewing it. Hold time and distance
use elapsed camera-frame time, including dropped frames. No new edge can qualify
from an opening, and its apparent extra width is never learned during a hold.

The filtered width is attached to the current opposite painted line. Curve shape
and vehicle motion therefore update immediately instead of being delayed by a
filter on camera-frame edge position. A credible narrowing reduces width promptly.
If centering was inactive, it reuses the edge's strong-anchor qualification rather
than starting a second 0.75 s timer. Initial engagement still needs currently
strong paint and an observed edge; a held boundary cannot start centering. Three
fresh observed frames are required before initial engagement after a hold.
During acquisition, a confidence-only dip may pause earned progress for at most
0.25 s while paint stays at or above 0.85. Edge uncertainty and width must still
pass admission checks, width must agree with the earned history, and painted-line
position/direction through 20 m must move by no more than 0.20 m per frame.
Weak frames earn no progress. Older strong observations expire after 1.25 s;
longer dips, missing paint, geometry changes or edge uncertainty above 0.65 clear
progress. A brief edge-uncertainty rise from 0.50 to 0.65 can pause normal
qualification under the same time/geometry limits, but cannot earn its progress.
The separate moderate-uncertainty path requires 30 strong painted-anchor
observations with tighter spatial and temporal width agreement.
Blinkers, model lane-change intent and invalid inputs retain the original fallback.
Steady-error bias learning is disabled for an inferred edge to avoid integrating
an uncertain curb offset; the painted-line centering gains remain unchanged.

The midpoint of this estimated corridor is still only an estimate. There is no
vehicle-footprint, obstacle, tire-clearance or construction-zone path planner in
this branch. It does not override reliable paint to cross a lane line around a curb.

## Display and logging

| Boundary | Candidate/armed | Used with lateral control active |
| --- | --- | --- |
| Painted line | Existing blue | Existing green |
| Qualified virtual edge replacing weak paint | Yellow, normal width | Green, double width |

The UI draws the same filtered virtual boundary used by the policy, through 55 m.
Unqualified edges keep their existing display. Fresh model/control state and no
steering override are required for active colours. `modelV2.lanePolicy` records
per-side sources, filtered geometry and short holds without overwriting raw model
lane lines, road edges or the model path. Older logs retain the legacy colour path.
Only the actively used virtual-edge side is thickened. The opposite painted line
keeps its original width and blue/green scheme. Colour and thickness are selected
at draw time, so disengagement/override removes both without waiting for another
model frame.

## Earlier speed selection

Enable **Settings → SLC → Advanced speed limit changes**. Lower limits/advisories
use a braking-distance approach trigger. A uniquely connected highway merge can
select its higher speed up to 8 s / 250 m before entry, after model/heading/curvature
agreement and two seconds of fresh, advancing position qualification. Mapped ramp
restrictions, ambiguous forks, ordinary higher speed zones, manual override and
the selected SLC acceptance policy retain their gates. See [SLC details](slc-offline-maps.md).

## Recorded-data checks

The on-ramp replay used route `000004e0--2ea04b3a41`, segment 2, recorded GPS,
vehicle motion and model paths, and refreshed public OSM geometry. It selected
100 km/h at 31.88 s, about 177 m before the mapped connection. The recorded first
100 km/h highway match was at 39.69 s: **7.81 s earlier target selection**.
The refreshed map is not a copy of the original device cache, and this replay
does not simulate acceleration, traffic response or the resulting driven path.

Road-edge qualification was replayed over `000004e2--6ad24210d9` segments 17–20
and 34–39, and `000004e5--29d4d078d4` segments 2–3 (14,400 model frames total),
with the supplied curb videos checked alongside the model estimates.
Segment 17's intersection/weak-paint geometry did not qualify. Segment 18's
left-hugging section gained a right-edge corridor; segments 19–20 provided right
and left curb cases. The two 4e5 segments exercised intermittent right edges.
Highway segments 34–38 did not select a road-edge fallback.

These are open-loop policy checks using recorded perception and vehicle motion.
Logged desired curvature already includes the recording branch's policy, so the
replay is suitable for checking boundary choice and transitions, not claiming a
closed-loop centering or oscillation improvement. On-device rendering, runtime
performance and supervised vehicle behaviour remain to be validated.

Host validation: 346 tests and 94 subtests passed across the map matcher, preview,
speed rules, SLC acceptance/E2E integration and lane-policy tests. The new cases
include unmarked roads, loss of the single painted anchor, brief openings, narrowing,
confidence chatter, exact pause duration, wire-schema round trips and source colours.
Changed Python files pass Ruff and compilation; the diff passes whitespace checks.
The native map-download/scheduler suites and a complete on-device UI/system build
were not run in this host environment.

### Follow-up: steady-curb continuity

Route `000004e6--d60f5fff86`, segments 1, 2 and 18, was recorded on the initial
version of this branch. The first lateral engagement is in segment 2. Replaying
the original filter reproduced its logged edge-availability states exactly.

The three active-centering releases in the steady curb section were caused by
the policy's width memory. Its inward geometry cap was repeatedly lowering the
stored width, and subsequent valid median-width observations then exceeded the
0.30 m compatibility threshold. Low model uncertainty and a confident opposite
painted line were still present at those releases.

The continuity reference now follows accepted observations independently of the
filtered median and conservative output cap. The inward cap also applies during
short holds; it cannot cause filter drift or justify adopting a large geometry
jump. Confidence thresholds, 3.0–4.6 m width bounds, 0.30 s / 5 m hold bounds,
paint-anchor requirements and curvature correction limits are unchanged.

On the same recorded inputs, the 4.83–15.73 s section now stays selected for
10.90 s, compared with 6.85 s and three releases previously. The earlier wide
geometry is still rejected, and the later opening still ends the edge fallback.
Segments 1 and 18 introduce no road-edge candidates. All three supplied videos
were reviewed and all 3,600 model frames replayed. This measures continuity of
policy selection, not closed-loop steering smoothness or tire clearance.

The follow-up passes 31 focused lane-policy tests, including the original 25 and
six additional cases for width drift, the recorded curb trace, bounded arming
holds, fresh recovery, strong-paint loss and inward movement during a hold.

### Follow-up: curb openings and confidence fluctuations

Reviewed the bookmark neighbourhoods in all 20 supplied videos from
`000004e7--05f29b7d3e`, segments 1, 5, 10, 13–19, 21–23, 25, 38–39 and 41–44,
and replayed all 24,001 model frames. The recording used `e3959e6`. The old-policy
replay reproduces edge availability except the first 2.4 s of segment 21, where
segment 20 was not supplied and the replay starts without the device's history.
Contiguous supplied segments retain state; missing segments reset replay state.

The main findings were an overly fragile temporal width window and short holds,
plus genuine loss of painted-line support at several intersections. A rolling
robust width window now tolerates an isolated old width outlier without restarting
qualification. It still needs 15 consecutive valid observations; missing paint,
excessive edge uncertainty, sustained width disagreement and an outlying latest
width do not earn qualification. Existing width/confidence limits remain.

On identical recorded perception and vehicle inputs (time counted in 20 Hz model
frames):

| Check | Previous policy | Updated policy |
| --- | ---: | ---: |
| Edge centering selected while lateral control active, no override | 106.50 s | 116.75 s |
| Segment 13 selected edge time | 30.30 s | 31.55 s |
| Segment 14 selected edge time | 23.45 s | 25.35 s |
| Segment 23 selected edge time | 3.80 s | 11.25 s |
| Segment 25 selected edge time | 2.25 s | 1.95 s |

The end of segment 13 shows a driveway opening ahead of an otherwise established
right curb. The updated policy bridges that interruption into segment 14 while
following current left paint at the established width. It still releases around
0.74 s into segment 14 when support deteriorates. Segment 23's solid curb is
selected at approximately 48.79 s instead of 56.24 s: the robust temporal window
allows earlier qualification despite isolated width fluctuations. The small loss
in segment 25 is intentional: three returning observations must agree before a
hold budget is refreshed, rather than accepting isolated good frames.

Wide corridors at the segment 10/21 bookmarks, low-confidence anchors at several
intersections, and the unmarked/construction examples do not justify wider or
unanchored virtual lanes. Segments 38, 39, 41, 43 and 44 still produce no qualified
edge. Segment 42 briefly qualifies the right curb, then rejects narrowing below
3.0 m. Segments 38–44 have lateral control off; those videos test detection only.
This change therefore bridges supported openings, not every intersection or
curb visible to a human. The earlier 4e6 segment 2 steady-curb regression remains
continuously selected through the previously analysed interval.

The curve reversal pause remains **0.15 s**. A diagnostic reconstruction of the
recorded base curvature (published action already includes our correction) was
checked against the rounded 1 Hz policy debug messages. Excluding cold-start
history, 185 matched samples agree within 0.000001 1/m. With identical recorded
inputs and the current guard logic, 0.15 s produces eight curve-guard entries
while controlling, seven in segment 5 and one in segment 13. Those occupy 1.20 s;
0.10 s occupies 0.80 s, while 0.30 s occupies 2.10 s across seven entries because
one closely spaced pair is combined. This compares timer settings in the current
algorithm, not every behaviour of the original batmobile timer. It does not
simulate how the changed steering would alter subsequent camera observations.
There is insufficient evidence here to claim a further reduction improves
handling, or that restoring 0.30 s fixes the remaining curve offset.

Validation: 52 focused tests pass, including three new recorded boundary traces,
left/right openings, time/distance expiry, paint loss, anchor jumps, narrowing,
confidence loss, intermittent recovery, admission robustness and active-side
colours. Changed Python files pass Ruff and compilation. The 3,600-frame earlier
4e6 sample was also replayed. No centering gains, correction limits, speed-preview
logic or vehicle-control interfaces were changed. These are open-loop selection
and regression results; on-device UI rendering and closed-loop steering behaviour
remain untested on this host.

### Follow-up: shared qualification and short painted-line confidence dips

Reviewed the eight supplied videos and replayed all 9,600 model frames from
`00000001--0383c98f11`, segments 2–4, and `00000005--c7a25a2e5c`, segments 13–17.
These were recorded on `batmobile` at `34c5a0f9`. Contiguous segments preserve
policy state. The baseline reproduces logged candidate and selected-edge time
exactly; active-edge time differs by one 20 Hz frame at a control-state boundary.

The edge's qualification is now shared with centering. Acquisition also preserves
strong observations across short confidence-only dips under the limits above.
This does not lower the confidence needed to earn admission or change steady
centering gain, width bounds, gap budgets, the 0.15 s curve pause, or UI colours.

| Replayed edge state | Previous policy | Updated policy |
| --- | ---: | ---: |
| Candidate available | 81.40 s | 82.45 s |
| Edge selected | 77.85 s | 82.45 s |
| Selected, centering active, lateral control active, no override | 68.15 s | 80.30 s |
| Route 5, segment 14 active edge | 0.00 s | 6.35 s |
| Route 5, segment 17 active edge | 3.35 s | 4.90 s |

No previously active edge frames are lost in this replay. Route 1 segment 4 and
route 5 segment 13 still do not qualify: a visible curb alone does not supply the
required painted anchor. Most of the gain comes from using an already qualified
boundary sooner; candidate availability increases by only 1.05 s. The new policy
does not claim that every curb in these videos can now be used.

Validation: 67 focused tests pass, including two additional recorded boundary
traces and cases for shared left/right qualification, paused versus earned
confidence, stale/disjoint evidence, elapsed-time expiry, anchor direction jumps,
width changes and recovery from holds. Existing rejection, curve, centering-gain,
schema and UI-colour regressions remain passing. Python lint, compilation and
whitespace checks pass. These are open-loop boundary-selection results; the
logged action already includes the recording policy's correction, so this replay
does not establish a closed-loop steering or oscillation improvement.

### Follow-up: earlier curb and barrier acquisition

On the speed-pause POC, route `00000008--1d6913bdff`, segments 1 and 7, exposed
another acquisition limitation. Some steady curb estimates had edge standard
deviation around 0.55–0.60, while the painted anchor was strong and the corridor
was within the existing width bounds. Repeated crossings of 0.50 also erased
otherwise consistent boundary evidence.

Normal qualification still needs 15 observations with paint probability at least
0.92 and edge standard deviation at most 0.50. Brief moderate edge-uncertainty
dips can pause it without earning progress. A second path requires 30 strong
observations within two seconds, permits edge standard deviation at most 0.65,
and tightens temporal width agreement to 0.20 m and spatial spread to 0.50 m.
At the normal 20 Hz model rate this is 1.5 s of earned evidence. It retains the
same painted-anchor, bracketing, 3.0–4.6 m width, jump, freshness and expiry gates.
The qualified boundary's output width is still capped inside the observed edge.

| New sample | Additional usable interval in replay |
| --- | --- |
| Segment 1, left curb | Active edge centering from approximately 14.04–19.29 s |
| Segment 7, right barrier | Candidate from 8.60 s; active edge centering from 9.40–15.61 s |
| Segment 7, later left curb | Candidate 0.20 s earlier; active centering still waits for lateral control |

These add 11.45 s of active edge selection on identical recorded inputs, with
no previously active edge frames lost. Segment 1 begins with a qualified boundary
inherited from missing segment 0; cold-start replay cannot reproduce that initial
state. The comparison excludes its first five seconds, after both replay and
recorded policy have released the boundary. Segment 7 availability matches the
baseline replay exactly; control-state alignment differs by one model frame.
The updated policy releases at subsequent openings and still rejects wide
corridors and lost paint. Review of both supplied videos confirms the added
intervals correspond to a continuous curb or barrier.

The previous eight supplied segments were also replayed (9,600 additional model
frames). Active edge selection changes from 80.30 to 80.80 s, with no active
frames lost. The unmarked/weak-anchor examples still produce no candidate.
Across all ten segments, 12,000 model frames were checked. These remain open-loop
boundary-selection checks, not a simulation of the resulting driven path.

Validation: 91 focused tests pass. Added coverage includes both sides, minimum
observation count, uncertainty spikes, spatial/temporal width variation, sparse
paint evidence, dropped-frame expiry, and recorded curb/barrier regressions.
Centering gains, correction limits, existing hold budgets, lane-change handling,
UI colours and the speed-dependent curve-pause POC are unchanged. On-device
steering behaviour remains to be evaluated.


### Longer hold and width memory (speed-pause POC)

The active opening hold is now capped at **2 seconds or 30 metres**, whichever
comes first. The ordinary 0.30 s / 5 m and uncertainty 0.50 s / 8 m budgets remain
unchanged. A remembered width is a prior, not permission to follow an unseen edge
for its entire lifetime. At least one current confident painted line is required.

After active hold ends, keep the last fully qualified width for up to **5 seconds
or 100 metres** since its last full confirmation. Render it offset from current
paint as a **thin yellow** virtual line, with `memoryOnly=true` and
`available=false`. It cannot supply the centerline. During active interpolation
or memory-assisted reacquisition, the selected virtual boundary is **thick
yellow**. Disengagement or steering override immediately removes thickness.
Fresh observed candidates remain yellow; fresh observed active edges remain
green. Painted lines keep blue/green. The virtual line disappears when its
reference expires or its supporting painted anchor becomes invalid.

Reacquisition using memory requires six consecutive matching observations
(0.30 s at 20 Hz), paint probability at least 0.92, edge uncertainty at most 0.65,
spatial width spread at most 0.50 m, and median width within 0.20 m of the saved
reference. It freezes the saved width, capped inward by current geometry. This
is stronger evidence than a missing edge but less than full acquisition, hence
the yellow display while used. Only the original full qualification windows
(15 strong or 30 stable moderate-uncertainty observations) can renew the memory
clock and teach new width. Short returns cannot keep it alive indefinitely.

Blinkers, lane-change intent, toggle-off, stale/backwards model timestamps and
lost paint discard the reference. Paint below 0.85 discards memory; large inward
changes require requalification. An anchor jump invalidates memory/held steering,
while simultaneous motion of valid observed paint and curb may retain observed
centering. Five seconds without lateral control (including driver override)
discards the prior; continuously observed boundaries can still qualify normally.
There is no two-road-edge fallback and no change to centering gains or the
speed-dependent reversal pause.

#### Replay against the preceding POC

Ten supplied segments, 12,000 model frames, compared against `968c678`:

| Metric | Previous | Width-memory POC |
| --- | ---: | ---: |
| Qualified edge available | 110.25 s | 110.90 s |
| Edge selected while controlling | 105.70 s | 105.80 s |
| Previously active frames lost | — | 0 |
| Display-only remembered boundary | — | 3.80 s |
| Active memory-assisted reacquisition | — | 0 s |

The added 0.10 s of active selection is in the right-barrier clip (route ending
`1d6913bdff`, segment 7). Most gaps end on other confidence/geometry gates rather
than the old timeout. This replay does not demonstrate a large availability gain
or exercise faster memory-assisted reacquisition; that behavior is covered by
synthetic tests and needs new recorded examples. Replay evaluates policy
selection against recorded model geometry, not closed-loop steering quality.
The earlier 1.1 s longest successful hold was constrained by the original cap
and did not establish that a longer hold would fail.

### Painted-lane width as a guarded edge prior

`RoadEdgePolicy` now keeps its own recent two-paint width reference, independent
of `LanePolicy.width` and that existing one-line hold's 9.95 s smoothing. It does
not reuse the old filter or teach painted width from a road edge.

Acquisition requires 15 consecutive observations with both inner painted-line
probabilities at least 0.92, a corridor bracketing the vehicle, widths within
3.0–4.6 m at every station through 55 m, spatial width spread at most 0.50 m and
temporal span/latest deviation at most 0.20 m. Near paint must move continuously.
A confident changed or implausible painted corridor invalidates the old reference
before collecting a new window.

The reference expires after 5 s / 100 m without renewed two-line evidence, on
lost/jumping remaining paint, on the existing policy resets (blinkers, lane-change
intent, toggle-off, stale/backwards frames), or after 5 s without lateral control.
Transfer into an edge candidate inherits the reference's original age/distance;
it cannot restart the clock. Each confirmed paint window may seed each side once.

It can assist acquisition only after paint on that side falls below 0.70, with
the opposite painted line still at least 0.92 and continuous. The road edge must
have uncertainty at most 0.65, median width within 0.20 m of the remembered width,
and all near-to-55 m stations within 0.30 m of the expected boundary. Shape
agreement is checked on subsequent observed frames too, not just the first seed.
Six agreeing observations are still required before use. Large openings cannot
start centering from paint memory alone. Existing qualified edge candidates are
not overwritten by a paint prior.

The virtual boundary always uses the current confident painted line's geometry
plus/minus the remembered width. It never replays a stored curve. Paint-assisted
memory follows the yellow display rules: thin when not actively controlling,
thick on the virtual side while used for centering. Full observed edge
qualification returns it to ordinary edge tracking/green active display. Existing
2 s / 30 m supported-opening and shorter uncertainty hold limits are unchanged.

Validation against `6f63771`: 140 focused tests and all 12,000 supplied replay
frames. Active edge time remained 105.80 s, available edge time 110.90 s; no
active selections were lost or added, and returned curvature was identical.
These recordings contain no eligible successful paint-assisted acquisition, so
this is regression evidence, not demonstrated real-drive benefit. Synthetic
tests cover both sides, live curve geometry, post-seed divergence, shoulder
spacing, expired references, repeated seeding, reset paths, and display state.

The added recorded fixture covers segment 15's mismatch: painted width around
3.39 m versus a left-edge corridor around 4.3–4.5 m near 38 s. The edge does not
receive accelerated qualification. Substituting that wider corridor would change
the target midpoint by roughly half a metre; these observations must not be
accepted merely because paint confidence and average edge uncertainty look good.

### Stable inward lane estimates constrain the curb corridor

The route ending `03e12d7807`, segments 9–15, contains an important disagreement:
while one painted anchor remains confident, a weaker same-side lane estimate
sometimes sits inside the accepted curb. The prior policy ignores that estimate
below its painted-line threshold and centres the wider paint-to-curb corridor.
The model's weak line may represent paint or an inferred boundary; the video
alone does not establish that each estimate is a real marking.

The test branch now uses consistent inward estimates to constrain the virtual
boundary. This does not change the perception model or promote a weak line into
an independent steering source. The curb must still qualify under every existing
gate and the opposite painted line remains the geometry anchor.

- Earn 15 observations with opposite paint probability at least 0.92 and inward
  line probability at least 0.10. Brief dips to 0.05 may pause, but cannot earn,
  qualification for at most 0.25 s within the existing 1.25 s observation window.
- Require the inward line to bracket the vehicle and form a 3.0–4.6 m corridor
  at every station through 55 m. Its spatial width spread must be at most 0.35 m,
  and temporal width span/latest deviation at most 0.20 m.
- Require the line to remain 0.10–0.80 m inside the observed edge throughout that
  horizon. Crossings, outward lines, speculative probabilities, and uncertain
  or held edges cannot earn this evidence.
- Apply only a width constraint, reconstructed from the **current opposite line's
  shape**. Acquisition and release move this width at no more than 0.20 m/s.
  An actual inward curb cap still takes effect immediately.
- Confidence loss on the weak side retains a previously confirmed constraint
  for up to 5 s or 100 m, then smoothly unwinds it toward the qualified curb.
  Only another complete qualifying window renews this reference. Opposite paint
  below 0.85, anchor jumps, lane-change/reset conditions and long disengagement
  clear it. It cannot extend active edge hold or permit memory-only steering.

The constraint does not alter raw curb width, edge admission, opening detection,
paint-width learning, centering gains, or the speed-dependent curve pause. UI
geometry follows the constrained boundary using the existing edge/memory colours.
Two confident painted lines still take priority after the existing handback time.
These are experimental evidence thresholds, not proof that a low-probability
line is a physical boundary or that a particular tire clearance is guaranteed.

#### Replay and checks

Compared against `158c664`, preserving state across consecutive segments:

| Metric | Before | With inward constraint |
| --- | ---: | ---: |
| New seven segments: available edge | 88.95 s | 88.95 s |
| New seven segments: active controlling edge | 75.55 s | 75.55 s |
| Earlier eleven segments: active controlling edge | 107.45 s | 107.45 s |
| Previously active frames lost, all eighteen segments | — | 0 |

In the new samples, the virtual midpoint moves at least 5 mm away from the curb
for 54.35 s of active edge centering. Among those frames, the median shift is
15.5 cm, the 90th percentile 22.9 cm, and the maximum 33.3 cm. These describe the
**inferred target**, not measured vehicle movement or curb clearance. Segment 9's
bookmark already uses confident painted lines, so this edge-only change does not
alter it. Segment 12 does not earn sufficient inward-line evidence and remains
unchanged. Other sections also retain the original target when the new gates fail.

Validation covers 21,600 recorded model frames and 166 focused tests, including
an eight-second anonymized boundary trace, both sides, live curve geometry,
confidence pauses, false/crossing lines, narrow corridors, source priority,
expiry, output slew and reset paths. Recorded model actions already include the
recording policy's correction. This is open-loop boundary/selection regression;
closed-loop curb clearance and handling still require a controlled vehicle test.
