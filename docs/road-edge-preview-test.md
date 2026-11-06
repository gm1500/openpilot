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
| Opposite paint probability | At least 0.92 to qualify; 0.70 to retain |
| Edge model standard deviation | At most 0.50 to qualify; 0.65 to retain |
| Geometric lane width | 3.0–4.6 m over 5–55 m; narrow near-field space rejected |
| Width spread over horizon | At most 0.75 m between the 10th/90th percentiles |
| Temporal qualification | 15 consecutive valid frames; width 10th–90th percentile span at most 0.30 m and latest width within 0.30 m of the window median |
| Median-width smoothing | 1 s time constant; change at most 0.20 m/s; immediate inward geometry cap kept separate |
| Ordinary short hold | At most 0.30 s or 5 m, whichever comes first |
| Established curb, outward opening | At most 1.25 s or 20 m, with stronger continuous paint support |
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
If centering was inactive, the qualified edge also passes the original policy's
0.75 s engagement timer before it begins applying correction. A bounded hold
pauses that timer while the opposite paint remains strong; held frames never
earn qualification time. Three fresh observed frames are required before initial
engagement after a hold. Loss of strong paint still resets engagement progress.
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
