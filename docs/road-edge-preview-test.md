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

| Check | Initial test setting |
| --- | --- |
| Opposite paint probability | At least 0.92 to qualify; 0.70 to retain |
| Edge model standard deviation | At most 0.50 to qualify; 0.65 to retain |
| Geometric lane width | 3.0–4.6 m over 5–55 m; narrow near-field space rejected |
| Width spread over horizon | At most 0.75 m between the 10th/90th percentiles |
| Temporal qualification | 15 consecutive model frames; width range at most 0.30 m |
| Width smoothing | 1 s time constant; widening at most 0.20 m/s |
| Short outward opening | Hold at most 0.30 s or 5 m, whichever comes first |
| Immediate rejection | Narrowing below minimum, loss of anchor, invalid geometry or lane-change intent |

These are experimental geometric gates derived from the supplied samples, not
certification of a road-design standard. Speed limits how far a held edge can
persist; speed alone does not manufacture a lane width. Adjacent-lane extrapolation
is not used to excuse a wide shoulder or multiple-lane corridor.

The filtered width is attached to the current opposite painted line. Curve shape
and vehicle motion therefore update immediately instead of being delayed by a
filter on camera-frame edge position. A credible narrowing reduces width promptly.
If centering was inactive, the qualified edge also passes the original policy's
0.75 s engagement timer before it begins applying correction.
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
| Qualified virtual edge replacing weak paint | Orange | Yellow |

The UI draws the same filtered virtual boundary used by the policy, through 55 m.
Unqualified edges keep their existing display. Fresh model/control state and no
steering override are required for active colours. `modelV2.lanePolicy` records
per-side sources, filtered geometry and short holds without overwriting raw model
lane lines, road edges or the model path. Older logs retain the legacy colour path.

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
