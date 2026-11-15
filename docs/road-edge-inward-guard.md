# Road-edge inward bounds

The disengaged preview previously discarded every inward-line constraint after
five seconds without lateral control, even when a stable inner line remained
visible. A valid but wider curb/railing estimate could then become the displayed
virtual boundary. Fresh inward evidence now keeps constraining an independently
qualified edge in that preview.

When the disengagement memory lease expires, painted-width reuse remains disabled
and missing inward evidence gets only the existing 0.25-second qualification
pause. Normal output-width smoothing still applies. Weak paint cannot qualify
an uncertain, overly wide, or unanchored road edge.

Virtual boundaries and their display memory also use the minimum observed width
over every model sample from 0 to 55 m. A single inward intrusion below the
minimum lane width rejects the candidate immediately. Percentiles still qualify
overall shape, but can no longer hide an intrusion beyond the near-car window.

The one-confident-painted-line requirement, width/confidence thresholds, two-edge
prohibition, and active correction limits remain unchanged. These checks bound
the model geometry; they do not identify barriers or guarantee physical vehicle
clearance when the model itself is wrong.

Validation: 207 tests across test_road_edge_policy.py,
test_road_edge_inner_line.py, test_road_edge_paint_memory.py, and
test_road_edge_width_refine.py. Added cases cover both sides, fresh versus expired
disengaged constraints, uncertain/wide edges, and near/mid/far inward intrusions
in observed and held states. Offline boundary replay preserved admission windows
in the sampled logs and narrowed the railing example. No closed-loop road test
has been performed.
