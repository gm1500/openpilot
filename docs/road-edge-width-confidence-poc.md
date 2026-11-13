# Confidence-weighted road-edge positioning POC

Base: `batmobile` at `e2e1d9126b3dd714b4be326c23893a0e9f2406dd`, after
fast-forwarding all nine commits from `batmobile-speed-pause-poc`.
Test branch: `road-edge-width-confidence-poc`.

A plausible but distant railing can place the inferred lane center too far
outward. This POC lets a recent, repeatedly confirmed painted-lane width reduce
the influence of a wider edge. The opposite live painted line supplies the
geometry on every frame, including curves; no old ego-frame coordinates are held.

## Evidence and weighting

The existing `PaintWidthMemory` must first qualify a stable window with two
strong painted lines. `WidthReference` takes the lower quartile of those
qualified window medians as a robust narrow-width reference, retaining the
median and variation too. It does not take a lifetime minimum. An isolated
narrow measurement cannot permanently ratchet it inward, and a confirmed width
change invalidates the old reference before learning the new one.

Only fresh two-paint confirmations earn support time. Existing road-edge
qualification and edge-width memory remain separate. Neither observed edges,
held edges, weak inner lines, generated boundaries nor clipped outputs teach
this new reference. This makes it independent of the current edge estimate,
not an independent perception sensor.

| Quantity | POC setting |
| --- | --- |
| Initial support | Existing 0.75 s qualified paint window |
| Full maturity | 8 s of confirmed paint support |
| Reference statistic | Lower quartile of up to 160 qualified window medians |
| Freshness expiry | Existing 5 s or 100 m, whichever comes first |
| Painted anchor while weighting | At least 0.85 probability, with existing continuity checks |
| Memory weight | 4 times confidence |
| Output width rate | 0.20 m/s; center moves at half this rate for a fixed anchor |

Confidence combines support duration, width stability, and remaining time and
distance budgets. Holding never increases support or refreshes the lease.
Blinkers, lane-change intent, model turn intent, a lost/discontinuous anchor,
long disengagement and stale frames invalidate the corresponding association.
Ordinary curves do not reset it. Turn detection uses model desire and the
existing blinker/geometry gates; this is not map-based road identity tracking.

An already qualified edge supplies the baseline width after existing inward
bounds and inner-line constraints. If that baseline is wider than the reference,
the target is `(memory_weight * reference + edge_weight * baseline) / sum(weights)`.
Edge weight decreases with edge uncertainty and disagreement with remembered
width. It cannot make an unqualified edge available or extend a hold window.

E2E supplies additional evidence only when its trajectory supports the remembered
center over the shared 8–55 m horizon. The plan must cover at least 8–30 m, be
finite and have increasing forward coordinates; there is no extrapolation.
Consistent improvement across the horizon reduces edge weight, provided the
remembered center is within 0.35 m of E2E at the 90th percentile. Missing or
incompatible plans supply no additional preference. E2E is never substituted
for the lane center or used to qualify an edge.

The blend can only narrow the existing candidate. Every existing inward bound
still wins immediately. On reference expiry or paint recovery, an existing
weighted output unwinds at 0.20 m/s of width while the candidate remains valid;
that filter tail does not retain the learned reference or prolong eligibility.
Memory-only display geometry is bounded consistently and cannot steer.

Two real painted lines retain their original target and correction gain.
The 0.30-to-0.05 s reversal pause by 100 km/h is unchanged. Memory-weighted
boundaries use the existing yellow indication: thin when inactive and thick
only on the side actively used for centering. Real paint keeps blue/green.

## Validation and limits

Run:

```sh
python -m pytest openpilot/selfdrive/modeld/tests/test_road_edge* -q
```

247 cases pass, including 40 new cases for confidence growth, stale evidence,
noise, width changes, both sides, curves, E2E consistency, invalid plans,
inward intrusions, paint recovery, expiry, intersection gaps, association resets,
two-edge rejection and UI serialization. Ruff and `git diff --check` pass.
The prior inner-line slew assertion now checks that limiter separately and
allows the new reference to narrow its output further.

A differential replay against the base covered all 11 checked-in recorded
boundary fixtures (1,564 frames). Acceptance/hold states and underlying edge
width learning were unchanged. Available boundary geometry also stayed unchanged:
these short fixtures did not exercise a qualified wider edge with a usable
narrower reference. Their plans are absent, so this replay deliberately omits
E2E weighting. It is regression evidence, not measured improvement on the
railing route.

Synthetic tests cover the new behavior: after sustained confirmation of a
3.6 m painted lane, a newly qualified 4.2 m edge with E2E supporting that lane
keeps the inferred center within 5 cm of the remembered center, instead of
the unweighted 30 cm displacement. This measures the inferred boundary center,
not actual closed-loop vehicle tracking.

No full-route replay with recorded plans or road test has been performed for
this POC. The current 5 s/100 m reference budget is intentionally unchanged;
older learned widths provide no new weighting. Lane boundaries remain model
hypotheses, without vehicle-footprint or obstacle-clearance guarantees.
