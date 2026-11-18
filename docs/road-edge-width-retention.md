# Retain painted lane width through confidence dips

When both painted-line probabilities briefly fall below 0.85, keep the
previously qualified scalar lane width within its existing 5-second / 100-metre
lifetime, whichever expires first. Confidence loss alone no longer erases it.
Weak observations cannot teach a new width, renew its lifetime, or increase
the independent width reference's support. Reference confidence keeps decaying.

This changes memory retention, not boundary qualification. Current paint and
edge geometry must still qualify before the prior can assist reacquisition or
positioning. Reacquisition still requires six matching observations, inherits
the prior's age, and cannot repeatedly reuse a rejected seed. Reliable anchor
jumps, conflicting strong painted widths, turn/lane-change resets, stale
frames, and disengagement expiry remain in place. Inward boundaries still win.

## Recorded result

For route `615b11d4c01d81ec/00000058--572e4c0242`, segment 6, the original policy
erased a 3.40856 m width at 7.552 seconds when the probabilities became
0.837 / 0.682. It was only 1.30 seconds old. The revised policy retains that
width at the 10.207-second bookmark (nearest model frame: 10.204 seconds),
then expires it at approximately 11.252 seconds under the unchanged time cap.

Retention alone does not change the inferred boundary at that bookmark:
the existing edge-agreement gates still block use of the width there. This
does not claim to fix the observed left bias or add a paint-only virtual
boundary. It preserves information needed for subsequent qualification.

| Recorded replay | Model frames | Additional frames retaining width | Availability/hold changes | Available boundary geometry changes |
| --- | ---: | ---: | ---: | ---: |
| Route `58`, segments 0–7 | 9,526 | 343 | 0 | 0 |
| Route `5c`, segments 0–7 | 8,917 | 170 | 0 | 0 |

Comparisons above use the unchanged base policy and the revised policy with
the same recorded inputs. The base reproduced logged right-edge availability
with one mismatch on route 58 and zero on route 5c. These are offline policy
replays, not closed-loop driving tests.

## Validation

All 259 road-edge tests pass. New regressions cover moderate and severe
confidence dips, both sides, matching reacquisition, inherited age, weak-anchor
rejection, returning anchor jumps, mismatched/wider edges, inward boundaries,
and time/distance expiry while both lines remain weak. Existing tests continue
to cover turns, blinkers, lane changes, stale frames and UI serialization.

The change is isolated as the initial commit on
`batmobile-future-lead-prediction-poc`, based on `batmobile` at `b6d5aa4`.
It does not implement future-lead prediction or alter longitudinal behavior.

## Mature edge width anchored to current paint

The route `70` intersection exposed a separate cache: a qualified road-edge
width was discarded by a 50 ms painted-anchor confidence dip, although the
learned scalar was only 2.6 seconds old. Ordinary two-paint lane width and the
strict painted-width cache had not qualified a usable prior for this interval.

A mature edge now has an explicit `remembered-width lane center` mode. Maturity
requires 2.5 seconds of fully qualified edge renewals after initial admission
(about 3.2 seconds of qualifying corridor evidence at 20 Hz). This is a
conservative evidence-duration setting, not an optimized threshold. Its private
scalar can survive confidence loss for 15 seconds or 250 metres, whichever
comes first. Immature edges and paint-seeded edge memory keep their existing
5-second / 100-metre limits. A held opening cannot earn maturity or renew a lease.

Remembering width and allowing it to position the car are separate decisions:

- Positioning requires six consecutive strong, continuous live-anchor frames
  with probability at least 0.92, finite current edge geometry and edge standard
  deviation no greater than 1.0. Once admitted, a continuous anchor may retain
  the mode down to 0.85; moderate confidence cannot start or reacquire it. Below
  0.85 it stops immediately. It cannot center through an intersection where the
  painted anchor is missing.
- The scalar includes the inward constraints actually applied when it was
  learned. An opening cannot widen it, and expiry of a narrower paint/reference
  constraint cannot expose the wider raw curb. Current inward edge samples still
  cap every point; large inward changes discard the old association.
- Both the raw corridor and the final synthetic boundary must bracket the car.
  Turns, blinkers, lane changes, reliable anchor jumps, stale input and five
  seconds of disengagement still discard memory. Temporary conflicting strong
  two-paint geometry blocks anchor-only reuse while preserving the curb scalar
  for fresh observed-edge validation. Fully qualified conflicting painted width
  clears the old corridor. Current valid two-paint geometry takes precedence
  over either form of memory-assisted positioning.
- Returning observed edges validate the frozen applied width against the
  current inward boundary. Unsupported narrowing still rejects immediately;
  additional outward space inside the ordinary corridor bounds cannot widen
  the frozen output or erase it merely by changing the edge median. Six matching
  observations of compatible geometry may reacquire the prior. Fresh observed medians update only the
  continuity check; a full fresh qualification is required to renew the lease
  or learn a different width.

The new mode uses the existing single-anchor correction fit, not the farther
shape fit for two observed boundaries. Diagnostics name the mode explicitly;
`lanePolicy` publishes its virtual side as an available held `roadEdge` (yellow).
It never increments observed-edge evidence for paint-only positioning. No
longitudinal or lead-tracking behavior changes.

Regressions cover both sides, maturity boundaries, weak-anchor suspension and
requalification, private lease expiry, fragmented returns, immediate inward
caps, final boundary bracketing, reset conditions, conflicting paint and
preservation of an applied narrower constraint through expiry/reacquisition.
Recorded replay and closed-loop vehicle behavior are different validations;
recorded inputs cannot establish the vehicle's counterfactual steering motion.

Final validation runs 295 road-edge tests, including two full `LanePolicy`
recorded-geometry regressions. Independent review adds 16 adversarial checks.
Paired replay covers 26,697 frames across routes 70, 58 and 5c, plus 1,564 frames
in 11 existing boundary fixtures. No replay violates the checked inward caps,
final boundary brackets or reset gates. Route 58 and all 11 fixtures are unchanged;
route 5c gains three active frames and loses none.

On route 70 the right boundary becomes continuously available from 191.799 to
200.005 seconds instead of starting at 193.249 seconds, an improvement of
1.45 seconds. Existing steering-override/control gaps still suspend active use.
The full route gains 126 active frames (6.3 seconds) and loses none. The retained
raw edge width is 4.27061 m, but the applied inward constraint is 3.80231 m;
carrying that narrower cap is intentional. These results establish retained
information and earlier eligibility, not measured dead-center vehicle tracking.
