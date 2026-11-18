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
