# One lane change per signal activation

The previous desire helper returned from `laneChangeStarting` directly to
`preLaneChange` when the signal was still on. It could therefore request another
lane change on the next model update (50 ms later). It also cleared its signal
history while lateral control was inactive, treating an existing signal as a
new activation on engagement.

The helper now accepts a request only when both signals were off on the previous
update, exactly one is on now, and lateral control was active on both updates.
It tracks the signals continuously, including while disengaged. Completion
returns to `off` regardless of the signal state.

| Situation | Behavior |
| --- | --- |
| Fresh signal activation while engaged and idle | One request, subject to the existing minimum speed and blind-spot checks |
| Signal held after completion or timeout | No repeat; switch off and back on to request another lane change |
| Engagement or re-engagement with a signal already on | No request |
| Signal activation in the same update as engagement, or helper startup with a signal on | No request; a later off/on cycle is required |
| Signal activated below minimum speed | No deferred request when speed rises |
| Valid request waiting for the blind spot to clear | The same request may proceed once clear |
| Pending request loses its signal, changes direction, or falls below minimum speed | Cancel; require a fresh off/on cycle |
| Hazards, including hazards changing directly to a single signal | No request until both signals have been off |
| Signal cycled during an ongoing maneuver | No queued second maneuver; the current maneuver retains its direction |

An activation is determined from the vehicle's `carState` blinker status, not
individual lamp flashes or a new raw stalk input. The existing completion
probability, minimum maneuver duration, timeout, speed threshold and behavior
after an already-started maneuver loses its signal remain unchanged.

## Validation

`openpilot/selfdrive/controls/tests/test_desire_helper.py` covers 18 regression
scenarios, including both directions, held signals, startup, engagement,
re-engagement, hazards, blind spots, cancellation, timeout and re-arming.
The new regression tests reproduce the unwanted behavior on the parent code
and pass with this change. Combined with the existing modeld lane/road-edge
tests, 277 tests passed (plus 14 subtests). Ruff and whitespace checks passed.

A 65-update synthetic comparison with the parent reproduced a second request
50 ms after completion. Holding the signal produced six starts in the parent
versus exactly one after the fix; engaging with an already-active signal
produced five starts in the parent versus zero after the fix. These are host
state-machine tests, not an on-device drive validation or a new model replay
of the reported overshoot.
