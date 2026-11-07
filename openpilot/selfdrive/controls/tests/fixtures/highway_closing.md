# Highway closing-speed regression

`highway_closing.json` contains 199 frames from seconds 50–60 of a recorded
highway approach, running commit `8129eaa177fb1df06034851aea0b31ad996982f9`.
Only relative time, ego motion, model lead estimates, logged lead acceptance
and logged output speed are retained. Location, device identifiers, absolute
time, video and unrelated parameters are excluded. Speeds and ranges are
estimates, not independently measured ground truth. Units are seconds, metres,
metres per second, metres per second squared and radians per second.

`modelSpeed` is the vision fallback speed after correcting for the difference
between model ego speed and measured ego speed. `filteredProbability` and
`present` preserve the recorded acceptance decisions. `loggedSpeed` is the
original published lead speed for comparison. The regression runs the ordinary
distance tracker, confidence filter and lead-transition recovery in order.

The original disagreement guard reset useful distance history at 55.50 s:
its private speed was about 72 km/h while the model reported 112 km/h. A
shrinking range with a substantially slower lead can produce this disagreement
without a target change. When both the private speed and the range slope's
two-sigma upper bound indicate closing within the existing 13-second
pre-closing horizon, retain the history. Far, ambiguous handoffs still reset.
Per-frame association gates, confidence requirements, covariance qualification,
speed blending, braking guards and planner settings remain unchanged.

Two older synthetic cases expected a reset for an 85 m gap shrinking at 15 m/s.
That sequence is also consistent with a real slower vehicle, so the tests now
require preserving its closing evidence. The reset and immediate stopped-lead
checks are retained with a farther, less urgent ambiguous handoff instead.
Separate tests cover abrupt range and lateral changes, cut-outs, steady noisy
following, and a 120-to-80 km/h approach with optimistic model velocity.

Run from the repository root:

```sh
python -m unittest openpilot.selfdrive.controls.tests.test_lead_control \
  openpilot.selfdrive.controls.tests.test_lead_transition \
  openpilot.selfdrive.controls.tests.test_highway_closing
```

The recorded approach regression fails on the original tracker. In the fixed
replay its distance estimate qualifies at 55.95 s instead of 57.50 s, about
1.55 seconds earlier, with the existing transition blending. The driver's brake
input was at 57.23 s; later recorded ego motion is affected by that intervention.

A host replay of the actual radard update logic and Capnp output covered 3,599
model updates in this approach and two earlier recordings. The mainline segment
had no changed outputs. The earlier lane-change segment changed for six frames
(about 0.30 s): lead-one speed was at most 0.22 km/h lower or 3.53 km/h higher
than baseline. Lead presence, probability, acceleration and acceleration decay
were unchanged across all three segments. The recovery observer supplied no
upward correction in the closing approach.

Replay uses recorded ego motion and host stand-ins for IPC infrastructure. It
does not validate native MPC, closed-loop braking, on-device timing, or behavior
on unobserved roads. The result establishes earlier usable lead-speed evidence,
not an equivalent measured improvement in vehicle braking time.
