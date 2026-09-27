# Confidence branch: stop feedback and speed-holding follow-up

Route 27c, segments 2–7, was recorded on `lead-confidence-poc` at `102ac92`.
The segment-5 bookmarks follow a brake override near the final stop. Immediately
before the override, ego speed was 0.42 m/s and the planner requested about
−0.212 m/s². A retained +0.051 m/s² integral correction reduced the controller
command to −0.161 m/s². Measured deceleration was about −0.12 m/s². This identifies
a contributor to weak final braking, not every physical cause of the event.

Confidence filtering was already bypassed at that speed. A native stock-planner
counterfactual using absolute/zero near-stop lead speed did not materially
strengthen the final creep command, so this update leaves lead estimates alone.

## Low-speed feedback

For Silverado/Sierra with openpilot longitudinal control enabled, Ki is 0.20
through 2 m/s, blends linearly to 0.05 at 5 m/s, and stays 0.05 above that speed.
Previously it was 0.05 everywhere. Faster low-speed feedback unwinds a stale
correction during the final approach and also corrects excessive braking.

The recorded-motion controller replay produces −0.259 m/s² at the last active
sample, versus −0.161 before, with the same −0.212 planner request. Mapped brake
requests at that controller sample increase from 23 to 38 command units. These
are computed requests, not a measured improvement in stopping distance.

The higher gain also applies during low-speed acceleration. A changed integral
state can persist above 5 m/s until feedback corrects it or the controller resets;
unchanged high-speed gain does not imply identical subsequent vehicle commands.

## Speed-holding crossover

Earlier route 272, segment 8, shows repeated zero/one-unit friction-brake entries
while holding about 65 km/h. Each entry also switches to the existing encoded
braking torque request. That encoding is not measured physical wheel torque.

The Silverado controller now suppresses a one-unit brake request only when:

- Speed is at least 5 m/s.
- The controller is outside the stopping state.
- No friction brake was previously applied.

Two-unit and larger requests pass on the current normal actuator update. Once
braking has started, one-unit requests are retained; zero releases immediately.
Lower speeds, stopping, other platforms and factory ACC keep their existing
behavior. Disengagement clears the applied-brake state. No timer or held minimum
brake request is added.

This deliberately omits some tiny friction-brake requests and can delay an
increasing onset until two units. It is an actuator comfort experiment, not a
rule to suppress lead braking. In the illustrated one-second route-272 window,
recorded-request replay reduces brake entries from three to one. Across all 34
segments, entries above 5 m/s decrease from 101 to 95. Route 27c itself has little
zero/one cycling: nine entries before and after. Its larger lead-controlled
slowdowns cannot be called phantom braking from these logs alone.

## Preserved policy

No changes to the stock planner/MPC, following costs, stop distance, 0.3 m/s stop
threshold, V3 tracker, confidence filter, controller state machine, actuator
limits, delay, stop-hold acceleration, brake lookup tables, CAN encoding or safety
limits. The opendbc revision changes to carry the two actuator adjustments.
The published `lead-cont-v3` and opendbc `brake-map` branches are unchanged.

## Validation

- 202,850 controller updates across 34 segments from six uploaded routes; 296
  unavailable/startup-input samples excluded. Baseline replay matches 169,658
  active PID commands within 1.2e-7 m/s². Controller state decisions match between
  variants when given the same recorded motion and stop flags.
- A full comparison of 36 GM parameter configurations changes only Silverado
  openpilot-longitudinal Ki. Factory ACC and other configurations are unchanged.
- 4,536 real controller/CAN comparisons against the previous implementation show
  only the three expected one-unit onset differences in the sampled grid.
- Twelve GM brake-map/fingerprint tests pass, including stateful hysteresis,
  bypasses, disengagement, command limits and CAN encoding.
- Five controller regressions pass, including 18 delayed-plant combinations
  covering underbraking/overbraking, launch feedback, high-speed equivalence from
  equal initial state, continuous gain boundaries, stop hold and disengagement.
  The simple test plant is not a validated truck model.
- Ruff and whitespace checks pass. Controller replay and the five new tests use
  actual LongControl/PID source with offline adapters for unavailable runtime
  imports; ordinary device process execution is not reproduced here.

No native device build or road test was performed. Recorded-motion replay cannot
prove an earlier stop, a shorter stopping distance, improved road feel, or a
general reduction in phantom braking. The stock stop decision stays identical
in replay because ego motion is held fixed.

On a built checkout:

```sh
python3 -m unittest openpilot.selfdrive.controls.tests.test_longcontrol_gm
cd opendbc_repo
python3 -m unittest opendbc.car.gm.tests.test_brake_map opendbc.car.gm.tests.test_gm
```
