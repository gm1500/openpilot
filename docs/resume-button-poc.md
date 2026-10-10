# Sierra manual resume experiment

Branch: `boat-anchor-resume-button-poc`, based on `boat-anchor` 9a0d1bb.

This is a supervised, closed-course diagnostic for the Sierra camera-harness
direct-longitudinal platform. It is not automatic stop-and-go. A deliberate
screen press temporarily overrides the existing model stop to test whether
GM accepts propulsion during a commanded brake-release gap.

## Gas full-stop flag experiment

This revision keeps `GasRegenFullStopActive = 0` on every outgoing gas/regen
command for the Sierra/Silverado camera-harness direct-longitudinal platform,
including the approach, full stop, manual release pulse, and rehold. It is
branch-wide and does not require pressing TEST RESUME to take effect.
The experiment tests whether omitting the flag prevents factory ACC from
entering standstill; clearing it after entering standstill did not resume
factory cruise in route `000002e5`.

Friction-brake full-stop mode (13), brake amount, the calibrated -540 Nm
braking torque request, and `GasRegenCmdActive` retain their existing behavior.
The button's pulse duration, acceleration limit, cancellation, and rehold
latch also retain their existing behavior. Only the gas full-stop flag and
its corresponding checksum change on the wire.

The existing GM camera-longitudinal safety configuration (`safetyParam = 3`,
confirmed in route `000002e5`) already blocks camera bus 2 gas/regen (0x2CB),
friction-brake (0x315), and ACC-status (0x370) messages from forwarding to
vehicle bus 0. Camera-side copies remain visible in logs; their presence is
not evidence of forwarding. This experiment does not modify Panda safety.

For a meaningful closed-course comparison, record an approach from moving
with cruise active into an openpilot-held stop, then a manual test pulse.
Starting the test after factory cruise is already in standstill does not
test whether the flag causes entry. Compare outgoing `GasRegenFullStopActive`
and brake mode against incoming `CruiseState`, wheel speed, and engine response.
The truck may still enter standstill based on brake mode or its own zero-speed
logic; the flag's effect and stationary hold must be verified on the vehicle.
Keep the driver ready to brake if hold weakens or movement is unexpected.

## Manual release button

The large **TEST RESUME** button appears only with fresh vehicle/control data,
openpilot active, Drive selected, measured speed below 0.05 m/s, and an actual
openpilot friction-brake hold. Release the pedals before using it.

Hold the button to request +0.3 m/s² through the existing longitudinal controller
and GM torque conversion. An attempt lasts at most one second and ends at
0.5 m/s (1.8 km/h), touch release, missing heartbeat (200 ms), invalid/stale
inputs, FCW/AEB, or a nearby/closing lead. The lead veto requires at least 6 m
plus two seconds of closing distance. These checks cannot establish that a
scene is clear; the driver must supervise the experiment.

After timeout/release, the calibrated stopping command returns immediately
and stays latched against model stop/departure flicker. A new press after a
one-second cooldown can repeat the test once the truck is stationary and held.
Holding a finger down never repeats an attempt. Physical Resume exits the
test hold back to the normal planner; it does not force that planner to go.
Pedals or disengagement also exit the test. No test request survives a restart.

No Panda safety limits, GM message layouts, hardware Resume messages, throttle
ceiling, or normal no-touch planner behavior are changed. Outgoing gas full-stop
flag behavior changes even without a screen press, as described above. The positive command
is a request, not a guarantee that the ECM will accelerate the truck.

Logged evidence:

- `resumeTestRequest`: touch ID (initial monotonic timestamp), held heartbeat.
- `controlsState.resumeTest`: readiness, active pulse, hold latch, ID, remaining
  time, and reason (`pulse`, `released`, `timeout`, `speedLimit`, `blocked`).
- Existing `carControl`, `carOutput`, `sendcan`, and wheel/engine CAN signals
  show commanded brake/torque versus the vehicle response.

The planner continues publishing its unmodified stop request during the manual
override, so compare it with the test state and actual actuator commands.
The POC has offline state-machine/control/CAN checks; on-vehicle behavior and
successful factory cruise resumption remain unvalidated.
