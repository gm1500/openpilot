# Offline GM Resume-button simulation

This adds an offline simulator on top of `boat-anchor-resume-button-poc`
commit `ee903a1`. It does not add a live Resume press to the screen button and
does not change vehicle control, firmware, safety permissions, or forwarding.

Run on a development computer with this checkout and its pinned opendbc:

```sh
python -m openpilot.tools.replay.gm_resume_button_sim /path/to/rlog.zst --check-safety
python -m unittest openpilot.tools.replay.tests.test_gm_resume_button_sim -v
```

For each complete recorded TEST RESUME attempt, the tool models one native
`ASCMSteeringButton` (0x1E1) frame as Resume (2), followed by an explicit
unpressed frame (1) in the next native slot. It preserves the slots' original
counters, handles counter wrap, and uses the existing GM button checksum.
Frames with physical button input, unknown fields, invalid checksums, stale
timing or counter discontinuities are excluded. Holding the screen button
does not produce repeated presses. Input logs are never modified.

The output is JSON describing hypothetical replacements. There is no CAN
transmit interface, `sendcan` publisher, vehicle connection, or live-enable
switch. `--check-safety` compiles and evaluates the existing safety hooks in
host memory; it does not access a Panda device. The tool does not simulate
ECM acceptance, actuator response, stopping distance, or factory cruise state.

Validation on route `000002e8`, segment 2 modeled all four recorded attempts
as one press/release pair each (approximately 21-39 ms between recorded slots).
All eight candidate frames were rejected by the unchanged Panda transmit
policy. Six tests cover packet/checksum encoding, counter wrap, repeated
requests, physical button priority, missing/stale/corrupt inputs and the
compiled safety policy. These results validate the offline model only.

## Why live injection is not included

Route `000002e8`, segment 2 shows the factory button stream on vehicle bus 0
at roughly 30 ms intervals. The camera harness does not intercept the source
of that stream before the vehicle ECUs receive it. Injecting another copy
would create a second sender; the replacement-slot model would no longer
describe the real bus. Reusing a counter or advancing it ahead of the native
sender may be rejected by receiving ECUs. Their response is unverified.

The pinned GM camera-longitudinal safety configuration (`safetyParam = 3`)
does not permit transmitting 0x1E1 on bus 0 or bus 2. The existing GM helper
is used for camera-side Cancel with stock longitudinal control; that does
not establish a vehicle-side Resume path for this configuration.

A live implementation needs a validated native Resume trace at a stopped,
openpilot-held vehicle and a bench check of the transmission/counter strategy,
followed by narrowly scoped independent safety tests. A physical Resume
press currently exits the test latch to the normal planner; account for
that behavior when interpreting such a trace. Do not simply allowlist this
message or enable a permissive safety mode to run the offline candidates.
