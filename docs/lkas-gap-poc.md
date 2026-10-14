# LKAS state and combined gap/lane control POC

Branch: `batmobile-lkas-gap-poc`, based on batmobile
`f43e7f0ac9b24cf76a7d9c304e04eac79a418142`. The resume/set POC is a
separate branch and is not included here.

## Combined control

The existing bottom-right lane-policy button is replaced with one circular
control in both UI layouts. It uses the gap icon's tapered side lines and
three trapezoid bars, with bars thickened from 6.5% to 9.5% of icon height.
Both ends of every bar use the same linear profile as the adjacent lane
line, offset inward by 5.5% of icon width. The bar sides are parallel to the
lane lines and leave the same clearance at the top and bottom of each bar.

- Tap: cycle the existing longitudinal personalities in the physical gap
  button's order. Aggressive lights the bottom bar, standard the bottom two,
  and relaxed all three. Only acknowledged `selfdriveState.personality` is
  displayed. No following-time, stopping-distance, or braking tune is changed.
- Hold for 0.6 seconds: toggle `LanePolicyEnabled` once. Continuing to hold
  and releasing afterward do not change the following gap.
- Side lines are green when the lane policy is selected, grey when it is off.
  They indicate selection, not whether lane geometry currently permits the
  policy to take control. Bar colours are independent of this selection.
- Sliding outside, a second touch, hiding the control, or going offroad
  cancels a gesture. A new press is required. Queued press/release events use
  their input timestamps, so a slow render frame cannot convert a hold into
  a gap-changing tap.
- Missing/stale personality data greys out the bars and prevents gap changes.
  The lane-policy hold remains available.

## Physical LKAS input

The GM parser reads bus 0 `ASCMSteeringButton.LKAButton` (CAN `0x1E1`, bit
23 in the DBC, high bit of the third byte). `carState` records:

- `gmLkaButton`: the last observed raw bit.
- `gmLkaButtonValid`: a received, fresh signal (under 100 ms); false for an
  unseen signal or stale data. The raw value is retained when invalid.
- `buttonEvents` of type `lkas` on observed transitions, without fabricating
  an initial press or a release when reception is lost.

For this state-following POC, the first valid bit establishes a baseline and
does not change `LanePolicyEnabled`. Subsequent observed bit transitions
still select the policy: 1 selects it, 0 deselects it.
Invalid CAN, bus timeout, and stale/unseen signals do not change the policy.
An unchanged bit does not repeatedly overwrite a HUD long-hold override.
The baseline is logged as `gm_lka_button_baseline` with `policyApplied=false`.
Later state applications are logged as `gm_lka_button_state`.

This remains a test interpretation, not a verified Sierra switch mapping.
Signal freshness (`gmLkaButtonValid`) does not prove that the bit represents
the dash switch or the stock system's on/off state. If a future capture
shows a momentary press/release instead of a latched selection, a
press-to-toggle adapter is needed. The HUD hold operates independently.

### Parked Sierra recording, 2026-10-14

The uploaded 34.23-second recording from the original POC commit `fc82358`
contained 1,140 received `0x1E1` frames on bus 0 and 155 on bus 2. The third
byte was `0x04` in every frame, so `LKAButton` was always zero on both buses.
There were no `lkas` button events. The original startup logic incorrectly
applied that first zero at about 3 seconds and turned the policy off.
Replaying all 3,201 recorded CarState samples through the corrected helper
produces no policy writes.

StarPilot uses this same bit and DBC position, choosing the source bus by
vehicle platform. Merely changing bus selection cannot fix this recording.
Its current GM implementation is in
[StarPilot carstate.py](https://github.com/firestar5683/StarPilot/blob/StarPilot/opendbc_repo/opendbc/car/gm/carstate.py).

GM's [2020 Sierra 1500 electrical manual](https://www.gmupfitter.com/wp-content/uploads/2021/05/20_Sierra_1500_Electrical_Body_Buil-1.pdf)
documents separate hardwired switch and indicator circuits. The B174W
camera connector, printed page 7-185 (PDF page 349), has switch circuit
3153 at pin 10 and indicator circuit 3152 at pin 4. The S171L dash switch
connector, printed page 7-443 (PDF page 607), lists those same circuits.
The camera also has a separate low-speed GMLAN connection at pin 11.
[Comma's GM harness schematic](https://github.com/commaai/hardware/blob/master/harness/v3/boards/gm/gm.sch)
routes those three pins through PT3, PT2, and PT4 respectively, separately
from the intercepted high-speed CAN pair. This is a concrete reason not
to assume the generic steering-wheel `LKAButton` bit describes this switch.

The bundled low-speed DBC contains switch and lane-status messages, but no
extended CAN identifiers appear in this recording. Their presence in a DBC
is not evidence that this harness/log exposes them, or that their bit layout
applies to this truck. No replacement mapping has been enabled.

Shane's [PR #26010](https://github.com/commaai/openpilot/pull/26010) suppresses
the factory camera's Take Steering alert by forwarding `PSCMStatus`
(`0x184`) with `HandsOffSWlDetectionStatus=1` and an adjusted checksum.
That modification is applied on camera platforms at 10 Hz regardless of
the dash switch. It is steering-module feedback to the camera, not an LKAS
switch-state report. In all 342 original bus-0 frames in this recording,
`HandsOffSWDetectionMode=1`, `HandsOffSWlDetectionStatus=1`, and
`LKATorqueDeliveredStatus=2`; none changes with the reported button presses.
The original bus-0 data, rather than our rewritten camera-bound copy, was
used for this check. The existing alert-suppression implementation is unchanged.

For further signal identification, capture a parked run after startup has
settled: leave the switch off for 10 seconds, on for 10, off for 10, on for
10, then off for 10. Record the actual press times and indicator state, and
include a separate two-second button hold to distinguish press/release
events from a latched state. Avoid other switch inputs during that window.
Compare native CAN changes with those labels before using a candidate for
policy selection. If no correlated message is present, a different read
interface to the factory camera or low-speed network will be necessary.

The reported touch issue was resolved by a device reboot. Touch input,
filtering, and gesture handling have not been changed in this update.

The existing model's centering blend, steering engagement, and CAN transmit
commands are unchanged. Selecting this lane policy does not engage steering
or disable openpilot lateral control. No Panda safety changes are required.
The branch must be installed with its matching opendbc submodule and a
normal rebuild because the logged CarState schema has two added fields.

## Validation

25 focused host tests (plus 22 subtests) pass. They cover real GM CAN packing/parsing, correct bus and raw
bit, transition events, concurrent Cancel preservation, missing/stale input,
startup baselining, policy state following and HUD overrides, tap/hold exclusion including
queued releases, repeated holds, cancellation, gap acknowledgements, and
all six bar/lane colour combinations, trapezoid slopes, and lane clearance.
All six icon states were also rendered for visual inspection. Hardware switch behavior and a
full device build remain to be verified. Ruff, Python compilation, whitespace
checks, and C++ generation from the updated Cap'n Proto schema pass.

The generic `test_car_interfaces_CHEVROLET_SILVERADO` test fails its
centre-of-gravity assertion: the existing `centerToFrontRatio=0.75` exceeds
that test's 0.7 upper bound. The exact failure was reproduced on the untouched
base `63c1fe0` with `FUZZ_SEED=3994426387935828367 FUZZ_EXAMPLE=0`.
This branch does not alter that vehicle setting.
