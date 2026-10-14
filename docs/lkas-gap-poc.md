# LKAS state and combined gap/lane control POC

Branch: `batmobile-lkas-gap-poc`, based on batmobile
`f43e7f0ac9b24cf76a7d9c304e04eac79a418142`. The resume/set POC is a
separate branch and is not included here.

## Combined control

The existing bottom-right lane-policy button is replaced with one circular
control in both UI layouts. It uses the gap icon's tapered side lines and
three horizontal bars, with bars thickened from 6.5% to 9.5% of icon height.

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

For this state-following POC, selfdrived applies the first valid bit and each
subsequent state change to `LanePolicyEnabled`: 1 selects it, 0 deselects it.
Invalid CAN, bus timeout, and stale/unseen signals do not change the policy.
An unchanged bit does not repeatedly overwrite a HUD long-hold override.
The next physical state change takes precedence. State applications are
also logged as `gm_lka_button_state`.

This is the requested test interpretation, not confirmation that the Sierra
reports a latched stock LKAS on/off state. Record a parked press, hold, and
release and compare the two raw fields with the side lines. If the bit is
momentary, the physical input will select the policy only while pressed;
that result means a press-to-toggle adapter is needed before normal use.
If the bit never changes, the truck's physical switch needs a different
signal mapping. The HUD long hold remains usable in either case.

The existing model's centering blend, steering engagement, and CAN transmit
commands are unchanged. Selecting this lane policy does not engage steering
or disable openpilot lateral control. No Panda safety changes are required.
The branch must be installed with its matching opendbc submodule and a
normal rebuild because the logged CarState schema has two added fields.

## Validation

23 focused host tests (plus 20 subtests) pass. They cover real GM CAN packing/parsing, correct bus and raw
bit, transition events, concurrent Cancel preservation, missing/stale input,
policy state following and HUD overrides, tap/hold exclusion including
queued releases, repeated holds, cancellation, gap acknowledgements, and
all six bar/lane colour combinations. Hardware switch behavior and a
full device build remain to be verified. Ruff, Python compilation, whitespace
checks, and C++ generation from the updated Cap'n Proto schema pass.

The generic `test_car_interfaces_CHEVROLET_SILVERADO` test fails its
centre-of-gravity assertion: the existing `centerToFrontRatio=0.75` exceeds
that test's 0.7 upper bound. The exact failure was reproduced on the untouched
base `63c1fe0` with `FUZZ_SEED=3994426387935828367 FUZZ_EXAMPLE=0`.
This branch does not alter that vehicle setting.
