# Sierra held Resume/Set rolling experiment

Branch: `batmobile-resume-set-poc`, based directly on `batmobile`
`15b257929323138f4aa67cb2ca09368f93ad1a90`. The matching opendbc branch starts
at `63c1fe02b20f9df99367671ae060652d16bbd00d`; openpilot pins its tested revision.

This is a manually supervised, closed-course diagnostic for the Sierra/Silverado
camera-harness direct-longitudinal configuration and the standard comma 3X HUD.
It sends real CAN messages. It is not automatic stop-and-go. Validate the
transmission/receiver behavior on a bench before controlled vehicle testing.
Keep the driver's brake available; commanding a stop is not proof of brake
pressure or a guarantee that the factory cruise state will return to Standstill.

## HUD behavior

Two bottom-center buttons, **RESUME TEST** and **SET TEST**, select separate
attempts. They arm only from a measured stationary openpilot brake hold with
factory `CruiseState = 4` (Standstill), openpilot fully enabled, Drive selected,
fresh input data, and healthy Panda safety configuration **11** (`1 | 2 | 8`).
There must be a released UI request before a new press can start; a held request
at process startup cannot launch an attempt.

Hold one button to request brake release and at most **+0.3 m/s²** through the
existing torque conversion. The ordinary planner continues running and its
stop request remains logged; this deliberate manual test temporarily overrides
that model stop. There is no learned launch integral during the attempt.

The selected button is transmitted in repeated bursts: five press frames,
two neutral/unpress frames, and at least 0.5 s between burst starts. Native
neutral steering-button observations set the cadence; each injected message
uses the next counter modulo four, 10–25 ms after that observation, at most one
message per observation and no faster than one per 25 ms. The recorded physical
Resume held five native frames for about 150 ms. Interleaving injected and native
messages is a hypothesis to verify on this Sierra, not a validated equivalent
of holding its physical switch.

When the factory reports **Active (1)**, further press bursts stop and a bounded
neutral release is sent. The low-acceleration rolling attempt remains active
while held. A later return to Standstill does not automatically restart bursts
within that attempt. The HUD displays the actual raw factory cruise state,
remaining test time, integrated movement, and attempted CAN frame count.

Release, slide off the selected button, touch a second finger, or slide to the
other button to end the attempt. Switching from Resume to Set requires release,
stopping, a one-second cooldown, and a fresh press. Both are never sent together.
The input thread supplies a separate fresh down-state snapshot; a stalled input
thread cannot keep a hold alive just because the UI is still rendering.

## Bounds and stopping

An attempt ends on the first of:

- Touch release or lost UI heartbeat (200 ms); input polling stale beyond 100 ms.
- **10 seconds**, **2.0 m/s (7.2 km/h)**, or **10 metres** of integrated attempt movement.
- Reverse movement, invalid vehicle/control data, disengagement, driver pedals,
  AEB/FCW, driver-monitoring forced deceleration, a door/seatbelt/gear fault,
  or a lead closer than 6 m plus two seconds of closing distance.
- Panda fault, invalid safety RX checks, lost controls permission, a mismatched
  safety configuration, or an increase in the safety TX-blocked count.
- Missing native button observations or a watchdog rejection in CarController.

These are abort thresholds, not guaranteed stopping distance/speed limits. The
10 m measurement excludes the distance needed to stop afterward. The lead gate
cannot establish that the path is clear. The recorded successful transition
was at about 6 km/h after downhill roll; stationary or flat-ground resumption
has not been demonstrated.

Ending an attempt latches a stopping request against later planner departures.
While moving, the POC requests approximately -0.8 m/s²; once measured stationary,
the existing calibrated `stopAccel = -0.37` hold is restored. Direct torque/brake
conversion and the existing hardware limits remain authoritative. CarController
independently enforces heartbeat, speed, distance, duration, and the hold latch,
so a frozen positive carControl request cannot continue the release indefinitely.

Normal `batmobile` full-stop signaling remains in use. During active rolling,
the gas full-stop flag is cleared and friction-brake release is requested. On
ending the attempt, stopping returns; when stationary, gas full-stop and friction
mode 13 return. The earlier branch-wide full-stop-flag removal is not included.

The HUD distinguishes **STOP / HOLD REQUESTED** from **STOPPED / BRAKE COMMANDED**.
The latter means fresh measured standstill and a positive brake command, not
measured hydraulic pressure. Factory Standstill is displayed separately.

Pedals or disengagement exit the test and restore ordinary driver/engagement
behavior. Use the brake or Cancel to finish testing. Physical RES/SET events are
suppressed within openpilot during the diagnostic and for 0.5 s afterward so
received copies cannot change its set speed or re-engage it. Cancel/gap events
are preserved. Do not use a physical Resume press to exit the diagnostic hold.

## Panda and build requirements

The dedicated feature/safety flags are applied only to the Sierra camera-harness
direct-longitudinal configuration on this branch. Panda gains a narrow vehicle
bus 0 `ASCMSteeringButton` permission only for the exact debug safety configuration
11. Presses require controls permission, fresh rear wheel/cruise/native button
observations, factory Standstill, low speed, no driver pedals, the expected next
counter, correct checksum/constant bits, and the transmit-rate bound. Other
button values, payload shapes and buses are rejected. Neutral release remains
possible after abort. Recently injected presses cannot re-arm Panda through RX
echoes. Production/release builds do not enable this diagnostic permission.

Direct-longitudinal torque/brake limits and camera ACC forwarding blocks are
unchanged. This does not enable either fork's `CC_LONG` mode. A full rebuild,
including Panda firmware, is required; this is not a Python-only branch switch.
The standard development build imports the pinned opendbc safety source, and
pandad compares/flashes the corresponding firmware signature on startup.
The development branch must be checked out with its pinned submodules.

Route `000003e4` reported `interruptRateCan2` even though controls were allowed.
If it persists, this POC will show **PANDAFAULT** and refuse to arm. Diagnose the
fault instead of removing the gate to get an attempt to run. Likewise,
**PANDANOTREADY** can indicate old safety configuration/firmware, and **INPUTSTALE**
indicates required data is unavailable. A blocked attempt does not initiate
experimental button presses or the brake-release override.

## Evidence and validation

`resumeSetTestRequest` logs the selected button, original press ID and held
heartbeat. `controlsState.resumeSetTest` records eligibility, active/hold state,
abort reason, movement/time, raw factory cruise state and whether Active was
observed. `carControl.gmResumeSetTest` retains the original UI heartbeat timestamp
across processes. `carOutput.actuatorsOutput.gmResumeSet*` records the independent
controller outcome. The HUD's TX count records attempted sends, not receiver
acknowledgments; inspect `sendcan`, incoming CAN and `pandaStates` together.

Focused tests cover gesture release/multitouch/polling loss, stale requests,
restart behavior, bounds, hazard gates, latched rehold, both button sequences,
the recorded Resume payloads, raw cruise-state parsing and echoed-event filtering.
Generated button/gas/brake frames are passed through compiled Panda safety.
The GM safety suite includes the new configuration plus the existing variants,
gas/brake/steering limits and forwarding rules. A release-mode safety build is
checked to reject the experimental button permission.

No complete comma-device build, bench receiver test, or on-vehicle validation
has been performed in this workspace. Successful button acceptance and reliable
factory-state recovery remain unverified.

References used to choose the sender strategy:

- [opgm GM sender](https://github.com/opgm/opendbc/blob/9130ffd4fd53147f6791685be2686c256d3115cf/opendbc/car/gm/gmcan.py)
- [StarPilot GM sender](https://github.com/firestar5683/StarPilot/blob/2a12dbd0ad94b46f8a7b6099d32d7216b7676f79/opendbc_repo/opendbc/car/gm/gmcan.py)

Their non-adaptive cruise set-speed controllers are not copied; this POC retains
the Sierra's existing direct-longitudinal controller.
