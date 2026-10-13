# Sentry mode — comma 3X

Integrated into `batmobile` as a single feature commit, including the camera
calibration, recording/upload fixes and motion eye refinements.

Manage the feature under **Settings → Sentry** in any vehicle state, including onroad.
All configuration controls remain editable; enabling Sentry onroad takes effect when parked.
Capture still requires an active parked session. On first use it defaults on;
previously saved enable/disable choices are retained, including an explicit STOP.
The status row reports ignition/offroad/health blockers, missing camera frames, missing encoded
video or an unavailable controller. Camera warmup and arming have separate countdowns once video is arriving; the
buffer counter confirms capture. The voltage row shows the current vehicle supply reading.
This implementation is ready for parked hardware evaluation; comma 3X encoding,
thermal behavior, battery draw, and Connect indexing/playback still require device validation.

## Behavior

- Requires a comma 3X, a recent valid panda/device state, ignition off and `IsOffroad`.
  Driving controls, modeld and locationd are never started for sentry. Ignition/onroad
  stops capture; driving and sentry encoders cannot run simultaneously.
- Starts filling the RAM buffer after the offroad transition. Once both wide cameras
  deliver frames and the recorder has buffered video, a separate 10-second camera warmup
  runs before the selected 15/30/60-second arming delay (default 30). With the default, motion
  arms 40 seconds after video is ready. The detector learns the changing exposure throughout
  both stages; automatic events and the eye wait until both finish. Manual saves work during
  either stage. Once armed, the capture session stays armed through quiet periods, completed
  events and brief camera/recorder gaps. A real camera reconnect or frame-counter reset gives
  that lens a new 10-second exposure warmup without repeating the user's arming delay or
  disarming the other lens. Missing frames before initial arming restart the initial countdown.
  A recorder-only status delay or empty buffer pauses new events, but confirmed motion can
  still extend an already-open event. A capture restart, including camera preview/livestream
  or thermal pause, starts a new warmup and arming cycle.
- Samples only the front-wide and cabin-wide cameras at 5 Hz for luminance changes. The eye reacts immediately
  to smaller local movements. Saving instead requires about 0.6 seconds of sustained
  movement in the same connected region, allowing gaps no longer than half a second.
  Saving area is scaled by each lens's configured focal length and image dimensions.
  Both wide lenses give the middle horizontal third a 2x weight across the full width,
  tapering continuously to the baseline 1x at the top and bottom. The same weight applies
  to the region's saving area and fresh-motion score for the eye and preview selector.
  At sensitivity 75, OX03C10 requires about 0.5% weighted area (0.25% actual connected area
  inside the central band); OS04C10 uses its own optics, about 0.73% weighted area.
  The narrow camera is still recorded for detail, but cannot trigger an event or take the
  preview. Separate snowflakes cannot pool their area. Fisheye size compensation is
  approximate, especially near the edges; this is not a physical object-size estimate.
  Exposure compensation, neighboring-pixel support and frame-to-frame change checks reject
  noise and stationary background afterimages. At sensitivity 50, both wide lenses use a
  maximum contrast threshold of 10 luma levels in brighter cells. Each cell adapts to its
  own background brightness, reaching 1.5 levels in very dark areas; bright sky or windows
  no longer choose the threshold for the entire image. The full bright-cell threshold is
  reached at background luma 128. The slider still changes contrast and saving area.
  Each cell in the
  128×80 detection image averages a 4×4 grid of sensor samples, reducing isolated-pixel noise
  without blurring a dim outline across neighboring cells. A measured frame-difference noise
  floor raises thresholds in noisy scenes. Fresh changes also need local spatial support,
  preventing scattered noise inside a departed subject's background footprint from sustaining
  an event. A fresh change must leave a meaningful difference from the learned background.
  Bright and dark regions are separated so a small passing speck cannot merge with its
  opposite-polarity departure trail into a larger object. The 0.6-second confirmation still applies.
  This is motion detection, not object identification;
  large nearby droplets, heavy snow, shadows, reflections and passing traffic can still
  trigger it. Small/distant objects or brief movement may not save an event. There are no
  person, vehicle or license-plate rules in this version.
- All three cameras are always recorded; obsolete camera-disable preferences are ignored. Device targets are 526×330, 20 fps, HEVC at 256 kbit/s
  per camera, plus a separate 256 kbit/s H264 preview buffer for each enabled camera.
  This retains pre-event previews from whichever lens confirms motion. With all cameras
  enabled, the nominal one-minute compressed-video payload is about 11.5 MB, before logs/overhead.
  The RAM ring retains native minute boundaries around the selected history; saved footage is
  trimmed to the selected seconds. Actual variable bitrate varies. Encoded payloads have a hard 128 MiB RAM cap; IPC queues,
  camera buffers and private watermark buffers use additional RAM.
- No route directories or video files are written while merely watching. Runtime control
  uses tmpfs. Existing system logs and battery-accounting Params still write normally;
  this is not a promise of zero device-wide NAND writes.
- Confirmed motion saves up to the selected 15/30/60 seconds of preceding footage (default 60),
  then continues through activity and the selected 15/30/60-second tail (default 60).
  The route starts at the first decodable preview frame retained within that window. Startup
  or RAM pressure can leave less history available; missing footage never pads the route's
  beginning. Saved segments are new 60-second intervals measured from that frame, independent
  of the encoders' source-minute boundaries. Each saved camera file starts on its next decodable
  keyframe, normally losing less than one second. Video and bookmark timestamps retain their real times.
  The confirmation period is included in this pre-roll.
  Only confirmed motion or a diagnostic manual trigger extends the tail; eye-only movement
  does not. Each confirmation renews the selected post-motion interval from that trigger.
  Further confirmations before that deadline remain on the same route,
  including when someone moves between lenses. Quiet periods turn off the eye without
  disarming capture. The route closes after the tail expires; the next event can start immediately
  without another arming delay. There is no fixed 15-minute split; parking, battery, thermal
  and storage limits still apply, and completed minute segments upload normally.
- Warmup and arming footage stays in the RAM history. A later event can therefore include
  the initial exposure/IR fade in its pre-roll even though that fade did not trigger a save.
  The motion bookmark identifies the confirmed trigger, rather than the start of pre-roll.
- Saved Sentry segments carry a log-only `selfdriveState` annotation labeled **Sentry recording**,
  with `alertStatus=userPrompt`, at their start and end. The first annotation uses the route's
  `initData` time origin, aligned with the first playable preview frame, so the timeline is
  orange from its actual beginning. Video and bookmark timestamps stay unchanged.
  Connect's existing timeline draws that
  alert interval orange; motion bookmarks remain yellow above it in both the route list and
  playback timeline. The annotation includes saved pre-roll and the post-motion tail, with
  driving state disabled and no audible alert. It is written directly to qlog/rlog, never
  published to live driving/UI processes. Existing uploaded routes are not recolored.
  Connect's server must index these states into route events; its deployed indexing and
  final appearance still require a newly uploaded event to verify.
- On ignition, shutdown, stale controller/health, thermal pause or disable, recording ends
  and the RAM buffer is discarded. This version does not retain the unsaved ring across reboots.

## Parking position and GPS

The first choice is SLC's `MapParkedPosition`: the same vehicle fingerprint/VIN hash,
a checkpoint no older than 24 hours, valid coordinates and accuracy at most 30 m.
No GPS process is requested when this checkpoint is usable.

Otherwise sentry requests GPS until one recent accurate fix arrives, for at most 90 seconds
per parked session. It then stops requesting GPS and keeps the single snapshot in RAM.
If no fix is available, video recording still works with location marked unavailable.
`SentryState` records the original location source, saved time and accuracy. The recorder
adds a **log-only stationary position estimate** under `gpsLocation` with source `fusion`
and the original sample timestamp; it never publishes that estimate onto a live GPS socket.
A cached position is not evidence of a fresh satellite observation. Whether Connect displays
this stationary estimate as intended must be checked on the backend.

The settings panel shows the current time instead of a parking-location status. Absolute
time comes from the existing device clock (AGNOS network time / openpilot `timed` GPS sync).
Sentry resolves the timezone offline from the parking position or GPS that is already
available while driving, and remembers its IANA name in `SentryTimeZone`. The resolver runs
in a background thread so loading the timezone dataset cannot stall ignition handling.
The cached timezone handles daylight-saving changes without a new GPS fix. An old SLC
checkpoint can also identify the timezone without qualifying as the route's current position.
If no geographic timezone is available, the existing modem daemon queries the EG25 modem's
network-supplied local time once per minute with read-only `AT+QLTS=2`. Sentry reads its UTC
offset from the daemon's RAM state, without opening a second AT connection or changing modem
settings. This offset already includes any network DST adjustment and is not stored as an
IANA timezone. The settings label identifies the cellular source, and watermarks use the
numeric offset. It requires the cellular network to supply time, not an active data session;
unsupported/unsynchronized modems or readings older than five minutes are ignored.
If neither source is available, time remains explicitly labeled UTC and recording continues.
The device's system clock and native log timestamps are unchanged. There is no extra GPS
request just to keep the clock running.

## Saved events and upload

Each event gets its own native route identifier and rebased segment/encode indexes:

- `fcamera.hevc`, `ecamera.hevc`, `dcamera.hevc`, `qcamera.ts` for all three cameras;
- native `rlog.zst` / `qlog.zst`, init data, saved car parameters, camera states,
  sentry metadata, timestamps, position estimate and segment/route sentinels;
- an internal `.sentry` marker for uploader identification and lock recovery.

All segments repeat one route-level `initData`, with its UTC wall time and monotonic
timestamp anchored to the first retained, independently decodable preview frame. Segment
numbers and sentinels use successive 60-second intervals from that same origin, even when
an output minute combines video from two source minutes. Earlier code kept the source-minute
origin after trimming, leaving an empty leading timeline interval; simply coloring that
interval orange did not make footage exist there. The recorder now trims the origin and
rebuilds the minute boundaries together. Individual frame and bookmark timestamps do not shift.
Timezone offsets apply only to the local display/watermark, never the native route clock.
The corrected metadata applies to new events; already uploaded routes are not rewritten.

The empty-lead-in regression was reproduced against the old recorder. The correction passed
12 native recorder cases, including 15/30/60-second trimming, only five seconds of playable
preview history, three output minutes crossing different source-minute boundaries, and
unchanged-deadline camera handoffs. Tests decode the saved streams and check the route/video
origin, real frame times, shared initData, segment numbers, bookmarks and orange annotations.
Connect's deployed backend still needs a fresh device event to verify its rendered timeline.

The recorder writes standard `userBookmark` events into both logs at the confirmed trigger,
and again when a new confirmed burst follows at least five seconds without confirmation.
Manual saves and new split routes also get a marker. Continuous movement does not create
a bookmark on every sample, and immediate eye-only movement does not create bookmarks.
These are written directly to the saved route; the UI retains ownership of the live bookmark
publisher. Log contents are tested locally; bookmark appearance in Connect still needs
verification against an uploaded device event.

The Connect preview (`qcamera.ts`) starts with a confirmed lens, including its available
pre-event footage. During an open recording, substantially stronger motion in the other
wide lens can take over at the next 5 Hz sample, even if the original lens still sees
movement. The score is the fraction of the sampled image occupied by changing pixels
in that region, multiplied by its 1x–2x middle-band weight. It is not a physical object-size
estimate. Close scores or no motion retain
the current view. A modest challenger must exceed the current score by 35% plus a small
margin for two samples (about 0.2 seconds), with a one-second hold after a cut. Decisive
motion can bypass that hold. Recent motion decays over 0.2 seconds to bridge brief pauses
in a stride without keeping the previous lens dominant as the person leaves its view.
Both wide lenses use the same broad horizontal preference, without assumed window outlines,
floor or dashboard masks. Top and bottom regions remain eligible, and there is no left/right
preference. The detector does not identify windows or distinguish
people from shadows; a strong, persistent shadow can still win. Lens area compensation
depends on the camera optics, and low-light thresholds adapt to scene brightness and noise,
rather than a particular vehicle interior. The handoff delay does not affect event detection
or the immediate red eye. Handoffs do not wait for recording confirmation; eye-only movement
can change an open preview but cannot start/extend a route or add a bookmark. New
events still choose a confirmed source, preserving the initiating event in the pre-roll.
This follows motion through the views without identifying or tracking a person.

Each handoff requests a fresh H264 keyframe from the newly selected preview encoder.
Playback switches when that independently decodable frame arrives, with the ordinary
one-second GOP as fallback. Only selection changes request a keyframe; all HEVC streams
retain their normal GOP. Actual device latency still depends on camera/encoder queues.
Timestamps remain increasing and each minute still has one `qcamera.ts`. Earlier preview
choices are retained when saved event histories overlap. Diagnostic manual saves use the
strongest active wide lens, then a front-wide/cabin fallback. Only the two wide cameras
drive the preview; the narrow recording remains available separately.

Only the preview changes lenses. Full recordings always retain their original mapping:
`fcamera.hevc` is narrow, `ecamera.hevc` is wide, and `dcamera.hevc` is cabin. Their
camera indexes and manual Connect upload destinations do not change.

Qlogs and the motion preview use the ordinary route uploader, so Connect can index events
and display their motion bookmarks. The stock metered-connection policy still applies:
qlogs upload normally, and previews on metered links require viewing the route in Connect.

Full logs (`rlog.zst`) and all three camera videos (`fcamera.hevc`, `ecamera.hevc`,
`dcamera.hevc`) stay on the device until requested manually from Connect. Open the event
route and use its upload controls to request the camera footage or all files. These requests
use the normal Athena upload queue. Sentry does not request batch upload URLs or retry full
files automatically, including files saved by earlier builds.

The optional Sentry account-token setup has been removed. No account token is needed on the
device for this workflow. On startup, the uploader removes the obsolete
`/persist/comma/sentry_connect.json` credential if present; it never reads or uses that file.
The settings panel no longer includes a separate upload-information row.

Failed or refused automatic Sentry preview/qlog uploads remain pending and use the normal
retry/backoff. They are not marked successful, and a failed file does not block the remaining
preview/qlog files. Local queue tests cover locks, restarts, metered policy, refusal handling,
and retaining all three camera files for manual requests. Connect's live server behavior is
not exercised by those tests.

Sentry no longer sets a preservation attribute or uses a separate event-storage budget.
Its events follow normal route cleanup when storage is low, including events from older
Sentry builds that still have preservation attributes. Active locked segments cannot be
deleted or uploaded. Recording pauses below 5 GiB / 10% free space, matching the normal
deleter's thresholds, and retries after cleanup. Events are not permanently pinned;
normal cleanup can remove an event that has not uploaded yet when storage runs low.

Completed files are flushed before unlocking. During an event, log frames and video are
flushed once per second, and completed segments are synced. After a recorder restart,
interrupted sentry segments are unlocked and marked `.sentry-incomplete`; the last partial
segment may be truncated. No software can guarantee preservation of the latest frames
through instantaneous power loss.

## Controls and display

| Setting | Default |
| --- | --- |
| Enable Sentry | On, for an unset first-use setting |
| Sensitivity | 75 / 100 |
| Arming delay | 30 s; one row with 15s / 30s / 60s presets |
| Pre-event history | 60 s; one row with 15s / 30s / 60s presets |
| Tail after motion | 60 s; one row with 15s / 30s / 60s presets |
| Battery cutoff | 11.8 V (stock threshold); adjustable 11.0–11.8 V |
| Maximum parked time | 4 h; inline Shut down switch defaults on |
| Thermal pause | 70 °C, resume 8 °C cooler; inline Shut down switch defaults off |
| Cameras | All three record; only the two wide lenses detect motion; no camera-selection row |
| Cabin IR | Stock automatic exposure/timeout behavior; no separate setting |
| Red eye | On |
| Video watermark | On: local timestamp with UTC offset, camera name and known parking coordinates |

The row order is Sentry status/STOP, Enable Sentry, Arming delay, Pre-event buffer,
Record after motion stops, Motion sensitivity, Battery cutoff, Maximum parked time,
Pause capture above, Red eye, Video watermark, then Buffered footage and Current time.
Sensitivity is the first slider. Battery cutoff includes the current voltage inline; stale
readings say unavailable. Camera selection, manual SAVE, Event uploads and Battery protection
rows are removed. All three duration rows use the same single-choice 15s / 30s / 60s buttons.
New defaults only initialize missing settings; older arming and post-event slider values
are interpreted as the nearest preset. The old 1–5 minute pre-event setting is replaced by
`SentryPreSeconds`, defaulting to 60 seconds; an existing seconds choice is retained.
The battery default remains openpilot's 11.8 V threshold.
Watermark changes pause capture briefly to restart the encoders. Watermarks are burned
into private low-resolution frames, never into shared camera buffers. Local timestamps include
a numeric UTC offset, with explicitly labeled UTC as the fallback. Native route/log timestamps
remain UTC. The coordinate line is omitted when the fixed parking position is unknown.
The single watermark switch turns all three fields on/off together. Existing voltage settings
above 11.8 V are clamped to the new maximum. There is no sentry energy-budget limit.
Full camera uploads are requested manually in Connect.

The eye follows the approximate image position of motion from the front wide or cabin lens;
the cabin direction is mirrored. Its target and cross-camera score use the pixels changing
in the current frame, avoiding the center of a large fading background footprint. Horizontal
travel is expanded 2.2 times for the cabin and 1.5 times for front-wide; vertical travel is
expanded 1.4 times. Cabin window activity around image x=0.72–0.80 therefore reaches the
opposite display edge, instead of staying about one-third of the way across. These are
approximate display mappings, not calibrated physical angles. Existing smoothing remains.
It reacts without the recording confirmation delay or larger
area threshold, while still respecting camera warmup, arming and touch suppression.
Three glass reflections form a symmetric arc across the upper lens when the eye is centered.
As the eye moves, they rotate together around the lens center toward a virtual overhead light,
with a subtle rotation limited to six degrees in either direction.
Their radius and angular spacing stay fixed, so the animation follows the rounded glass
instead of shifting the reflections sideways/upward. Each reflection and its halo turns
tangent to the curve, including the middle reflection as the arc rotates.
Its diameter is four times the original, including the glow. Motion is bounded by the actual
outer glow radius, with no extra border/padding: the full eye stays inside the display and
the glow can touch its edge. It appears on black at 100% brightness only after the normal
screen timeout, and fades off when motion stops. **Any touch clears the eye and suppresses it
throughout the interactive screen period, even if motion continues. It becomes eligible again
only at the next inactivity timeout.** Settings remain accessible through the normal interface.
To stop an active session, tap the display, open **Settings → Sentry**, and press **STOP**
at the top or turn **Enable Sentry** off. Turning it off remains available even if ignition
is reported. Capture/GPS process gates and the eye honor the master switch directly;
the current event is finalized and saved events remain available for manual upload from
Connect. Sentry stays disabled until you explicitly enable it again. The switch can be changed
onroad; capture still waits for ignition off and an offroad state.

Sentry's battery cutoff remains mandatory while its parked controller lease is fresh.
`DisablePowerDown` does not bypass that cutoff or an enabled Sentry shutdown switch.
The parked-time switch defaults on: expiry shuts down the device. With it off, expiry ends
capture for this parking session and restores ordinary offroad power management. Cooling,
missing readings, changing the time slider after expiry, or toggling Sentry off/on cannot
renew the budget; ignition/onroad starts a new parking session. The timer starts when Sentry
first has a valid parked session, and keeps running during pauses.

The temperature shutdown switch defaults off: reaching the threshold pauses capture, and
capture warms up/re-arms automatically at 8 °C below the threshold. Turning that switch on
requests device shutdown instead. Missing/non-finite temperature readings pause capture
without being treated as proof of overheating. The parked-time limit takes priority over
thermal resumption; cooling after expiry cannot restart capture.

Returning to ordinary offroad mode does not guarantee the device stays on: stock voltage,
capacity and maximum-offroad-time rules still apply. A lost controller lease also restores
stock power policy. Hardware protections remain active.
The fan can exceed the usual offroad limit during capture/cooldown. Camera processing keeps
the necessary CPU power state available, even with the display off.

There is no measured maximum power figure for this implementation yet. The eye now uses a
larger portion of the OLED at full brightness. The eye, cameras, ISP, encoders, IR, fan and
uploads all contribute to consumption.
Measure at the vehicle supply across daylight/darkness, fan load and Wi-Fi/LTE upload; device
power telemetry alone can miss upstream losses. Choose the voltage threshold for the vehicle's
battery and verify with a meter before unattended operation.

## Validation

The dusk crossing in `000003c2--4b23da1686` runs the preceding build at sensitivity 50.
The controller is already Watching throughout the front approach: it reports small wide-camera
motion but never confirms it, then records the first cabin bookmark at 14.896 seconds.
The bright sky selected the old frame-wide daylight threshold, fragmenting the darker
silhouette below the recording area floor. Local contrast adaptation and the requested
middle-horizontal-third preference confirm the front approach at 3.58–3.88 seconds in the
decoded-video replay, across all four 20-fps frame phases sampled at 5 Hz. The preceding
detector misses the crossing in every phase. The narrow camera remains excluded from detection
and preview selection.

The focused detector/controller run passes 154 checks; 14 UI-dependent cases are excluded
because this environment lacks `pyray`. New checks cover a dim crossing below bright sky,
mount tilts, middle-band priority across the full width, continued outside-band detection,
and repeated small bright/dark passes that must not grow into large afterimage triggers.
This update changes Python detection only and needs a controller restart after updating,
not a native recorder rebuild. Live raw-camera behavior still needs a parked device test.

The later dark crossing in route `000003b9--0145a7f041` exposed another missed approach at
the recorded 50% setting: the person is visible in the front-wide footage around 22–24.6
seconds while the preview remains on the empty cabin. The old sampling and low-light blur
leave disconnected pieces smaller than the required area. Averaging within sample cells,
preserving outlines and lowering the dark-scene contrast threshold confirm the crossing at
22.81 seconds in replay. Comparing the revised wide score against the original recorded
cabin scores switches the preview wide at 23.01 seconds and retains it through the crossing.
This is a partial-input comparison, not a simultaneous replay of both raw cameras. There
are no confirmed events in the tested quiet portions before 9 seconds or after 35 seconds.

The earlier dark-garage approach was replayed at the recorded 50% setting as well. The revised
wide detector confirms motion at 11.20 seconds (11.40 with the preceding low-light fix and
15.00 before that); the flashlight first appears at about 13.6 seconds. There are no triggers
in the preceding quiet portion. Replays sample decoded Y planes at 5 Hz with watermark rows
excluded, so compression and downsampling make them an approximation of live raw-camera detection.
The middle-band update also retains the dark-garage detection (11.00 seconds in replay)
and detects the seated person's cabin movement earlier in the daylight clip (8.19 seconds,
compared with 14.79 for the previous sampling-only change). The walk-around preview only
contains the camera selected at each instant; it cannot reconstruct both simultaneous
camera inputs. A regression using its observed winning scores, without spatial weighting,
checks the selector's hold and switching rules during that crossing,
including real 5 Hz scheduling jitter. Swapping the lens identities gives the same behavior.
Full simultaneous-camera behavior still needs a new parked walk-around test. Synthetic
checks cover dim moving subjects through the real sampling path with padded image strides
and both sensor resolutions, noise inside a departed subject's footprint, brightness changes, scattered specks,
near-ties, pauses, large new motion, and returning to the previous lens. Large subjects remain
detectable throughout the image, with left/right symmetry; dim-approach checks include +/-12 degree
tilts at different image heights. Other vehicles and mounting angles still need real footage
validation; those checks do not establish performance for every installation.

The 50% calibration was replayed against the supplied two consecutive one-minute segments
from all three cameras: garage opening/closing in both forward views and cabin-window
activity. The forward cameras already confirmed these events at 50%; the cabin now confirms
the window activity as well. No saves occurred in the sampled quiet intervals. Replay uses
the decoded Y plane at 5 Hz and excludes the timestamp/GPS watermark from detection; live
detection uses the original unwatermarked camera buffers. HEVC/downsampling makes this an
approximation of live detection, and this one scene does not validate every lighting/weather
condition. Synthetic regressions also cover small scattered regions, noise/exposure changes,
the motion confirmation, fresh-motion targeting and complete eye visibility at the edges.

Build the native encoder and recorder, params and messaging extensions using the project's
normal SCons environment. Run:

```sh
scons -j4 --minimal openpilot/selfdrive/pandad/pandad openpilot/system/loggerd/encoderd openpilot/system/loggerd/sentry_loggerd
pytest openpilot/system/sentry/tests
```

The native video tests additionally require PyAV (`pip install av`), and the VisionIPC test
requires permission to create Unix sockets.
The native state-reader regression compiles a small C++ program with the installed Cap'n Proto
dependencies. It covers subscriptions before their first message, invalid/wrong-type messages,
and expired leases. The recorder and watermark rendering share this guarded reader so
neither can crash merely because the sentry publisher starts later. The recorder is also tested
with no publishers initially and staggered controller/panda/device messages.
Encoded-video sockets use the service registry's queue sizes, matching the encoder's
publishers. Native tests start them in both orders, send real HEVC/H264 past the
10 MiB queue boundaries, save two successive events, and decode every saved frame from all
four video files. This catches the smaller socket mapping that previously stopped capture
after a short burst even while motion detection kept running.
The native service-registry checks link the built messaging library and start panda,
recorder and encoder subscriptions/publications without hardware. This catches stale
native service tables that Python-only tests cannot detect. Unknown-service errors now
print the missing name. The socketmaster object explicitly depends on both `services.py`
and its generated header so additions invalidate the native library's build cache entry.
Panda firmware and `pandad` retain their pre-sentry code; there is no `sentryState`
subscription in panda. The selected battery cutoff is enforced in
`openpilot/system/hardware/power_monitoring.py`, with capture stopping in `sentryd`
at the same threshold. Cabin IR follows the existing camera exposure and timeout logic.

If panda repeatedly connects and then aborts at the `services.count(...)` assertion on
an older build, check the console error before treating it as a hardware/firmware failure.
After updating, remove only the generated registry/library artifacts and let startup
rebuild them:

```sh
cd /data/openpilot
rm -f prebuilt openpilot/cereal/services.h openpilot/cereal/messaging/socketmaster.o openpilot/cereal/libsocketmaster.a
sudo reboot
```

The tests cover brightness/noise rejection, sensitivity and centroid tracking, checkpoint
validation, bounded one-shot GPS acquisition, expired leases, battery override, driving-process
gates, tap suppression/full brightness, ordinary event cleanup, manual camera upload policy,
preview retry handling, timezone/DST and native video/log creation. The recorder test replays
real HEVC/H264 packets, checks no idle route writes,
decodes every saved camera stream, validates indexes and tests ignition finalization.
Preview tests cover all three recorder inputs, strongest-wide-motion selection during overlapping
motion, requested and periodic H264 keyframe handoffs within a minute, increasing timestamps, retained preview
choices in overlapping events, and byte-for-byte unchanged full-camera files. The encoder test
uses three synthetic VisionIPC camera streams and checks each preview's watermark output
without modifying the shared camera buffer. It also checks that all six encoded streams
remain present even when obsolete camera-disable preferences are on disk, and that a handoff
requests one independently decodable frame promptly without forcing every following frame.
PC main-camera encoding uses the existing lossless codec; production hardware uses HEVC.

The faster-handoff/0.6-second-confirmation update passed 104 controller/detector checks and
two native mixed-camera decoding cases. PC encoder and recorder targets compiled. Two
VisionIPC encoder cases were skipped because this environment blocks Unix socket creation;
the requested-keyframe latency still needs measurement on a comma 3X.

Duration tests save real encoded footage with each 15/30/60-second history setting, check
the cutoff in every saved stream, decode partial minute files, and verify that the route
starts at its first playable frame with correctly aligned later minutes. Settings checks cover preset
selection, old values, and agreement between native and Python first-use defaults.
The duration update passed 105 controller, settings and native-recorder checks. Two encoder
cases were skipped because the encoder binary was not built for this focused check.

Before considering it ready for unattended use, verify on a parked comma 3X:

1. Hardware encoding for all three cameras and their three H264 previews, decodable saved
   clips, timestamp/coordinate watermarks, IR behavior and realistic night-time motion
   sensitivity. Walk around the car to check camera handoffs in the Connect preview.
2. Screen timeout → motion eye → touch → uninterrupted settings → timeout → eye again.
3. Cached parking position leaves GPS processes off; missing checkpoint starts/stops a
   bounded GPS acquisition; no-fix recording still works.
4. Ignition promptly stops sentry and restores normal camera encoding without interfering
   with driving process startup. Repeat onroad/offroad and watermark-setting transitions.
5. Battery cutoff, thermal pause/resume, time limits, actual input power and RAM
   consumption under sustained motion and upload. Test controlled shutdown and interrupted
   event recovery.
6. Connect discovers the stationary event route, shows the intended location and plays
   preview/full requested camera files. Do not assume backend compatibility solely from
   successful local decoding.

Local verification for the motion-preview change: the PC encoder and recorder targets compiled;
82 controller, recorder, native-service and upload checks passed, including decoding the
mixed-camera preview. Both VisionIPC encoder cases were skipped in the development sandbox
because Unix socket creation is blocked. Six concurrent hardware encoder sessions and Connect
playback of the changing preview still need comma 3X validation. The touch/timeout
policy was unit tested; the rendered settings and eye still need a device visual check.
The eye's drawing calls are checked against the installed raylib C signatures using
callbacks instead of an OpenGL context, including its first visible frame and the older
raylib 5 gradient signature. This prevents the raylib 6 argument-count error that could
terminate the UI when motion first displayed the eye after the screen timeout.
