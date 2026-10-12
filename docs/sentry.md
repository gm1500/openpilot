# Sentry mode — comma 3X prototype

Branch: `sentry-mode-poc`, based on `boat-anchor` at `2ee8df154017362e3a791c90d788172508f428df`.

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
  either stage. Missing camera frames or a capture restart resets both delays. A recorder-only
  status delay or empty buffer pauses automatic save requests but keeps the camera arming clock
  and existing event deadline; once buffered video is available again, detection resumes without
  another countdown. Saving an event does not re-arm the cameras. Camera preview/livestream
  pauses capture.
- Samples only the front-wide and cabin-wide cameras at 5 Hz for luminance changes. The eye reacts immediately
  to smaller local movements. Saving instead requires about one second of sustained
  movement in the same connected region, allowing gaps no longer than half a second.
  Saving area is scaled by each lens's configured focal length and image dimensions.
  At sensitivity 75, the OX03C10 cameras in this comma 3X require about 0.5% of either wide
  image. OS04C10 uses its own configured optics (about 0.73% for either wide image).
  The narrow camera is still recorded for detail, but cannot trigger an event or take the preview. A connected region must cover at least 0.5% regardless of lens;
  scattered snowflakes cannot pool their area to meet that floor. Fisheye size compensation
  is approximate, especially near the edges; this is not a physical object-size estimate.
  Exposure compensation, neighboring-pixel support and frame-to-frame change checks reject
  noise and stationary background afterimages. At sensitivity 50, the minimum luma change is
  18 levels for the forward cameras and 10 for the cabin's lower-contrast IR/window footage.
  The cabin slider curve retains the same least/most-sensitive endpoints as the forward
  cameras; its middle is calibrated separately. The sensitivity slider changes pixel-change
  thresholds and minimum saving area. This is motion detection, not object identification;
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
  The first minute is trimmed at the frame level, and each camera starts on its next decodable
  keyframe, normally shortening history by less than one second. Startup or RAM pressure can
  leave less history available. Original native minute time anchors are retained even when
  the first minute is partial, so subsequent segments and bookmarks keep their real timestamps.
  The confirmation period is included in this pre-roll.
  Only confirmed motion or manual Save extends the tail; eye-only movement does not.
  Events split after 15 minutes of continuous activity.
- Warmup and arming footage stays in the RAM history. A later event can therefore include
  the initial exposure/IR fade in its pre-roll even though that fade did not trigger a save.
  The motion bookmark identifies the confirmed trigger, rather than the start of pre-roll.
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
timestamp anchored to the first retained pre-event segment. Segment sentinels and frame
timestamps still identify each minute separately. Previously, Sentry advanced `initData`
for every minute, which could move the route start forward and shorten Connect's timeline.
Timezone offsets apply only to the local display/watermark, never the native route clock.
The corrected metadata applies to new events; already uploaded routes are not rewritten.

The recorder writes standard `userBookmark` events into both logs at the confirmed trigger,
and again when a new confirmed burst follows at least five seconds without confirmation.
Manual saves and new split routes also get a marker. Continuous movement does not create
a bookmark on every sample, and immediate eye-only movement does not create bookmarks.
These are written directly to the saved route; the UI retains ownership of the live bookmark
publisher. Log contents are tested locally; bookmark appearance in Connect still needs
verification against an uploaded device event.

The Connect preview (`qcamera.ts`) starts with the lens that first confirms motion,
including its available pre-event footage. If movement then reaches another camera,
the preview can switch within the same minute. The first lens keeps priority while
its confirmed motion continues; after it clears, the earliest other lens with confirmed
motion takes over. A stronger score or eye-only movement cannot steal the current view.
This follows motion through the camera views, without identifying or tracking a person.

Handoffs wait for the next H264 keyframe, normally within one second. Playback holds
the previous view until that frame; timestamps remain increasing and each minute still
has one `qcamera.ts`. Earlier preview choices are retained when saved event histories
overlap. Manual saves use a confirming camera, otherwise the strongest currently active
wide camera, then a front-wide/cabin fallback. Only the two wide cameras drive the preview;
the narrow recording remains available separately.

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

The 50% calibration was replayed against the supplied two consecutive one-minute segments
from all three cameras: garage opening/closing in both forward views and cabin-window
activity. The forward cameras already confirmed these events at 50%; the cabin now confirms
the window activity as well. No saves occurred in the sampled quiet intervals. Replay uses
the decoded Y plane at 5 Hz and excludes the timestamp/GPS watermark from detection; live
detection uses the original unwatermarked camera buffers. HEVC/downsampling makes this an
approximation of live detection, and this one scene does not validate every lighting/weather
condition. Synthetic regressions also cover small scattered regions, noise/exposure changes,
the one-second confirmation, fresh-motion targeting and complete eye visibility at the edges.

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
Preview tests cover all three trigger lenses, first-confirmed priority during overlapping
motion, real H264 keyframe handoffs within a minute, increasing timestamps, retained preview
choices in overlapping events, and byte-for-byte unchanged full-camera files. The encoder test
uses three synthetic VisionIPC camera streams and checks each preview's watermark output
without modifying the shared camera buffer. It also checks that all six encoded streams
remain present even when obsolete camera-disable preferences are on disk.
PC main-camera encoding uses the existing lossless codec; production hardware uses HEVC.

Duration tests save real encoded footage with each 15/30/60-second history setting, check
the cutoff in every saved stream, decode partial minute files, and verify that trimming
does not move the shared route clock or later segment times. Settings checks cover preset
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
