# Sentry mode — comma 3X prototype

Branch: `sentry-mode-poc`, based on `boat-anchor` at `2ee8df154017362e3a791c90d788172508f428df`.

Manage the feature under **Settings → Sentry** while parked. On first use it defaults on;
previously saved enable/disable choices are retained, including an explicit STOP.
The status row reports ignition/offroad/health blockers, missing camera frames, missing encoded
video or an unavailable controller. Camera warmup and arming have separate countdowns once video is arriving; the
buffer counter confirms capture. The voltage row shows the current vehicle supply reading.
**Save an event now** also works during arming once capture has started.
This implementation is ready for parked hardware evaluation; comma 3X encoding,
thermal behavior, battery draw, and Connect indexing/playback still require device validation.

## Behavior

- Requires a comma 3X, a recent valid panda/device state, ignition off and `IsOffroad`.
  Driving controls, modeld and locationd are never started for sentry. Ignition/onroad
  stops capture; driving and sentry encoders cannot run simultaneously.
- Starts filling the RAM buffer after the offroad transition. Once all selected cameras
  deliver frames and the recorder has buffered video, a separate 10-second camera warmup
  runs before the selected 15/30/60-second arming delay (default 30). With the default, motion
  arms 40 seconds after video is ready. The detector learns the changing exposure throughout
  both stages; automatic events and the eye wait until both finish. Manual saves work during
  either stage. Missing camera frames or a capture restart resets both delays. A recorder-only
  status delay or empty buffer pauses automatic save requests but keeps the camera arming clock
  and existing event deadline; once buffered video is available again, detection resumes without
  another countdown. Saving an event does not re-arm the cameras. Camera preview/livestream
  pauses capture.
- Samples all selected cameras at 5 Hz for luminance changes. The eye reacts immediately
  to smaller local movements. Saving instead requires about one second of sustained
  movement in the same connected region, allowing gaps no longer than half a second.
  Saving area is scaled by each lens's configured focal length and image dimensions:
  narrow requires a larger region, while front-wide and cabin require smaller ones.
  At sensitivity 75, the OX03C10 cameras in this comma 3X require about 5.25% of the narrow
  image or 0.5% of either wide image. OS04C10 uses its own configured optics (about 0.73%
  for either wide image). A connected region must cover at least 0.5% regardless of lens;
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
- All three cameras default on. Device targets are 526×330, 20 fps, HEVC at 256 kbit/s
  per camera, plus a separate 256 kbit/s H264 front preview. The nominal five-minute
  compressed-video payload is about 38.4 MB including the preview, before logs/overhead.
  Actual variable bitrate varies. Encoded payloads have a hard 128 MiB RAM cap; IPC queues,
  camera buffers and private watermark buffers use additional RAM.
- No route directories or video files are written while merely watching. Runtime control
  uses tmpfs. Existing system logs and battery-accounting Params still write normally;
  this is not a promise of zero device-wide NAND writes.
- Confirmed motion saves available preceding native minute segments, then continues through activity
  and the default 60-second tail. To preserve normal segment boundaries, pre-roll is rounded
  down to complete available source-segment starts: normally 4–5 minutes with a full buffer,
  less during startup or RAM pressure. The confirmation period is included in this pre-roll.
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

- `fcamera.hevc`, `ecamera.hevc`, `dcamera.hevc`, `qcamera.ts` for selected cameras;
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

Logs and preview enter the ordinary uploader. Once those are queued, it automatically
uploads Sentry's `rlog.zst` and every available selected-camera file (`fcamera.hevc`,
`ecamera.hevc`, `dcamera.hevc`) on unmetered connections, usually Wi-Fi. No separate uploader
toggle is required. Metered links retain the stock preview/log policy; full Sentry videos
wait for an unmetered link. Disabling the narrow camera also disables the normal front preview.
Full Sentry logs/videos request signed URLs through the documented batch upload endpoint
(`POST /v1/:dongle_id/upload_urls/`), as Connect's Upload button does, using the device's
own signed token. The previous automatic upload flow returned HTTP 412 for these files
despite manual uploads working. Preview/qlog uploads retain their ordinary
endpoint. The batch flow still requires server authorization; it does not override a refusal.
Uploader logs distinguish `sentry.upload_url_failed` (URL request) from
`sentry.upload_put_failed` (file transfer). Only a successful transfer marks a file uploaded.
Uploads resume from on-disk files and per-file upload attributes after a restart. Failed or
refused Sentry uploads, including HTTP 401/403/412, remain pending and use the uploader's
normal retry/backoff; they are not marked as successful. Within each priority tier, remaining
files get a turn before a failed file is retried, so one rejected log or camera cannot block
the other files in that tier. Failures include an explicit HTTP status in the uploader log.
This does not override Connect's
account or server policies. Native files, indexes and queue behavior are tested locally;
server-side route recognition, map display and all-camera playback still need device validation.

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
| Pre-event history | Up to 5 min |
| Tail after motion | 60 s |
| Battery cutoff | 11.8 V (stock threshold); adjustable 11.0–11.8 V |
| Maximum parked time | 4 h |
| Thermal pause | 70 °C, resume 8 °C cooler |
| Cameras: Narrow / Wide / Cabin | All on; three independent buttons in one row |
| Cabin IR | Stock automatic exposure/timeout behavior; no separate setting |
| Red eye | On |
| Video watermark | On: local timestamp with UTC offset, camera name and known parking coordinates |

Enable/STOP, cameras, red eye, watermark and arming presets are at the top. Sensitivity and
manual save follow; recording lengths, battery/time limits and thermal pause come farther
down. New defaults only initialize missing settings; older custom arming slider values
are interpreted as the nearest preset. The battery default remains openpilot's 11.8 V threshold.
Camera/watermark changes pause capture briefly to restart the encoders. Watermarks are burned
into private low-resolution frames, never into shared camera buffers. Local timestamps include
a numeric UTC offset, with explicitly labeled UTC as the fallback. Native route/log timestamps
remain UTC. The coordinate line is omitted when the fixed parking position is unknown.
The single watermark switch turns all three fields on/off together. Existing voltage settings
above 11.8 V are clamped to the new maximum. There is no sentry energy-budget limit; voltage
and maximum parked time remain the shutdown limits. Upload behavior is automatic and has no
settings row.

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
the current event is finalized and already saved events stay in the upload queue. Sentry stays
disabled until you explicitly enable it again while offroad.

Sentry's cutoff/time limits replace the stock offroad software limits only while the
controller has a fresh parked-session lease. `DisablePowerDown` does not bypass sentry's cutoff.
A lost lease restores stock behavior. Stock thermal shutdown and hardware protections remain.
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
publishers. Native tests start them in both orders, send real HEVC/H264 past multiple
10 MiB queue wraps, save two successive events, and decode every saved frame from all
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
gates, tap suppression/full brightness, ordinary event cleanup, all-camera upload/retry handling, timezone/DST and native video/log
creation. The recorder test replays real HEVC/H264 packets, checks no idle route writes,
decodes every saved camera stream, validates indexes and tests ignition finalization.
The encoder test uses three synthetic VisionIPC camera streams and checks watermark output
without modifying the shared camera buffer. PC main-camera encoding uses the existing lossless
codec; production hardware uses HEVC.

Before considering it ready for unattended use, verify on a parked comma 3X:

1. Hardware encoding for all three cameras, decodable saved clips, timestamp/coordinate
   watermarks, IR behavior and realistic night-time motion sensitivity.
2. Screen timeout → motion eye → touch → uninterrupted settings → timeout → eye again.
3. Cached parking position leaves GPS processes off; missing checkpoint starts/stops a
   bounded GPS acquisition; no-fix recording still works.
4. Ignition promptly stops sentry and restores normal camera encoding without interfering
   with driving process startup. Repeat onroad/offroad and camera-setting transitions.
5. Battery cutoff, thermal pause/resume, time limits, actual input power and RAM
   consumption under sustained motion and upload. Test controlled shutdown and interrupted
   event recovery.
6. Connect discovers the stationary event route, shows the intended location and plays
   preview/full requested camera files. Do not assume backend compatibility solely from
   successful local decoding.

Local verification for this change: the PC encoder and recorder targets compiled, and the
native recorder integration decoded all saved streams. The VisionIPC encoder test could not
run in the development sandbox because Unix socket creation is blocked. The touch/timeout
policy was unit tested; the rendered settings and eye still need a device visual check.
The eye's drawing calls are checked against the installed raylib C signatures using
callbacks instead of an OpenGL context, including its first visible frame and the older
raylib 5 gradient signature. This prevents the raylib 6 argument-count error that could
terminate the UI when motion first displayed the eye after the screen timeout.
