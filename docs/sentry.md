# Sentry mode — comma 3X prototype

Branch: `sentry-mode-poc`, based on `boat-anchor` at `2ee8df154017362e3a791c90d788172508f428df`.

Manage the feature under **Settings → Sentry** while parked. On first use it defaults on;
previously saved enable/disable choices are retained, including an explicit STOP.
The status row reports ignition/offroad/health blockers, missing camera frames, missing encoded
video or an unavailable controller. An arming countdown appears once video is arriving; the
buffer counter confirms capture. The voltage row shows the current vehicle supply reading.
**Save an event now** also works during arming once capture has started.
This implementation is ready for parked hardware evaluation; comma 3X encoding,
thermal behavior, battery draw, and Connect indexing/playback still require device validation.

## Behavior

- Requires a comma 3X, a recent valid panda/device state, ignition off and `IsOffroad`.
  Driving controls, modeld and locationd are never started for sentry. Ignition/onroad
  stops capture; driving and sentry encoders cannot run simultaneously.
- Starts filling the RAM buffer after the offroad transition. The selected 15/30/60-second
  arming delay (default 30) starts only once all selected cameras deliver frames and the
  recorder has buffered video. The detector learns the changing exposure during this delay;
  automatic events and the eye wait until it finishes. Manual saves still work during warmup.
  Missing frames/recording readiness or a capture restart resets the delay. Camera
  preview/livestream pauses capture.
- Samples all selected cameras at 5 Hz for luminance changes. Exposure compensation,
  neighboring-pixel support and three consecutive detections reduce noise. The sensitivity
  slider changes both minimum area and pixel-change thresholds. This is motion detection;
  shadows, rain, reflections and passing traffic can still trigger it. There are no object,
  person, vehicle or license-plate rules in this version.
- All three cameras default on. Device targets are 526×330, 20 fps, HEVC at 256 kbit/s
  per camera, plus a separate 256 kbit/s H264 front preview. The nominal five-minute
  compressed-video payload is about 38.4 MB including the preview, before logs/overhead.
  Actual variable bitrate varies. Encoded payloads have a hard 128 MiB RAM cap; IPC queues,
  camera buffers and private watermark buffers use additional RAM.
- No route directories or video files are written while merely watching. Runtime control
  uses tmpfs. Existing system logs and battery-accounting Params still write normally;
  this is not a promise of zero device-wide NAND writes.
- Motion saves available preceding native minute segments, then continues through activity
  and the default 60-second tail. To preserve normal segment boundaries, pre-roll is rounded
  down to complete available source-segment starts: normally 4–5 minutes with a full buffer,
  less during startup or RAM pressure. Events split after 15 minutes of continuous activity.
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
The cached timezone handles daylight-saving changes without a new GPS fix. If neither a
timezone nor a usable position is available, time is explicitly labeled UTC; recording
continues. There is no extra GPS request just to keep the clock running.

## Saved events and upload

Each event gets its own native route identifier and rebased segment/encode indexes:

- `fcamera.hevc`, `ecamera.hevc`, `dcamera.hevc`, `qcamera.ts` for selected cameras;
- native `rlog.zst` / `qlog.zst`, init data, saved car parameters, camera states,
  sentry metadata, timestamps, position estimate and segment/route sentinels;
- an internal `.sentry` marker for uploader identification and lock recovery.

Logs and preview enter the ordinary uploader. Once those are queued, it automatically
uploads Sentry's `rlog.zst` and every available selected-camera file (`fcamera.hevc`,
`ecamera.hevc`, `dcamera.hevc`) on unmetered connections, usually Wi-Fi. No separate uploader
toggle is required. Metered links retain the stock preview/log policy; full Sentry videos
wait for an unmetered link. Disabling the narrow camera also disables the normal front preview.
Uploads resume from on-disk files and per-file upload attributes after a restart. Failed or
refused Sentry uploads, including HTTP 401/403/412, remain pending and use the uploader's
normal retry/backoff; they are not marked as successful. This does not override Connect's
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
the cabin direction is mirrored. Its diameter is four times the original, including the glow,
and its full extent stays within the display. It appears on black at 100% brightness only after the normal
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
