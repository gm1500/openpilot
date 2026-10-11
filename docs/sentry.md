# Sentry mode — comma 3X prototype

Branch: `sentry-mode-poc`, based on `boat-anchor` at `2ee8df154017362e3a791c90d788172508f428df`.

Enable **Settings → Sentry → Enable Sentry** while parked. The feature defaults off.
This implementation is ready for parked hardware evaluation; comma 3X encoding,
thermal behavior, battery draw, and Connect indexing/playback still require device validation.

## Behavior

- Requires a comma 3X, a recent valid panda/device state, ignition off and `IsOffroad`.
  Driving controls, modeld and locationd are never started for sentry. Ignition/onroad
  stops capture; driving and sentry encoders cannot run simultaneously.
- Starts filling the RAM buffer after the offroad transition, with a default 60-second
  arming delay before motion can save an event. Camera preview/livestream pauses capture.
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

## Saved events and upload

Each event gets its own native route identifier and rebased segment/encode indexes:

- `fcamera.hevc`, `ecamera.hevc`, `dcamera.hevc`, `qcamera.ts` for selected cameras;
- native `rlog.zst` / `qlog.zst`, init data, saved car parameters, camera states,
  sentry metadata, timestamps, position estimate and segment/route sentinels;
- `user.preserve` on **every** event segment, plus an internal `.sentry` marker.

Logs and preview enter the ordinary uploader. Its normal connectivity, metered-network,
account and upload policies still apply. Full camera files remain available for requested
upload. Disabling the narrow camera also disables the normal front preview. No forced
upload option is implemented. Native files and indexes are tested locally; server-side route
recognition, map display and requested playback are not yet verified.

The deleter retains all `.sentry` segments, independently of the stock latest-five preservation
rule. The default event budget is 2 GiB, configurable to 1–10 GiB. Recording pauses near the
budget (128 MiB reserved) or below 6 GiB / 12% free space. Existing events are not silently
removed to make room. Remove saved events with the device's file-management workflow to
resume when full; a dedicated event browser/delete UI is future work. Raising the budget
resumes saving on the recorder's next storage retry (up to 30 seconds).

Completed files are flushed before unlocking. During an event, log frames and video are
flushed once per second, and completed segments are synced. After a recorder restart,
interrupted sentry segments are unlocked and marked `.sentry-incomplete`; the last partial
segment may be truncated. No software can guarantee preservation of the latest frames
through instantaneous power loss.

## Controls and display

| Setting | Default |
| --- | --- |
| Enable Sentry | Off |
| Sensitivity | 50 / 100 |
| Arming delay | 60 s |
| Pre-event history | Up to 5 min |
| Tail after motion | 60 s |
| Battery cutoff | 12.3 V |
| Maximum parked time | 4 h |
| Parked energy budget | 60 Wh |
| Thermal pause | 70 °C, resume 8 °C cooler |
| Event storage budget | 2 GiB |
| Narrow / front wide / cabin | All on |
| Cabin IR | On, exposure controlled |
| Red eye | On |
| Timestamp / camera watermark | On |
| GPS coordinate watermark | Off |

Camera/watermark changes pause capture briefly to restart the encoders. Watermarks are burned
into private low-resolution frames, never into shared camera buffers. Timestamp is UTC.
The coordinate watermark uses the fixed parking position.

The eye follows the approximate image position of motion from the front wide or cabin lens;
the cabin direction is mirrored. It appears on black at low brightness only after the normal
screen timeout, and fades off when motion stops. **Any touch clears the eye and suppresses it
throughout the interactive screen period, even if motion continues. It becomes eligible again
only at the next inactivity timeout.** Settings remain accessible through the normal interface.

Sentry's cutoff/time/energy limits replace the stock offroad software limits only while the
controller has a fresh parked-session lease. `DisablePowerDown` does not bypass sentry's cutoff.
A lost lease restores stock behavior. Stock thermal shutdown and hardware protections remain.
The fan can exceed the usual offroad limit during capture/cooldown. Camera processing keeps
the necessary CPU power state available, even with the display off.

There is no measured maximum power figure for this implementation yet. The eye uses only a
small portion of the OLED, but cameras, ISP, encoders, IR, fan and uploads dominate consumption.
Measure at the vehicle supply across daylight/darkness, fan load and Wi-Fi/LTE upload; device
power telemetry alone can miss upstream losses. Choose the voltage threshold for the vehicle's
battery and verify with a meter before unattended operation.

## Validation

Build the native encoder and recorder, params and messaging extensions using the project's
normal SCons environment. Run:

```sh
scons -j4 --minimal openpilot/system/loggerd/encoderd openpilot/system/loggerd/sentry_loggerd
pytest openpilot/system/sentry/tests
```

The native video tests additionally require PyAV (`pip install av`), and the VisionIPC test
requires permission to create Unix sockets.

The tests cover brightness/noise rejection, sensitivity and centroid tracking, checkpoint
validation, bounded one-shot GPS acquisition, expired leases, battery override, driving-process
gates, tap suppression, all-segment retention, uploader lock handling, and native video/log
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
5. Battery cutoff, thermal pause/resume, energy/time limits, actual input power and RAM
   consumption under sustained motion and upload. Test controlled shutdown and interrupted
   event recovery.
6. Connect discovers the stationary event route, shows the intended location and plays
   preview/full requested camera files. Do not assume backend compatibility solely from
   successful local decoding.

Local verification for this change: the PC encoder and recorder targets compiled, and the
native recorder integration decoded all saved streams. The VisionIPC encoder test could not
run in the development sandbox because Unix socket creation is blocked. The touch/timeout
policy was unit tested; the rendered settings and eye still need a device visual check.
