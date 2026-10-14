# Sentry mode — comma 3X

Sentry is integrated into `batmobile`. The changes developed on `sentry-session-routes-poc`
are included as one additional squashed commit: one route per armed session, with small
logs and black preview video bridging quiet gaps. Blue means no saved camera footage,
orange marks saved preview intervals, and yellow marks confirmed motion.

Manage the feature under **Settings → Sentry** in any vehicle state, including onroad.
All configuration controls remain editable; enabling Sentry onroad takes effect when parked.
Capture still requires an active parked session. On first use it defaults on;
previously saved enable/disable choices are retained, including an explicit STOP.
The status row reports ignition/offroad/health blockers, missing camera frames, missing encoded
video or an unavailable controller. Camera warmup and arming have separate countdowns once video is arriving; the
buffer counter confirms capture. The voltage row shows the current vehicle supply reading.
This implementation is ready for parked hardware evaluation; comma 3X encoding,
thermal behavior, battery draw, and Connect indexing/playback still require device validation.

## Parking risk choice

Each drive ends with a full-screen parking overlay on comma 3X. The top left shows
the controller's camera warmup/arming countdown; the top right X dismisses it without
changing the default Low Risk choice. Low Risk (yellow) and High Risk (red) split the
upper selection area. The green lower third turns Sentry off for this drive only.
Every choice returns to the regular offroad UI. The prompt also closes when arming
finishes, stays awake while visible, and cannot intercept the driving screen.
An explicit global Enable Sentry/STOP setting is still respected.

- Low Risk saves events from sudden sound changes or accelerometer/gyroscope activity.
  While idle, camera region analysis is skipped; a small live background is kept so
  tracking can start on the sensor-triggered frame. All three camera buffers remain
  active so pre-event footage is preserved. After a sensor hit, the preview and eye
  follow the strongest motion in the two wide cameras with the existing handoff
  smoothing. Visual motion alone cannot start or extend a Low Risk event. With no
  visible motion, the preview holds its last view (initially front wide).
- High Risk adds the existing two-wide-camera motion detector and preview handoffs.
  Its red eye follows wide-camera motion while watching as well as recording.
- Sound is compared with a learned ambient level (12 dB rise, using micd's uncalibrated
  readings); IMU changes are compared with a learned gravity/bias vector (0.6 m/s² or
  0.004 rad/s). Moderate gyro hits still require 80 ms confirmation; stronger hits can trigger immediately,
  and bursts are debounced for two seconds. Each input settles for three seconds and
  relearns after missing data. The gyro floor is based on stationary samples from
  `000003ef--91bc370306/5`: after excluding stopping/pull-away transitions, deviation
  from the median bias was about 0.0047 rad/s near the bookmark and peaked near
  0.0057 rad/s. The new floor is about 15% below the bookmarked amplitude. This qlog
  retains only ~1 Hz IMU samples, so it cannot verify the native 104 Hz confirmation
  timing or distinguish wind from engine vibration. Tests use that measured amplitude
  in a synthetic sustained pulse; actual parked-device false-trigger behavior still
  needs validation. Audio and acceleration thresholds were deliberately left unchanged.
- The existing micd and sensord processes run during parked capture; no new microphone
  recorder is added. Sentry consumes sound levels, not raw audio. It drains queued IMU
  samples to avoid missing a short impact between controller ticks.
- Both modes use the same camera warmup, arming delay, pre/post-event buffer, route,
  bookmark, thermal, battery and parking-time rules. Mode selection does not restart
  arming or extend the parking budget. Risk mode is also available immediately below
  Enable Sentry in settings. Choosing a mode there clears the temporary off override.
- Low Risk, the temporary off override and the prompt acknowledgment reset for the
  next drive; manager startup also resets them. Permanent disabling stays permanent.

The native parameter table changed: update and rebuild before restarting UI/manager.
The standard Sentry updater explicitly rebuilds both the recorder and Python parameter library,
then reboots after success. No panda changes are needed.

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
- No route directories or video files are written before the first event. After a later
  event resumes, small logs and black previews bridge the intervening whole minutes. Runtime control
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
  disarming capture. After the tail expires, saving pauses and the controller returns to
  Watching; the armed session keeps its route identifier. Subsequent events, even after
  long quiet gaps, append to that route without another arming delay. There is no fixed
  15-minute split; parking, battery, thermal and storage limits still apply.
- Warmup and arming footage stays in the RAM history. A later event can therefore include
  the initial exposure/IR fade in its pre-roll even though that fade did not trigger a save.
  The motion bookmark identifies the confirmed trigger, rather than the start of pre-roll.
- Connect's timeline is orange only where saved preview footage is available, blue across
  unsaved gaps, and yellow at confirmed-motion bookmarks. Log-only state annotations mark
  the first saved frame and the end of its continuous run, including pre-roll and the tail.
  These annotations never publish live driving alerts. Existing uploaded routes are not
  rewritten or recolored.
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

Sentry groups events from one uninterrupted armed capture
session under one native route identifier. No route is allocated until the first saved
event, so an arm with no detections creates no empty route. Disarming/capture shutdown or
a recorder/encoder restart ends the grouping; the next armed capture creates a new route.
Each event still saves its selected pre-roll and post-motion tail:

- `fcamera.hevc`, `ecamera.hevc`, `dcamera.hevc`, `qcamera.ts` for all three cameras;
- native `rlog.zst` / `qlog.zst`, init data, saved car parameters, camera states,
  sentry metadata, timestamps, position estimate and segment/route sentinels;
- an internal `.sentry` marker for uploader identification and lock recovery.

All footage segments repeat one route-level `initData`, with its UTC wall time and monotonic
timestamp anchored to the first retained, independently decodable preview frame. Segment
numbers and sentinels use real 60-second intervals from that same origin, even when
an output minute combines video from two source minutes. Earlier code kept the source-minute
origin after trimming, leaving an empty leading timeline interval; simply coloring that
interval orange did not make footage exist there. The recorder now trims the origin and
rebuilds the minute boundaries together. Individual frame and bookmark timestamps do not shift.
Timezone offsets apply only to the local display/watermark, never the native route clock.
The corrected metadata applies to new sessions; already uploaded routes are not rewritten.

Quiet gaps retain their real duration. When footage resumes, intervening whole minutes
receive a small `qlog.zst`, a valid black `qcamera.ts` and the internal `.sentry` marker.
For example, footage in segments 0 and 3 produces black-preview segments 1 and 2.
Each gap log has compact `initData`, start-of-segment, disabled/normal timeline state,
preview encode indexes, and end-of-segment.
The compact header preserves the original route clock, device and software identity;
it omits repeated parameter snapshots, hardware command output and car parameters. The
full snapshots remain in footage segments. There are no sensor-camera messages,
bookmarks, `rlog.zst` or HEVC files in a gap minute. Gap logs and previews use the ordinary
uploader, including its existing metered-network policy; full camera files still require
a Connect request. No account token or forced full-video upload is added.

The filler is an embedded 104-byte black H.264 IDR with the exact SPS/PPS used by the
comma 3X hardware preview: 526×330, High profile/level 2.1. Matching the full decoder
configuration matters; matching only resolution/profile does not suffice. The first
filler version used x264's different reference, prediction, quantizer, deblocking and
VUI settings. It could decode with a decoder accepting parameter-set changes while
failing when the initial configuration remained in use. The current asset keeps the
hardware parameter sets unchanged through black-to-camera transitions. No software
encoder runs on the device. The recorder remuxes one independently
decodable frame per second at the original route timestamps. Two final frames at 59.90
and 59.95 seconds give hls.js its expected last-sample duration, producing a complete
60-second minute. Empty files and three-frame minimal files are not used: an empty file
cannot decode, and the 1,692-byte three-frame experiment was misidentified as MPEG-PS by
automatic probing. Very long still-frame holds also make seeking less predictable.

Partial-minute gaps are filled too: before resumed footage, after an immutable event
tail, and between non-overlapping events in the same minute. A brief lens-handoff wait
retains the existing last-frame hold. Real video rejoins after filler only on an IDR with
its own SPS/PPS restored. The three HEVC streams are never padded or relabeled. Preview
indexes include filler so in-file indexes stay correct; filler uses `frameId = UINT32_MAX`
to distinguish it from sensor frames. Blue/orange availability is derived exclusively from
accepted real preview frames, not these synthetic indexes. Bookmarks retain their actual
motion times. A final open partial minute is only filled through its logged end on disarm;
already finalized minutes are not rewritten.

An arm without an event still writes no route, and waiting after the last event does not
keep creating gap logs. Only a later recording supplies the far boundary. Backfilling
runs one minute per recorder tick; disarming completes the remaining queue. A recent gap
stays pending until its minute ends at least 60 seconds in the past, or earlier segments
have already been finalized past it. If newly selected pre-roll fills it meanwhile, the
real footage takes its place before any minimal log is published. Existing segment
directories are never replaced by filler. Already uploaded routes are not repaired.

Events inside the same minute append to that minute's open files; overlapping pre-roll is saved
only once. After an unsaved gap, each video resumes on a decodable keyframe. Qcamera PTS,
camera indexes, log clocks and bookmarks retain the original real-time offsets. The three
HEVC files remain assigned to their original lenses.

Orange availability follows accepted qcamera frames, not the requested recording deadline
or the start of a segment minute. Each continuous run ends one frame period after its last
saved frame. Its closing `normal` state is written into the same event log, so it cannot
carry orange across missing minutes. Overlapping pre-roll that fills a pause stays orange;
an actual unsaved gap gets a blue interval. There are only two state annotations per saved
run in each footage segment. Each wholly quiet minute between events instead has just one
normal state in its minimal gap log; there is no periodic idle telemetry stream.

The complete black TS measured **46,060 bytes per minute** using the native VideoWriter;
the compact native-test log with all 62 preview indexes stayed below 1 kB. Ten hours is
about **28 MB of video plus logs**, excluding filesystem allocation and HTTP/TLS overhead.
That is about one tenth of the measured 20 fps black-video payload (474,512 bytes/minute).
It still means two uploads per quiet minute, or 1,200 files for ten hours; small payloads
do not remove per-request latency or server work. Quiet minutes are generated only once
another event resumes, not continuously during idle parking. The native regression covers
a ten-hour gap, recent gaps filled by expanded pre-roll, shared route clocks, decoded
footage and unchanged bookmarks.

An incomplete output minute stays locked until its end is at least 60 seconds behind the
capture clock, covering the maximum selectable pre-roll even if the setting changes. This
can delay its automatic qlog/preview upload by up to about two minutes after saving pauses.
Disarming closes it immediately, and completed continuous-recording minutes can upload
as before. Once unlocked, files are never reopened or rewritten. If all files have already
closed when disarming, no empty final segment is created merely to add an end-of-route sentinel.

Connect compatibility was reviewed at
[`d6d8ab03f67d78d028f58adef2e12cb9a8c9cbb0`](https://github.com/commaai/connect/commit/d6d8ab03f67d78d028f58adef2e12cb9a8c9cbb0):
its [player](https://github.com/commaai/connect/blob/d6d8ab03f67d78d028f58adef2e12cb9a8c9cbb0/src/components/DriveVideo/index.jsx)
selects hls.js **1.4.8** with a 40-second target buffer, while iPhones use native HLS.
It uses one route-relative video clock, and reports missing-TS HTTP 404 errors. Its
[event loader](https://github.com/commaai/connect/blob/d6d8ab03f67d78d028f58adef2e12cb9a8c9cbb0/src/actions/cached.js)
loads timeline colors separately from video. Filler therefore keeps the normal qcamera
format and real elapsed time; it does not require player changes or a new playlist type.

`system/sentry/tests/test_connect_preview.cjs` exercises the exact hls.js 1.4.8 transmuxer
with native recorder output, both sequentially and after forward/backward seeks. It
checks fragment timing, dropped samples and parser errors, and exports MP4s for decoding.
`system/sentry/tests/test_filler.py` also validates the filler against the original
comma 3X decoder configuration. Set `SENTRY_PREVIEW_SAMPLE` to a local device
`qcamera.ts` to check decoded camera pixels across repeated filler transitions,
with decoder parameters held fixed. The development generator refuses unexpected
encoder headers instead of silently emitting an incompatible filler asset.
This does not emulate native iPhone playback or the private server that builds playlists.
New device uploads must confirm seeking, playback, later appended events and manual
camera requests. The filler removes missing video locally; it cannot eliminate network
delays, backend processing delays or gaps while uploads are still pending. It does not
automatically skip quiet time; the timeline remains a real-time view of the armed session.

### Update and rebuild

With the vehicle parked and offroad, run `bash tools/sentry_update.sh` through a tmux session.
The script supports `batmobile` and `sentry-session-routes-poc`, keeps the installed branch,
and stops Sentry so saved files can close. It
fast-forwards the installed branch, synchronizes its pinned submodules and LFS files, and
forces a fresh native recorder build. It then checks `AGNOS_VERSION` from `launch_env.sh`
against `/VERSION`, uses the stock AGNOS updater if needed, restores the previous Sentry
enable setting, and reboots. A failed update or build does not reboot; local changes and
diverged history are never discarded. Re-enable Sentry manually after resolving a failure.
For an update that must finish with Sentry enabled, run `bash tools/sentry_update.sh --enable-sentry`.
This also clears the temporary "off for this drive" choice after a successful build. Without
that option, a retry after an earlier failed update can preserve the disabled state left by
that failure. The updater uses blocking parameter writes and verifies the saved value before
rebooting. Its output is retained in `/data/sentry-update.log`, including the starting and
restored enable settings. Failures still leave capture paused until the build is repaired;
the option does not bypass battery, temperature or parking limits.
The updater also stops before disabling Sentry if a shutdown/reboot/uninstall request is
pending, `ForcePowerDown` is set, or the saved parked-energy estimate is already exhausted
while stock power-down is enabled. This last case can otherwise make the running hardware
daemon shut down as soon as Sentry is disabled, even with a healthy measured battery voltage.
The check does not disable protections or guarantee the battery/energy estimate will remain
sufficient throughout a long build.
The next captured route will include minimal logs and black preview video between saved events.

### Investigating a complete power-off

`python tools/sentry_power_check.py` is read-only and prints only an allowlist of power
settings, the Sentry phase, and matching parameters from the latest three saved bootlogs.
The current-settings portion reads files directly, so it also works when native Params
cannot load. Manager copies parameters into a bootlog before clearing transient keys;
these snapshots can retain the previous `DoShutdown`, `LastManagerExitReason`, AGNOS
watchdog and power-drop records after those values have disappeared from current settings.
Account tokens and location are excluded from the report.

The shutdown paths differ:

- `DoShutdown` clears on manager startup; `ForcePowerDown` is persistent and is not written
  by the Sentry updater. With a live Sentry session, a set force flag can shut down immediately.
- A Sentry `Battery cutoff` phase also requests immediate shutdown. The controller reaches
  that phase after ten seconds of valid low readings. This path bypasses the stock first-boot
  minimum-on-time gate; increasing arming delay does not defer battery protection.
- `SentryShutdownOnOverheat` persists (default off), so an enabled temperature shutdown can
  operate on a hot first boot. `SentryShutdownOnTimeLimit` defaults on, but its parking clock
  and expiry latch live in RAM and restart after a real reboot.
- While Sentry is active, the estimated `CarBatteryCapacity` is not a shutdown condition.
  Disabling Sentry restores stock policy, including that estimate. A reproduction at 12.5 V,
  a zero estimate and an elapsed stock offroad delay changes from no shutdown to shutdown
  on that enable-switch transition. This establishes a possible updater failure, not proof
  that it caused a particular device's power-off.

The empty-lead-in regression was reproduced against the old recorder. The correction passed
12 native recorder cases, including 15/30/60-second trimming, only five seconds of playable
preview history, three output minutes crossing different source-minute boundaries, and
unchanged-deadline camera handoffs. Tests decode the saved streams and check the route/video
origin, real frame times, shared initData, segment numbers and bookmarks.
Connect's deployed backend still needs a fresh device event to verify its rendered timeline.

The recorder writes standard `userBookmark` events into both logs at the confirmed trigger,
and again when a new confirmed burst follows at least five seconds without confirmation.
Manual saves and resumed event windows also get a marker. Continuous movement does not create
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

The row order is Sentry status/STOP, Enable Sentry, Risk mode, Arming delay, Pre-event buffer,
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

The armed-session route change passes 98 checks: 12 native IPC recorder cases,
one deterministic native session replay, and 85 controller/upload checks. The replay
uses real H264/HEVC keyframes and dependent frames, decodes all four saved outputs,
checks overlapping events and an unsaved gap within a minute, skips two whole quiet
minutes, resumes partway through a later minute, verifies byte-for-byte immutable
finalized files, and starts a new route after disarming. Real timestamps, shared
initData, sequential in-file indexes and motion bookmarks are checked, with no
synthetic alert states. The recorder and native test executable compile on the PC.
This change needs the native recorder rebuilt after switching branches; a Python
restart alone cannot load it. Hardware encoding and Connect's sparse playback,
route growth and manual camera requests still need the parked-device test.

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
