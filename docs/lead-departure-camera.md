# Offline camera continuity experiment

`tools/lead_departure_camera.py` tests whether **manually selected original rear
features** remain visually continuous when the model's lead heads split. It is
an offline experiment, with no imports into radard, modeld or the planner. It
does not detect vehicles automatically, identify a physical vehicle uniquely,
track lane dividers or establish that a destination lane is clear. Its output
always has `can_ignore_lead=false`, including for a strong match.

The image tracker acquires before the split and retains those features without
reseeding. It uses pyramidal Lucas–Kanade flow in both directions, rejects bad
round trips and image errors, and estimates a partial affine transform with
RANSAC. At least 12 original features, 60% of the initial population and 70%
geometric inliers must remain. A lost track stays lost until an explicit new
acquisition. Duplicate/reversed timestamps, gaps above 150 ms, invalid images,
range values or rotation matrices revoke the track. Observation expires after
2.5 seconds; the feature population is bounded at 80.

The optional `rotation_homography(K, R)` helper compensates image rotation when
time-aligned camera intrinsics and an **old-camera-to-new-camera** rotation are
available. Vehicle yaw alone is not that rotation. Recorded results below did
not use this helper: they are uncompensated visual scales, not calibrated depth
measurements. Translation, changing vehicle orientation, articulated trailers,
illumination and mixed foreground/background texture remain confounders.

For an unchanged object face, an initial distance `d0` and current distance `d`
would imply image scale `d0 / d`. The tool reports this value, observed cumulative
scale, and `log(observed / implied)`. A disagreement is an association/depth
diagnostic. It cannot distinguish a bad model distance from selection of another
real vehicle. Partial occlusion can leave enough original features for a valid
continuity observation; that is deliberately not an identity guarantee.

## Reproducible use

OpenCV is an optional workstation dependency. It is not added to the device
environment. The validation used OpenCV 4.12.0.88 and NumPy 2.3.5.

```sh
python3 -m pip install opencv-python-headless==4.12.0.88
python3 openpilot/selfdrive/controls/tests/test_lead_departure_camera.py
python3 tools/lead_departure_camera.py fcamera.hevc \
  --start 47.4 --duration 2 --roi 854 132 160 200 \
  --frame-times frame_times.json --ranges ranges.json
```

The ROI is `left top width height`, in decoded image pixels, selected from the
acquisition frame before inspecting the split. Raw HEVC lacks usable PTS here;
the tool requires explicit capture timestamps instead of assuming 20 Hz. It
rejects unusable PTS, a missing/empty video and an acquisition outside the video
with a nonzero exit status. Container PTS is usable only when verified to share
the supplied range clock.

`frame_times.json` is an array indexed by decoded frame. Build it from the same
segment's `narrowRoadEncodeIdx.segmentId` and `timestampSof`. Verify that IDs are
exactly `0..N-1`; subtract the first capture time. `ranges.json` contains
`{"time": seconds, "distance": metres}` rows from valid `modelV2` messages,
using `logMonoTime` on that same origin and `leadsV3[0].x[0]`. This preserves
publication causality. The tool accepts only the latest already published
sample, no older than 100 ms, and never matches a future sample. Missing range
data produces no scale residual. Example extraction in an installed checkout:

```python
import json
from openpilot.tools.lib.logreader import LogReader

events = list(LogReader('rlog.zst'))
index = sorted((int(e.narrowRoadEncodeIdx.segmentId),
                int(e.narrowRoadEncodeIdx.timestampSof))
               for e in events if e.which() == 'narrowRoadEncodeIdx')
assert [i for i, _ in index] == list(range(len(index)))
origin = index[0][1] / 1e9
times = [sof / 1e9 - origin for _, sof in index]
ranges = [{'time': e.logMonoTime / 1e9 - origin,
           'distance': float(e.modelV2.leadsV3[0].x[0])}
          for e in events if e.which() == 'modelV2' and e.valid
          and len(e.modelV2.leadsV3) and e.modelV2.leadsV3[0].x[0] > 0]
json.dump(times, open('frame_times.json', 'w'))
json.dump(ranges, open('ranges.json', 'w'))
```

## Recorded observations

The four known braking examples were evaluated at the original narrow camera
resolution, 1928×1208. These are discovery cases, not an independent accuracy
benchmark. Each run retained original features for all 40 frames in its selected
two-second window. The table samples the first capture after the previously
identified raw-head split. Scales are relative to acquisition, not velocities.

| Case | Requested start | Manual ROI | Sample time | Features, initial → last | Visual scale | Range-implied scale |
|---|---:|---|---:|---:|---:|---:|
| A | 47.4 s | 854,132,160,200 | 47.849 s | 62 → 55 | 1.002 | 1.253 |
| B | 26.5 s | 884,108,139,195 | 27.050 s | 63 → 62 | 1.004 | 1.200 |
| C | 40.2 s | 918,210,99,83 | 40.750 s | 20 → 18 | 0.987 | 1.244 |
| D | 11.3 s | 886,261,119,72 | 11.850 s | 33 → 32 | 1.001 | 1.367 |

The original rear features did not show the growth implied by the model's
selected range. That supports investigating reassociation/depth discontinuity;
it does not prove the other lead is fictitious. Reduced 526×330 video acquired
only cases A/B; C/D correctly rejected insufficient texture. Full-resolution
input matters for this particular observation.

The real dark semi cut-in was also attempted at 16.1 s with ROI
`1002,243,106,94`, before the driver's first brake at 19.411 s. It acquired 13
features and revoked the track on the next frame at 16.151 s for inconsistent
flow. **This is a failure to obtain usable visual evidence, not successful
cut-in classification.** No thresholds were relaxed to make that example pass.

Video SHA-256 identities (raw video and telemetry remain outside the repository):

| Case | SHA-256 |
|---|---|
| A | `f5b1b0db67aa2943020e4d5007b5647c15a8264e275aa755efc64441f5adff49` |
| B | `089106574b5b7b276180e6941bf487378c494fd4e88339b629483bb4fcc1a5f2` |
| C | `d5aaf19565074d25fbcd85aa158c38359d46363d62ef8ab6791b8489fa7e524c` |
| D | `92dcd19c7a591e7db35f20ff891550df136228a639dc734752888795dadbcd22` |
| Semi | `91b8cdbe129f7e9ae38ee6c7820e94b030ef4f724229018954ccdcc71004836f` |

## Validation boundary

Twenty-two unit tests cover translation, expansion, range disagreement, causal
range lookup, low texture, full occlusion, replaced texture, a partial
distractor, image/rotation/range corruption, frame gaps, fixed expiry, explicit
reacquisition, input immutability, rotation compensation and CLI failure paths.
Full occlusion/replacement rejects the track; partial occlusion may retain a
subset and still cannot authorize ignoring a lead.

Automatic acquisition, reliable vehicle/lane association through cut-ins,
calibrated real-drive rotation compensation, independent clips and device
runtime evaluation are still required before any control integration. The
shadow split observer and the offline feature tracker do not remove friction
braking, alter a lead velocity or change planner authority.
