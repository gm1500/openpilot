import unittest

import numpy as np

from openpilot.selfdrive.controls.lib.vision_lead_tracker import (
  ACCEL_NOISE_BP,
  ACCEL_NOISE_V,
  VisionLeadObservation,
  VisionLeadTracker,
  _DistanceTrack,
)
from openpilot.selfdrive.controls.tests.test_vision_lead_tracker import lead, observation


class TestVisionLeadAdaptive(unittest.TestCase):
  def track(self, distance, ego, speed):
    track = _DistanceTrack(observation(distance), 0.0, ego)
    track.x[1] = speed
    return track

  def test_same_time_gap_is_more_responsive_when_closer(self):
    slow = self.track(30.0, 20.0, 20.0)
    fast = self.track(45.0, 30.0, 30.0)
    self.assertGreater(slow.acceleration_noise(30.0, 20.0), fast.acceleration_noise(45.0, 30.0))

  def test_route_derived_process_noise_schedule(self):
    track = self.track(40.0, 25.0, 25.0)
    for distance, expected in zip(ACCEL_NOISE_BP, ACCEL_NOISE_V, strict=True):
      # Choose ego so headway <= 1 s; this isolates the distance-interpolated base.
      ego = max(distance, 5.0)
      track.x[1] = ego
      self.assertAlmostEqual(track.acceleration_noise(distance, ego), expected)

  def test_close_response_and_far_smoothing_are_bounded(self):
    for distance in np.linspace(2.0, 200.0, 20):
      for ego in (0.0, 15.0, 25.0, 60.0):
        for speed in (-5.0, 0.0, 25.0, 70.0):
          track = self.track(distance, ego, speed)
          q = track.acceleration_noise(distance, ego)
          self.assertGreaterEqual(q, 0.01)
          self.assertLessEqual(q, 24.0)
    track = self.track(20.0, 25.0, 25.0)
    self.assertAlmostEqual(track.acceleration_noise(20.0, 25.0), 0.2)
    self.assertAlmostEqual(track.acceleration_noise(100.0, 25.0), 0.01)

  def test_fast_closing_boosts_far_response(self):
    track = self.track(100.0, 35.0, 15.0)
    self.assertAlmostEqual(track.acceleration_noise(100.0, 35.0), 0.04)
    track.x[1] = 35.0
    self.assertAlmostEqual(track.acceleration_noise(100.0, 35.0), 0.01)

  def test_schedule_has_no_boundary_jumps(self):
    track = self.track(40.0, 25.0, 25.0)
    for boundary in ACCEL_NOISE_BP:
      self.assertLess(abs(track.acceleration_noise(boundary + 1e-5, max(boundary, 5.0))
                          - track.acceleration_noise(boundary - 1e-5, max(boundary, 5.0))), 1e-4)

  def test_measurement_floor_is_unchanged(self):
    for distance in (5.0, 10.0, 20.0, 50.0):
      obs = VisionLeadObservation(distance, 0.0, 0.1, 1.0)
      self.assertAlmostEqual(obs.variance, 1.0)

  def test_far_range_is_not_more_assertive_than_previous_nominal_tune(self):
    for distance in (35.0, 50.0, 70.0, 100.0):
      ego = 25.0
      track = self.track(distance, ego, ego)
      headway = distance / max(ego, 5.0)
      previous = 0.1 * float(np.interp(headway, [1.0, 2.5], [1.0, 0.5]))
      self.assertLessEqual(track.acceleration_noise(distance, ego), previous)

  def test_corroborated_closing_adds_bounded_response_only_when_both_cues_agree(self):
    track = self.track(50.0, 25.0, 25.0)
    track.age = 5.0
    track.P[1, 1] = 0.1
    track.time = 1.0
    track.dt = 0.05
    track.transition = 1.0
    track.range_history.clear()
    for i in range(13):
      t = 0.4 + 0.05 * i
      # 1 m/s closing with a constant 25 m/s ego implies a 24 m/s lead.
      track.range_history.append((t, 50.6 - (t - 0.4), 1.0, 25.0))

    noise_before = track.acceleration_noise(50.0, 25.0)
    out = track.closing_response_target(25.0, 24.0, observation(50.0))
    self.assertLess(out, 25.0)
    self.assertGreaterEqual(out, 24.0)
    self.assertLessEqual(25.0 - out, track.CLOSING_RESPONSE_RATE * track.dt + 1e-8)
    # Correct the published lag without making the private distance state noisier.
    self.assertAlmostEqual(track.acceleration_noise(50.0, 25.0), noise_before)

  def test_model_only_or_range_only_slowdown_does_not_trigger_response(self):
    for baseline, range_rate in ((24.0, 0.0), (25.0, -1.0)):
      with self.subTest(baseline=baseline, range_rate=range_rate):
        track = self.track(50.0, 25.0, 25.0)
        track.age = 5.0
        track.P[1, 1] = 0.1
        track.time = 1.0
        track.dt = 0.05
        track.transition = 1.0
        track.range_history.clear()
        for i in range(13):
          t = 0.4 + 0.05 * i
          track.range_history.append((t, 50.0 + range_rate * (t - 0.4), 1.0, 25.0))
        out = track.closing_response_target(25.0, baseline, observation(50.0))
        self.assertAlmostEqual(out, 25.0)
        self.assertAlmostEqual(track.acceleration_noise(50.0, 25.0), 0.02)

  def test_integer_range_history_is_valid_for_both_closing_paths(self):
    track = self.track(25, 25, 23)
    track.P[1, 1] = 0.1
    track.time, track.dt, track.age = 0.7, 0.1, 1.0
    track.range_history.clear()
    for i in range(8):
      track.range_history.append((i * 0.1, 32 - i, 1.0, 25))
    self.assertTrue(np.isfinite(track.pre_closing_target(25, observation(25), 25)))
    track.age = 5.0
    self.assertTrue(np.isfinite(track.closing_response_target(25, 24, observation(25))))

  def test_low_speed_closing_cannot_publish_an_optimistic_range_speed(self):
    track = self.track(11.3, 3.6, 4.1)
    track.age = 5.0
    track.P[1, 1] = 0.1
    track.time, track.dt = 1.0, 0.05
    model = lead(11.3, speed=2.5, ego=3.6, acceleration=-0.25)
    out = track.speed([model, model], observation(11.3), 3.6)
    self.assertLessEqual(out, 2.5)
    self.assertAlmostEqual(track.x[1], 4.1)

  def test_low_speed_pullaway_retains_distance_filter(self):
    track = self.track(12.0, 3.0, 3.5)
    track.age = 5.0
    track.P[1, 1] = 0.1
    track.time, track.dt = 1.0, 0.05
    model = lead(12.0, speed=4.0, ego=3.0, acceleration=0.1)
    track.speed([model, model], observation(12.0), 3.0)
    self.assertEqual(track.mode, 'distance')

  def test_near_stop_hold_preserves_model_output_while_range_state_keeps_learning(self):
    track = self.track(7.0, 0.5, 0.9)
    track.age = 5.0
    track.P[1, 1] = 0.1
    track.time = 1.0
    track.dt = 0.05
    model = lead(7.0, speed=0.2, ego=0.5)
    out = track.speed([model, model], observation(7.0), 0.5)
    self.assertEqual(track.mode, 'stop-hold')
    self.assertAlmostEqual(out, 0.2)
    self.assertAlmostEqual(track.x[1], 0.9)


  def test_stop_hold_release_never_starts_below_moving_model(self):
    track = self.track(7.0, 0.5, 1.4)
    track.age = 5.0
    track.P[1, 1] = 0.1
    track.time = 1.0
    track.dt = 0.05
    stopped = lead(7.0, speed=0.2, ego=0.5)
    self.assertAlmostEqual(track.speed([stopped, stopped], observation(7.0), 0.5), 0.2)
    moving = lead(7.0, speed=1.2, ego=0.5)
    out = track.speed([moving, moving], observation(7.0), 0.5)
    self.assertGreaterEqual(out, 1.2)

  def test_confirmed_stopped_lead_clamps_small_model_creep_to_zero(self):
    track = self.track(50.0, 13.0, 2.0)
    track.age = 5.0
    track.P[1, 1] = 0.1
    track.dt = 0.05
    stopped = lead(50.0, speed=0.3, ego=13.0, acceleration=0.0)

    for i in range(7):
      track.time = 1.0 + i * 0.05
      out = track.speed([stopped, stopped], observation(50.0), 13.0)

    self.assertEqual(track.mode, 'stopped-lead')
    self.assertAlmostEqual(out, 0.0)

  def test_stopped_lead_confirmation_rejects_creeping_or_unsettled_model(self):
    for speed, acceleration in ((0.9, 0.0), (0.3, -0.4)):
      with self.subTest(speed=speed, acceleration=acceleration):
        track = self.track(50.0, 13.0, 2.0)
        track.age = 5.0
        track.P[1, 1] = 0.1
        track.dt = 0.05
        candidate = lead(50.0, speed=speed, ego=13.0, acceleration=acceleration)
        for i in range(10):
          track.time = 1.0 + i * 0.05
          out = track.speed([candidate, candidate], observation(50.0), 13.0)
        self.assertNotEqual(track.mode, 'stopped-lead')
        self.assertGreater(out, 0.0)

  def test_stopped_lead_release_never_starts_below_moving_model(self):
    track = self.track(50.0, 13.0, 2.0)
    track.age = 5.0
    track.P[1, 1] = 0.1
    track.dt = 0.05
    stopped = lead(50.0, speed=0.3, ego=13.0, acceleration=0.0)
    for i in range(7):
      track.time = 1.0 + i * 0.05
      track.speed([stopped, stopped], observation(50.0), 13.0)
    self.assertEqual(track.mode, 'stopped-lead')

    moving = lead(50.0, speed=1.2, ego=13.0, acceleration=0.0)
    track.time += 0.05
    out = track.speed([moving, moving], observation(50.0), 13.0)
    self.assertGreaterEqual(out, 1.2)

  def test_far_track_filters_identical_distance_noise_more(self):
    rng = np.random.default_rng(7)
    near, far = VisionLeadTracker(), VisionLeadTracker()
    near_speeds, far_speeds = [], []
    for i in range(600):
      noise = rng.normal(0, 0.5)
      for tracker, distance, output in [(near, 25.0, near_speeds), (far, 100.0, far_speeds)]:
        out = tracker.update([observation(distance + noise)] * 2, [lead(distance, speed=25.0)] * 2, i * 0.05, 25.0)
        if i >= 200:
          output.append(out[0]['vLead'])
    self.assertLess(np.std(far_speeds), np.std(near_speeds))

  def test_near_track_responds_faster_to_changing_speed(self):
    speeds = []
    for initial_gap in (25.0, 100.0):
      tracker = VisionLeadTracker()
      for i in range(260):
        # A lead increases from 25 to 26 m/s after ten seconds. Model speed is held fixed.
        gap = initial_gap + max(i * 0.05 - 10.0, 0.0)
        out = tracker.update([observation(gap)] * 2, [lead(gap, speed=25.0)] * 2, i * 0.05, 25.0)
      speeds.append(out[0]['vLead'])
    self.assertGreater(speeds[0], speeds[1])
    self.assertLess(abs(speeds[0] - 26.0), 0.5)


if __name__ == '__main__':
  unittest.main()
