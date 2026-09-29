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
    self.assertAlmostEqual(track.acceleration_noise(20.0, 25.0), 1.2)
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
