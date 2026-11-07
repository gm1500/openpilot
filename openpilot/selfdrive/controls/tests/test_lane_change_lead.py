import copy
import math
import json
from pathlib import Path
import unittest

from openpilot.selfdrive.controls.lib.lane_change_lead import LaneChangeLeadRecovery
from openpilot.selfdrive.controls.lib.vision_lead_tracker import VisionLeadObservation


class TestLaneChangeLeadRecovery(unittest.TestCase):
  def setUp(self):
    self.recovery = LaneChangeLeadRecovery()
    self.time = 0.0

  def sample(self, distance, ego=30.0, speed=29.0, acceleration=0.0, lateral=0.0, std=8.0, probability=0.99,
             lane_change=True, yaw=0.0, valid=True, dt=0.05, speed_std=2.0, modify=None):
    self.time += dt
    obs = [VisionLeadObservation(distance, lateral, std, probability) for _ in range(2)]
    raw = [{'present': True, 'radar': False, 'dRel': distance, 'yRel': lateral, 'vLead': speed, 'vLeadK': speed, 'vRel': speed - ego,
            'aLeadK': acceleration, 'aLeadTau': 0.3, 'modelProb': probability, 'radarTrackId': -1} for _ in range(2)]
    leads = copy.deepcopy(raw)
    speed_stds = [speed_std, speed_std]
    if modify is not None:
      modify(obs, raw, leads, speed_stds)
    originals = copy.deepcopy((obs, raw, leads, speed_stds))
    out = self.recovery.update(obs, raw, leads, speed_stds, self.time, ego, lane_change, yaw, valid)
    self.assertEqual((obs, raw, leads, speed_stds), originals)
    for a, b in zip(leads, out, strict=True):
      self.assertEqual({k: v for k, v in a.items() if k not in ('vLead', 'vLeadK', 'vRel')},
                       {k: v for k, v in b.items() if k not in ('vLead', 'vLeadK', 'vRel')})
      if a.get('present') and a.get('vLead') is not None and math.isfinite(a['vLead']):
        self.assertGreaterEqual(b['vLead'], a['vLead'])
    return leads, out

  def acquire(self, frames=36, noise=False):
    changed = []
    for i in range(frames):
      distance = 60.0 + 4.0 * (i + 1) * 0.05
      if noise:
        distance += 3.0 * (-1) ** i
      before, out = self.sample(distance, speed=30.0)
      changed.append(out[0]['vLead'] - before[0]['vLead'])
    return changed

  def test_confirmed_opening_recovers_with_bounded_speed(self):
    changes = self.acquire()
    self.assertGreater(max(changes), 0.5)
    self.assertLessEqual(max(changes), 2.0)
    for previous, current in zip(changes[:-1], changes[1:], strict=True):
      self.assertLessEqual(current - previous, 0.100001)
    self.assertTrue(all(change == 0.0 for change in changes[:14]))

  def test_distance_jitter_keeps_private_history(self):
    changes = self.acquire(frames=30, noise=True)
    self.assertGreaterEqual(len(self.recovery.history), 20)
    # Every alternating step exceeds the ordinary tracker's 5 m gate.
    self.assertGreater(max(changes), 0.0)

  def test_not_changing_lane_is_exact_passthrough(self):
    for i in range(60):
      before, out = self.sample(60.0 + i * 0.3, lane_change=False)
      self.assertEqual(before, out)
    self.assertFalse(self.recovery.history)

  def test_equal_speed_and_closing_do_not_recover(self):
    for rate in (0.0, -0.5, -2.0, -5.0):
      with self.subTest(rate=rate):
        self.setUp()
        for i in range(50):
          before, out = self.sample(90.0 + rate * i * 0.05)
          self.assertEqual(before, out)

  def test_braking_immediately_revokes_a_correction(self):
    self.assertGreater(max(self.acquire()), 0.0)
    before, out = self.sample(67.0, acceleration=-0.2)
    self.assertEqual(before, out)
    self.assertEqual(self.recovery.correction, 0.0)
    self.assertFalse(self.recovery.history)

  def test_other_hypothesis_braking_revokes_correction(self):
    self.acquire()
    def braking(obs, raw, leads, speed_stds):
      raw[1]['aLeadK'] = leads[1]['aLeadK'] = -1.0
    before, out = self.sample(67.0, modify=braking)
    self.assertEqual(before, out)

  def test_existing_tracker_braking_guard_is_preserved(self):
    self.acquire()
    def braking(obs, raw, leads, speed_stds):
      leads[0]['vLead'] = 27.0
    before, out = self.sample(67.0, modify=braking)
    self.assertEqual(before, out)

  def test_close_stopped_and_urgent_leads_are_unchanged(self):
    for distance, speed in ((20.0, 30.0), (44.0, 30.0), (70.0, 0.0), (70.0, 20.0), (40.0, 27.0)):
      with self.subTest(distance=distance, speed=speed):
        self.setUp()
        self.acquire()
        before, out = self.sample(distance, speed=speed)
        self.assertEqual(before, out)
        self.assertEqual(self.recovery.correction, 0.0)

  def test_large_target_switch_has_no_speed_bridge(self):
    self.acquire()
    for i in range(12):
      before, out = self.sample(95.0 - 0.05 * i)
      self.assertEqual(before, out)
    self.assertLess(self.recovery.history[-1].time - self.recovery.history[0].time, self.recovery.MIN_HISTORY)

  def test_gradual_target_switch_does_not_create_opening(self):
    for i in range(50):
      # Per-frame changes are small, but this is a switch to a farther car,
      # not a 50 m/s lead pulling away. Model motion contradicts the drift.
      before, out = self.sample(60.0 + 1.0 * i)
      self.assertEqual(before, out)

  def test_single_outlier_is_quarantined_without_speed_output(self):
    self.acquire(frames=30)
    count = len(self.recovery.history)
    before, out = self.sample(90.0)
    self.assertEqual(before, out)
    self.assertEqual(len(self.recovery.history), count)
    before, out = self.sample(66.4, speed=30.0)
    self.assertGreater(len(self.recovery.history), 20)
    self.assertEqual(before, out)  # reconfirm before permitting any correction

  def test_temporary_hypothesis_split_keeps_private_history_only(self):
    self.acquire(frames=30)
    def split(obs, raw, leads, speed_stds):
      obs[1] = VisionLeadObservation(72.0, 2.0, 8.0, 0.99)
    before, out = self.sample(66.2, modify=split)
    self.assertEqual(before, out)
    self.assertGreater(len(self.recovery.history), 20)
    before, out = self.sample(66.4, speed=30.0)
    self.assertEqual(before, out)
    self.assertGreater(len(self.recovery.history), 20)

  def test_persistent_hypothesis_split_discards_history(self):
    self.acquire()
    def split(obs, raw, leads, speed_stds):
      obs[1] = VisionLeadObservation(80.0, 3.0, 8.0, 0.99)
    for _ in range(8):
      before, out = self.sample(67.0, modify=split)
      self.assertEqual(before, out)
    self.assertFalse(self.recovery.history)

  def test_yaw_compensation_preserves_same_target(self):
    # At 100 m, a 0.12 rad/s yaw moves the image laterally by 0.6 m/frame.
    # Raw lateral gating would reject each frame despite a consistent car.
    for i in range(25):
      self.sample(100.0 + i * 0.2, lateral=-0.6 * i, yaw=-0.12, speed=30.0)
    self.assertGreater(len(self.recovery.history), 20)

  def test_lateral_cut_in_is_not_borrowed_from_old_target(self):
    self.acquire()
    for _ in range(3):
      before, out = self.sample(67.0, lateral=2.0)
      self.assertEqual(before, out)

  def test_recovery_window_expires(self):
    self.acquire()
    for i in range(45):
      before, out = self.sample(67.0 + i * 0.2, lane_change=False)
    self.assertEqual(before, out)
    self.assertEqual(self.recovery.reason, 'inactive')
    self.assertFalse(self.recovery.history)

  def test_input_loss_time_gap_and_invalid_yaw_revoke(self):
    for kwargs in ({'valid': False}, {'dt': 0.3}, {'dt': -0.1}, {'yaw': None}, {'yaw': float('nan')},
                   {'yaw': 0.6}, {'probability': 0.5}, {'std': 40.0}, {'speed_std': 0.0}, {'ego': 5.0}):
      with self.subTest(kwargs=kwargs):
        self.setUp()
        self.acquire()
        before, out = self.sample(67.0, **kwargs)
        self.assertEqual(before, out)
        self.assertEqual(self.recovery.correction, 0.0)

  def test_radar_and_absent_lead_passthrough(self):
    for key, value in (('radar', True), ('present', False)):
      with self.subTest(key=key):
        self.setUp()
        self.acquire()
        def mutate(obs, raw, leads, speed_stds, key=key, value=value):
          raw[0][key] = leads[0][key] = value
        before, out = self.sample(67.0, modify=mutate)
        self.assertEqual(before, out)

  def test_speed_uncertainty_caps_correction(self):
    changes = []
    for i in range(36):
      before, out = self.sample(60.0 + 4.0 * (i + 1) * 0.05, speed=30.0, speed_std=0.4)
      changes.append(out[0]['vLead'] - before[0]['vLead'])
    self.assertAlmostEqual(max(changes), 0.4)

  def test_symmetric_lane_changes(self):
    outputs = []
    for sign in (-1, 1):
      self.setUp()
      current = []
      for i in range(30):
        _, out = self.sample(70.0 + i * 0.2, lateral=sign * 0.1 * i, yaw=sign * 0.025, speed=30.0)
        current.append(out[0]['vLead'])
      outputs.append(current)
    self.assertEqual(outputs[0], outputs[1])


class TestRecordedLaneChange(unittest.TestCase):
  def test_recorded_target_handoff_and_recovery(self):
    fixture = json.loads((Path(__file__).parent / 'fixtures/lane_change_lead.json').read_text())
    recovery = LaneChangeLeadRecovery()
    changed = []
    for timestamp, ego, changing, yaw, observations, published in fixture['frames']:
      obs = [VisionLeadObservation(*o[:4]) for o in observations]
      raw = [{'present': True, 'radar': False, 'dRel': o[0], 'vLead': o[4], 'aLeadK': o[5]} for o in observations]
      leads = [{'dRel': p[0], 'vLead': p[1], 'aLeadK': p[2], 'present': p[3], 'radar': p[4],
                'vLeadK': p[5], 'vRel': p[6]} for p in published]
      result = recovery.update(obs, raw, leads, [o[6] for o in observations], timestamp, ego, changing, yaw)
      for original, updated in zip(leads, result, strict=True):
        self.assertEqual(original['dRel'], updated['dRel'])
        self.assertEqual(original['aLeadK'], updated['aLeadK'])
        self.assertGreaterEqual(updated['vLead'], original['vLead'])
        self.assertLessEqual(updated['vLead'] - original['vLead'], recovery.MAX_CORRECTION)
      if result != leads:
        changed.append(timestamp)
      # Both heads disagree during the initial acquisition. Do not invent a
      # faster lead to suppress this recorded slowdown before opening is known.
      if timestamp < 33.0 or timestamp > 36.4:
        self.assertEqual(result, leads)
    self.assertTrue(changed)
    self.assertLess(min(changed), 35.0)  # ordinary distance history matures around 35.6 s


if __name__ == '__main__':
  unittest.main()
