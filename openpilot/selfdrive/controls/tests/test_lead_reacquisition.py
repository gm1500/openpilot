import copy
from dataclasses import dataclass
import math
import unittest

from openpilot.selfdrive.controls.lib.lead_reacquisition import LeadReacquisitionShadow
from openpilot.selfdrive.controls.lib.vision_lead_tracker import VisionLeadObservation, VisionLeadTracker


@dataclass(eq=False)
class Track:
  ready: bool = True
  age: float = 10.0
  mode: str = 'distance'
  transition: float = 1.0


class TestLeadReacquisitionShadow(unittest.TestCase):
  def setUp(self):
    self.shadow = LeadReacquisitionShadow()
    self.anchor = Track()
    self.new = Track(ready=False, age=0.0, mode='fallback')
    self.time = 0.0

  def step(
    self,
    distance=60.0,
    speed=30.0,
    std=3.0,
    lateral=0.0,
    yaw=0.0,
    ego=30.0,
    tracks=None,
    acceleration=0.0,
    probability=0.99,
    present=True,
    second=None,
    dt=0.05,
    valid=True,
  ):
    self.time += dt
    a = VisionLeadObservation(distance, lateral, std, probability)
    observations = [a, second or a]
    leads = [
      {
        'present': present and o.valid(),
        'radar': False,
        'dRel': o.distance,
        'yRel': o.lateral,
        'vLead': speed,
        'vLeadK': speed,
        'vRel': speed - ego,
        'aLeadK': acceleration,
        'modelProb': o.probability,
      }
      for o in observations
    ]
    tracks = tracks or [self.anchor, self.anchor]
    before = copy.deepcopy((observations, leads, [vars(t) if t is not None else None for t in tracks]))
    result = self.shadow.update(observations, leads, leads, tracks, self.time, ego, yaw, valid)
    self.assertIsNone(result)  # An observer cannot be assigned as production lead output.
    self.assertEqual(before, (observations, leads, [vars(t) if t is not None else None for t in tracks]))
    return self.shadow.status

  def prime(self, **kw):
    for _ in range(30):
      self.step(**kw)
    self.assertTrue(self.shadow.evidence)

  def jump(self, **kw):
    return self.step(distance=67.0, speed=27.0, std=12.0, tracks=[self.new, self.new], **kw)

  def test_shadow_predicts_range_and_speed_without_changing_control(self):
    self.prime()
    status = self.jump()[0]
    self.assertTrue(status.active)
    self.assertAlmostEqual(status.distance, 60.0)
    self.assertAlmostEqual(status.speed, 30.0)
    self.assertAlmostEqual(status.delta_distance, -7.0)
    self.assertAlmostEqual(status.delta_speed, 3.0)
    self.assertEqual(len(self.shadow.episodes), 1)
    self.assertEqual(self.shadow.bridge_frames, [1, 1])

  def test_baseline_detects_gradual_uncertainty_growth(self):
    self.prime()
    for std in [4.0, 5.0, 6.0, 8.0, 10.0]:
      self.step(std=std)
    # 12 is not twice the previous frame's 10, but is four times stable 3.
    status = self.jump()[0]
    self.assertTrue(status.active)
    self.assertAlmostEqual(status.std_ratio, 4.0)

  def test_expiration_is_total_elapsed_time_not_restartable_frames(self):
    self.prime()
    self.assertTrue(self.jump()[0].active)
    for d in [71.0, 67.0, 71.0]:
      self.assertTrue(self.step(distance=d, speed=27.0, std=12.0, tracks=[Track(False), Track(False)])[0].active)
    status = self.step(distance=67.0, speed=27.0, std=12.0, tracks=[self.new, self.new])[0]
    self.assertFalse(status.active)
    self.assertEqual(status.reason, 'expired')
    for _ in range(10):
      self.assertFalse(self.jump()[0].active)
    self.assertEqual(self.shadow.bridge_frames, [4, 4])

  def test_coherent_replacement_ends_prediction_early(self):
    self.prime()
    self.jump()
    self.assertTrue(self.step(distance=66.85, speed=27.0, std=12.0, tracks=[self.new, self.new])[0].active)
    status = self.step(distance=66.7, speed=27.0, std=12.0, tracks=[self.new, self.new])[0]
    self.assertEqual(status.reason, 'consistent_replacement')
    self.assertFalse(status.active)

  def test_precise_closer_replacement_does_not_borrow_speed(self):
    self.prime()
    status = self.step(distance=45.0, speed=28.0, std=3.0, tracks=[self.new, self.new])[0]
    self.assertFalse(status.active)

  def test_immediate_braking_in_either_hypothesis_clears_episode(self):
    for slot in (0, 1):
      with self.subTest(slot=slot):
        self.setUp()
        self.prime()
        self.jump()
        observations = [VisionLeadObservation(67.0, 0.0, 12.0, 0.99)] * 2
        leads = [{'present': True, 'radar': False, 'dRel': 67.0, 'vLead': 27.0, 'aLeadK': 0.0} for _ in range(2)]
        leads[slot]['aLeadK'] = -0.3
        self.time += 0.05
        self.shadow.update(observations, leads, leads, [self.new, self.new], self.time, 30.0, 0.0)
        self.assertEqual(self.shadow.status[0].reason, 'braking')
        self.assertFalse(self.shadow.episodes)
        self.assertFalse(self.jump()[0].active)

  def test_short_ttc_or_gap_bypasses_shadow(self):
    for distance, speed in [(29.0, 30.0), (60.0, 20.0), (10.0, 0.0)]:
      with self.subTest(distance=distance, speed=speed):
        self.setUp()
        self.prime()
        status = self.step(distance=distance, speed=speed, std=20.0, tracks=[self.new, self.new])[0]
        self.assertFalse(status.active)
        self.assertEqual(status.reason, 'short_gap_or_closing')

  def test_single_accepted_hypothesis_can_be_observed(self):
    absent = VisionLeadObservation(0.0, 0.0, 3.0, 0.0)
    self.prime(second=absent, tracks=[self.anchor, None])
    status = self.step(distance=67.0, speed=27.0, std=12.0, second=absent, tracks=[self.new, None])
    self.assertTrue(status[0].active)
    self.assertFalse(status[1].active)

  def test_old_track_in_other_slot_is_not_copied_to_candidate(self):
    self.prime()
    second = VisionLeadObservation(60.0, 1.0, 3.0, 0.99)
    status = self.step(distance=67.0, speed=27.0, std=12.0, second=second, tracks=[self.new, self.anchor])[0]
    self.assertFalse(status.active)
    self.assertEqual(status.reason, 'anchor_still_assigned')

  def test_two_distinct_candidates_cannot_borrow_one_anchor(self):
    self.prime()
    second = VisionLeadObservation(70.0, 0.1, 12.0, 0.99)
    statuses = self.step(distance=67.0, speed=27.0, std=12.0, second=second, tracks=[self.new, Track(False)])
    self.assertTrue(all(not s.active and s.reason == 'ambiguous_target' for s in statuses))

  def test_slot_swap_retains_identity_without_creating_transition(self):
    other = Track()
    self.prime(second=VisionLeadObservation(90.0, 3.0, 3.0, 0.99), tracks=[self.anchor, other])
    statuses = self.step(distance=90.0, lateral=3.0, second=VisionLeadObservation(60.0, 0.0, 3.0, 0.99), tracks=[other, self.anchor])
    self.assertFalse(any(s.active for s in statuses))
    self.assertEqual(set(self.shadow.evidence), {self.anchor, other})

  def test_yaw_compensation_matches_rotating_view(self):
    self.prime(distance=100.0, yaw=0.08)
    # Across 50ms, ego rotation moves the observed lateral point by .4m.
    status = self.step(distance=107.0, speed=28.0, std=12.0, lateral=0.4, yaw=0.08, tracks=[self.new, self.new])[0]
    self.assertTrue(status.active)

  def test_uncompensated_lateral_change_rejects_candidate(self):
    self.prime()
    self.assertFalse(self.jump(lateral=1.0)[0].active)

  def test_stale_baseline_does_not_qualify(self):
    self.prime()
    for _ in range(23):
      self.step(std=9.0)
    self.assertFalse(self.jump()[0].active)

  def test_immature_track_does_not_supply_anchor(self):
    for _ in range(30):
      self.step(tracks=[self.new, self.new])
    self.assertFalse(self.shadow.evidence)
    self.assertFalse(self.jump()[0].active)

  def test_loss_invalid_timing_or_yaw_removes_private_episode(self):
    for kwargs in [
      {'valid': False},
      {'yaw': None},
      {'yaw': math.nan},
      {'dt': 0.2},
      {'dt': -0.05},
      {'present': False},
      {'probability': 0.4},
      {'distance': math.nan},
    ]:
      with self.subTest(kwargs=kwargs):
        self.setUp()
        self.prime()
        self.jump()
        values = {'distance': 67.0, 'speed': 27.0, 'std': 12.0, 'tracks': [self.new, self.new]}
        values.update(kwargs)
        status = self.step(**values)
        self.assertFalse(any(s.active for s in status))
        self.assertFalse(self.jump()[0].active)

  def test_established_tracker_is_identical_with_observer(self):
    baseline, observed = VisionLeadTracker(), VisionLeadTracker()
    for frame in range(160):
      t = frame * 0.05
      d = 80.0 + 0.05 * math.sin(frame) if frame < 130 else 87.0 + 3.0 * math.sin(frame)
      std = 3.0 if frame < 125 else 12.0
      obs = [VisionLeadObservation(d, 0.0, std, 0.99)] * 2
      raw = [
        {
          'present': True,
          'radar': False,
          'dRel': d,
          'yRel': 0.0,
          'vLead': 30.0 if frame < 130 else 27.0,
          'vLeadK': 30.0,
          'vRel': 0.0,
          'aLeadK': 0.0,
          'modelProb': 0.99,
        }
        for _ in range(2)
      ]
      expected = baseline.update(obs, copy.deepcopy(raw), t, 30.0, yaw_rate=0.0)
      output = observed.update(obs, raw, t, 30.0, yaw_rate=0.0)
      before = copy.deepcopy(output)
      self.shadow.update(obs, raw, output, observed.slots, t, 30.0, 0.0)
      self.assertEqual(output, before)
      self.assertEqual(output, expected)
      for a, b in zip(baseline.slots, observed.slots, strict=True):
        self.assertEqual(a.age, b.age)
        self.assertEqual(a.x.tolist(), b.x.tolist())
        self.assertEqual(a.P.tolist(), b.P.tolist())
