import unittest
from types import SimpleNamespace

from openpilot.cereal import log
from openpilot.selfdrive.controls.radard import RadarD, get_RadarState_from_vision


class FakeSubMaster(dict):
  def __init__(self):
    super().__init__()
    model = log.ModelDataV2.new_message()
    model.velocity.x = [25.0]
    model.init('leadsV3', 2)
    for i, lead in enumerate(model.leadsV3):
      lead.prob = 1.0
      lead.x = [41.52 + 20.0 * i]
      lead.y = [0.0]
      lead.v = [24.25]
      lead.a = [0.0]
      lead.xStd = [2.0]
      lead.yStd = [0.5]
      lead.vStd = [1.0]
    self['modelV2'] = model
    self['carState'] = SimpleNamespace(vEgo=25.0)
    self.seen = {'modelV2': True}
    self.recv_frame = {'carState': 0}
    self.logMonoTime = {'modelV2': 1_000_000_000}
    self.valid = True

  def all_checks(self):
    return self.valid

  def advance(self):
    self.recv_frame['carState'] += 1
    self.logMonoTime['modelV2'] += 50_000_000


class TestRadarDPrelead(unittest.TestCase):
  def setUp(self):
    self.rd, self.sm = RadarD(), FakeSubMaster()
    self.radar = SimpleNamespace(points=[], errors={})

  def step(self, count=1):
    for _ in range(count):
      self.sm.advance()
      self.rd.update(self.sm, self.radar)

  def assert_original(self, slot=0):
    original = get_RadarState_from_vision(
      self.sm['modelV2'].leadsV3[slot], self.sm['carState'].vEgo, self.sm['modelV2'].velocity.x[0], self.rd.lead_prob_filters[slot].x
    )
    actual = getattr(self.rd.radar_state, ('leadOne', 'leadTwo')[slot])
    for key, value in original.items():
      if isinstance(value, float):
        self.assertAlmostEqual(getattr(actual, key), value, places=4)
      else:
        self.assertEqual(getattr(actual, key), value)

  def test_both_slots_feed_speed_fields_used_by_mpc(self):
    self.step(240)
    for field in ('leadOne', 'leadTwo'):
      lead = getattr(self.rd.radar_state, field)
      self.assertAlmostEqual(lead.vLead, 25.0, delta=0.03)
      self.assertAlmostEqual(lead.vLeadK, 25.0, delta=0.03)
      self.assertAlmostEqual(lead.vRel, 0.0, delta=0.03)
      self.assertEqual(lead.aLeadK, 0.0)
      self.assertFalse(lead.radar)
    self.assertAlmostEqual(self.rd.radar_state.leadOne.dRel, 40.0, places=4)
    self.assertEqual(self.rd.radar_state.mdMonoTime, self.sm.logMonoTime['modelV2'])

  def test_private_history_does_not_lower_acceptance_threshold(self):
    for lead in self.sm['modelV2'].leadsV3:
      lead.prob = 0.4
    self.step(240)
    self.assertFalse(self.rd.radar_state.leadOne.present)
    self.assertFalse(self.rd.radar_state.leadTwo.present)
    self.assertTrue(all(t.ready for t in self.rd.vision_lead_tracker.slots))
    for lead in self.sm['modelV2'].leadsV3:
      lead.prob = 0.5
    self.step()
    self.assertFalse(self.rd.radar_state.leadOne.present)
    for lead in self.sm['modelV2'].leadsV3:
      lead.prob = 0.6
    self.step()
    self.assertTrue(self.rd.radar_state.leadOne.present)
    self.assertAlmostEqual(self.rd.radar_state.leadOne.vLead, 25.0, delta=0.03)

  def test_missing_one_observation_does_not_clear_other(self):
    self.step(240)
    other = self.rd.vision_lead_tracker.slots[1]
    self.sm['modelV2'].leadsV3[0].prob = 0.0
    self.step()
    self.assertIsNone(self.rd.vision_lead_tracker.slots[0])
    self.assertIs(self.rd.vision_lead_tracker.slots[1], other)
    self.assert_original(0)

  def test_moderate_confidence_drop_keeps_ready_track(self):
    self.step(240)
    track = self.rd.vision_lead_tracker.slots[0]
    self.sm['modelV2'].leadsV3[0].prob = 0.7
    self.step()
    self.assertIs(self.rd.vision_lead_tracker.slots[0], track)
    self.assertAlmostEqual(self.rd.radar_state.leadOne.vLead, 25.0, delta=0.03)

  def test_invalid_message_resets_history(self):
    self.step(240)
    self.sm.valid = False
    self.step()
    self.assertFalse(self.rd.radar_state_valid)
    self.assertEqual(self.rd.vision_lead_tracker.tracks, [])
    self.assert_original(0)
    self.assert_original(1)

  def test_missing_lead_array_resets(self):
    self.step(240)
    self.sm['modelV2'].leadsV3 = []
    self.step()
    self.assertFalse(self.rd.radar_state.leadOne.present)
    self.assertFalse(self.rd.radar_state.leadTwo.present)
    self.assertEqual(self.rd.vision_lead_tracker.tracks, [])

  def test_radar_track_is_unchanged(self):
    self.step(240)
    self.radar.points = [SimpleNamespace(trackId=7, dRel=40.0, yRel=0.0, vRel=-0.75)]
    self.step()
    self.assertTrue(self.rd.radar_state.leadOne.radar)
    self.assertEqual(self.rd.radar_state.leadOne.radarTrackId, 7)
    self.assertAlmostEqual(self.rd.radar_state.leadOne.vLead, 24.25)

  def test_braking_is_immediate(self):
    self.step(240)
    self.sm['modelV2'].leadsV3[0].a = [-1.0]
    self.step()
    self.assertLessEqual(self.rd.radar_state.leadOne.vLead, 24.25)
    self.assertEqual(self.rd.radar_state.leadOne.aLeadK, -1.0)
    self.assertTrue(self.rd.vision_lead_tracker.slots[0].ready)

  def test_missing_uncertainty_uses_original(self):
    self.step(240)
    self.sm['modelV2'].leadsV3[0].xStd = []
    self.step()
    self.assert_original()

  def test_same_car_hypotheses_share_speed(self):
    self.sm['modelV2'].leadsV3[1].x = [42.0]
    self.sm['modelV2'].leadsV3[1].v = [26.0]
    self.step(240)
    self.assertEqual(self.rd.radar_state.leadOne.vLead, self.rd.radar_state.leadTwo.vLead)
    self.assertNotEqual(self.rd.radar_state.leadOne.dRel, self.rd.radar_state.leadTwo.dRel)

  def test_alignment_only_affects_fallback_and_guards(self):
    self.sm['modelV2'].velocity.x = [23.0]
    self.step()
    self.assert_original()
    self.step(240)
    self.assertAlmostEqual(self.rd.radar_state.leadOne.vLead, 25.0, delta=0.03)
    self.sm['modelV2'].velocity.x = [26.0]
    self.step()
    self.assertAlmostEqual(self.rd.radar_state.leadOne.vLead, 25.0, delta=0.03)


if __name__ == '__main__':
  unittest.main()
