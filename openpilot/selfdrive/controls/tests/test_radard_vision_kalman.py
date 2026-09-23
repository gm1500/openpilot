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
      lead.x = [41.52 + 20.0*i]
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


class TestRadarDVisionKalman(unittest.TestCase):
  def setUp(self):
    self.rd = RadarD()
    self.sm = FakeSubMaster()
    self.radar = SimpleNamespace(points=[], errors={})

  def step(self, count=1):
    for _ in range(count):
      self.sm.advance()
      self.rd.update(self.sm, self.radar)

  def assert_original(self, slot=0):
    lead = self.sm['modelV2'].leadsV3[slot]
    original = get_RadarState_from_vision(lead, 25.0, 25.0, self.rd.lead_prob_filters[slot].x)
    actual = getattr(self.rd.radar_state, ('leadOne', 'leadTwo')[slot])
    for key, value in original.items():
      if isinstance(value, float):
        self.assertAlmostEqual(getattr(actual, key), value, places=5)
      else:
        self.assertEqual(getattr(actual, key), value)

  def test_both_lead_slots_update_fields_consumed_by_mpc(self):
    self.step(240)
    for field in ('leadOne', 'leadTwo'):
      lead = getattr(self.rd.radar_state, field)
      self.assertAlmostEqual(lead.vLead, 25.0, delta=0.03)
      self.assertAlmostEqual(lead.vLeadK, lead.vLead, places=5)
      self.assertAlmostEqual(lead.vRel, 0.0, delta=0.03)
      self.assertEqual(lead.aLeadK, 0.0)
      self.assertFalse(lead.radar)
    self.assertAlmostEqual(self.rd.radar_state.leadOne.dRel, 40.0, places=4)
    self.assertEqual(self.rd.radar_state.mdMonoTime, self.sm.logMonoTime['modelV2'])

  def test_one_lead_lost_does_not_clear_the_other(self):
    self.step(240)
    self.sm['modelV2'].leadsV3[0].prob = 0.0
    self.step()
    self.assertIsNone(self.rd.vision_lead_filters[0].x)
    self.assertTrue(self.rd.vision_lead_filters[1].active)
    self.assert_original(0)

  def test_raw_confidence_loss_falls_back_before_held_probability(self):
    self.step(240)
    self.sm['modelV2'].leadsV3[0].prob = 0.9
    self.step()
    self.assertGreater(self.rd.lead_prob_filters[0].x, 0.95)
    self.assertIsNone(self.rd.vision_lead_filters[0].x)
    self.assert_original(0)

  def test_invalid_messages_reset_both_slots(self):
    self.step(240)
    self.sm.valid = False
    self.step()
    self.assertFalse(self.rd.radar_state_valid)
    for slot in (0, 1):
      self.assertIsNone(self.rd.vision_lead_filters[slot].x)
      self.assert_original(slot)

  def test_missing_model_leads_reset_both_slots(self):
    self.step(240)
    self.sm['modelV2'].leadsV3 = []
    self.step()
    self.assertFalse(self.rd.radar_state.leadOne.present)
    self.assertFalse(self.rd.radar_state.leadTwo.present)
    self.assertTrue(all(f.x is None for f in self.rd.vision_lead_filters))

  def test_radar_track_bypasses_vision_estimate(self):
    self.step(240)
    self.radar.points = [SimpleNamespace(trackId=7, dRel=40.0, yRel=0.0, vRel=-0.75)]
    self.step()
    self.assertTrue(self.rd.radar_state.leadOne.radar)
    self.assertEqual(self.rd.radar_state.leadOne.radarTrackId, 7)
    self.assertAlmostEqual(self.rd.radar_state.leadOne.vLead, 24.25)
    self.assertIsNone(self.rd.vision_lead_filters[0].x)

  def test_model_braking_is_immediate(self):
    self.step(240)
    self.sm['modelV2'].leadsV3[0].a = [-0.3]
    self.step()
    self.assert_original(0)
    self.assertIsNone(self.rd.vision_lead_filters[0].x)

  def test_missing_distance_uncertainty_uses_original_lead(self):
    self.step(240)
    self.sm['modelV2'].leadsV3[0].xStd = []
    self.step()
    self.assert_original(0)
    self.assertIsNone(self.rd.vision_lead_filters[0].x)


if __name__ == '__main__':
  unittest.main()
