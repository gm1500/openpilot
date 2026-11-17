import unittest
from unittest.mock import patch

from openpilot.cereal import log
from opendbc.car.structs import car
from openpilot.selfdrive.controls.radard import RadarD


class ModelInputs(dict):
  """The SubMaster inputs consumed by RadarD.update, without running IPC."""
  def __init__(self, model, frame, ego=33., valid=True):
    super().__init__(modelV2=model, carState=car.CarState.new_message(vEgo=ego))
    self.seen = {'modelV2': True}
    self.recv_frame = {'carState': frame}
    self.logMonoTime = {'modelV2': int((1. + frame * .05) * 1e9)}
    self.valid = valid

  def all_checks(self):
    return self.valid


def closing_model(frame, count=3, ego=33.):
  t = frame * .05
  braking_time = max(0., t - 2.)
  distance = 140. - 3. * t - .7 * braking_time**2
  speed = 30. - 1.4 * braking_time
  model = log.ModelDataV2.new_message()
  model.velocity.x = [ego]
  model.orientationRate.z = [0.]
  for i, lead in enumerate(model.init('leadsV3', count)):
    times = [0., 2., 4., 6., 8., 10.]
    lead.t = times
    lead.prob = .99
    lead.x = [distance + 1.52 + speed * dt - .7 * dt**2 for dt in times]
    lead.xStd = [2.] * 6
    lead.y, lead.yStd = [0.] * 6, [1.] * 6
    lead.v, lead.vStd = [32.] * 6, [1.] * 6
    lead.a, lead.aStd = [0.] * 6, [1.] * 6
    if i == 2:
      # This extra prediction is not one of radard's two accepted lead slots.
      lead.x, lead.y, lead.v = [10.] * 6, [3.] * 6, [0.] * 6
  return model


class TestRadarDLeadMotion(unittest.TestCase):
  def setUp(self):
    self.logging = patch('openpilot.selfdrive.controls.radard.cloudlog')
    self.logging.start()
    self.addCleanup(self.logging.stop)
    self.radar = car.RadarData.new_message()

  def update(self, rd, model, frame, **kwargs):
    # Exercise the serialized model shape received by the real caller.
    payload = model.to_bytes()
    model.clear_write_flag()
    with log.ModelDataV2.from_bytes(payload) as decoded:
      rd.update(ModelInputs(decoded, frame, **kwargs), self.radar)
    return [getattr(rd.radar_state, field).to_dict() for field in ('leadOne', 'leadTwo')]

  def warm(self, rd, count=3):
    peak = 0.
    for frame in range(180):
      self.update(rd, closing_model(frame, count), frame)
      self.assertIsNotNone(rd.lead_responsive_motion)
      peak = max(peak, -rd.lead_responsive_motion.delta)
    return peak

  def test_three_model_predictions_reach_responsive_filter(self):
    two, three, baseline = RadarD(), RadarD(), RadarD()
    baseline.lead_responsive_motion = None
    peak = 0.
    for frame in range(180):
      model = closing_model(frame)
      actual = self.update(three, model, frame)
      expected = self.update(two, closing_model(frame, 2), frame)
      prior = self.update(baseline, model, frame)
      self.assertEqual(actual, expected)
      self.assertIsNotNone(three.lead_responsive_motion)
      self.assertNotEqual(three.lead_responsive_motion.reason, 'unsupported')
      for before, after in zip(prior, actual, strict=True):
        for field in ('present', 'dRel', 'yRel', 'aLeadK', 'aLeadTau'):
          self.assertEqual(before[field], after[field])
        self.assertLessEqual(after['vLead'], before['vLead'])
        peak = max(peak, before['vLead'] - after['vLead'])
    self.assertGreater(peak, .1)
    self.assertGreater(three.lead_responsive_motion.motion.forecast.updates, 3)

  def test_low_speed_and_invalid_messages_revoke_active_correction(self):
    for ego, valid in ((14., True), (33., False)):
      with self.subTest(ego=ego, valid=valid):
        rd = RadarD()
        self.assertGreater(self.warm(rd), .1)
        self.update(rd, closing_model(180, ego=ego), 180, ego=ego, valid=valid)
        self.assertEqual(rd.lead_responsive_motion.reason, 'unsupported')
        self.assertEqual(rd.lead_responsive_motion.delta, 0.)
        self.assertIsNone(rd.lead_responsive_motion.motion.kalman)

  def test_distinct_lead_and_missing_model_revoke_active_correction(self):
    for change in ('distinct', 'missing'):
      with self.subTest(change=change):
        rd = RadarD()
        self.assertGreater(self.warm(rd), .1)
        model = closing_model(180, count=1 if change == 'missing' else 3)
        if change == 'distinct':
          model.leadsV3[1].y = [3.] * 6
        self.update(rd, model, 180)
        self.assertEqual(rd.lead_responsive_motion.delta, 0.)
        self.assertIsNone(rd.lead_responsive_motion.motion.kalman)
        if change == 'distinct':
          self.assertEqual(rd.lead_responsive_motion.reason, 'distinct_or_missing')


if __name__ == '__main__':
  unittest.main()
