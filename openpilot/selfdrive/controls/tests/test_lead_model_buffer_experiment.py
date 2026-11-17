import unittest
from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.lead_model_buffer_experiment import LeadModelBufferExperiment
from openpilot.selfdrive.controls.tests.test_lead_motion_fusion import sample


class TestModelBufferExperiment(unittest.TestCase):
  def setUp(self):
    self.buffer = LeadModelBufferExperiment()
    self.time = -.05

  def step(self, distance=140., model_speed=35., derived=30., mode='distance', ready=True, std=1., ego=33.):
    self.time += .05
    model, obs, raw = sample(distance, speed=model_speed, ego=ego)
    model.vStd = [std] * 6
    baseline = dict(raw, vLead=derived, vLeadK=derived, vRel=derived - ego)
    track = SimpleNamespace(ready=ready, mode=mode, age=10. + self.time)
    out = self.buffer.update([model, model], [obs, obs], [raw, raw], [baseline, baseline],
                             self.time, ego, 0., tracks=[track, track])
    for lead in out:
      for key in ('present', 'dRel', 'yRel', 'aLeadK', 'aLeadTau'):
        self.assertEqual(lead[key], baseline[key])
    return baseline, out

  def test_weak_anchor_does_not_accumulate_to_model_velocity(self):
    for _ in range(200):
      baseline, out = self.step()
    self.assertAlmostEqual(out[0]['vLead'], 30.5)
    self.assertEqual(baseline['vLead'], 30.)

  def test_close_range_returns_existing_pipeline_exactly(self):
    for _ in range(100):
      self.step()
    for d in (80., 60., 30.):
      baseline, out = self.step(distance=d)
      self.assertEqual(out, [baseline, baseline])

  def test_braking_and_acquisition_cannot_be_overridden(self):
    for kwargs in ({'mode': 'braking'}, {'mode': 'pre-closing'}, {'mode': 'stop-hold'}, {'ready': False}):
      self.setUp()
      for _ in range(100):
        self.step()
      baseline, out = self.step(**kwargs)
      self.assertEqual(out, [baseline, baseline])

  def test_short_closing_time_does_not_raise_velocity(self):
    for _ in range(100):
      self.step()
    baseline, out = self.step(distance=100., model_speed=30., derived=20., ego=40.)
    self.assertEqual(out, [baseline, baseline])

  def test_uncertain_model_cannot_gain_more_weight(self):
    offsets = []
    for std in (1., 3., 5.):
      self.setUp()
      for _ in range(100):
        baseline, out = self.step(std=std)
      offsets.append(out[0]['vLead'] - baseline['vLead'])
    self.assertGreater(offsets[0], offsets[1])
    self.assertGreater(offsets[1], offsets[2])
    self.assertEqual(offsets[2], 0.)

  def test_missing_track_and_empty_lane_clear_buffer(self):
    for _ in range(100):
      self.step()
    model, obs, _ = sample(140.)
    absent = [{'present': False}] * 2
    out = self.buffer.update([model, model], [obs, obs], absent, absent, self.time + .05, 33., 0., tracks=[None, None])
    self.assertEqual(out, absent)
    self.assertEqual(self.buffer.offsets, [0., 0.])
