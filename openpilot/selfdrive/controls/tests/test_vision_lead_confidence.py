import math
import unittest
from dataclasses import replace
from types import SimpleNamespace

import numpy as np

from openpilot.selfdrive.controls.lib.vision_lead_confidence import LeadUncertainty, VisionLeadConfidenceFilter


def fixture(timestamp=0.0, distance=50.0, acceleration=0.0, ego=25.0, track=None):
  if track is None:
    track = SimpleNamespace(age=10.0, ready=True, mode='distance', transition=1.0, uncertain_handoff_until=-math.inf,
                            x=np.array([distance, ego]), P=np.diag([1.0, 0.04]))
  lead = {'present': True, 'radar': False, 'dRel': distance, 'vLead': ego, 'vLeadK': ego, 'vRel': 0.0, 'aLeadK': acceleration,
          'aLeadTau': 0.3, 'modelProb': 0.99, 'yRel': 0.0, 'radarTrackId': -1}
  return {'raw_leads': [dict(lead), dict(lead)], 'leads': [dict(lead), dict(lead)],
          'uncertainties': [LeadUncertainty(0.99, 5.0, 2.0, 0.7)] * 2, 'tracks': [track, track],
          'timestamp': timestamp, 'ego': ego, 'valid': True}


class TestVisionLeadConfidence(unittest.TestCase):
  def warm(self):
    filt, track = VisionLeadConfidenceFilter(), None
    for i in range(40):
      args = fixture(i * 0.05, 50.0 + 0.35 * math.sin(i * 1.2), 0.10 * math.sin(i * 0.8), track=track)
      track = args['tracks'][0]
      filt.update(**args)
    self.assertTrue(all(status.reason == 'active' for status in filt.status))
    return filt, track

  def check_bypass(self, mutator):
    filt, track = self.warm()
    args = fixture(2.0, track=track)
    mutator(args)
    self.assertEqual(filt.update(**args), args['leads'])
    self.assertTrue(all(status.reason != 'active' for status in filt.status))

  def test_bounded_changes_and_unchanged_fields(self):
    filt, track, changed = VisionLeadConfidenceFilter(), None, 0
    for i in range(200):
      args = fixture(i * 0.05, 50.0 + 0.5 * math.sin(i * 0.8), 0.1 * math.sin(i * 0.7), track=track)
      track = args['tracks'][0]
      original = [dict(lead) for lead in args['leads']]
      output = filt.update(**args)
      self.assertEqual(args['leads'], original)
      for lead, baseline in zip(output, original, strict=True):
        self.assertLessEqual(abs(lead['dRel'] - baseline['dRel']), 0.750000001)
        self.assertLessEqual(abs(lead['aLeadK'] - baseline['aLeadK']), 0.080000001)
        for key in baseline.keys() - {'dRel', 'aLeadK'}:
          self.assertEqual(lead[key], baseline[key])
      changed += output != original
    self.assertGreater(changed, 100)

  def test_braking_bypasses_immediately(self):
    for acceleration in (-2.0, -0.2):
      with self.subTest(acceleration=acceleration):
        self.check_bypass(lambda args, accel=acceleration: [lead.update(aLeadK=accel) for lead in args['raw_leads'] + args['leads']])

  def test_rapid_closing(self):
    self.check_bypass(lambda args: [lead.update(vLead=15.0, vRel=-10.0) for lead in args['raw_leads'] + args['leads']])

  def test_short_gap(self):
    self.check_bypass(lambda args: [lead.update(dRel=20.0) for lead in args['raw_leads'] + args['leads']])

  def test_new_lead(self):
    self.check_bypass(lambda args: setattr(args['tracks'][0], 'age', 0.0))

  def test_identity_change(self):
    def change(args):
      args['tracks'] = [SimpleNamespace(**vars(args['tracks'][0]))] * 2
    self.check_bypass(change)

  def test_minimal_missing_lead(self):
    self.check_bypass(lambda args: args.update(raw_leads=[{'present': False}] * 2, leads=[{'present': False}] * 2))

  def test_missing_slot_does_not_clear_other(self):
    filt, track = self.warm()
    args = fixture(2.0, track=track)
    args['raw_leads'][0] = args['leads'][0] = {'present': False}
    output = filt.update(**args)
    self.assertEqual(output[0], {'present': False})
    self.assertIsNone(filt.states[0])
    self.assertEqual(filt.status[1].reason, 'active')

  def test_radar_bypass(self):
    self.check_bypass(lambda args: [lead.update(radar=True) for lead in args['raw_leads'] + args['leads']])

  def test_invalid_clears_state(self):
    filt, track = self.warm()
    args = fixture(2.0, track=track)
    self.assertEqual(filt.update(**dict(args, valid=False)), args['leads'])
    self.assertEqual(filt.states, [None, None])
    self.assertEqual(filt.update(**dict(args, timestamp=2.05)), args['leads'])
    self.assertTrue(all(status.reason == 'reanchor' for status in filt.status))

  def test_nonfinite_uncertainty(self):
    for field in ('probability', 'distance_std', 'speed_std', 'acceleration_std'):
      with self.subTest(field=field):
        self.check_bypass(lambda args, key=field: args.update(uncertainties=[replace(u, **{key: math.nan}) for u in args['uncertainties']]))

  def test_nonfinite_raw_acceleration(self):
    self.check_bypass(lambda args: [lead.update(aLeadK=math.nan) for lead in args['raw_leads']])

  def test_invalid_uncertainty_range(self):
    for uncertainty in (LeadUncertainty(0.4, 5.0, 2.0, 0.7), LeadUncertainty(0.99, 0.0, 2.0, 0.7), LeadUncertainty(0.99, 5.0, 2.0, 0.0)):
      with self.subTest(uncertainty=uncertainty):
        self.check_bypass(lambda args, value=uncertainty: args.update(uncertainties=[value] * 2))

  def test_time_gap(self):
    self.check_bypass(lambda args: args.update(timestamp=4.0))

  def test_low_speed(self):
    self.check_bypass(lambda args: args.update(ego=8.0))

  def test_range_jump(self):
    self.check_bypass(lambda args: [lead.update(dRel=45.0) for lead in args['raw_leads'] + args['leads']])

  def test_acceleration_jump(self):
    self.check_bypass(lambda args: [lead.update(aLeadK=0.5) for lead in args['raw_leads'] + args['leads']])

  def test_preclosing_and_transition_bypass(self):
    self.check_bypass(lambda args: setattr(args['tracks'][0], 'mode', 'pre-closing'))
    self.check_bypass(lambda args: setattr(args['tracks'][0], 'transition', 0.5))
    self.check_bypass(lambda args: setattr(args['tracks'][0], 'uncertain_handoff_until', 3.0))

  def test_guard_on_shared_secondary(self):
    self.check_bypass(lambda args: args['raw_leads'][1].update(aLeadK=-1.0))

  def test_uncertainty_changes_response(self):
    movements = []
    for std in (1.0, 10.0):
      filt, track = VisionLeadConfidenceFilter(), None
      for i, distance in enumerate((50.0, 50.5)):
        args = fixture(i * 0.05, distance, track=track)
        track = args['tracks'][0]
        args['uncertainties'] = [replace(u, distance_std=std) for u in args['uncertainties']]
        output = filt.update(**args)
      movements.append(output[0]['dRel'] - 50.0)
    self.assertGreater(movements[0], movements[1])

  def test_reset_discards_offsets(self):
    filt, track = self.warm()
    filt.reset()
    args = fixture(2.0, track=track)
    self.assertEqual(filt.update(**args), args['leads'])

  def test_incomplete_arrays_bypass_and_reset(self):
    filt, track = self.warm()
    args = fixture(2.0, track=track)
    args['uncertainties'] = []
    self.assertEqual(filt.update(**args), args['leads'])
    self.assertEqual(filt.states, [None, None])


if __name__ == '__main__':
  unittest.main()
