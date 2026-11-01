import json
from pathlib import Path
import unittest

from openpilot.selfdrive.controls.lib.lead_transition import LeadTransitionRecovery
from openpilot.selfdrive.controls.lib.vision_lead_confidence import LeadUncertainty, VisionLeadConfidenceFilter
from openpilot.selfdrive.controls.lib.vision_lead_tracker import VisionLeadObservation, VisionLeadTracker


class TestHighwayClosing(unittest.TestCase):
  def test_recorded_approach_keeps_closing_history(self):
    fixture = json.loads((Path(__file__).parent / 'fixtures' / 'highway_closing.json').read_text())
    tracker, confidence, recovery = VisionLeadTracker(), VisionLeadConfidenceFilter(), LeadTransitionRecovery()
    ready_at = None
    rows = []
    for timestamp, ego, valid, yaw, samples in fixture['frames']:
      observations, raw, uncertainties = [], [], []
      for distance, lateral, distance_std, prob, speed, accel, speed_std, accel_std, filtered_prob, present, _ in samples:
        observations.append(VisionLeadObservation(distance, lateral, distance_std, prob))
        uncertainties.append(LeadUncertainty(prob, distance_std, speed_std, accel_std))
        raw.append({'present': True, 'radar': False, 'dRel': distance, 'yRel': lateral, 'vLead': speed, 'vLeadK': speed,
                    'vRel': speed - ego, 'aLeadK': accel, 'aLeadTau': .3, 'modelProb': filtered_prob, 'radarTrackId': -1}
                   if present else {'present': False})
      leads = tracker.update(observations, raw, timestamp, ego, valid)
      leads = confidence.update(raw, leads, uncertainties, tracker.slots, timestamp, ego, valid)
      leads = recovery.update(observations, raw, leads, [u.speed_std for u in uncertainties], timestamp, ego, yaw, valid)
      self.assertEqual(recovery.correction, 0.)
      for original, lead in zip(raw, leads, strict=True):
        # This change cannot accept a missing lead or alter its acceleration.
        self.assertEqual(original['present'], lead['present'])
        if original['present']:
          self.assertEqual(original['aLeadK'], lead['aLeadK'])
      if 53.85 < timestamp < 57.23:
        self.assertGreater(tracker.slots[0].age, .04)
        if tracker.slots[0].ready and ready_at is None:
          ready_at = timestamp
      if leads[0]['present']:
        if 53.05 < timestamp < 55.45:
          self.assertAlmostEqual(leads[0]['vLead'], samples[0][-1], places=3)
        rows.append((timestamp, leads[0]['vLead'], samples[0][-1]))
    self.assertIsNotNone(ready_at)
    self.assertLess(ready_at, 56.1)  # previously 57.50, after the driver's brake input
    approaching = [(speed, logged) for t, speed, logged in rows if 56.65 < t < 57.20]
    self.assertTrue(approaching)
    for speed, logged in approaching:
      self.assertLess(speed * 3.6, 85.)
      self.assertGreater((logged - speed) * 3.6, 15.)


if __name__ == '__main__':
  unittest.main()
