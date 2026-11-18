"""Recorded departure/lead-exit comparisons for the read-only observer.

The fixture stores only scalar observations, with current model geometry in FRD
and relative time. Yaw is derived from deviceMotion with measured calibration,
not the carControl cache or model orientation prediction. Events are discovery/regression cases, not an independent
validation set or evidence that a detected hypothesis can be discarded.
"""
from copy import deepcopy
from dataclasses import asdict
import json
from pathlib import Path
import unittest

from openpilot.selfdrive.controls.lib.lead_departure_shadow import DepartureHead, LeadDepartureShadow

FIXTURE = Path(__file__).parent / 'fixtures' / 'lead_departure_cases.json'
STATES = ('off', 'preLaneChange', 'laneChangeStarting', 'laneChangeFinishing')
SIDES = {-1: 'left', 0: 'none', 1: 'right'}


def load_cases():
  return json.loads(FIXTURE.read_text())['cases']


def replay(case, mirror=False, swap_heads=False, limit=None):
  observer = LeadDepartureShadow()
  results = []
  for row in case['frames'][:limit]:
    t, delay, ego, yaw, valid, state, side, raw, cs_offset, motion_offset = row
    heads = [DepartureHead(*head) for head in raw]
    if mirror:
      heads = [DepartureHead(h.x, -h.y, h.speed, h.probability, h.x_std, h.y_std) for h in heads]
      yaw, side = -yaw, -side
    if swap_heads:
      heads.reverse()
    before = deepcopy(heads)
    returned = observer.update(heads, t, ego, yaw, STATES[state], SIDES[side], bool(valid))
    if heads != before or returned is not None or observer.status.can_ignore_lead:
      raise AssertionError('A diagnostic observer must not mutate lead observations or request suppression')
    results.append((t + delay, asdict(observer.status)))
  return results


class TestRecordedLeadDeparture(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.cases = load_cases()

  def test_fixture_has_complete_causal_observations_and_no_location_data(self):
    self.assertEqual(len(self.cases), 11)
    self.assertEqual(sum(len(case['frames']) for case in self.cases), 2073)
    for case in self.cases:
      with self.subTest(case=case['name']):
        self.assertEqual(len(case['source_sha256']), 64)
        self.assertEqual(case['frames'][0][0], 0.)
        previous = -1.
        for t, delay, ego, yaw, valid, state, side, heads, cs_offset, motion_offset in case['frames']:
          self.assertGreater(t, previous)
          self.assertLessEqual(t - previous, .101 if previous >= 0 else 1.1)
          self.assertGreaterEqual(delay, 0.)
          self.assertLessEqual(delay, .1)
          self.assertTrue(-.1 <= cs_offset <= 0.)
          self.assertTrue(-.1 <= motion_offset <= 0.)
          self.assertGreater(ego, 0.)
          self.assertLess(abs(yaw), 1.)
          self.assertIn(valid, (0, 1))
          self.assertIn(state, range(4))
          self.assertIn(side, SIDES)
          self.assertEqual([len(head) for head in heads], [6, 6])
          previous = t
    text = FIXTURE.read_text()
    for private_key in ('latitude', 'longitude', 'gpsLocation', 'dongleId', 'routeName'):
      self.assertNotIn(private_key, text)

  def test_all_four_departures_report_ambiguity_before_recorded_brake(self):
    cases = [case for case in self.cases if case['expected']['ambiguous_split_expected']]
    self.assertEqual(len(cases), 4)
    for case in cases:
      with self.subTest(case=case['name']):
        results = replay(case)
        active = [(t, status) for t, status in results if status['active']]
        self.assertTrue(active)
        # Availability is included: a retrospectively matched model timestamp
        # alone must not count as early detection.
        self.assertLess(active[0][0], case['expected']['friction_onset'])
        self.assertTrue(all(status['reason'] == 'split_ambiguous' for _, status in active))
        self.assertFalse(results[-1][1]['active'])
        self.assertTrue(all(not status['can_ignore_lead'] for _, status in results))

  def test_clean_exits_and_real_cutin_comparisons_do_not_report_ego_departure(self):
    cases = [case for case in self.cases if not case['expected']['ambiguous_split_expected']]
    self.assertEqual(len(cases), 7)
    for case in cases:
      with self.subTest(case=case['name']):
        results = replay(case)
        self.assertFalse(any(status['active'] for _, status in results))
        self.assertTrue(all(not status['can_ignore_lead'] for _, status in results))
    # The two intervention-limited comparisons stop before the driver brakes;
    # subsequent driver behavior cannot be attributed to this observer.
    limited = {c['name'] for c in cases if c['expected'].get('ends_before_driver_brake')}
    self.assertEqual(limited, {'real_semi_cut_in', 'slower_lead_approach'})

  def test_left_right_mirror_preserves_ambiguity_without_head_identity(self):
    for case in self.cases:
      with self.subTest(case=case['name']):
        baseline = replay(case)
        mirrored = replay(case, mirror=True)
        swapped = replay(case, swap_heads=True)
        self.assertEqual([s['active'] for _, s in baseline], [s['active'] for _, s in mirrored])
        self.assertEqual([s['active'] for _, s in baseline], [s['active'] for _, s in swapped])
        for (_, before), (_, after) in zip(baseline, swapped, strict=True):
          if before['active']:
            self.assertEqual(before['near_head'], 1 - after['near_head'])

  def test_future_observations_cannot_change_prefix_results(self):
    for case in self.cases:
      with self.subTest(case=case['name']):
        complete = replay(case)
        for length in (len(complete) // 2, len(complete) - 1):
          self.assertEqual(replay(case, limit=length), complete[:length])


if __name__ == '__main__':
  unittest.main()
