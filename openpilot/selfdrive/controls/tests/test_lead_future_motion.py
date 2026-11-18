import copy
import math
from types import SimpleNamespace
import unittest

import numpy as np

from openpilot.selfdrive.controls.lib.lead_future_motion import FutureMotion, LeadFutureMotion
from openpilot.selfdrive.controls.lib.lead_transition import LeadTransitionRecovery
from openpilot.selfdrive.controls.lib.vision_lead_confidence import LeadUncertainty, VisionLeadConfidenceFilter
from openpilot.selfdrive.controls.lib.vision_lead_tracker import VisionLeadObservation, VisionLeadTracker, _DistanceTrack


def prediction(distance=80.0, speed=20.0, acceleration=0.0, std=1.0):
  times = [0.0, 2.0, 4.0, 6.0, 8.0, 10.0]
  return SimpleNamespace(t=times, x=[distance + 1.52 + speed * t + 0.5 * acceleration * t**2 for t in times],
                         xStd=[std] * 6, prob=0.99)


def lead(distance=80.0, speed=25.0, ego=25.0, acceleration=0.0):
  return {'present': True, 'radar': False, 'dRel': distance, 'vLead': speed, 'vLeadK': speed, 'vRel': speed - ego,
          'aLeadK': acceleration, 'aLeadTau': 0.3, 'modelProb': 0.99, 'yRel': 0.0, 'radarTrackId': -1}


def track(speed=20.0):
  result = _DistanceTrack(VisionLeadObservation(80.0, 0.0, 1.0, 0.99), 0.0, speed)
  result.age = 2.0
  result.P[1, 1] = 1.0
  result.mode = 'distance'
  result.continuity.continuous = True
  result.continuity.speed = speed
  return result


class TestFutureMotion(unittest.TestCase):
  def test_serialized_model_message_uses_capnp_list_readers(self):
    from openpilot.cereal import log

    message = log.ModelDataV2.new_message()
    sample = prediction(speed=25.0, acceleration=-2.0)
    model_lead = message.init('leadsV3', 2)[0]
    model_lead.t, model_lead.x, model_lead.xStd, model_lead.prob = sample.t, sample.x, sample.xStd, sample.prob
    with log.ModelDataV2.from_bytes(message.to_bytes()) as decoded:
      motion = FutureMotion.from_model(decoded.leadsV3[0])
      self.assertAlmostEqual(motion.speed, 25.0, places=5)
      self.assertAlmostEqual(motion.acceleration, -2.0, places=5)
      self.assertIsNone(FutureMotion.from_model(decoded.leadsV3[1]))

  def test_constant_speed_uses_world_motion_without_ego_addition(self):
    for speed in (0.0, 20.0, 40.0):
      motion = FutureMotion.from_model(prediction(speed=speed))
      self.assertAlmostEqual(motion.speed, speed)
      self.assertAlmostEqual(motion.acceleration, 0.0)

  def test_quadratic_recovers_current_speed_during_deceleration(self):
    motion = FutureMotion.from_model(prediction(speed=25.0, acceleration=-2.0))
    self.assertAlmostEqual(motion.speed, 25.0)
    self.assertAlmostEqual(motion.acceleration, -2.0)
    # The first two-second average would be 23 m/s, not the current speed.
    self.assertNotAlmostEqual(motion.speed, 23.0)

  def test_distance_origin_does_not_change_motion(self):
    a, b = (FutureMotion.from_model(prediction(distance=distance)) for distance in (10.0, 110.0))
    self.assertAlmostEqual(a.speed, b.speed)
    self.assertAlmostEqual(a.acceleration, b.acceleration)

  def test_bad_arrays_and_uncertain_predictions_are_rejected(self):
    for field, value in (('t', []), ('x', [1.0]), ('xStd', []), ('t', [0.0, 1.0, 2.0]),
                         ('x', [0.0, math.nan, 3.0]), ('xStd', [1.0, math.inf, 1.0]),
                         ('xStd', [0.0] * 3), ('prob', 0.8), ('prob', math.nan), ('x', [80.0, 70.0, 60.0])):
      with self.subTest(field=field, value=value):
        sample = prediction()
        setattr(sample, field, value)
        self.assertIsNone(FutureMotion.from_model(sample))
    self.assertIsNone(FutureMotion.from_model(prediction(std=10.0)))
    self.assertIsNone(FutureMotion.from_model(prediction(acceleration=5.0)))

  def test_uncertainty_does_not_assume_independent_future_points(self):
    motion = FutureMotion.from_model(prediction(std=2.0))
    self.assertEqual(motion.speed_std, 4.0)


class TestLeadFutureMotion(unittest.TestCase):
  def setUp(self):
    self.filter = LeadFutureMotion()
    self.track = track()
    self.timestamp = 0.0

  def step(self, speed=25.0, future_speed=20.0, measured_speed=20.0, distance=None, ego=25.0,
           acceleration=0.0, std=1.0, valid=True):
    self.timestamp += 0.05
    self.track.age += 0.05
    self.track.continuity.speed = measured_speed
    distance = 80.0 + (measured_speed - ego) * self.timestamp if distance is None else distance
    raw = [lead(distance, speed, ego, acceleration), {'present': False}]
    models = [prediction(distance, future_speed, std=std), prediction()]
    result = self.filter.update(models, raw, raw, [self.track, None], self.timestamp, ego, valid)
    return result, raw

  def warm(self, count=25, **kwargs):
    for _ in range(count):
      result, raw = self.step(**kwargs)
    return result, raw

  def test_speed_correction_is_bounded_and_does_not_mutate_inputs_or_kalman(self):
    state, covariance = self.track.x.copy(), self.track.P.copy()
    previous_delta = 0.0
    for _ in range(50):
      output, raw = self.step()
      delta = output[0]['vLead'] - raw[0]['vLead']
      self.assertGreaterEqual(delta, -2.0)
      self.assertLessEqual(delta, 0.0)
      self.assertLessEqual(abs(delta - previous_delta), 0.100001)
      self.assertEqual(raw[0]['vLead'], 25.0)
      self.assertEqual(output[0]['vLeadK'], output[0]['vLead'])
      self.assertAlmostEqual(output[0]['vRel'], output[0]['vLead'] - 25.0)
      for key in ('aLeadK', 'aLeadTau', 'present', 'radarTrackId'):
        self.assertEqual(output[0][key], raw[0][key])
      previous_delta = delta
    self.assertLess(previous_delta, -0.5)
    np.testing.assert_array_equal(self.track.x, state)
    np.testing.assert_array_equal(self.track.P, covariance)

  def test_wrong_future_slowdown_without_observed_motion_does_not_slow(self):
    output, raw = self.warm(measured_speed=25.0)
    self.assertEqual(output[0]['vLead'], raw[0]['vLead'])

  def test_optimistic_prediction_cannot_overrule_observed_closing(self):
    output, raw = self.warm(speed=20.0, future_speed=30.0, measured_speed=20.0)
    self.assertEqual(output[0]['vLead'], raw[0]['vLead'])
    self.assertLessEqual(output[0]['dRel'], raw[0]['dRel'])

  def test_supported_faster_motion_has_smaller_upward_bound(self):
    output, raw = self.warm(speed=20.0, future_speed=26.0, measured_speed=26.0)
    self.assertGreater(output[0]['vLead'], raw[0]['vLead'])
    self.assertLessEqual(output[0]['vLead'] - raw[0]['vLead'], 1.0)

  def test_preclosing_cue_cannot_be_undone(self):
    self.track.mode = 'pre-closing'
    output, raw = self.warm(speed=20.0, future_speed=26.0, measured_speed=26.0)
    self.assertEqual(output, raw)

  def test_range_prior_uses_ego_travel_and_is_not_future_gap(self):
    for _ in range(25):
      output, raw = self.step(speed=20.0, future_speed=20.0, measured_speed=20.0)
      self.assertAlmostEqual(output[0]['dRel'], raw[0]['dRel'], places=7)
    self.assertLess(output[0]['dRel'], 80.0)

  def test_range_jitter_is_reduced_with_a_bounded_correction(self):
    errors, raw_errors = [], []
    for i in range(100):
      distance = 80.0 + (0.2 if i % 2 else -0.2)
      output, raw = self.step(speed=25.0, future_speed=25.0, measured_speed=25.0, distance=distance)
      if i > 10:
        errors.append(output[0]['dRel'] - 80.0)
        raw_errors.append(raw[0]['dRel'] - 80.0)
      self.assertLessEqual(abs(output[0]['dRel'] - raw[0]['dRel']), 0.30)
    self.assertLess(np.std(errors), np.std(raw_errors))

  def test_cut_in_range_jump_is_immediate_and_reanchors(self):
    self.warm()
    output, raw = self.step(distance=55.0)
    self.assertEqual(output, raw)
    self.assertEqual(self.filter.status[0].reason, 'reanchor')

  def test_generation_change_discards_correction_even_with_mature_age(self):
    output, raw = self.warm()
    self.assertLess(output[0]['vLead'], raw[0]['vLead'])
    old_identity = self.filter.states[self.track].identity
    self.track.generation += 1
    output, raw = self.step()
    self.assertEqual(output, raw)
    self.assertEqual(self.filter.status[0].reason, 'reanchor')
    self.assertNotEqual(self.filter.states[self.track].identity, old_identity)

  def test_replaced_track_and_in_place_reset_discard_old_correction(self):
    self.warm()
    self.track = track()
    output, raw = self.step()
    self.assertEqual(output, raw)
    self.warm()
    self.track.age = 0.0
    output, raw = self.step()
    self.assertEqual(output, raw)

  def test_braking_stopped_lead_low_speed_and_ttc_passthrough(self):
    for changes in ({'acceleration': -1.0}, {'ego': 3.0}, {'speed': 0.0}, {'distance': 20.0, 'speed': 18.0}):
      with self.subTest(changes=changes):
        self.warm()
        output, raw = self.step(**changes)
        self.assertEqual(output, raw)
        self.assertEqual(self.filter.states, {})

  def test_ambiguous_tracking_prediction_loss_and_missing_lead_clear_state(self):
    self.warm()
    self.track.continuity.continuous = False
    output, raw = self.step()
    self.assertEqual(output, raw)
    self.assertEqual(self.filter.states, {})
    self.track.continuity.continuous = True
    self.warm()
    output, raw = self.step(std=10.0)
    self.assertEqual(output, raw)
    self.assertEqual(self.filter.states, {})
    self.warm()
    missing = [{'present': False}, {'present': False}]
    self.assertEqual(self.filter.update([prediction()] * 2, missing, missing, [self.track, None], 10.0, 25.0), missing)
    self.assertEqual(self.filter.states, {})

  def test_radar_and_invalid_inputs_passthrough(self):
    self.warm()
    raw = [lead(), {'present': False}]
    raw[0]['radar'] = True
    self.assertEqual(self.filter.update([prediction()] * 2, raw, raw, [self.track, None], 2.0, 25.0), raw)
    self.assertEqual(self.filter.states, {})
    for valid, timestamp, ego in ((False, 2.0, 25.0), (True, math.nan, 25.0), (True, 2.0, math.nan)):
      self.assertEqual(self.filter.update([prediction()] * 2, raw, raw, [self.track, None], timestamp, ego, valid), raw)

  def test_prediction_change_discards_old_speed_correction(self):
    self.warm()
    output, raw = self.step(future_speed=25.0)
    self.assertEqual(output, raw)

  def test_duplicate_slots_do_not_increase_weight_or_correction(self):
    duplicate = LeadFutureMotion()
    for _ in range(25):
      single, raw = self.step()
      models = [prediction(raw[0]['dRel'])] * 2
      duplicated = [copy.copy(raw[0]), copy.copy(raw[0])]
      output = duplicate.update(models, duplicated, duplicated, [self.track, self.track], self.timestamp, 25.0)
      self.assertEqual(single[0], output[0])
      self.assertEqual(output[0], output[1])
      self.assertEqual(self.filter.status[0].weight, duplicate.status[0].weight)

  def test_prediction_disagreement_rejects_shared_track(self):
    raw = [lead(), lead()]
    self.filter.update([prediction(speed=20.0), prediction(speed=30.0)], raw, raw, [self.track] * 2, 1.0, 25.0)
    self.assertEqual(self.filter.states, {})
    self.assertEqual(self.filter.status[0].reason, 'prediction_disagreement')

  def test_slot_swap_keeps_correction_with_vehicle(self):
    output, _ = self.warm()
    old_delta = output[0]['vLead'] - 25.0
    self.timestamp += 0.05
    self.track.age += 0.05
    distance = 80.0 - 5.0 * self.timestamp
    raw = [{'present': False}, lead(distance)]
    output = self.filter.update([prediction(), prediction(distance)], raw, raw, [None, self.track], self.timestamp, 25.0)
    self.assertEqual(output[0], raw[0])
    self.assertAlmostEqual(output[1]['vLead'] - 25.0, old_delta, places=5)
    self.assertEqual(self.filter.status[1].reason, 'active')

  def test_time_gap_reanchors_instead_of_extrapolating_stale_prediction(self):
    self.warm()
    self.timestamp += 0.5
    output, raw = self.step()
    self.assertEqual(output, raw)


class TestConfidenceIdentity(unittest.TestCase):
  def test_generation_change_reanchors_range_without_smoothing_new_target(self):
    confidence = VisionLeadConfidenceFilter()
    owner = track(speed=25.)
    uncertainty = LeadUncertainty(.99, 8., 1., 1.)
    for frame, distance in enumerate((80., 81.)):
      raw = [lead(distance=distance), {'present': False}]
      output = confidence.update(raw, raw, [uncertainty] * 2, [owner, None], frame * .05, 25.)
    self.assertEqual(confidence.status[0].reason, 'active')
    self.assertLess(output[0]['dRel'], raw[0]['dRel'])
    owner.generation += 1
    raw = [lead(distance=82.), {'present': False}]
    output = confidence.update(raw, raw, [uncertainty] * 2, [owner, None], .1, 25.)
    self.assertEqual(output, raw)
    self.assertEqual(confidence.status[0].reason, 'reanchor')
    self.assertEqual(confidence.states[0].identity, owner.identity)


class TestTrackerFutureIntegration(unittest.TestCase):
  def test_high_closing_speed_improves_early_estimate(self):
    tracker, future = VisionLeadTracker(), LeadFutureMotion()
    confidence, transition = VisionLeadConfidenceFilter(), LeadTransitionRecovery()
    ego, actual_speed = 33.0, 22.0
    baseline_errors, future_errors = [], []
    for i in range(80):
      timestamp = i * 0.05
      distance = 120.0 + (actual_speed - ego) * timestamp
      observations = [VisionLeadObservation(distance, 0.0, 1.0, 0.99)] * 2
      raw = [lead(distance, 32.0, ego)] * 2
      baseline = tracker.update(observations, raw, timestamp, ego, yaw_rate=0.0)
      uncertainty = [LeadUncertainty(0.99, 1.0, 1.0, 0.5)] * 2
      baseline = confidence.update(raw, baseline, uncertainty, tracker.slots, timestamp, ego)
      baseline = transition.update(observations, raw, baseline, [1.0] * 2, timestamp, ego, 0.0)
      output = future.update([prediction(distance, actual_speed)] * 2, raw, baseline, tracker.slots, timestamp, ego)
      if 1.1 < timestamp < 2.0:
        baseline_errors.append(abs(baseline[0]['vLead'] - actual_speed))
        future_errors.append(abs(output[0]['vLead'] - actual_speed))
      self.assertLessEqual(output[0]['vLead'], baseline[0]['vLead'])
      self.assertGreaterEqual(output[0]['vLead'], baseline[0]['vLead'] - 2.0)
    self.assertLess(np.mean(future_errors), np.mean(baseline_errors) - 0.1)

  def test_gradual_cut_in_lateral_motion_disables_prediction_correction(self):
    tracker, future = VisionLeadTracker(), LeadFutureMotion()
    for i in range(60):
      timestamp = i * 0.05
      distance = 100.0 - 5.0 * timestamp
      lateral = max(0.0, timestamp - 1.5)
      observations = [VisionLeadObservation(distance, lateral, 1.0, 0.99)] * 2
      raw = [lead(distance)] * 2
      baseline = tracker.update(observations, raw, timestamp, 25.0, yaw_rate=0.0)
      output = future.update([prediction(distance)] * 2, raw, baseline, tracker.slots, timestamp, 25.0)
      if timestamp > 2.0:
        self.assertEqual(output, baseline)


if __name__ == '__main__':
  unittest.main()
