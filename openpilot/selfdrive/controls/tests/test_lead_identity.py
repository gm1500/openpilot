"""Vehicle identity must own history without delaying a newly observed hazard."""

import copy
import math
import unittest

from openpilot.selfdrive.controls.lib.vision_lead_tracker import VisionLeadObservation, VisionLeadTracker


def observed_lead(distance=75., lateral=0., speed=30., ego=30., acceleration=0.):
  return {'present': True, 'radar': False, 'dRel': distance, 'yRel': lateral,
          'vLead': speed, 'vLeadK': speed, 'vRel': speed - ego, 'aLeadK': acceleration,
          'aLeadTau': .3, 'modelProb': .99, 'radarTrackId': -1}


class TestLeadIdentity(unittest.TestCase):
  def setUp(self):
    self.tracker = VisionLeadTracker()
    self.timestamp = 1.
    self.ego = 30.

  def step(self, samples=((75., 0., 30.), (75., 0., 30.)), yaw=0., acceleration=0., valid=True):
    self.timestamp += .05
    observations, raw = [], []
    for sample in samples:
      if sample is None:
        observations.append(VisionLeadObservation(0., 0., 1., 0.))
        raw.append({'present': False})
      else:
        distance, lateral, speed = sample
        observations.append(VisionLeadObservation(distance, lateral, 1., .99))
        raw.append(observed_lead(distance, lateral, speed, self.ego, acceleration))
    before = copy.deepcopy(raw)
    output = self.tracker.update(observations, raw, self.timestamp, self.ego, valid=valid, yaw_rate=yaw)
    self.assertEqual(raw, before, 'Association must not mutate the current measurements')
    return output, raw

  def warm(self, samples=((75., 0., 30.), (75., 0., 30.)), yaw=0.):
    for _ in range(160):
      output, raw = self.step(samples, yaw=yaw)
    self.assertTrue(all(track is not None and track.ready for track in self.tracker.slots))
    return output, raw

  def test_duplicate_model_heads_share_one_identity_and_history(self):
    self.warm()
    first, second = self.tracker.slots
    self.assertIs(first, second)
    self.assertGreater(first.track_id, 0)
    self.assertEqual(first.identity, (first.track_id, first.generation))
    identity = first.identity
    age = first.age
    self.step(((75., -.02, 30.), (75.08, .02, 30.)))
    self.assertIs(self.tracker.slots[0], self.tracker.slots[1])
    self.assertEqual(self.tracker.slots[0].identity, identity)
    self.assertAlmostEqual(self.tracker.slots[0].age - age, .05)

  def test_slot_swap_keeps_history_with_the_vehicle(self):
    pair = ((65., -1., 30.), (85., 1., 30.))
    self.warm(pair)
    old_tracks = tuple(self.tracker.slots)
    old_ids = tuple(track.identity for track in old_tracks)
    self.assertNotEqual(*old_ids)
    self.step(tuple(reversed(pair)))
    self.assertEqual(tuple(track.identity for track in self.tracker.slots), tuple(reversed(old_ids)))
    self.assertIs(self.tracker.slots[0], old_tracks[1])
    self.assertIs(self.tracker.slots[1], old_tracks[0])

  def test_nearby_slower_cut_in_starts_fresh_and_responds_immediately(self):
    self.warm()
    old_identity = self.tracker.slots[0].identity
    output, raw = self.step(((72., 0., 18.), (72., 0., 18.)))
    new_track = self.tracker.slots[0]
    self.assertNotEqual(new_track.identity, old_identity)
    self.assertIs(new_track, self.tracker.slots[1])
    self.assertEqual(new_track.age, 0.)
    self.assertEqual(len(new_track.range_history), 1)
    for actual, measured in zip(output, raw, strict=True):
      # This is outside the old five-second urgent-closing bypass: identity
      # ownership itself must prevent the old 30 m/s estimate being blended in.
      self.assertEqual(actual['dRel'], measured['dRel'])
      self.assertEqual(actual['vLead'], 18.)
      self.assertEqual(actual['vLeadK'], 18.)
      self.assertEqual(actual['vRel'], -12.)
      self.assertEqual(actual['aLeadK'], 0.)
      self.assertTrue(actual['present'])

  def test_split_retains_old_car_without_lending_speed_to_new_car(self):
    self.warm()
    old_identity = self.tracker.slots[0].identity
    output, raw = self.step(((72., .2, 18.), (75., 0., 30.)))
    self.assertEqual(self.tracker.slots[1].identity, old_identity)
    self.assertNotEqual(self.tracker.slots[0].identity, old_identity)
    self.assertEqual(output[0]['vLead'], raw[0]['vLead'])
    self.assertEqual(output[0]['dRel'], raw[0]['dRel'])
    self.assertEqual(self.tracker.slots[0].age, 0.)
    self.assertGreater(self.tracker.slots[1].age, 5.)

  def test_continuously_braking_vehicle_keeps_identity(self):
    self.warm()
    old_id = self.tracker.slots[0].track_id
    for frame in range(1, 41):
      elapsed = .05 * frame
      speed = 30. - 3. * elapsed
      distance = 75. - 1.5 * elapsed**2
      output, _ = self.step(((distance, 0., speed),) * 2, acceleration=-3.)
      self.assertEqual(self.tracker.slots[0].track_id, old_id)
      self.assertLessEqual(output[0]['vLead'], speed)
      self.assertEqual(output[0]['aLeadK'], -3.)

  def test_model_velocity_change_alone_does_not_invent_a_new_vehicle(self):
    self.warm()
    old_identity = self.tracker.slots[0].identity
    self.step(((75., 0., 21.),) * 2, acceleration=-1.)
    self.assertEqual(self.tracker.slots[0].identity, old_identity)

  def test_range_correction_alone_does_not_invent_a_new_vehicle(self):
    self.warm()
    old_identity = self.tracker.slots[0].identity
    self.step(((73., 0., 30.),) * 2)
    self.assertEqual(self.tracker.slots[0].identity, old_identity)

  def test_small_joint_correction_keeps_history_but_large_joint_change_reanchors(self):
    # Distinguish normal estimation adjustments from the previously missed
    # nearby cut-in. Neither cue alone establishes a replacement vehicle.
    for distance, speed, expected_new in ((73., 28., False), (74., 24., False), (73., 24., True)):
      with self.subTest(distance=distance, speed=speed):
        self.tracker.reset()
        self.warm()
        old_identity = self.tracker.slots[0].identity
        output, raw = self.step(((distance, 0., speed),) * 2)
        self.assertEqual(self.tracker.slots[0].identity != old_identity, expected_new)
        if expected_new:
          self.assertEqual(output[0]['vLead'], raw[0]['vLead'])
          self.assertEqual(output[0]['dRel'], raw[0]['dRel'])

  def test_coincident_heads_with_conflicting_speeds_do_not_share_history(self):
    self.warm()
    old_identity = self.tracker.slots[0].identity
    output, raw = self.step(((75., 0., 18.), (75., 0., 30.)))
    self.assertIsNot(self.tracker.slots[0], self.tracker.slots[1])
    self.assertEqual(self.tracker.slots[1].identity, old_identity)
    self.assertNotEqual(self.tracker.slots[0].identity, old_identity)
    self.assertEqual(output[0]['vLead'], raw[0]['vLead'])
    self.assertEqual(output[1]['vLead'], raw[1]['vLead'])

  def test_ego_yaw_preserves_identity_across_camera_lateral_motion(self):
    # A rightward ego rotation makes a stationary relative target move left
    # in yRel. Use camera-frame coordinates with the existing 1.52 m offset.
    yaw = .25
    self.warm()
    old_identity = self.tracker.slots[0].identity
    angle = yaw * .05
    distance = (75. + 1.52) * math.cos(angle) - 1.52
    lateral = (75. + 1.52) * math.sin(angle)
    self.assertGreater(lateral, .75)
    self.step(((distance, lateral, 30.),) * 2, yaw=yaw)
    self.assertEqual(self.tracker.slots[0].identity, old_identity)

  def test_reset_does_not_reuse_a_retired_id(self):
    self.step()
    old_id = self.tracker.slots[0].track_id
    self.tracker.reset()
    self.step()
    self.assertGreater(self.tracker.slots[0].track_id, old_id)

  def test_brief_missing_observation_retains_private_identity_not_presence(self):
    self.warm()
    old_identity = self.tracker.slots[0].identity
    output, _ = self.step((None, None))
    self.assertEqual(output, [{'present': False}, {'present': False}])
    self.assertEqual(self.tracker.slots, [None, None])
    self.step()
    self.assertEqual(self.tracker.slots[0].identity, old_identity)

  def test_ambiguous_split_does_not_duplicate_mature_history(self):
    self.warm()
    old_identity = self.tracker.slots[0].identity
    output, raw = self.step(((75., -.3, 20.), (75., .3, 20.)))
    identities = [track.identity for track in self.tracker.slots]
    self.assertEqual(len(set(identities)), 2)
    # Both assignments fit equally well. Neither branch may claim the mature
    # lead's trusted state just because its model slot happens to come first.
    self.assertNotIn(old_identity, identities)
    for actual, measured, track in zip(output, raw, self.tracker.slots, strict=True):
      self.assertEqual(track.age, 0.)
      self.assertEqual(actual['vLead'], measured['vLead'])
      self.assertEqual(actual['dRel'], measured['dRel'])

  def test_ambiguous_merge_does_not_pick_an_old_identity_arbitrarily(self):
    self.warm(((75., -.4, 30.), (75., .4, 30.)))
    old_identities = {track.identity for track in self.tracker.slots}
    output, raw = self.step(((75., 0., 18.),) * 2)
    self.assertIs(self.tracker.slots[0], self.tracker.slots[1])
    self.assertNotIn(self.tracker.slots[0].identity, old_identities)
    self.assertEqual(output[0]['vLead'], raw[0]['vLead'])

  def test_in_place_reanchor_advances_history_generation(self):
    self.warm()
    track = self.tracker.slots[0]
    old_id, old_generation = track.identity
    observation = VisionLeadObservation(74., 0., 1., .99)
    track.__init__(observation, self.timestamp, self.ego)
    self.assertEqual(track.track_id, old_id)
    self.assertGreater(track.generation, old_generation)
    self.assertNotEqual(track.identity, (old_id, old_generation))
    self.assertEqual(track.age, 0.)
    self.assertEqual(len(track.range_history), 1)
    self.assertIsNone(track.output)


if __name__ == '__main__':
  unittest.main()
