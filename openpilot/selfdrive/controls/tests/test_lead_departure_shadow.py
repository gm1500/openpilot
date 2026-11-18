import copy
from dataclasses import FrozenInstanceError, replace
import math
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from openpilot.selfdrive.controls.lib.lead_departure_shadow import DepartureHead, LeadDepartureShadow


def head(x=60., y=0., speed=25., **kwargs):
  return DepartureHead(x, y, speed, kwargs.get('probability', .99), kwargs.get('x_std', 2.), kwargs.get('y_std', .2))


class TestLeadDepartureShadow(unittest.TestCase):
  def setUp(self):
    self.observer = LeadDepartureShadow()
    self.time = -.05

  def step(self, heads=None, state='off', direction='none', yaw=0., dt=.05, **kwargs):
    self.time += dt
    heads = heads if heads is not None else (head(), head())
    before = copy.deepcopy(heads)
    result = self.observer.update(heads, self.time, kwargs.get('ego', 30.), yaw, state, direction, kwargs.get('valid', True))
    self.assertIsNone(result)
    self.assertEqual(heads, before)
    self.assertFalse(self.observer.status.can_ignore_lead)
    return self.observer.status

  def warm(self):
    for _ in range(12):
      self.step()
    self.assertEqual(self.observer.status.reason, 'coherent_pair_armed')

  def split(self, direction='left', **kwargs):
    sign = -1. if direction == 'left' else 1.
    return self.step((head(52., 0., 25.), head(64., sign * .8, 28.)),
                     'laneChangeStarting', direction, sign * .02, **kwargs)

  def test_confirmation_never_creates_control_authority(self):
    self.warm()
    for _ in range(3):
      self.assertFalse(self.split().active)
    status = self.split()
    self.assertTrue(status.active)
    self.assertEqual(status.reason, 'split_ambiguous')
    self.assertLess(status.range_residual, -3.)
    with self.assertRaises(FrozenInstanceError):
      status.can_ignore_lead = True

  def test_no_mid_maneuver_acquisition_and_no_indefinite_renewal(self):
    for _ in range(20):
      self.step(state='laneChangeStarting', direction='left', yaw=-.02)
    self.assertIsNone(self.observer.anchor)
    self.warm()
    anchor = self.observer.anchor
    for _ in range(20):
      self.split()
      self.assertIs(self.observer.anchor, anchor)
    for _ in range(60):
      self.split()
    self.assertFalse(self.observer.status.active)
    self.assertIsNone(self.observer.anchor)
    self.assertIsNone(self.observer.split_since)
    self.warm()
    for _ in range(4):
      status = self.split()
    self.assertTrue(status.active)

  def test_direction_change_and_abort_revoke(self):
    for action in ('reverse', 'abort'):
      with self.subTest(action=action):
        self.setUp()
        self.warm()
        for _ in range(4):
          self.split()
        status = self.split('right') if action == 'reverse' else self.step()
        self.assertFalse(status.active)
        self.assertIsNone(self.observer.anchor)

  def test_gap_duplicate_backward_and_invalid_samples_revoke(self):
    for change in ({'dt': 0.}, {'dt': -.05}, {'dt': .2}, {'valid': False}, {'ego': math.nan}, {'ego': 5.}):
      with self.subTest(change=change):
        self.setUp()
        self.warm()
        for _ in range(4):
          self.split()
        self.assertFalse(self.split(**change).active)
        self.assertIsNone(self.observer.anchor)
        for _ in range(4):
          self.assertFalse(self.split().active)

  def test_malformed_or_missing_head_cannot_preserve_an_anchor(self):
    bad_heads = [(), (head(),), (head(), head(), head()), (head(), None)]
    for field in ('x', 'y', 'speed', 'probability', 'x_std', 'y_std'):
      for value in (math.nan, math.inf, None):
        bad_heads.append((head(), replace(head(), **{field: value})))
    for field, value in (('x', -1.), ('x_std', 0.), ('y_std', -1.), ('probability', .1), ('probability', 1.1)):
      bad_heads.append((head(), replace(head(), **{field: value})))
    for heads in bad_heads:
      with self.subTest(heads=heads):
        self.setUp()
        self.warm()
        self.step(heads)
        self.assertIsNone(self.observer.anchor)
        self.assertEqual(self.observer.status.reason, 'invalid_context')
    self.observer.update(None, self.time + .05, 30., 0., 'off', 'none')
    self.assertEqual(self.observer.status.reason, 'invalid_context')

  def test_directional_measured_motion_and_confirmed_split_are_required(self):
    for state, direction, yaw in (('off', 'left', -.02), ('preLaneChange', 'left', -.02),
                                   ('laneChangeStarting', 'left', .02), ('laneChangeStarting', 'none', -.02),
                                   ('laneChangeStarting', 'left', 0.), ('unknown', 'left', -.02)):
      with self.subTest(state=state, direction=direction, yaw=yaw):
        self.setUp()
        self.warm()
        for _ in range(8):
          self.assertFalse(self.step((head(52.), head(64., -.8, 28.)), state, direction, yaw).active)

  def test_head_order_and_left_right_symmetry(self):
    records = []
    for sign, reverse in ((-1., False), (-1., True), (1., False), (1., True)):
      self.setUp()
      self.warm()
      sequence = []
      for _ in range(12):
        heads = (head(52.), head(64., sign * .8, 28.))
        status = self.step(heads[::-1] if reverse else heads, 'laneChangeStarting', 'left' if sign < 0. else 'right', sign * .02)
        self.assertEqual(status.near_head, 1 if reverse else 0)
        sequence.append(replace(status, near_head=-1))
      records.append(sequence)
    self.assertTrue(all(r == records[0] for r in records[1:]))

  def test_true_cutin_closer_stopped_and_braking_vehicles_remain_ambiguous(self):
    # A distinct vehicle can move into the destination corridor while producing
    # the same split. The observer must never rename it a false/ignorable lead.
    for speed in (0., 10., 25.):
      for sign in (-1., 1.):
        with self.subTest(speed=speed, sign=sign):
          self.setUp()
          self.warm()
          for frame in range(8):
            near = head(45. - frame * .1, sign * frame * .03, speed)
            far = head(65., sign * 1.2, 28.)
            status = self.step((near, far), 'laneChangeStarting', 'left' if sign < 0. else 'right', sign * .02)
          self.assertTrue(status.active)
          self.assertEqual(status.reason, 'split_ambiguous')
          self.assertFalse(status.can_ignore_lead)

  def test_short_split_and_lost_coherence_do_not_arm(self):
    for _ in range(11):
      self.step()
    self.step((head(), head(70.)))
    for _ in range(4):
      self.assertFalse(self.split().active)
    self.warm()
    for _ in range(3):
      self.assertFalse(self.split().active)
    self.step(state='laneChangeStarting', direction='left', yaw=-.02)
    for _ in range(3):
      self.assertFalse(self.split().active)


class TestLeadDepartureRadarIntegration(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    from openpilot.cereal import log
    from opendbc.car.structs import car
    from openpilot.selfdrive.controls import radard
    from openpilot.selfdrive.controls.tests.test_radard_lead_motion import closing_model
    cls.log, cls.car, cls.radard = log, car, radard
    cls.closing_model = staticmethod(closing_model)

  def setUp(self):
    self.logging = patch('openpilot.selfdrive.controls.radard.cloudlog')
    self.log_mock = self.logging.start()
    self.addCleanup(self.logging.stop)
    self.radar = self.car.RadarData.new_message()

  def model(self, frame):
    model = self.closing_model(frame, count=2)
    model.orientationRate.z = [.15]  # Deliberately opposite to measured left yaw.
    for i, lead in enumerate(model.leadsV3):
      lead.x, lead.y, lead.v = [60.] * 6, [0.] * 6, [25.] * 6
      if frame >= 12:
        lead.x = [52. if i == 0 else 64.] * 6
        lead.y = [0. if i == 0 else -.8] * 6
        lead.v = [25. if i == 0 else 28.] * 6
    if frame >= 12:
      model.meta.laneChangeState, model.meta.laneChangeDirection = 'laneChangeStarting', 'left'
    return model

  def update(self, rd, frame, control='valid', sample_offset=-.01, model=None, calibration='valid', source_delay=.02, motion_yaw=-.02):
    model = self.model(frame) if model is None else model
    timestamp = 1. + frame * .05
    # A fresh carControl envelope does not prove its cached angularVelocity is
    # fresh. Keep this decoy valid even when the actual measured source fails.
    cc = self.car.CarControl.new_message(angularVelocity=[0., 0., .15])
    motion = self.log.DeviceMotion.new_message(timestamp=round((timestamp + sample_offset - source_delay) * 1e9),
                                               inputsOK=True, sensorsOK=True, posenetOK=True)
    motion.angularVelocityDevice = {'x': 0., 'y': 0., 'z': motion_yaw, 'xStd': .01, 'yStd': .01, 'zStd': .01, 'valid': True}
    if control == 'nonfinite':
      motion.angularVelocityDevice.z = math.nan
    if control == 'malformed':
      motion.angularVelocityDevice.valid = False
    if control == 'nonfinite_std':
      motion.angularVelocityDevice.zStd = math.nan
    if control == 'negative_std':
      motion.angularVelocityDevice.zStd = -1.
    if control in ('inputsOK', 'sensorsOK', 'posenetOK'):
      setattr(motion, control, False)
    extrinsics = self.log.ExtrinsicsCalibration.new_message(calStatus='calibrated', rpyCalib=[0., 0., 0.])
    if calibration == 'uncalibrated':
      extrinsics.calStatus = 'uncalibrated'
    if calibration == 'nonfinite':
      extrinsics.rpyCalib = [0., math.nan, 0.]
    if calibration == 'malformed':
      extrinsics.rpyCalib = [0., 0.]
    if calibration == 'rotated':
      extrinsics.rpyCalib = [.2, 0., 0.]
      motion.angularVelocityDevice.y, motion.angularVelocityDevice.z = .1, -.01
    inputs = {'modelV2': model, 'carState': self.car.CarState.new_message(vEgo=33.), 'carControl': cc,
              'deviceMotion': motion, 'extrinsicsCalibration': extrinsics}
    sm = type('Inputs', (dict,), {})(inputs)
    sm.seen = {'modelV2': True, 'carState': True, 'carControl': True, 'deviceMotion': control != 'missing',
               'extrinsicsCalibration': calibration != 'missing'}
    sm.valid = {'modelV2': True, 'carState': True, 'carControl': True, 'deviceMotion': control != 'invalid',
                'extrinsicsCalibration': calibration != 'invalid'}
    sm.recv_frame = {'carState': frame}
    sm.logMonoTime = {'modelV2': round(timestamp * 1e9), 'carState': round((timestamp + sample_offset) * 1e9),
                     'carControl': round((timestamp - .001) * 1e9), 'deviceMotion': round((timestamp + sample_offset) * 1e9),
                     'extrinsicsCalibration': round((timestamp - (3. if calibration == 'stale' else .1)) * 1e9)}
    sm.all_checks = lambda: True  # Optional measured input never owns radar health.
    payload = model.to_bytes()
    model.clear_write_flag()
    with self.log.ModelDataV2.from_bytes(payload) as decoded:
      sm['modelV2'] = decoded
      rd.update(sm, self.radar)
    return rd.radar_state_valid, rd.radar_state.to_dict()

  def test_active_shadow_has_exactly_the_same_published_radar_state(self):
    observed, baseline = self.radard.RadarD(), self.radard.RadarD()
    baseline.lead_departure_shadow = None
    any_active = False
    for frame in range(40):
      self.assertEqual(self.update(observed, frame), self.update(baseline, frame))
      any_active |= observed.lead_departure_shadow.status.active
    self.assertTrue(any_active)
    for rd in (observed, baseline):
      pm = Mock()
      rd.publish(pm)
      self.assertTrue(pm.send.call_args.args[1].valid)
      self.assertEqual(pm.send.call_args.args[1].radarState.to_dict(), rd.radar_state.to_dict())

  def test_missing_stale_invalid_and_malformed_optional_input_preserve_control(self):
    for control, offset in (('missing', -.01), ('invalid', -.01), ('valid', -.2), ('valid', .3),
                            ('nonfinite', -.01), ('malformed', -.01), ('nonfinite_std', -.01), ('negative_std', -.01),
                            ('inputsOK', -.01), ('sensorsOK', -.01), ('posenetOK', -.01)):
      with self.subTest(control=control, offset=offset):
        observed, baseline = self.radard.RadarD(), self.radard.RadarD()
        baseline.lead_departure_shadow = None
        for frame in range(30):
          self.assertEqual(self.update(observed, frame, control, offset), self.update(baseline, frame, control, offset))
          self.assertFalse(observed.lead_departure_shadow.status.active)
        self.assertIsNotNone(observed.lead_departure_shadow)

  def test_invalid_measured_input_revokes_an_existing_warning_without_changing_control(self):
    for control in ('missing', 'invalid', 'nonfinite', 'malformed', 'inputsOK', 'sensorsOK', 'posenetOK'):
      with self.subTest(control=control):
        observed, baseline = self.radard.RadarD(), self.radard.RadarD()
        baseline.lead_departure_shadow = None
        for frame in range(20):
          self.assertEqual(self.update(observed, frame), self.update(baseline, frame))
        self.assertTrue(observed.lead_departure_shadow.status.active)
        self.assertEqual(self.update(observed, 20, control), self.update(baseline, 20, control))
        self.assertFalse(observed.lead_departure_shadow.status.active)
        self.assertIsNone(observed.lead_departure_shadow.anchor)

  def test_previous_causal_sample_used_when_latest_measurement_is_newer(self):
    observed, baseline = self.radard.RadarD(), self.radard.RadarD()
    baseline.lead_departure_shadow = None
    active = False
    for frame in range(40):
      self.assertEqual(self.update(observed, frame, sample_offset=.01), self.update(baseline, frame, sample_offset=.01))
      active |= observed.lead_departure_shadow.status.active
      self.assertLessEqual(len(observed.departure_ego_samples), 4)
      self.assertLessEqual(len(observed.departure_motion_samples), 4)
      self.assertLessEqual(len(observed.departure_calibration_samples), 4)
    self.assertTrue(active)

  def test_device_motion_filter_time_must_be_fresh_and_no_later_than_publication(self):
    for delay in (.2, -.02):
      with self.subTest(delay=delay):
        observed, baseline = self.radard.RadarD(), self.radard.RadarD()
        baseline.lead_departure_shadow = None
        for frame in range(20):
          self.assertEqual(self.update(observed, frame), self.update(baseline, frame))
        self.assertTrue(observed.lead_departure_shadow.status.active)
        self.assertEqual(self.update(observed, 20, source_delay=delay, sample_offset=-.05),
                         self.update(baseline, 20, source_delay=delay, sample_offset=-.05))
        self.assertFalse(observed.lead_departure_shadow.status.active)
        self.assertIsNone(observed.lead_departure_shadow.anchor)

  def test_received_motion_may_postdate_model_only_when_source_time_precedes_it(self):
    rd = self.radard.RadarD()
    for frame in range(20):
      self.update(rd, frame)
    self.update(rd, 20, sample_offset=.03, source_delay=.04, motion_yaw=-.045)
    self.assertTrue(rd.lead_departure_shadow.status.active)
    self.assertAlmostEqual(rd.lead_departure_shadow.last[2], -.045, places=7)
    # The next message has a future measurement timestamp. Only the previous
    # causally available measurement may contribute, not its different yaw.
    self.update(rd, 21, sample_offset=.03, source_delay=.01, motion_yaw=.08)
    self.assertAlmostEqual(rd.lead_departure_shadow.last[2], -.045, places=7)
    # Even an old measurement with an implausibly late envelope is rejected.
    self.update(rd, 22, sample_offset=.15, source_delay=.16, motion_yaw=-.03)
    self.assertFalse(rd.lead_departure_shadow.status.active)
    self.assertIsNone(rd.lead_departure_shadow.anchor)

  def test_calibration_must_be_valid_finite_and_recent(self):
    for mode in ('missing', 'invalid', 'uncalibrated', 'nonfinite', 'malformed', 'stale'):
      with self.subTest(calibration=mode):
        observed, baseline = self.radard.RadarD(), self.radard.RadarD()
        baseline.lead_departure_shadow = None
        for frame in range(20):
          self.assertEqual(self.update(observed, frame), self.update(baseline, frame))
        self.assertTrue(observed.lead_departure_shadow.status.active)
        self.assertEqual(self.update(observed, 20, calibration=mode), self.update(baseline, 20, calibration=mode))
        self.assertFalse(observed.lead_departure_shadow.status.active)
        self.assertIsNone(observed.lead_departure_shadow.anchor)

  def test_device_to_calibrated_rotation_uses_the_pose_calibrator_convention(self):
    rd = self.radard.RadarD()
    for frame in range(20):
      self.update(rd, frame, calibration='rotated')
    # device y=+.1, z=-.01 at +.2 roll maps to negative calibrated yaw.
    # The non-transposed matrix, model rate and cached CC yaw all point right.
    self.assertTrue(rd.lead_departure_shadow.status.active)
    self.assertAlmostEqual(rd.lead_departure_shadow.last[2], -.1 * math.sin(.2) - .01 * math.cos(.2), places=7)

  def test_observer_exception_disables_only_the_observer(self):
    observed, baseline = self.radard.RadarD(), self.radard.RadarD()
    baseline.lead_departure_shadow = None
    observer = Mock()
    observer.update.side_effect = RuntimeError('injected observer failure')
    observed.lead_departure_shadow = observer
    for frame in range(20):
      self.assertEqual(self.update(observed, frame), self.update(baseline, frame))
    self.assertIsNone(observed.lead_departure_shadow)
    observer.update.assert_called_once()

  def test_missing_model_revokes_observer(self):
    observed = self.radard.RadarD()
    for frame in range(20):
      self.update(observed, frame)
    self.assertTrue(observed.lead_departure_shadow.status.active)
    self.update(observed, 20, model=self.closing_model(20, count=1))
    self.assertEqual(observed.lead_departure_shadow.status.reason, 'missing_model')
    self.assertIsNone(observed.lead_departure_shadow.anchor)

  def test_optional_subscription_is_excluded_from_radar_health_checks(self):
    class SubscriptionReached(Exception):
      pass
    with patch.object(self.radard, 'config_realtime_process'), patch.object(self.radard, 'Params'), \
         patch.object(self.radard.messaging, 'log_from_bytes', return_value=SimpleNamespace(radarDelay=0.)), \
         patch.object(self.radard.messaging, 'SubMaster', side_effect=SubscriptionReached) as submaster:
      with self.assertRaises(SubscriptionReached):
        self.radard.main()
    args, kwargs = submaster.call_args
    self.assertNotIn('carControl', args[0])
    for name in ('ignore_alive', 'ignore_avg_freq', 'ignore_valid'):
      for service in ('deviceMotion', 'extrinsicsCalibration'):
        self.assertIn(service, args[0])
        self.assertIn(service, kwargs[name])

  def test_diagnostic_reasons_are_visible_with_bounded_log_rate(self):
    rd = self.radard.RadarD()
    emitted = []
    for frame in range(60):
      self.log_mock.info.reset_mock()
      self.update(rd, frame, control='invalid' if frame % 2 == 0 else 'valid')
      lines = [call.args[0] for call in self.log_mock.info.call_args_list if call.args[0].startswith('lead-departure-shadow:')]
      self.assertLessEqual(len(lines), 1)
      if lines:
        emitted.append(frame * .05)
    self.assertGreater(len(emitted), 1)
    self.assertTrue(all(b - a >= .2 - 1e-9 for a, b in zip(emitted, emitted[1:], strict=False)))


if __name__ == '__main__':
  unittest.main()
