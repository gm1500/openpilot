import math
import unittest
from types import SimpleNamespace as NS

import numpy as np

from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.model_intent import model_intent
from openpilot.selfdrive.ui.onroad.model_stop import ModelStop, front_offset


def model(speed=8., decel=2., yaw=0.):
  t = np.asarray(ModelConstants.T_IDXS)
  travel_time = np.minimum(t, speed / decel) if decel > 0 else t
  distance = speed * travel_time - .5 * decel * travel_time**2
  velocity = np.maximum(0., speed - decel * t)
  zeros = np.zeros(len(t))
  return NS(position=NS(x=distance * math.cos(yaw), y=distance * math.sin(yaw), z=zeros.copy()),
            velocity=NS(x=velocity * math.cos(yaw), y=velocity * math.sin(yaw)),
            orientation=NS(y=zeros.copy(), z=zeros + yaw))


class SM(dict):
  def __init__(self, prediction):
    super().__init__(modelV2=prediction, carState=NS(vEgo=8., gearShifter='drive'),
                     selfdriveState=NS(experimentalMode=False, conditionalExperimental=True))
    self.valid = dict.fromkeys(self, True)
    self.recv_frame = dict.fromkeys(self, 10)
    self.stamp(100.)

  def stamp(self, now):
    self.recv_time = dict.fromkeys(self, now)
    self.logMonoTime = dict.fromkeys(self, round(now * 1e9))


class TestModelStop(unittest.TestCase):
  def test_planner_margin_moves_marker_earlier_once_without_changing_bumper_offset(self):
    from openpilot.selfdrive.controls.lib.e2e_stop import E2EStopTarget

    prediction = model()
    target = E2EStopTarget()
    target.update(eligible=True, model=prediction, now=100., model_time=100.,
                  v_ego=8., e2e_accel=-1., e2e_stop=False)
    display = ModelStop()
    raw = display.predict(prediction, 8., 100.)
    planned = display.predict_target(prediction, 8., target.distance)
    self.assertEqual(planned.label, 'STOP TARGET')
    self.assertAlmostEqual(raw.distance - planned.distance, 1.5)
    self.assertAlmostEqual(raw.position[0] - planned.position[0], 1.5)
    self.assertAlmostEqual(planned.position[0] - planned.distance, 1.52)

  def test_marker_uses_shared_stop_prediction(self):
    prediction = model()
    marker = ModelStop().predict(prediction, 8., 100.)
    self.assertAlmostEqual(marker.distance, model_intent(prediction, 8.).stop_distance)

  def test_bumper_offset_moves_forward_without_shortening_remaining_travel(self):
    a = ModelStop(offset=0.).predict(model(), 8., 100.)
    b = ModelStop(offset=2.4).predict(model(), 8., 100.)
    self.assertIsNotNone(a)
    self.assertAlmostEqual(b.position[0] - a.position[0], 2.4)
    self.assertAlmostEqual(a.distance, b.distance)
    self.assertLessEqual(a.distance, 16.)
    self.assertGreater(a.distance, 15.)

  def test_offset_follows_future_heading_and_pitch(self):
    prediction = model(yaw=.5)
    prediction.orientation.y[:] = .1
    a = ModelStop(offset=0.).predict(prediction, 8., 100.)
    b = ModelStop(offset=2.).predict(prediction, 8., 100.)
    np.testing.assert_allclose(np.subtract(b.position, a.position),
                               [2 * math.cos(.5) * math.cos(.1), 2 * math.sin(.5) * math.cos(.1), -2 * math.sin(.1)])

  def test_no_stop_hides_marker_but_invalid_geometry_keeps_status(self):
    self.assertIsNone(ModelStop().predict(model(decel=0.), 8., 100.))
    prediction = model()
    prediction.position.z[8] = np.nan
    marker = ModelStop().predict(prediction, 8., 100.)
    self.assertIsNone(marker.position)
    self.assertAlmostEqual(marker.distance, 16., delta=.1)
    prediction = model()
    prediction.orientation.z = []
    self.assertIsNone(ModelStop().predict(prediction, 8., 100.).position)
    prediction.velocity.x[0] = np.nan
    self.assertIsNone(ModelStop().predict(prediction, 8., 100.))

  def test_brief_speed_dip_is_not_a_stop(self):
    prediction = model(decel=0.)
    prediction.velocity.x[10] = 0.
    self.assertIsNone(ModelStop().predict(prediction, 8., 100.))

  def test_forward_gears_modes_and_data_freshness(self):
    sm, stop = SM(model()), ModelStop()
    stop.update(sm, 1, 100.)
    self.assertIsNotNone(stop.marker)
    for service in sm:
      sm.valid[service] = False
      stop.update(sm, 1, 100.)
      self.assertIsNone(stop.marker)
      sm.valid[service] = True
    for gear in ('reverse', 'park', 'neutral', 'unknown'):
      sm['carState'].gearShifter = gear
      stop.update(sm, 1, 100.)
      self.assertIsNone(stop.marker)
    sm['carState'].gearShifter = 'drive'
    stop.update(sm, 1, 100.31)
    self.assertIsNone(stop.marker)
    stop.update(sm, 11, 100.)
    self.assertIsNone(stop.marker)
    sm['selfdriveState'].conditionalExperimental = False
    stop.update(sm, 1, 100.)
    self.assertIsNotNone(stop.marker)
    sm['selfdriveState'].experimentalMode = True
    stop.update(sm, 1, 100.)
    self.assertIsNotNone(stop.marker)

  def test_fixed_stop_has_no_smoothing_lag_as_car_moves(self):
    stop = ModelStop()
    before = stop.predict(model(), 8., 100.)
    after_model = model(speed=7.9)
    # A fixed world stop after 0.05 s at 8 m/s: translate all future positions.
    before_model = model()
    after_model.position.x = before_model.position.x - .4
    # Its current reference starts at zero and the stop is now .4 m closer.
    after_model.position.x = np.maximum(after_model.position.x, 0.)
    after_model.velocity = before_model.velocity
    after = stop.predict(after_model, 8., 100.05)
    self.assertAlmostEqual(before.distance - after.distance, .4, places=6)
    self.assertAlmostEqual(before.position[0] - after.position[0], .4, places=6)

  def test_invalid_prediction_clears_immediately_and_same_frame_does_not_filter_twice(self):
    sm, stop = SM(model()), ModelStop()
    stop.update(sm, 1, 100.)
    marker = stop.marker
    stop.update(sm, 1, 100.01)
    self.assertIs(stop.marker, marker)
    sm['modelV2'] = model(decel=0.)
    sm.stamp(100.05)
    stop.update(sm, 1, 100.05)
    self.assertIsNone(stop.marker)

  def test_stationary_path_and_offset_validation(self):
    marker = ModelStop().predict(model(speed=0.), 0., 100.)
    self.assertAlmostEqual(marker.distance, 0.)
    self.assertAlmostEqual(marker.position[0], 1.52)
    for value in (None, 'bad', float('nan'), float('inf'), -1., 6.):
      self.assertEqual(front_offset(value), 1.52)
    self.assertEqual(front_offset('2.35'), 2.35)

  def test_explicit_stop_action_without_trajectory_keeps_status_without_inventing_distance(self):
    prediction = model(decel=0.)
    prediction.action = NS(shouldStop=True)
    sm, stop = SM(prediction), ModelStop()
    sm['carState'].vEgo = 0.
    for frame in range(30):
      stamp = 100. + frame * .05
      sm.stamp(stamp)
      stop.update(sm, 1, stamp + 1e-6)
      self.assertIsNotNone(stop.marker)
      self.assertIsNone(stop.marker.position)
      self.assertIsNone(stop.marker.distance)
    sm.stamp(102.)
    prediction.action.shouldStop = False
    stop.update(sm, 1, 102.)
    self.assertIsNone(stop.marker)

  def test_docked_marker_does_not_survive_stale_data_or_shift_to_reverse(self):
    sm, stop = SM(model(speed=0.)), ModelStop(offset=0.)
    sm['carState'].vEgo = 0.
    stop.update(sm, 1, 100.)
    self.assertIsNotNone(stop.marker)
    stop.update(sm, 1, 100.31)
    self.assertIsNone(stop.marker)

    sm.stamp(101.)
    stop.update(sm, 1, 101.)
    self.assertIsNotNone(stop.marker)
    sm['carState'].gearShifter = 'reverse'
    sm.stamp(101.05)
    stop.update(sm, 1, 101.05)
    self.assertIsNone(stop.marker)

  def test_acc_uses_published_target_without_model_distance_or_ui_smoothing(self):
    sm, stop = SM(model()), ModelStop()
    sm['selfdriveState'].enabled = True
    sm['longitudinalPlan'] = NS(stopTarget=NS(active=True, distance=7.), e2eStopActive=True)
    sm.valid['longitudinalPlan'] = True
    sm.recv_frame['longitudinalPlan'] = 10
    sm.stamp(100.)
    stop.update(sm, 1, 100.)
    self.assertEqual(stop.marker.label, 'STOP TARGET')
    self.assertEqual(stop.marker.distance, 7.)
    self.assertAlmostEqual(stop.marker.position[0], 8.52)
    sm['longitudinalPlan'].stopTarget.distance = 4.
    stop.update(sm, 1, 100.01)
    self.assertEqual(stop.marker.distance, 4.)
    sm['longitudinalPlan'].stopTarget.active = False
    sm['longitudinalPlan'].e2eStopActive = False
    stop.update(sm, 1, 100.02)
    self.assertIsNone(stop.marker)  # a raw prediction cannot resurrect a released target

  def test_target_marker_requires_fresh_planner_and_handles_unknown_projection(self):
    sm, stop = SM(model()), ModelStop()
    sm['selfdriveState'].enabled = True
    sm['longitudinalPlan'] = NS(stopTarget=NS(active=True, distance=40.), e2eStopActive=True)
    sm.valid['longitudinalPlan'] = True
    sm.recv_frame['longitudinalPlan'] = 10
    sm.stamp(100.)
    stop.update(sm, 1, 100.)
    self.assertEqual(stop.marker.distance, 40.)
    self.assertIsNone(stop.marker.position)  # target beyond path: no clamping
    sm.valid['longitudinalPlan'] = False
    stop.update(sm, 1, 100.)
    self.assertIsNone(stop.marker)
    sm.valid['longitudinalPlan'] = True
    sm.recv_time['longitudinalPlan'] = 99.
    stop.update(sm, 1, 100.)
    self.assertIsNone(stop.marker)
