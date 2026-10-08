import math
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

import numpy as np
import pyray as rl

from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.conditional_experimental import ConditionalExperimental, MapApproach
from openpilot.selfdrive.ui.onroad.model_stop import ModelStop, StopMarker, PATH_HALF_WIDTH, draw_stop_flag, front_offset


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
  def test_flag_and_cem_share_the_stop_prediction(self):
    prediction = model()
    marker = ModelStop().predict(prediction, 8., 100.)
    policy = ConditionalExperimental(.05, .5)
    accel, stop, _ = policy.update(enabled=True, eligible=True, approach=MapApproach(True), now=100.,
                                  model=prediction, v_ego=8., v_set=15., personality=1,
                                  regular_accel=.5, regular_stop=False, e2e_accel=-1., e2e_stop=False)
    self.assertAlmostEqual(marker.distance, policy.intent.stop_distance)
    self.assertEqual((accel, stop, policy.reason), (-1., False, 'modelStop'))
    self.assertTrue(policy.active)

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

  def test_no_stop_and_invalid_geometry_hide_marker(self):
    for prediction in (model(decel=0.), model()):
      if prediction.velocity.x[-1] == 0:
        prediction.position.z[8] = np.nan
      self.assertIsNone(ModelStop().predict(prediction, 8., 100.))
    prediction = model()
    prediction.orientation.z = []
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
    self.assertIsNone(stop.marker)
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

  def test_flag_foot_is_not_clamped_and_units_are_converted(self):
    marker = StopMarker((21.52, 0., 0.), 0., 20.)
    rect = rl.Rectangle(30, 30, 2100, 1020)
    def projector(x, y, z):
      return (1080 + y * 30, 520 + z * 60)
    with patch.multiple(rl, draw_circle_v=lambda *a: None,
                        draw_rectangle_rounded=lambda *a: None, draw_rectangle_rounded_lines_ex=lambda *a: None), \
         patch.object(rl, 'draw_line_ex') as line, patch.object(rl, 'draw_text_ex') as text:
      draw_stop_flag(marker, 1.22, projector, rect, None, True)
      self.assertEqual(text.call_args_list[-1].args[1], '~20.0 m')
      self.assertEqual(line.call_args_list[0].args[0], projector(21.52, PATH_HALF_WIDTH, 1.22))
      self.assertGreater(line.call_args_list[0].args[0][0], projector(21.52, 0., 1.22)[0])
      self.assertEqual(marker.position, (21.52, 0., 0.))
      draw_stop_flag(marker, 1.22, projector, rect, None, False)
      self.assertEqual(text.call_args_list[-1].args[1], '~65.6 ft')
      text.reset_mock()
      draw_stop_flag(marker, 1.22, lambda *a: (1080., 1100.), rect, None, True)
      text.assert_not_called()
