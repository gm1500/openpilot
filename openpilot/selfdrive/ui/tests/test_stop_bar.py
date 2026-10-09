import unittest
import sys
from types import SimpleNamespace as NS
from unittest.mock import patch

import numpy as np
import pyray as rl

from openpilot.selfdrive.ui.onroad.lead_geometry import lead_fill_alpha
from openpilot.selfdrive.ui.onroad.model_stop import ModelStop, StopMarker
from openpilot.selfdrive.ui.onroad.stop_bar import draw_stop_bar, stop_bar_geometry
from openpilot.selfdrive.ui.tests.test_model_stop import SM, model


class TestStopBar(unittest.TestCase):
  def setUp(self):
    self.rect = rl.Rectangle(30, 30, 2100, 1020)
    self.sm = SM(model(speed=0.))
    self.sm['carState'].vEgo = 0.
    self.sm['carState'].brakePressed = False
    self.sm['carState'].brakeHoldActive = False
    self.sm['selfdriveState'].enabled = True
    self.sm['longitudinalPlan'] = NS(stopTarget=NS(active=True, distance=0.), e2eStopActive=True)
    self.sm['carControl'] = NS(longActive=True)
    self.sm['carOutput'] = NS(actuatorsOutput=NS(brake=40.))
    self.sm.valid = dict.fromkeys(self.sm, True)
    self.sm.recv_frame = dict.fromkeys(self.sm, 10)
    self.sm.stamp(100.)
    self.stop = ModelStop()
    self.stop.update(self.sm, 1, 100.)

  def release_target(self):
    self.sm['longitudinalPlan'].stopTarget.active = False
    self.sm['longitudinalPlan'].e2eStopActive = False

  def test_hold_only_visual_until_all_brake_sources_release(self):
    self.release_target()
    self.stop.update(self.sm, 1, 100.01)
    self.assertTrue(self.stop.marker.held)
    self.assertFalse(self.sm['longitudinalPlan'].stopTarget.active)
    self.sm['carState'].brakePressed = True
    self.sm['carOutput'].actuatorsOutput.brake = 0.
    self.stop.update(self.sm, 1, 100.02)
    self.assertTrue(self.stop.marker.held)
    self.sm['carState'].brakePressed = False
    self.sm['carState'].brakeHoldActive = True
    self.stop.update(self.sm, 1, 100.03)
    self.assertTrue(self.stop.marker.held)
    self.sm['carState'].brakeHoldActive = False
    self.stop.update(self.sm, 1, 100.04)  # even without a new model frame
    self.assertIsNone(self.stop.marker)

  def test_manual_pedal_takeover_retains_planner_label_and_distance(self):
    self.release_target()
    self.sm['selfdriveState'].enabled = False
    self.sm['carControl'].longActive = False
    self.sm['carState'].brakePressed = True
    self.sm['modelV2'] = model()  # a different raw prediction must not move the held target
    self.sm.stamp(100.05)
    self.stop.update(self.sm, 1, 100.05)
    self.assertEqual((self.stop.marker.label, self.stop.marker.distance), ('STOP TARGET', 0.))
    self.assertTrue(self.stop.marker.held)
    self.sm['carState'].brakePressed = False
    self.stop.update(self.sm, 1, 100.06)
    self.assertIsNone(self.stop.marker)

  def test_hold_tracks_remaining_travel_while_driver_finishes_stop(self):
    self.sm['carState'].vEgo = 4.
    self.sm['modelV2'] = model()
    self.sm['longitudinalPlan'].stopTarget.distance = 8.
    self.stop.update(self.sm, 1, 100.01)
    self.release_target()
    self.stop.update(self.sm, 1, 100.11)
    self.assertAlmostEqual(self.stop.marker.distance, 7.6)
    self.assertAlmostEqual(self.stop.marker.position[0], 9.12)

  def test_no_ghost_from_braking_without_a_target_or_stale_output(self):
    self.release_target()
    empty = ModelStop()
    empty.update(self.sm, 1, 100.)
    self.assertIsNone(empty.marker)
    self.sm.valid['carOutput'] = False
    self.stop.update(self.sm, 1, 100.01)
    self.assertIsNone(self.stop.marker)

  def test_held_marker_clears_on_reverse_stale_data_new_drive_or_gas(self):
    for change in ('reverse', 'stale', 'new_drive', 'gas'):
      with self.subTest(change=change):
        self.setUp()
        self.release_target()
        self.stop.update(self.sm, 1, 100.01)
        self.assertTrue(self.stop.marker.held)
        if change == 'reverse':
          self.sm['carState'].gearShifter = 'reverse'
        elif change == 'gas':
          self.sm['carState'].gasPressed = True
        self.stop.update(self.sm, 11 if change == 'new_drive' else 1, 100.31 if change == 'stale' else 100.02)
        self.assertIsNone(self.stop.marker)

  @staticmethod
  def project(x, y, z):
    return (1080 + 900 * y / x, 400 + 900 * z / x) if x > 0 else None

  def test_bar_follows_road_perspective_and_has_bounded_thickness(self):
    marker = StopMarker((21.52, 0., 0.), 0., 20.)
    points, docked = stop_bar_geometry(marker, 1.5, self.project, self.rect)
    self.assertFalse(docked)
    np.testing.assert_allclose(points[:2], [self.project(21.52, -.9, 1.5), self.project(21.52, .9, 1.5)])
    self.assertGreater(points[2, 0] - points[3, 0], points[1, 0] - points[0, 0])
    self.assertLessEqual(np.ptp(points[:, 1]), 12 * self.rect.height / 540 + 1e-6)

  def test_close_passed_and_unprojectable_stops_remain_visible_as_a_bar(self):
    for x in (0., -2., 1.52, 2., 5.):
      marker = StopMarker((x, 0., 0.), 0., 0., True, 'STOP TARGET')
      points, docked = stop_bar_geometry(marker, 1.5, self.project, self.rect)
      if x <= 2.:
        self.assertTrue(docked)
      self.assertEqual(points.shape, (4, 2))  # a rectangle, never a triangle
      self.assertGreater(points[:, 0].min(), self.rect.x)
      self.assertLess(points[:, 0].max(), self.rect.x + self.rect.width)
      self.assertGreater(points[:, 1].min(), self.rect.y)
      self.assertLess(points[:, 1].max(), self.rect.y + self.rect.height)
    unknown = StopMarker(None, 0., None, True)
    points, docked = stop_bar_geometry(unknown, 1.5, self.project, self.rect)
    self.assertTrue(docked)
    self.assertTrue(np.all(np.isfinite(points)))

  def test_regular_lead_color_response_and_opaque_bar_fill(self):
    self.assertEqual(lead_fill_alpha(50., -8.), 0)
    self.assertLess(lead_fill_alpha(30., 0.), lead_fill_alpha(30., -2.))
    self.assertEqual(lead_fill_alpha(0., 0.), 255)
    marker = StopMarker((21.52, 0., 0.), 0., 20., label='STOP TARGET')
    with patch.dict(sys.modules, {'openpilot.system.ui.lib.text_measure': NS(measure_text_cached=lambda *args: NS(x=240., y=30.))}), \
         patch.object(rl, 'draw_triangle_fan') as polygon, patch.object(rl, 'draw_rectangle_rounded'), \
         patch.object(rl, 'draw_text_ex') as text:
      draw_stop_bar(marker, 4., 1.5, self.project, self.rect, None, True)
      self.assertEqual(polygon.call_count, 2)
      for call in polygon.call_args_list:
        self.assertEqual(call.args[2].a, 255)
      self.assertEqual(text.call_args.args[1], 'STOP TARGET  20.0 m')
      draw_stop_bar(marker, 4., 1.5, self.project, self.rect, None, False)
      self.assertEqual(text.call_args.args[1], 'STOP TARGET  65.6 ft')
