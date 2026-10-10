import unittest
import sys
from types import SimpleNamespace as NS
from unittest.mock import patch

import numpy as np
import pyray as rl

from openpilot.selfdrive.ui.onroad.lead_geometry import lead_fill_alpha, rounded_polygon
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

  def set_lead(self, distance, second=False):
    def lead(present):
      return NS(present=present, dRel=distance, vLead=0.)
    self.sm['radarState'] = NS(leadOne=lead(not second), leadTwo=lead(second))
    self.sm.valid['radarState'] = True
    self.sm.recv_frame['radarState'] = 10
    self.sm.stamp(100.02)

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

  def test_distinct_earlier_stop_survives_real_lead_and_brake_hold(self):
    for second in (False, True):
      for held in (False, True):
        with self.subTest(second=second, held=held):
          self.setUp()
          if held:
            self.release_target()
            self.stop.update(self.sm, 1, 100.01)
            self.assertTrue(self.stop.marker.held)
          self.set_lead(30., second)
          self.stop.update(self.sm, 1, 100.02)
          self.assertEqual(self.stop.marker.label, 'STOP TARGET')
          self.assertEqual(self.stop.marker.distance, 0.)
          self.assertEqual(self.stop.marker.held, held)

  def test_either_lead_in_overlap_band_clears_live_and_brake_held_bar(self):
    for second in (False, True):
      for held in (False, True):
        with self.subTest(second=second, held=held):
          self.setUp()
          if held:
            self.release_target()
            self.stop.update(self.sm, 1, 100.01)
            self.assertTrue(self.stop.marker.held)
          self.set_lead(11., second)  # 5 m ego stop after the ordinary 6 m gap
          self.stop.update(self.sm, 1, 100.02)
          self.assertIsNone(self.stop.marker)
          # Lead loss alone cannot resurrect the old brake-held bar.
          self.sm['radarState'].leadOne.present = self.sm['radarState'].leadTwo.present = False
          self.stop.update(self.sm, 1, 100.03)
          if held:
            self.assertIsNone(self.stop.marker)
          else:
            self.assertIsNotNone(self.stop.marker)  # fresh published target still exists

  def test_manual_and_experimental_displays_share_overlap_and_reentry_band(self):
    for experimental in (False, True):
      with self.subTest(experimental=experimental):
        self.setUp()
        self.sm['selfdriveState'].enabled = experimental
        self.sm['selfdriveState'].experimentalMode = experimental
        self.stop.reset()
        self.set_lead(11.)
        self.stop.update(self.sm, 1, 100.02)
        self.assertIsNone(self.stop.marker)
        self.set_lead(12.)  # exactly 6 m is still inside the re-entry guard
        self.stop.update(self.sm, 1, 100.02)
        self.assertIsNone(self.stop.marker)
        self.set_lead(12.1)
        self.stop.update(self.sm, 1, 100.02)
        self.assertEqual(self.stop.marker.label, 'MODEL STOP')

  def test_held_position_is_checked_after_model_withdraws_its_stop(self):
    self.release_target()
    self.sm['modelV2'] = model(decel=0.)
    self.set_lead(12.)  # held raw position 1 m; real-lead stop 6 m: 5 m overlap
    self.stop.update(self.sm, 1, 100.02)
    self.assertIsNone(self.stop.marker)

  def test_stale_lead_does_not_suppress_a_current_target(self):
    self.set_lead(11.)
    self.sm.valid['radarState'] = False
    self.stop.update(self.sm, 1, 100.02)
    self.assertIsNotNone(self.stop.marker)

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

  def test_bar_tracks_stop_point_but_stays_horizontal(self):
    scale = min(self.rect.width / 1080., self.rect.height / 540.)
    for distance, yaw in ((20., 0.), (60., 0.), (20., .4), (20., -.4)):
      x = distance + 1.52
      marker = StopMarker((x, 0., 0.), yaw, distance)
      points, docked = stop_bar_geometry(marker, 1.5, self.project, self.rect)
      self.assertFalse(docked)
      np.testing.assert_allclose(points[:2].mean(axis=0), self.project(x, 0., 1.5))
      self.assertAlmostEqual(np.ptp(points[:, 0]), 450.)
      self.assertEqual(points[0, 1], points[1, 1])
      self.assertEqual(points[2, 1], points[3, 1])
      self.assertGreater(points[0, 0], points[3, 0])
      self.assertLess(points[1, 0], points[2, 0])
      self.assertAlmostEqual(points[1, 0] - points[0, 0], .8 * (points[2, 0] - points[3, 0]))
      self.assertLessEqual(np.ptp(points[:, 1]), 24 * scale + 1e-6)
      if yaw == 0.:
        depth = self.project(x - min(6., .25 * x), 0., 1.5)[1] - self.project(x, 0., 1.5)[1]
        self.assertAlmostEqual(np.ptp(points[:, 1]), 2 * np.clip(depth, 3 * scale, 12 * scale))

  def test_close_passed_and_unprojectable_stops_remain_visible_as_a_bar(self):
    for x in (0., -2., 1.52, 2., 5.):
      marker = StopMarker((x, 0., 0.), 0., 0., True, 'STOP TARGET')
      points, docked = stop_bar_geometry(marker, 1.5, self.project, self.rect)
      if x <= 2.:
        self.assertTrue(docked)
      self.assertEqual(points.shape, (4, 2))  # ground-perspective trapezoid
      self.assertGreater(points[:, 0].min(), self.rect.x)
      self.assertLess(points[:, 0].max(), self.rect.x + self.rect.width)
      self.assertGreater(points[:, 1].min(), self.rect.y)
      self.assertLess(points[:, 1].max(), self.rect.y + self.rect.height)
    unknown = StopMarker(None, 0., None, True)
    points, docked = stop_bar_geometry(unknown, 1.5, self.project, self.rect)
    self.assertTrue(docked)
    self.assertTrue(np.all(np.isfinite(points)))
    self.assertAlmostEqual(np.ptp(points[:, 0]), 450.)
    self.assertAlmostEqual(np.ptp(points[:, 1]), 24 * min(self.rect.width / 1080., self.rect.height / 540.))

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

  def test_rounded_bar_fans_face_the_camera_with_either_input_winding(self):
    # raylib DrawTriangleFan submits (center, i, i+1) with back-face culling.
    # Screen Y is downward, so every visible triangle needs negative area.
    for position in ((21.52, 0., 0.), (1.52, 0., 0.), None):
      marker = StopMarker(position, 0., 0., True, 'STOP TARGET')
      points, _ = stop_bar_geometry(marker, 1.5, self.project, self.rect)
      for vertices in (points, points[::-1]):
        fan = np.asarray(rounded_polygon(vertices, 4.))
        a, b = fan[1:-1] - fan[0], fan[2:] - fan[0]
        areas = a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]
        self.assertTrue(np.all(areas <= 1e-6), areas)
        self.assertLess(areas.sum(), -1.)
