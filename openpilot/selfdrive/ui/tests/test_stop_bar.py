import unittest
import sys
from types import SimpleNamespace as NS
from unittest.mock import patch

import numpy as np
import raylib_nb as rl

from openpilot.selfdrive.ui.onroad.lead_geometry import lead_fill_alpha
from openpilot.selfdrive.ui.onroad.model_stop import ModelStop, StopMarker
from openpilot.selfdrive.ui.onroad.stop_bar import _flat_fan, _inset_quad, draw_stop_bar, stop_bar_geometry
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
    self.sm['longitudinalPlan'].stopTarget.approaching = False
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

  def test_horizon_target_with_no_velocity_stop_uses_published_distance_for_overlap(self):
    prediction = model(speed=10.)
    prediction.velocity.x = np.maximum(.7, prediction.velocity.x)
    self.sm['modelV2'] = prediction
    self.sm['longitudinalPlan'].stopTarget.distance = 10.
    self.sm['longitudinalPlan'].e2eStopDistance = 20.
    self.set_lead(40.)  # real-lead ego stop at 34 m; model target is separate
    self.stop.update(self.sm, 1, 100.02)
    self.assertIsNotNone(self.stop.marker)
    self.assertEqual(self.stop.marker.distance, 10.)
    self.set_lead(31.)  # real-lead ego stop at 25 m overlaps the raw 20 m target
    self.stop.update(self.sm, 1, 100.02)
    self.assertIsNone(self.stop.marker)

  def test_trajectory_style_promotes_on_stop_and_persists_until_brake_release(self):
    self.set_trajectory_only()
    self.assertTrue(self.stop.marker.trajectory_only)
    self.sm['modelV2'].velocity.x = model().velocity.x  # sustained stop point, still no shouldStop flag
    self.sm['longitudinalPlan'].stopTarget.active = True
    self.sm['longitudinalPlan'].stopTarget.approaching = False
    self.sm['longitudinalPlan'].stopTarget.distance = 16.
    self.stop.update(self.sm, 1, 100.01)
    self.assertFalse(self.stop.marker.trajectory_only)
    self.sm['modelV2'].velocity.x = np.maximum(.7, self.sm['modelV2'].velocity.x)
    self.stop.update(self.sm, 1, 100.02)
    self.assertFalse(self.stop.marker.trajectory_only)  # braking retains the stop style
    self.release_target()
    self.stop.update(self.sm, 1, 100.03)
    self.assertTrue(self.stop.marker.held)
    self.assertFalse(self.stop.marker.trajectory_only)
    self.sm['carOutput'].actuatorsOutput.brake = 0.
    self.stop.update(self.sm, 1, 100.04)
    self.assertIsNone(self.stop.marker)

  def test_planner_hold_promotes_trajectory_style_without_model_stop_flag(self):
    self.set_trajectory_only()
    self.assertTrue(self.stop.marker.trajectory_only)
    self.sm['longitudinalPlan'].stopTarget.approaching = False
    self.sm['longitudinalPlan'].stopTarget.holding = True
    self.stop.update(self.sm, 1, 100.01)
    self.assertFalse(self.stop.marker.trajectory_only)

  def set_trajectory_only(self):
    self.sm['modelV2'] = model()
    self.sm['modelV2'].velocity.x = np.maximum(.7, self.sm['modelV2'].velocity.x)
    self.sm['longitudinalPlan'].stopTarget.active = False
    self.sm['longitudinalPlan'].stopTarget.approaching = True
    self.sm['longitudinalPlan'].stopTarget.approachDistance = 16.
    self.sm['longitudinalPlan'].stopTarget.distance = -1.
    self.stop = ModelStop()
    self.stop.update(self.sm, 1, 100.)

  def test_green_approach_uses_preview_distance_and_clears_when_assistance_releases(self):
    self.set_trajectory_only()
    self.assertEqual(self.stop.marker.label, 'E2E APPROACH')
    self.assertEqual(self.stop.marker.distance, 16.)
    self.assertFalse(self.sm['longitudinalPlan'].stopTarget.active)
    self.release_target()
    self.stop.update(self.sm, 1, 100.01)
    self.assertIsNone(self.stop.marker)  # applied brakes cannot retain a green control indication

  def test_green_projection_clamps_to_new_path_without_moving_confirmed_target(self):
    self.set_trajectory_only()
    self.sm['modelV2'].position.x *= .8  # newer model ends before the planner's 16 m preview
    self.sm.stamp(100.05)
    self.stop.update(self.sm, 1, 100.05)
    self.assertIsNotNone(self.stop.marker.position)
    self.assertAlmostEqual(self.stop.marker.distance, 12.8)
    self.assertAlmostEqual(self.stop.marker.position[0], 12.8 + 1.52)
    self.assertEqual(self.sm['longitudinalPlan'].stopTarget.approachDistance, 16.)
    self.sm['longitudinalPlan'].stopTarget.active = True
    self.sm['longitudinalPlan'].stopTarget.approaching = False
    self.sm['longitudinalPlan'].stopTarget.distance = 15.
    self.stop.update(self.sm, 1, 100.06)
    self.assertFalse(self.stop.marker.trajectory_only)
    self.assertEqual(self.stop.marker.distance, 15.)  # real stop keeps its exact planner distance
    self.assertAlmostEqual(self.stop.marker.position[0], 12.8 + 1.52)

  def test_adjacent_path_endpoints_do_not_send_confirmed_stop_to_dock(self):
    self.sm['carState'].vEgo = 8.
    self.sm['longitudinalPlan'].stopTarget.distance = 30.
    for frame, length in enumerate((31., 29., 30.2, 28.8, 32.)):
      prediction = model()
      prediction.position.x *= length / 16.
      self.sm['modelV2'] = prediction
      now = 100. + frame * .05
      self.sm.stamp(now)
      self.stop.update(self.sm, 1, now)
      marker = self.stop.marker
      self.assertEqual(marker.distance, 30.)
      self.assertAlmostEqual(marker.position[0], min(30., length) + 1.52)
      def project(x, y, z):
        return (1080. + 900. * y / x, 300. + 900. * z / x)
      _, docked = stop_bar_geometry(marker, 1.5, project, self.rect)
      self.assertFalse(docked)
      self.assertEqual(self.sm['longitudinalPlan'].stopTarget.distance, 30.)
    # A truly nearer planner target moves immediately, with no smoothing lag.
    self.sm['longitudinalPlan'].stopTarget.distance = 12.
    self.stop.update(self.sm, 1, now + .01)
    self.assertAlmostEqual(self.stop.marker.position[0], 13.52)
    self.assertEqual(self.stop.marker.distance, 12.)

  def test_green_preview_smooths_range_changes_in_both_directions(self):
    self.set_trajectory_only()
    self.sm['modelV2'] = model(speed=20.)
    self.sm['longitudinalPlan'].stopTarget.approachDistance = 60.
    self.stop.reset()
    self.stop.update(self.sm, 1, 100.)
    self.assertEqual(self.stop.marker.distance, 60.)
    self.sm['longitudinalPlan'].stopTarget.approachDistance = 40.
    self.sm.stamp(100.05)
    self.stop.update(self.sm, 1, 100.05)
    closer = self.stop.marker.distance
    self.assertGreater(closer, 55.)
    self.assertLess(closer, 60.)
    self.sm['longitudinalPlan'].stopTarget.approachDistance = 75.
    self.sm.stamp(100.10)
    self.stop.update(self.sm, 1, 100.10)
    self.assertGreater(self.stop.marker.distance, closer)
    self.assertLess(self.stop.marker.distance, 65.)

  def test_green_bridges_brief_withdrawal_then_fades_and_expires(self):
    self.set_trajectory_only()
    self.sm['modelV2'].action = NS(desiredAcceleration=-1.)
    self.release_target()
    for now, opacity in ((100.1, 1.), (100.2, 1.), (100.3, 2. / 3.)):
      self.sm.stamp(now)
      self.stop.update(self.sm, 1, now)
      self.assertTrue(self.stop.marker.trajectory_only)
      self.assertFalse(self.stop.marker.held)
      self.assertAlmostEqual(self.stop.marker.opacity, opacity)
    self.sm.stamp(100.41)
    self.stop.update(self.sm, 1, 100.41)
    self.assertIsNone(self.stop.marker)

  def test_green_retention_clears_for_departure_override_or_invalid_data(self):
    for change in ('departure', 'gas', 'brake', 'disabled', 'control_off', 'reverse', 'stale', 'invalid_path', 'lead'):
      with self.subTest(change=change):
        self.setUp()
        self.set_trajectory_only()
        self.sm['modelV2'].action = NS(desiredAcceleration=-1.)
        self.release_target()
        self.sm.stamp(100.1)
        if change == 'departure':
          self.sm['modelV2'].action.desiredAcceleration = .5
        elif change in ('gas', 'brake'):
          setattr(self.sm['carState'], change + 'Pressed', True)
        elif change == 'disabled':
          self.sm['selfdriveState'].enabled = False
        elif change == 'control_off':
          self.sm['carControl'].longActive = False
        elif change == 'reverse':
          self.sm['carState'].gearShifter = 'reverse'
        elif change == 'invalid_path':
          self.sm['modelV2'].position.x[5] = np.nan
        elif change == 'lead':
          self.set_lead(20.)
        self.stop.update(self.sm, 1, 100.5 if change == 'stale' else 100.1)
        self.assertIsNone(self.stop.marker)

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
    self.set_lead(11.)  # held position 0 m; real-lead stop 5 m: 5 m overlap
    self.stop.update(self.sm, 1, 100.02)
    self.assertIsNone(self.stop.marker)

  def test_green_preview_yields_to_physical_moving_lead_despite_distant_projected_stop(self):
    for second in (False, True):
      with self.subTest(second=second):
        self.setUp()
        self.set_trajectory_only()
        self.set_lead(20., second)
        lead = self.sm['radarState'].leadTwo if second else self.sm['radarState'].leadOne
        lead.vLead = 20.  # projected lead stop is far beyond the green preview
        self.stop.update(self.sm, 1, 100.02)
        self.assertIsNone(self.stop.marker)  # 16 m preview is within physical clearance
        lead.dRel = 22.1
        self.stop.update(self.sm, 1, 100.02)
        self.assertIsNotNone(self.stop.marker)
        self.assertTrue(self.stop.marker.trajectory_only)

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

  def test_lane_wide_bar_tracks_exact_stop_point_and_camera_perspective(self):
    scale = min(self.rect.width / 1080., self.rect.height / 540.)
    for distance, yaw in ((20., 0.), (60., 0.), (20., .4), (20., -.4)):
      x = distance + 1.52
      marker = StopMarker((x, 0., 0.), yaw, distance)
      points, docked = stop_bar_geometry(marker, 1.5, self.project, self.rect)
      self.assertFalse(docked)
      np.testing.assert_allclose(points[:2].mean(axis=0), self.project(x, 0., 1.5))
      self.assertAlmostEqual(points[1, 0] - points[0, 0], 900 * 3.6 / x)
      self.assertEqual(points[0, 1], points[1, 1])
      self.assertEqual(points[2, 1], points[3, 1])
      center_depth = np.linalg.norm(points[[2, 3]].mean(axis=0) - points[:2].mean(axis=0))
      self.assertLessEqual(center_depth, 10 * scale + 1e-6)
      self.assertGreaterEqual(center_depth, 3 * scale - 1e-6)
      if yaw == 0.:
        depth = self.project(x - min(6., .25 * x), 0., 1.5)[1] - self.project(x, 0., 1.5)[1]
        self.assertAlmostEqual(np.ptp(points[:, 1]), np.clip(depth, 3 * scale, 10 * scale))

  def test_curved_path_controls_near_edge_without_moving_target_anchor(self):
    marker = StopMarker((30., 3., .2), .2, 200.)
    path = np.array([(0., 0., 0.), (24., 1., .1), (30., 3., .2), (60., 10., .3)])
    points, docked = stop_bar_geometry(marker, 1.5, self.project, self.rect, path=path, edges=(-1.6, 2.))
    self.assertFalse(docked)
    np.testing.assert_allclose(points[0], self.project(30., 1.4, 1.7))
    np.testing.assert_allclose(points[1], self.project(30., 5., 1.7))
    near_center = np.mean([self.project(24., -.6, 1.6), self.project(24., 3., 1.6)], axis=0)
    direction = near_center - points[:2].mean(axis=0)
    actual_direction = points[[3, 2]].mean(axis=0) - points[:2].mean(axis=0)
    np.testing.assert_allclose(actual_direction / np.linalg.norm(actual_direction), direction / np.linalg.norm(direction))
    self.assertEqual(marker.distance, 200.)  # published range never becomes a geometry estimate

  def test_ground_bar_follows_camera_roll(self):
    marker = StopMarker((30., 0., 0.), 0., 28.48)
    def project_roll(x, y, z):
      px, py = self.project(x, y, z)
      return px, py + .1 * (px - 1080)
    points, docked = stop_bar_geometry(marker, 1.5, project_roll, self.rect)
    self.assertFalse(docked)
    self.assertAlmostEqual(points[1, 1] - points[0, 1], .1 * (points[1, 0] - points[0, 0]))

  def test_red_fill_stays_inside_rotated_or_skewed_yellow_outline(self):
    for points in (np.array([(30., 100.), (300., 70.), (302., 77.), (27., 107.)]),
                   np.array([(30., 100.), (300., 160.), (304., 166.), (25., 106.)])):
      for vertices in (points, points[::-1]):
        inner = _inset_quad(vertices, 3.)
        directions = np.roll(vertices, -1, axis=0) - vertices
        normals = np.column_stack((-directions[:, 1], directions[:, 0]))
        if np.sum(normals * (vertices.mean(axis=0) - vertices)) < 0:
          normals *= -1
        for edge, normal in zip(vertices, normals, strict=True):
          self.assertTrue(np.all((inner - edge) @ normal > 0.))

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
    self.assertAlmostEqual(np.ptp(points[:, 1]), 10 * min(self.rect.width / 1080., self.rect.height / 540.))

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

  def test_trajectory_bar_is_lane_wide_green_with_the_same_profile_and_thickness(self):
    for position in ((21.52, 0., 0.), None):
      marker = StopMarker(position, 0., 20., label='STOP TARGET', trajectory_only=True)
      points, _ = stop_bar_geometry(marker, 1.5, self.project, self.rect)
      full, _ = stop_bar_geometry(StopMarker(position, 0., 20.), 1.5, self.project, self.rect)
      np.testing.assert_array_equal(points, full)
      with patch.dict(sys.modules, {'openpilot.system.ui.lib.text_measure': NS(measure_text_cached=lambda *args: NS(x=240., y=30.))}), \
           patch.object(rl, 'draw_triangle_fan') as polygon, patch.object(rl, 'draw_rectangle_rounded'), patch.object(rl, 'draw_text_ex'):
        draw_stop_bar(marker, 8., 1.5, self.project, self.rect, None, True)
        self.assertEqual(polygon.call_count, int(position is not None))
        if position is not None:
          color = polygon.call_args.args[2]
          self.assertEqual((color.r, color.g, color.b, color.a), (23, 134, 68, 255))

  def test_green_fade_applies_to_bar_and_label_and_never_draws_in_stop_dock(self):
    marker = StopMarker((61.52, 0., 0.), 0., 60., label='E2E APPROACH', trajectory_only=True, opacity=.5)
    with patch.dict(sys.modules, {'openpilot.system.ui.lib.text_measure': NS(measure_text_cached=lambda *args: NS(x=240., y=30.))}), \
         patch.object(rl, 'draw_triangle_fan') as polygon, patch.object(rl, 'draw_rectangle_rounded') as background, \
         patch.object(rl, 'draw_text_ex') as text:
      draw_stop_bar(marker, 8., 1.5, self.project, self.rect, None, True)
      self.assertEqual(polygon.call_args.args[2].a, 128)
      self.assertEqual(text.call_args.args[-1].a, 128)
      self.assertEqual(background.call_args.args[-1].a, 95)
      polygon.reset_mock()
      draw_stop_bar(marker, 8., 1.5, lambda *p: None, self.rect, None, True)
      self.assertEqual(polygon.call_count, 0)

  def test_flat_bar_fans_face_the_camera_with_either_input_winding(self):
    # raylib DrawTriangleFan submits (center, i, i+1) with back-face culling.
    # Screen Y is downward, so every visible triangle needs negative area.
    for position in ((21.52, 0., 0.), (1.52, 0., 0.), None):
      marker = StopMarker(position, 0., 0., True, 'STOP TARGET')
      points, _ = stop_bar_geometry(marker, 1.5, self.project, self.rect)
      for vertices in (points, points[::-1]):
        fan = np.asarray(_flat_fan(vertices))
        self.assertEqual(len(fan), 6)
        a, b = fan[1:-1] - fan[0], fan[2:] - fan[0]
        areas = a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]
        self.assertTrue(np.all(areas <= 1e-6), areas)
        self.assertLess(areas.sum(), -1.)
