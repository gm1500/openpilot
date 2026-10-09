import unittest
from types import SimpleNamespace as NS

from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.selfdrive.controls.tests.test_conditional_experimental import FakeSM
from openpilot.selfdrive.ui.onroad.camera_zoom import CameraZoom, camera_inputs_valid


class TestCameraZoom(unittest.TestCase):
  def setUp(self):
    self.zoom = CameraZoom()
    self.now = 100.

  def advance(self, frames, fps=20, wide=True):
    for _ in range(frames):
      self.now += 1 / fps
      self.zoom.update(wide, self.now)

  def test_narrow_to_wide_matches_crop_before_animating_and_waits_before_return(self):
    self.zoom.request(True, 4., True, self.now)
    self.assertTrue(self.zoom.use_wide_stream(False, True))
    self.advance(10, wide=False)  # delayed connection must not consume the animation
    self.assertEqual(self.zoom.transition, 1.)
    self.advance(1)
    self.assertEqual(self.zoom.transition, 1.)
    self.advance(10)
    self.assertGreater(self.zoom.transition, 0.)
    self.assertLess(self.zoom.transition, 1.)
    self.advance(10)
    self.assertEqual(self.zoom.transition, 0.)
    self.zoom.request(True, 11., True, self.now)
    self.advance(10)
    self.assertTrue(self.zoom.use_wide_stream(True, True))
    self.advance(10)
    self.assertEqual(self.zoom.transition, 1.)
    self.assertFalse(self.zoom.use_wide_stream(True, True))

  def test_same_easing_at_20_and_60_fps(self):
    positions = []
    for fps in (20, 60):
      self.setUp()
      self.zoom.request(True, 0., True, self.now)
      self.zoom.update(True, self.now)
      self.advance(fps // 2, fps)
      positions.append(self.zoom.transition)
    self.assertAlmostEqual(*positions, places=10)

  def test_reversal_keeps_current_crop_and_hysteresis_keeps_requested_direction(self):
    self.zoom.request(True, 0., True, self.now)
    self.zoom.update(True, self.now)
    self.advance(10)
    before = self.zoom.transition
    self.zoom.request(True, 11., True, self.now)
    self.zoom.request(True, 7., True, self.now)
    self.assertFalse(self.zoom.wide_requested)
    self.assertEqual(self.zoom.transition, before)
    self.advance(1)
    self.assertGreater(self.zoom.transition, before)
    self.zoom.request(True, 4., True, self.now)
    self.zoom.request(True, 7., True, self.now)
    self.assertTrue(self.zoom.wide_requested)
    before = self.zoom.transition
    self.advance(1)
    self.assertLess(self.zoom.transition, before)

  def test_disabled_mode_returns_to_narrow_and_missing_camera_does_not_wait_for_animation(self):
    self.zoom.request(True, 0., True, self.now)
    self.zoom.update(True, self.now)
    self.advance(20)
    self.zoom.request(False, 0., True, self.now)
    self.assertFalse(self.zoom.wide_requested)
    self.assertTrue(self.zoom.use_wide_stream(True, True))  # finish the exit
    self.assertFalse(self.zoom.use_wide_stream(True, False))  # failed stream: recover immediately
    self.advance(20)
    self.assertFalse(self.zoom.use_wide_stream(True, True))
    for speed in (0., 4., 7., 11.):
      self.zoom.request(False, speed, True, self.now)
      self.assertFalse(self.zoom.use_wide_stream(False, True))
    self.zoom.request(True, 0., False, self.now)
    self.assertFalse(self.zoom.wide_requested)

  def test_reset_and_clock_discontinuity_do_not_jump_animation(self):
    self.zoom.request(True, 0., True, self.now)
    self.zoom.update(True, self.now)
    before = self.zoom.update(True, self.now - 1)
    self.assertEqual(before, 1.)
    self.assertGreater(self.zoom.update(True, self.now + 10), .9)
    self.zoom.reset()
    self.assertEqual(self.zoom.transition, 1.)
    self.assertFalse(self.zoom.wide_requested)

  def test_unknown_inputs_never_start_zoom_and_only_briefly_preserve_an_existing_request(self):
    for enabled, speed in ((None, 0.), (True, float('nan'))):
      with self.subTest(enabled=enabled, speed=speed):
        self.zoom.reset()
        self.zoom.request(enabled, speed, True, 100.)
        self.assertFalse(self.zoom.wide_requested)
        self.zoom.request(True, 0., True, 100.)
        self.zoom.request(enabled, speed, True, 100.5)
        self.assertTrue(self.zoom.wide_requested)
        self.zoom.request(enabled, speed, True, 101.01)
        self.assertFalse(self.zoom.wide_requested)
        self.zoom.request(True, 0., True, 102.)
        self.zoom.reset()
        self.zoom.request(enabled, speed, True, 102.1)
        self.assertFalse(self.zoom.wide_requested)

  def test_explicit_mode_off_clears_grace_even_when_speed_is_invalid(self):
    self.zoom.request(True, 0., True, 100.)
    self.zoom.request(False, float('nan'), True, 100.05)
    self.assertFalse(self.zoom.wide_requested)
    self.zoom.request(None, 0., True, 100.1)
    self.assertFalse(self.zoom.wide_requested)

  def test_crop_matches_narrow_fov_at_handoff_for_both_devices_and_sensors(self):
    for device, width, height, narrow_zoom, wide_zoom in (('tici', 2160, 1080, 1.1, 2.), ('mici', 420, 960, .8, 1.05)):
      for sensor in ('ar0231', 'os04c10'):
        with self.subTest(device=device, sensor=sensor):
          config = DEVICE_CAMERAS[device, sensor]
          narrow = self.zoom.zoom(False, config, narrow_zoom, wide_zoom, width, height)
          wide = self.zoom.zoom(True, config, narrow_zoom, wide_zoom, width, height)
          self.assertAlmostEqual(narrow * config.narrow_road.focal_length, wide * config.wide_road.focal_length)


class TestCameraMode(unittest.TestCase):
  def setUp(self):
    self.sm = FakeSM(selfdriveState=NS(experimentalMode=False, conditionalExperimental=False, enabled=True),
                     carState=NS(gasPressed=False, brakePressed=False), carControl=NS(longActive=True),
                     longitudinalPlan=NS(conditionalExperimental=NS(e2eEnabled=False)))

  def test_zoom_available_in_all_modes_independent_of_e2e_activation(self):
    self.assertTrue(camera_inputs_valid(self.sm, 1, 100.))
    self.sm['selfdriveState'].conditionalExperimental = True
    self.assertTrue(camera_inputs_valid(self.sm, 1, 100.))
    self.sm['longitudinalPlan'].conditionalExperimental.e2eEnabled = True
    self.assertTrue(camera_inputs_valid(self.sm, 1, 100.))
    self.sm['longitudinalPlan'].conditionalExperimental.e2eEnabled = False
    self.sm['carState'].gasPressed = True
    self.assertTrue(camera_inputs_valid(self.sm, 1, 100.))
    self.sm['selfdriveState'].conditionalExperimental = False
    self.assertTrue(camera_inputs_valid(self.sm, 1, 100.))

  def test_full_experimental_preserves_original_selection_behavior(self):
    self.sm['selfdriveState'].experimentalMode = True
    self.assertTrue(camera_inputs_valid(self.sm, 1, 100.))

  def test_stale_or_previous_drive_mode_and_speed_cannot_start_animation(self):
    self.sm['selfdriveState'].experimentalMode = True
    self.assertFalse(camera_inputs_valid(self.sm, 11, 100.))
    self.assertIsNone(camera_inputs_valid(self.sm, 1, 101.))
    self.sm.valid['carState'] = False
    self.assertIsNone(camera_inputs_valid(self.sm, 1, 100.))
    self.sm['selfdriveState'].experimentalMode = False
    self.assertIsNone(camera_inputs_valid(self.sm, 1, 100.))
