import unittest
from types import SimpleNamespace as NS
import numpy as np
from openpilot.common.transformations.camera import DEVICE_CAMERAS, view_frame_from_device_frame
from openpilot.common.transformations.orientation import rot_from_euler
from openpilot.selfdrive.ui.tests.helpers import FakeSM
from openpilot.selfdrive.ui.onroad.camera_zoom import CameraZoom, camera_inputs_valid


class TestCameraZoom(unittest.TestCase):
  def test_engagement_zooms_in_at_every_speed_even_with_invalid_speed(self):
    for speed in (0., 4., 7., 11., 35., float('nan')):
      z = CameraZoom()
      z.request(False, 0., True, 100.)
      z.update(True, 100.)
      for t in np.arange(100.05, 101.1, .05):
        z.update(True, t)
      self.assertEqual(z.transition, 0.)
      z.request(True, speed, True, 101.1)
      self.assertFalse(z.wide_requested)
      self.assertTrue(z.use_wide_stream(True, True))
      for t in np.arange(101.15, 102.3, .05):
        z.update(True, t)
      self.assertEqual(z.transition, 1.)
      self.assertFalse(z.use_wide_stream(True, True))

  def test_disengaged_low_speed_view_and_grace(self):
    z = CameraZoom()
    z.request(None, 0., True, 100.)
    self.assertFalse(z.wide_requested)
    z.request(False, 4., True, 100.)
    z.request(False, 7., True, 100.1)
    self.assertTrue(z.wide_requested)
    z.request(None, 7., True, 100.5)
    self.assertTrue(z.wide_requested)
    z.request(None, 7., True, 101.2)
    self.assertFalse(z.wide_requested)
    z.request(False, 4., False, 101.3)
    self.assertFalse(z.wide_requested)

  def test_fresh_engagement_does_not_depend_on_longitudinal_mode(self):
    sm = FakeSM(selfdriveState=NS(enabled=True), carState=NS())
    self.assertTrue(camera_inputs_valid(sm, 1, 100.))
    sm['selfdriveState'].enabled = False
    self.assertFalse(camera_inputs_valid(sm, 1, 100.))
    self.assertIsNone(camera_inputs_valid(sm, 11, 100.))
    self.assertIsNone(camera_inputs_valid(sm, 1, 101.))

  def test_calibrated_crop_matches_at_sensor_handoff(self):
    for device, width, height, narrow_zoom, wide_zoom in (('tici', 2160, 1080, 1.1, 2.), ('mici', 420, 960, .8, 1.05)):
      for sensor in ('ar0231', 'os04c10'):
        config = DEVICE_CAMERAS[device, sensor]
        z = CameraZoom()
        nc = view_frame_from_device_frame @ rot_from_euler([.01, .02, -.015])
        wc = view_frame_from_device_frame @ rot_from_euler([.01, .025, -.01])
        nz, nx, ny = z.framing(False, config, nc, wc, narrow_zoom, wide_zoom, width, height)
        wz, wx, wy = z.framing(True, config, nc, wc, narrow_zoom, wide_zoom, width, height)
        self.assertAlmostEqual(nz * config.narrow_road.focal_length, wz * config.wide_road.focal_length)
        narrow_pixel = np.append(config.narrow_road.intrinsics[:2, 2] + np.array([nx, ny]) / nz, 1.)
        ray = config.wide_road.intrinsics @ wc @ nc.T @ config.narrow_road.intrinsics_inv @ narrow_pixel
        expected = (ray[:2] / ray[2] - config.wide_road.intrinsics[:2, 2]) * wz
        self.assertTrue(np.allclose(expected, [wx, wy], atol=1e-6))
