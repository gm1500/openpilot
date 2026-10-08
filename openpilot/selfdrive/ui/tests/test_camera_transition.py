"""Real camera switch/matrix code with only frame sources and GPU uploads replaced."""
import importlib
import os
import unittest
from contextlib import ExitStack
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import numpy as np
import pyray as rl

from openpilot.common.transformations.camera import DEVICE_CAMERAS, view_frame_from_device_frame
from openpilot.common.transformations.orientation import rot_from_euler
from openpilot.selfdrive.controls.tests.test_conditional_experimental import FakeSM
from openpilot.selfdrive.ui.onroad.camera_zoom import CameraZoom


class FrameClient:
  num_buffers = 4

  def __init__(self, *args, **kwargs):
    self.connected = True
    self.frames = []

  def is_connected(self):
    return self.connected

  def connect(self, _):
    return self.connected

  def recv(self, timeout_ms=0):
    return self.frames.pop(0) if self.frames else None


class TestCameraTransition(unittest.TestCase):
  def make_view(self, device, stack):
    package = 'openpilot.selfdrive.ui.' + ('mici.' if device == 'mici' else '') + 'onroad'
    # Avoid opening a window just to query monitor scaling in headless tests.
    with patch.dict(os.environ, {'SCALE': '1'}):
      mod = importlib.import_module(package + '.augmented_road_view')
      base = importlib.import_module(package + '.cameraview')
    view = mod.AugmentedRoadView.__new__(mod.AugmentedRoadView)
    view.texture_y = view.texture_uv = view.egl_texture = view.shader = None
    view._name, view._stream_type = 'camerad', mod.NARROW_ROAD_CAM
    view.available_streams = [mod.NARROW_ROAD_CAM, mod.WIDE_CAM]
    view.frame, view.client = NS(width=1928, height=1208), FrameClient()
    view._target_client = view._target_stream_type = None
    view._switching = False
    view.last_connection_attempt = 0.
    view._last_target_connection_attempt = float('-inf')
    view._last_stream_check = float('-inf')
    view._fade = None
    view._crossfade_enabled = True
    view._last_frame_at = 100.
    view._texture_needs_update = True
    view.egl_images = {}
    view._initialize_textures = lambda: None  # no GPU context needed for stream/projection checks
    view._camera_zoom = CameraZoom()
    view.device_camera = DEVICE_CAMERAS[device, 'ar0231']
    view._content_rect = rl.Rectangle(0, 0, 2160 if device == 'tici' else 420, 1080 if device == 'tici' else 960)
    view.view_from_calib = view_frame_from_device_frame.copy()
    view.view_from_wide_calib = view_frame_from_device_frame.copy()
    view._matrix_cache_key = view._cached_matrix = None
    view.transforms = []
    view.model_renderer = view._model_renderer = NS(set_transform=view.transforms.append)
    sm = FakeSM(selfdriveState=NS(experimentalMode=True, conditionalExperimental=False, enabled=True),
                 carState=NS(vEgo=0., gasPressed=False, brakePressed=False), carControl=NS(longActive=True),
                 longitudinalPlan=NS(conditionalExperimental=NS(e2eEnabled=False)), extrinsicsCalibration=NS())
    stack.enter_context(patch.object(mod, 'ui_state', NS(sm=sm, started_frame=1)))
    stack.enter_context(patch.object(base, 'ui_state', NS(is_onroad=lambda: False)))
    stack.enter_context(patch.object(base, 'VisionIpcClient', FrameClient))
    clock = stack.enter_context(patch.object(mod.time, 'monotonic', return_value=100.))
    stack.callback(view.close)

    def step():
      clock.return_value += .05
      for service in sm.services:
        sm.recv_time[service] = clock.return_value
        sm.logMonoTime[service] = int((clock.return_value - 1e-6) * 1e9)
      view._switch_stream_if_needed(sm)
      view._handle_switch()
      view._calc_frame_matrix(view._content_rect)

    return view, sm, step, mod

  def test_live_handoffs_keep_crop_and_overlay_aligned_with_delayed_target_frames(self):
    for device in ('tici', 'mici'):
      with self.subTest(device=device), ExitStack() as stack:
        view, sm, step, mod = self.make_view(device, stack)
        step()
        narrow = view.transforms[-1].copy()
        for _ in range(10):
          step()
        self.assertEqual(view.stream_type, mod.NARROW_ROAD_CAM)
        self.assertEqual(view._camera_zoom.transition, 1.)
        view._target_client.frames.append(view.frame)
        step()
        self.assertEqual(view.stream_type, mod.WIDE_CAM)
        np.testing.assert_allclose(view.transforms[-1], narrow)
        before = view._cached_matrix.copy()
        step()
        self.assertFalse(np.array_equal(view._cached_matrix, before))  # animation invalidates the projection cache
        for _ in range(20):
          step()
        self.assertEqual(view._camera_zoom.transition, 0.)
        sm['carState'].vEgo = 11.
        step()
        self.assertEqual(view.stream_type, mod.WIDE_CAM)
        self.assertFalse(view._switching)  # no narrow swap before the wide zoom-in completes
        for _ in range(20):
          step()
        self.assertEqual(view._camera_zoom.transition, 1.)
        self.assertTrue(view._switching)
        crop = view.transforms[-1].copy()
        for _ in range(5):
          step()
        self.assertEqual(view.stream_type, mod.WIDE_CAM)  # keep live wide view while narrow is delayed
        view._target_client.frames.append(view.frame)
        step()
        self.assertEqual(view.stream_type, mod.NARROW_ROAD_CAM)
        np.testing.assert_allclose(view.transforms[-1], crop)

  def test_mode_exit_cancels_pending_wide_connection(self):
    for device in ('tici', 'mici'):
      with self.subTest(device=device), ExitStack() as stack:
        view, sm, step, mod = self.make_view(device, stack)
        step()
        self.assertTrue(view._switching)
        view._target_client.frames.append(view.frame)
        sm['selfdriveState'].experimentalMode = False
        sm.valid['carState'] = False  # explicit mode-off must bypass the input grace
        step()
        self.assertFalse(view._switching)
        self.assertIsNone(view._target_client)
        self.assertEqual(view.stream_type, mod.NARROW_ROAD_CAM)

  def test_brief_invalid_input_does_not_lose_pending_zoom_in_hysteresis_band(self):
    for device in ('tici', 'mici'):
      for service in ('carState', 'selfdriveState'):
        with self.subTest(device=device, service=service), ExitStack() as stack:
          view, sm, step, mod = self.make_view(device, stack)
          step()  # wide requested below 18 km/h, waiting for its first frame
          target = view._target_client
          sm['carState'].vEgo = 7.
          sm.valid[service] = False
          step()
          sm.valid[service] = True
          step()
          self.assertIs(view._target_client, target)
          self.assertTrue(view._camera_zoom.wide_requested)
          target.frames.append(view.frame)
          step()
          self.assertEqual(view.stream_type, mod.WIDE_CAM)
          for _ in range(20):
            step()
          self.assertEqual(view._camera_zoom.transition, 0.)

  def test_stale_input_preserves_active_zoom_briefly_then_exits_and_recovers(self):
    for device in ('tici', 'mici'):
      with self.subTest(device=device), ExitStack() as stack:
        view, sm, step, mod = self.make_view(device, stack)
        step()
        view._target_client.frames.append(view.frame)
        step()
        for _ in range(20):
          step()
        sm['carState'].vEgo = 7.
        step()
        # Exercise the real freshness gate without refreshing message timestamps.
        mod.time.monotonic.return_value += .5
        view._switch_stream_if_needed(sm)
        view._calc_frame_matrix(view._content_rect)
        self.assertTrue(view._camera_zoom.wide_requested)
        self.assertEqual(view._camera_zoom.transition, 0.)
        step()  # fresh inputs in the hysteresis band retain wide
        self.assertTrue(view._camera_zoom.wide_requested)
        mod.time.monotonic.return_value += 1.01
        view._switch_stream_if_needed(sm)
        self.assertFalse(view._camera_zoom.wide_requested)
        for _ in range(22):
          step()
        self.assertTrue(view._switching)
        view._target_client.frames.append(view.frame)
        step()
        self.assertEqual(view.stream_type, mod.NARROW_ROAD_CAM)
        sm['carState'].vEgo = 4.
        step()
        view._target_client.frames.append(view.frame)
        step()
        self.assertEqual(view.stream_type, mod.WIDE_CAM)

  def test_zero_buffer_connections_retry_for_primary_and_target_streams(self):
    for device in ('tici', 'mici'):
      for target in (False, True):
        with self.subTest(device=device, target=target), ExitStack() as stack:
          view, _, step, mod = self.make_view(device, stack)
          frame = view.frame
          step()
          client = view._target_client if target else view.client
          client.num_buffers = 0
          client.connect = Mock(return_value=True)
          client.available_streams = Mock(return_value=[mod.NARROW_ROAD_CAM, mod.WIDE_CAM])
          retry = view._handle_switch if target else view._ensure_connection
          retry()
          client.connect.assert_called_once_with(False)
          retry()
          self.assertEqual(client.connect.call_count, 1)  # retry is throttled
          mod.time.monotonic.return_value += .25

          def connected(_, client=client, frame=frame):
            client.num_buffers = 4
            client.frames.append(frame)
            return True

          client.connect.side_effect = connected
          retry()
          self.assertEqual(client.connect.call_count, 2)
          if target:
            self.assertEqual(view.stream_type, mod.WIDE_CAM)
            self.assertIs(view.frame, frame)
          else:
            self.assertTrue(view._ensure_connection())

  def test_missing_wide_stream_is_rediscovered_without_primary_disconnect(self):
    for device in ('tici', 'mici'):
      with self.subTest(device=device), ExitStack() as stack:
        view, _, step, mod = self.make_view(device, stack)
        view.available_streams = [mod.NARROW_ROAD_CAM]
        view.client.available_streams = Mock(side_effect=[[], [mod.NARROW_ROAD_CAM, mod.WIDE_CAM]])
        step()
        self.assertFalse(view._switching)
        self.assertEqual(view.client.available_streams.call_count, 1)
        step()
        self.assertEqual(view.client.available_streams.call_count, 1)
        mod.time.monotonic.return_value += 1.
        step()
        self.assertTrue(view._switching)
        view._target_client.frames.append(view.frame)
        step()
        self.assertEqual(view.stream_type, mod.WIDE_CAM)

  def test_calibrated_handoff_preserves_the_actual_clamped_narrow_viewport(self):
    for device in ('tici', 'mici'):
      for sensor in ('ar0231', 'os04c10'):
        with self.subTest(device=device, sensor=sensor), ExitStack() as stack:
          view, sm, step, mod = self.make_view(device, stack)
          view.device_camera = DEVICE_CAMERAS[device, sensor]
          device_from_calib = rot_from_euler([.005, .14, -.011])
          view.view_from_calib = view_frame_from_device_frame @ device_from_calib
          view.view_from_wide_calib = view_frame_from_device_frame @ rot_from_euler([-.001, .036, -.028]) @ device_from_calib
          step()
          width, height = view._content_rect.width, view._content_rect.height
          pixels = np.array([[width / 2, height / 2, 1.], [width * .25, height * .25, 1.],
                             [width * .75, height * .75, 1.], [width * .5, height * .85, 1.]]).T
          rays = np.linalg.solve(view.transforms[-1], pixels)
          view._target_client.frames.append(view.frame)
          step()
          projected = view.transforms[-1] @ rays
          projected /= projected[2]
          # Same central ray at the first live wide frame. Small residuals away
          # from it reflect perspective rotation, not a large framing jump.
          np.testing.assert_allclose(projected[:, 0], pixels[:, 0], atol=1e-7)
          self.assertLess(np.max(np.abs(projected - pixels)), 20.)
          for _ in range(20):
            step()
          sm['carState'].vEgo = 11.
          for _ in range(22):
            step()
          crop = view.transforms[-1].copy()
          view._target_client.frames.append(view.frame)
          step()
          narrow_ray = np.linalg.solve(view.transforms[-1], pixels[:, 0])
          returned = crop @ narrow_ray
          np.testing.assert_allclose(returned / returned[2], pixels[:, 0], atol=1e-7)

  def test_conditional_selection_allows_camera_between_e2e_activations_and_offroad_resets_switch(self):
    for device in ('tici', 'mici'):
      with self.subTest(device=device), ExitStack() as stack:
        view, sm, step, mod = self.make_view(device, stack)
        sm['selfdriveState'].experimentalMode = False
        step()
        self.assertFalse(view._switching)
        sm['selfdriveState'].conditionalExperimental = True
        step()
        self.assertFalse(sm['longitudinalPlan'].conditionalExperimental.e2eEnabled)
        self.assertTrue(view._switching)
        target = view._target_client
        sm['longitudinalPlan'].conditionalExperimental.e2eEnabled = True
        step()
        sm['longitudinalPlan'].conditionalExperimental.e2eEnabled = False
        step()
        self.assertTrue(view._switching)
        self.assertIs(view._target_client, target)
        view._offroad_transition()
        self.assertFalse(view._switching)
        self.assertIsNone(view._target_client)
        self.assertEqual(view._camera_zoom.transition, 1.)
        self.assertFalse(view._camera_zoom.wide_requested)

  def test_lost_wide_stream_can_recover_without_waiting_for_animation(self):
    for device in ('tici', 'mici'):
      with self.subTest(device=device), ExitStack() as stack:
        view, sm, step, mod = self.make_view(device, stack)
        step()
        view._target_client.frames.append(view.frame)
        step()
        for _ in range(20):
          step()
        self.assertEqual(view._camera_zoom.transition, 0.)
        view.client.connected = False
        view.available_streams = [mod.NARROW_ROAD_CAM]
        step()
        self.assertTrue(view._switching)
        view._target_client.frames.append(view.frame)
        step()
        self.assertEqual(view.stream_type, mod.NARROW_ROAD_CAM)


class TestSignPulse(unittest.TestCase):
  def test_override_and_missing_limit_clear_animation_clock_but_new_map_speed_pulses(self):
    with patch.dict(os.environ, {'SCALE': '1'}):
      mod = importlib.import_module('openpilot.selfdrive.ui.onroad.speed_limit')
    ui = NS(map_cruise_pulsing=True, map_cruise_enabled=True, map_cruise_supported=True, map_cruise_state='active',
            map_cruise_e2e=False, speed_limit=70/3.6, speed_limit_is_advisory=False, is_metric=True,
            sm={'mapSpeedLimit': NS(positionEstimated=False)})
    with patch.object(mod, 'ui_state', ui), patch.object(mod, 'draw_speed_limit') as draw, patch.object(mod.rl, 'get_time', return_value=100.):
      button = mod.SpeedLimitButton()
      rect = rl.Rectangle(0, 0, 180, 204)
      button._render(rect)
      self.assertEqual(button._pulse_since, 100.)
      ui.map_cruise_e2e = True
      button._render(rect)
      self.assertIsNone(button._pulse_since)
      self.assertIsNone(draw.call_args.args[6])
      ui.map_cruise_e2e, ui.speed_limit = False, None
      button._render(rect)
      self.assertIsNone(button._pulse_since)
      ui.speed_limit = 50/3.6
      button._render(rect)
      self.assertEqual(button._pulse_since, 100.)
