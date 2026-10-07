"""Real camera switch/matrix code with only frame sources and GPU uploads replaced."""
import importlib
import os
import unittest
from contextlib import ExitStack
from types import SimpleNamespace as NS
from unittest.mock import patch

import numpy as np
import pyray as rl

from openpilot.common.transformations.camera import DEVICE_CAMERAS, view_frame_from_device_frame
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
        step()
        self.assertFalse(view._switching)
        self.assertIsNone(view._target_client)
        self.assertEqual(view.stream_type, mod.NARROW_ROAD_CAM)

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
