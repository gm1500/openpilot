import importlib
import unittest
from contextlib import ExitStack
from types import SimpleNamespace as NS
from unittest.mock import patch

import numpy as np

from openpilot.selfdrive.ui.tests import test_camera_transition
from openpilot.common.transformations.orientation import rot_from_euler
from openpilot.common.transformations.camera import view_frame_from_device_frame


class TestCameraFade(unittest.TestCase):
  make_view = test_camera_transition.TestCameraTransition.make_view

  def setup_render(self, device, stack):
    view, sm, step, mod = self.make_view(device, stack)
    base = importlib.import_module('openpilot.selfdrive.ui.' + ('mici.' if device == 'mici' else '') + 'onroad.cameraview')
    draws = []
    view.frame.sequence = 0
    view._render_textures = lambda src, dst, camera, alpha: draws.append(
      (camera.stream_type, camera.frame.sequence, alpha, dst.width, dst.height))

    def tick():
      step()
      base.CameraView._render(view, view._content_rect)

    def queue(client, sequence):
      client.frames.append(NS(width=1928, height=1208, sequence=sequence))

    return view, sm, tick, queue, draws, mod, base

  def test_both_live_streams_crossfade_during_zoom_out_and_after_zoom_in(self):
    for device in ('tici', 'mici'):
      with self.subTest(device=device), ExitStack() as stack:
        view, sm, tick, queue, draws, mod, _ = self.setup_render(device, stack)
        narrow = view.client
        for i in range(10):
          queue(narrow, i)
          tick()
          self.assertIsNone(view._fade)  # target frames are delayed
          self.assertEqual(view._camera_zoom.transition, 1.)
        wide = view._target_client
        queue(wide, 100)
        queue(narrow, 10)
        tick()
        self.assertEqual(draws[-2][:3], (mod.WIDE_CAM, 100, 1.))
        self.assertEqual(draws[-1][:3], (mod.NARROW_ROAD_CAM, 10, 1.))
        initial_width = draws[-1][3]
        for i in range(1, 4):
          queue(wide, 100 + i)
          queue(narrow, 10 + i)
          tick()
          self.assertEqual(draws[-2][:3], (mod.WIDE_CAM, 100 + i, 1.))
          self.assertEqual(draws[-1][:2], (mod.NARROW_ROAD_CAM, 10 + i))
          self.assertLess(draws[-1][2], 1.)
          self.assertGreater(draws[-1][2], 0.)
        self.assertLess(draws[-1][3], initial_width)  # both crops follow the running zoom
        for i in range(20):
          queue(wide, 200 + i)
          tick()
        self.assertIsNone(view._fade)
        self.assertEqual(view._camera_zoom.transition, 0.)
        sm['carState'].vEgo = 11.
        for i in range(22):
          queue(wide, 300 + i)
          tick()
        self.assertEqual(view._camera_zoom.transition, 1.)
        self.assertEqual(view.stream_type, mod.WIDE_CAM)
        returning_narrow = view._target_client
        queue(returning_narrow, 400)
        queue(wide, 322)
        tick()
        self.assertEqual(draws[-2][:3], (mod.NARROW_ROAD_CAM, 400, 1.))
        self.assertEqual(draws[-1][:3], (mod.WIDE_CAM, 322, 1.))
        for i in range(1, 6):
          queue(returning_narrow, 400 + i)
          queue(wide, 322 + i)
          tick()
        self.assertIsNone(view._fade)
        self.assertEqual(view.stream_type, mod.NARROW_ROAD_CAM)

  def test_reversal_reuses_two_clients_and_preserves_blend_weights(self):
    for device in ('tici', 'mici'):
      with self.subTest(device=device), ExitStack() as stack:
        view, _, tick, queue, _, mod, base = self.setup_render(device, stack)
        narrow = view.client
        queue(narrow, 1)
        tick()
        wide = view._target_client
        queue(wide, 2)
        queue(narrow, 3)
        tick()
        queue(wide, 4)
        queue(narrow, 5)
        tick()
        previous_opacity = view._fade.opacity(mod.time.monotonic())
        with patch.object(base, 'VisionIpcClient') as constructor:
          view.switch_stream(mod.NARROW_ROAD_CAM)
          queue(narrow, 6)
          view._handle_switch()
          constructor.assert_not_called()
        self.assertIs(view.client, narrow)
        self.assertIs(view._fade.client, wide)
        self.assertAlmostEqual(view._fade.opacity(mod.time.monotonic()), 1. - previous_opacity)

  def test_stale_disconnected_and_offroad_outgoing_streams_are_released(self):
    for device in ('tici', 'mici'):
      for reason in ('stale', 'disconnected', 'offroad'):
        with self.subTest(device=device, reason=reason), ExitStack() as stack:
          view, _, tick, queue, _, mod, _ = self.setup_render(device, stack)
          narrow = view.client
          queue(narrow, 1)
          tick()
          wide = view._target_client
          queue(wide, 2)
          queue(narrow, 3)
          tick()
          self.assertIsNotNone(view._fade)
          if reason == 'offroad':
            view._offroad_transition()
          else:
            if reason == 'disconnected':
              narrow.connected = False
            else:
              mod.time.monotonic.return_value += .16
            view._render_fade(view._content_rect, view._cached_matrix)
          self.assertIsNone(view._fade)

  def test_outgoing_calibrated_centre_tracks_incoming_crop_through_fade(self):
    for device in ('tici', 'mici'):
      with self.subTest(device=device), ExitStack() as stack:
        view, _, tick, queue, _, mod, _ = self.setup_render(device, stack)
        view.view_from_calib = view_frame_from_device_frame @ rot_from_euler([.005, .14, -.011])
        view.view_from_wide_calib = view_frame_from_device_frame @ rot_from_euler([-.001, .036, -.028]) @ rot_from_euler([.005, .14, -.011])
        old = view.client
        tick()
        queue(view._target_client, 1)
        for i in range(4):
          queue(old, i)
          tick()
          rect = view._content_rect
          fade_matrix = view._calc_fade_matrix(rect, view._cached_matrix)
          cam = view.device_camera.narrow_road
          zoom = fade_matrix[0, 0] * rect.width / cam.width
          offset = -fade_matrix[:2, 2] * (rect.width / 2, rect.height / 2)
          ray = view.view_from_calib.T @ cam.intrinsics_inv @ np.append(cam.intrinsics[:2, 2] + offset / zoom, 1.)
          screen = view.transforms[-1] @ ray
          np.testing.assert_allclose(screen[:2] / screen[2], [rect.width / 2, rect.height / 2], atol=1e-7)

  def test_retained_gpu_resources_are_released_without_touching_incoming_camera(self):
    for device in ('tici', 'mici'):
      with self.subTest(device=device), ExitStack() as stack:
        view, _, tick, queue, _, _, base = self.setup_render(device, stack)
        tick()
        old_image, new_image = object(), object()
        old_texture, new_texture = NS(id=1), NS(id=2)
        view.egl_images, view.egl_texture = {0: old_image}, old_texture
        queue(view._target_client, 1)
        view._handle_switch()
        self.assertEqual(view._fade.egl_images, {0: old_image})
        view.egl_images, view.egl_texture = {0: new_image}, new_texture
        with patch.object(base, 'COMMA_HARDWARE', True), patch.object(base, 'destroy_egl_image') as destroy, \
             patch.object(base.rl, 'unload_texture') as unload:
          view._clear_fade()
          destroy.assert_called_once_with(old_image)
          unload.assert_called_once_with(old_texture)
        self.assertEqual(view.egl_images, {0: new_image})
        self.assertIs(view.egl_texture, new_texture)
        view.egl_images, view.egl_texture = {}, None

  def test_external_texture_draw_uses_per_camera_buffers_and_resets_alpha(self):
    for device in ('tici', 'mici'):
      with self.subTest(device=device), ExitStack() as stack:
        view, _, _, _, _, _, base = self.setup_render(device, stack)
        # Both producers use buffer index zero. They must import and bind
        # different images rather than accidentally drawing the same stream.
        def camera(fd, texture):
          return NS(frame=NS(idx=0, width=1928, height=1208, stride=2048, fd=fd, uv_offset=2048*1208),
                    egl_texture=NS(id=texture), egl_images={})
        incoming, outgoing = camera(10, 100), camera(11, 101)
        view._alpha_value, view._alpha_loc = base.rl.ffi.new('float[1]', [0.]), 7
        view._update_texture_color_filtering = lambda: None
        uniforms, bindings = [], []
        with patch.object(base, 'create_egl_image', side_effect=lambda w, h, s, fd, o: ('image', fd)), \
             patch.object(base, 'bind_egl_image_to_texture', side_effect=lambda tex, img, bindings=bindings: bindings.append((tex, img))), \
             patch.object(base.rl, 'begin_shader_mode'), patch.object(base.rl, 'end_shader_mode'), \
             patch.object(base.rl, 'draw_texture_pro'), \
             patch.object(base.rl, 'set_shader_value', side_effect=lambda shader, loc, value, kind, uniforms=uniforms: uniforms.append(float(value[0]))):
          rect = view._content_rect
          view._render_egl(rect, rect, incoming, 1.)
          view._render_egl(rect, rect, outgoing, .5)
          view._render_egl(rect, rect, incoming, 1.)
        self.assertEqual(bindings, [(100, ('image', 10)), (101, ('image', 11)), (100, ('image', 10))])
        self.assertEqual(uniforms, [1., .5, 1.])
