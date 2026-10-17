"""Short overlap of two live road streams; GPU resources remain per-stream."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np
import pyray as rl

from openpilot.cereal.visionipc import VisionStreamType
from msgq.visionipc import VisionIpcClient, VisionBuf
from openpilot.system.ui.lib.egl import EGLImage

CAMERA_FADE_TIME = .2
CAMERA_FADE_STALE = .15


@dataclass
class FadingCamera:
  client: VisionIpcClient
  stream_type: VisionStreamType
  frame: VisionBuf
  texture_y: rl.Texture | None
  texture_uv: rl.Texture | None
  egl_texture: rl.Texture | None
  egl_images: dict[int, EGLImage]
  _texture_needs_update: bool
  started_at: float
  last_frame_at: float

  def opacity(self, now: float) -> float:
    if not 0 <= now - self.started_at < CAMERA_FADE_TIME or not 0 <= now - self.last_frame_at <= CAMERA_FADE_STALE:
      return 0.
    progress = (now - self.started_at) / CAMERA_FADE_TIME
    return 1. - progress * progress * (3. - 2. * progress)


def aligned_camera_matrix(matrix, active_camera, active_calibration, other_camera, other_calibration, width, height):
  """Draw the outgoing lens at the incoming lens's current scale and centre ray.

  The outgoing narrow image may cover less than the full wide viewport during
  zoom-out. Do not clamp it back to filling the screen: the incoming wide image
  already covers the background and the shrinking overlap stays aligned.
  """
  zoom = matrix[0, 0] * width / active_camera.width
  offset = -matrix[:2, 2] * (width / 2, height / 2)
  pixel = np.append(active_camera.intrinsics[:2, 2] + offset / zoom, 1.)
  ray = other_camera.intrinsics @ other_calibration @ active_calibration.T @ active_camera.intrinsics_inv @ pixel
  if not np.all(np.isfinite(ray)) or ray[2] <= 1e-6:
    return None
  other_zoom = zoom * active_camera.focal_length / other_camera.focal_length
  other_offset = (ray[:2] / ray[2] - other_camera.intrinsics[:2, 2]) * other_zoom
  return np.array([[other_zoom * other_camera.width / width, 0., -other_offset[0] * 2 / width],
                   [0., other_zoom * other_camera.height / height, -other_offset[1] * 2 / height],
                   [0., 0., 1.]])
