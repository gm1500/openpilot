"""Wide-camera easing adapted from upstream 33ddd8eb (Qt) for both raylib UIs."""
import math
import numpy as np

WIDE_CAM_MAX_SPEED = 5.0  # m/s (18 km/h)
ROAD_CAM_MIN_SPEED = 10.0  # m/s (36 km/h)
CAMERA_INPUT_GRACE = 1.0  # preserve an existing request through brief input gaps


def camera_inputs_valid(sm, started_frame: int, now: float) -> bool | None:
  """Fresh current-drive inputs in any mode; None means temporarily unknown."""
  if any(sm.recv_frame[service] < started_frame for service in ('selfdriveState', 'carState')):
    return False  # never carry a request across drives
  for service in ('selfdriveState', 'carState'):
    if (not sm.valid[service] or
        not 0 <= now - sm.recv_time[service] <= .3 or
        not 0 <= now - sm.logMonoTime[service] * 1e-9 <= .3):
      return None
  return True


class CameraZoom:
  def __init__(self):
    self.reset()

  def reset(self):
    self.wide_requested = False
    self.transition = 1.0  # wide camera cropped to match the narrow field of view
    self._last_time: float | None = None
    self._was_wide = False
    self._last_valid_request: float | None = None

  def request(self, enabled: bool | None, speed: float, wide_available: bool, now: float):
    if enabled is False or not wide_available:
      self._last_valid_request = None
      self.wide_requested = False
    elif enabled is None or not math.isfinite(speed):
      # Invalid data must not start a transition, or erase the hysteresis latch
      # after one missed update. Prolonged loss still returns to narrow.
      if self._last_valid_request is None or not 0 <= now - self._last_valid_request <= CAMERA_INPUT_GRACE:
        self.wide_requested = False
    else:
      self._last_valid_request = now
      if speed < WIDE_CAM_MAX_SPEED:
        self.wide_requested = True
      elif speed > ROAD_CAM_MIN_SPEED:
        self.wide_requested = False
    # In the hysteresis band retain the requested view, including mid-transition.

  def use_wide_stream(self, wide_active: bool, can_animate: bool) -> bool:
    # On exit, keep drawing live wide frames until their crop matches narrow.
    # A lost stream must not stall recovery waiting for an undrawable animation.
    return self.wide_requested or (wide_active and can_animate and self.transition < 1.)

  def update(self, wide_active: bool, now: float) -> float:
    dt = 0. if self._last_time is None else min(max(now - self._last_time, 0.), .1)
    self._last_time = now
    if not wide_active or not self._was_wide:
      self.transition = 1.
    else:
      # Original 20 Hz recurrence: x += x * .2 + .01. Integrate over elapsed
      # time so tici and mici have the same roughly 0.85-second animation.
      factor = 1.2 ** (20 * dt)
      if self.wide_requested:
        self.transition = 1.05 - (1.05 - self.transition) * factor
      else:
        self.transition = (self.transition + .05) * factor - .05
      self.transition = min(max(self.transition, 0.), 1.)
    self._was_wide = wide_active
    return self.transition

  def zoom(self, wide_active, device_camera, narrow_zoom, wide_zoom, width, height):
    narrow, wide = device_camera.narrow_road, device_camera.wide_road
    narrow_zoom = max(narrow_zoom, width / narrow.width, height / narrow.height)
    if not wide_active:
      return narrow_zoom
    wide_zoom = max(wide_zoom, width / wide.width, height / wide.height)
    matched_zoom = narrow_zoom * narrow.focal_length / wide.focal_length
    return wide_zoom + self.transition * (matched_zoom - wide_zoom)

  def framing(self, wide_active, device_camera, narrow_calibration, wide_calibration,
              narrow_zoom, wide_zoom, width, height, vertical_offset=0.):
    """Animate around the narrow viewport's calibrated ray, including its crop.

    Matching focal lengths alone is insufficient: narrow framing often clamps
    at a sensor edge while the magnified wide image can centre its vanishing
    point. Map the ACTUAL narrow crop centre into the other lens, then ease to
    the normal wide framing. Video and model overlays share these offsets.
    """
    def normal_offset(camera, calibration, zoom):
      point = camera.intrinsics @ calibration[:, 0]
      offset = np.zeros(2)
      if np.all(np.isfinite(point)) and point[2] > 1e-6:
        offset = (point[:2] / point[2] - camera.intrinsics[:2, 2]) * zoom + (0., vertical_offset)
      margin = np.maximum(0., np.array(camera.size) * zoom / 2 - np.array((width, height)) / 2 - 5.)
      return np.clip(offset, -margin, margin)

    narrow, wide = device_camera.narrow_road, device_camera.wide_road
    fitted_narrow = self.zoom(False, device_camera, narrow_zoom, wide_zoom, width, height)
    narrow_offset = normal_offset(narrow, narrow_calibration, fitted_narrow)
    if not wide_active:
      return fitted_narrow, *narrow_offset
    fitted_wide = max(wide_zoom, width / wide.width, height / wide.height)
    wide_offset = normal_offset(wide, wide_calibration, fitted_wide)
    zoom = self.zoom(True, device_camera, narrow_zoom, wide_zoom, width, height)
    # The calibrated lens rotation maps a viewing direction; no assumption that
    # either optical centre or vanishing point sits at the viewport centre.
    narrow_pixel = np.append(narrow.intrinsics[:2, 2] + narrow_offset / fitted_narrow, 1.)
    ray = wide.intrinsics @ wide_calibration @ narrow_calibration.T @ narrow.intrinsics_inv @ narrow_pixel
    if np.all(np.isfinite(ray)) and ray[2] > 1e-6:
      matched_anchor = ray[:2] / ray[2] - wide.intrinsics[:2, 2]
      anchor = self.transition * matched_anchor + (1. - self.transition) * wide_offset / fitted_wide
      wide_offset = anchor * zoom
    margin = np.maximum(0., np.array(wide.size) * zoom / 2 - np.array((width, height)) / 2 - 5.)
    return zoom, *np.clip(wide_offset, -margin, margin)
