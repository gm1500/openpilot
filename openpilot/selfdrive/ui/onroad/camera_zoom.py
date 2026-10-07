"""Wide-camera easing adapted from upstream 33ddd8eb (Qt) for both raylib UIs."""
import math

WIDE_CAM_MAX_SPEED = 5.0  # m/s (18 km/h)
ROAD_CAM_MIN_SPEED = 10.0  # m/s (36 km/h)


def experimental_camera_active(sm, started_frame: int, now: float) -> bool:
  """Camera behavior follows the selected mode, independently of E2E activation."""
  for service in ('selfdriveState', 'carState'):
    if (not sm.valid[service] or sm.recv_frame[service] < started_frame or
        not 0 <= now - sm.recv_time[service] <= .3 or
        not 0 <= now - sm.logMonoTime[service] * 1e-9 <= .3):
      return False
  state = sm['selfdriveState']
  return state.experimentalMode or state.conditionalExperimental


class CameraZoom:
  def __init__(self):
    self.reset()

  def reset(self):
    self.wide_requested = False
    self.transition = 1.0  # wide camera cropped to match the narrow field of view
    self._last_time: float | None = None
    self._was_wide = False

  def request(self, enabled: bool, speed: float, wide_available: bool):
    if not enabled or not wide_available or not math.isfinite(speed):
      self.wide_requested = False
    elif speed < WIDE_CAM_MAX_SPEED:
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
