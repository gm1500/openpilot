import math
import time
import pyray as rl

from openpilot.selfdrive.ui.ui_state import ui_state, device
from openpilot.system.ui.widgets import Widget


def draw_glow(center, radius, inner, outer):
  try:
    rl.draw_circle_gradient(center, radius, inner, outer)
  except (TypeError, RuntimeError):  # Older AGNOS installations use raylib 5.
    rl.draw_circle_gradient(int(center.x), int(center.y), radius, inner, outer)


def eye_target(x, y, camera):
  # Mirror the inward camera. Side windows occupy the outer quarters of its
  # fisheye image, so they should already aim toward the display's edges.
  x = 1-x if camera == 1 else x
  gain = 2.2 if camera == 1 else 1.5
  return max(0., min(1., .5+(x-.5)*gain)), max(0., min(1., .5+(y-.5)*1.4))


class SentryEye(Widget):
  def __init__(self):
    super().__init__()
    self.x = self.y = .5
    self.last = time.monotonic()
    self.was_visible = False
    self.record_until = 0
    self.sound_level = 0.
    self.pulse_phase = 0.
    self.gyro_level = 0.
    self.gyro_phase = 0.

  def _render(self, rect):
    rl.draw_rectangle_rec(rect, rl.BLACK)
    now = time.monotonic()
    elapsed, self.last = now-self.last, now
    dt = min(.1, elapsed)
    if not device.sentry_eye_visible:
      self.was_visible = False
      self.x = self.y = .5
      self.sound_level = 0.
      self.pulse_phase = 0.
      self.gyro_level = 0.
      self.gyro_phase = 0.
      return
    state = ui_state.sm['sentryState']
    motion = state.motion and state.motionCamera in (1, 2)
    new_event = self.record_until <= now*1e9 < state.recordUntilMonoTime
    self.record_until = state.recordUntilMonoTime
    # Sleeping screens may stop rendering entirely, so also detect a render gap.
    # A sound/IMU hit has no bearing: begin at the exact center, not the previous
    # subject's location. Once a subject appears, retain normal smooth tracking.
    if not self.was_visible or elapsed > 1. or new_event and not motion:
      self.x = self.y = .5
      self.sound_level = 0.
      self.pulse_phase = 0.
      self.gyro_level = 0.
      self.gyro_phase = 0.
    self.was_visible = True
    if motion:
      x, y = eye_target(state.motionX, state.motionY, state.motionCamera)
      self.x += (x-self.x)*min(1., dt*5)
      self.y += (y-self.y)*min(1., dt*5)
    fade = max(0., min(1., (device.sentry_motion_until-now)/.5))
    level = state.soundLevel if math.isfinite(state.soundLevel) else 0.
    level = max(0., min(1., level))
    self.sound_level += (level-self.sound_level)*(1-math.exp(-max(0., dt)/(.04 if level > self.sound_level else .25)))
    self.pulse_phase = (self.pulse_phase+max(0., dt)*math.tau*(1.4+1.6*self.sound_level)) % math.tau
    pulse = .75+.25*math.sin(self.pulse_phase)
    gyro = float(state.gyroActive)
    self.gyro_level += (gyro-self.gyro_level)*(1-math.exp(-max(0., dt)/(.04 if gyro > self.gyro_level else .2)))
    self.gyro_phase = (self.gyro_phase+max(0., dt)*math.tau*4.) % math.tau
    gyro_pulse = .65+.35*math.sin(self.gyro_phase)
    glow = max(self.sound_level*pulse, self.gyro_level*gyro_pulse)*fade
    scale = min(4., rect.width/190, rect.height/190)
    # The gyro halo nearly spans the screen's short dimension. Clamp to its
    # maximum extent, independent of pulse phase, so the eye does not wobble.
    margin = 95*scale+self.gyro_level*max(0., .48*min(rect.width, rect.height)-95*scale)
    center = rl.Vector2(rect.x+margin+self.x*max(0, rect.width-2*margin), rect.y+margin+self.y*max(0, rect.height-2*margin))
    if self.gyro_level > .005:
      draw_glow(center, margin*(.82+.18*gyro_pulse),
                rl.Color(255, 0, 0, int(225*self.gyro_level*gyro_pulse*fade)), rl.BLANK)
    draw_glow(center, 95*scale, rl.Color(130, 0, 0, int(110*fade)), rl.BLANK)
    # Red light blooms outward from the lens rim. Quiet returns to the normal
    # eye; louder sound makes the bloom brighter without moving its target.
    if glow > .005:
      for i in range(12):
        rl.draw_ring(center, (44+4*i)*scale, (48+4*i)*scale, 0, 360, 96,
                     rl.Color(255, 0, 0, int(145*glow*(1-i/12)**2)))
    rl.draw_circle_v(center, 44*scale, rl.Color(18, 5, 5, int(255*fade)))
    rl.draw_ring(center, 42*scale, 44*scale, 0, 360, 64,
                 rl.Color(int(110+145*glow), int(40*(1-glow)), int(40*(1-glow)), int(180*fade+75*glow)))
    draw_glow(center, 35*scale, rl.Color(255, 65, 25, int(255*fade)), rl.Color(75, 0, 0, int(255*fade)))
    draw_glow(center, 16*scale, rl.Color(255, 190, 130, int(255*fade)), rl.Color(220, 0, 0, int(120*fade)))
    # Rotate three lights together along the upper glass. A fixed radius and
    # angular spacing make them sweep around the lens rather than slide over it.
    # The virtual overhead light changes the arc's angle with screen position.
    light_x = .5-(center.x-rect.x)/max(rect.width, 1)
    light_y = 1.+.6*(.5-(center.y-rect.y)/max(rect.height, 1))
    rotation = max(-6., min(6., .3*math.degrees(math.atan2(light_x, light_y))))
    for base_angle, rx, ry in ((-36., 4.4, 2.2), (0., 5.2, 2.4), (36., 4.4, 2.2)):
      angle = base_angle+rotation
      offset_x = 29.*math.sin(math.radians(angle))
      offset_y = -29.*math.cos(math.radians(angle))
      rl.rl_push_matrix()
      try:
        rl.rl_translatef(center.x+offset_x*scale, center.y+offset_y*scale, 0.)
        rl.rl_rotatef(angle, 0., 0., 1.)
        rl.draw_ellipse(0, 0, (rx+1.3)*scale, (ry+1.)*scale, rl.Color(255, 185, 170, int(40*fade)))
        rl.draw_ellipse(0, 0, rx*scale, ry*scale, rl.Color(255, 235, 230, int(170*fade)))
      finally:
        rl.rl_pop_matrix()
