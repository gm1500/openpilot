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

  def _render(self, rect):
    rl.draw_rectangle_rec(rect, rl.BLACK)
    now = time.monotonic()
    dt, self.last = min(.1, now-self.last), now
    if not device.sentry_eye_visible:
      return
    state = ui_state.sm['sentryState']
    if state.motion and state.motionCamera in (1, 2):
      x, y = eye_target(state.motionX, state.motionY, state.motionCamera)
      self.x += (x-self.x)*min(1., dt*5)
      self.y += (y-self.y)*min(1., dt*5)
    # Clamp by the full glow radius, with no extra border/padding. Even at
    # maximum travel the entire eye is rendered inside the display.
    scale = min(4., rect.width/190, rect.height/190)
    margin = 95*scale
    center = rl.Vector2(rect.x+margin+self.x*max(0, rect.width-2*margin), rect.y+margin+self.y*max(0, rect.height-2*margin))
    fade = max(0., min(1., (device.sentry_motion_until-now)/.5))
    draw_glow(center, 95*scale, rl.Color(130, 0, 0, int(110*fade)), rl.BLANK)
    rl.draw_circle_v(center, 44*scale, rl.Color(18, 5, 5, int(255*fade)))
    rl.draw_ring(center, 42*scale, 44*scale, 0, 360, 64, rl.Color(110, 40, 40, int(180*fade)))
    draw_glow(center, 35*scale, rl.Color(255, 65, 25, int(255*fade)), rl.Color(75, 0, 0, int(255*fade)))
    draw_glow(center, 16*scale, rl.Color(255, 190, 130, int(255*fade)), rl.Color(220, 0, 0, int(120*fade)))
    # Three lights across the upper glass, symmetric at the display center.
    # Stronger parallax follows the smoothed display position, keeping every
    # highlight inside the lens as it shifts toward the display center.
    shift_x = (.5-(center.x-rect.x)/max(rect.width, 1))*24
    shift_y = (.5-(center.y-rect.y)/max(rect.height, 1))*24
    for dx, dy, rx, ry in ((-17, -23, 4.4, 2.2), (0, -29, 5.2, 2.4), (17, -23, 4.4, 2.2)):
      x = int(center.x+(dx+shift_x)*scale)
      y = int(center.y+(dy+shift_y)*scale)
      rl.draw_ellipse(x, y, (rx+1.3)*scale, (ry+1.)*scale, rl.Color(255, 185, 170, int(40*fade)))
      rl.draw_ellipse(x, y, rx*scale, ry*scale, rl.Color(255, 235, 230, int(170*fade)))
