import time
import pyray as rl

from openpilot.selfdrive.ui.ui_state import ui_state, device
from openpilot.system.ui.widgets import Widget


def draw_glow(center, radius, inner, outer):
  try:
    rl.draw_circle_gradient(center, radius, inner, outer)
  except (TypeError, RuntimeError):  # Older AGNOS installations use raylib 5.
    rl.draw_circle_gradient(int(center.x), int(center.y), radius, inner, outer)


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
    # Approximate image-space tracking; mirror the inward camera on the display.
    x = 1-state.motionX if state.motionCamera == 1 else state.motionX
    if state.motion and state.motionCamera in (1, 2):
      self.x += (max(0., min(1., x))-self.x)*min(1., dt*5)
      self.y += (max(0., min(1., state.motionY))-self.y)*min(1., dt*5)
    # Four times the previous diameter, with the entire glow kept inside the display.
    scale = min(4., rect.width/200, rect.height/200)
    margin = 100*scale
    center = rl.Vector2(rect.x+margin+self.x*max(0, rect.width-2*margin), rect.y+margin+self.y*max(0, rect.height-2*margin))
    fade = max(0., min(1., (device.sentry_motion_until-now)/.5))
    draw_glow(center, 95*scale, rl.Color(130, 0, 0, int(110*fade)), rl.BLANK)
    rl.draw_circle_v(center, 44*scale, rl.Color(18, 5, 5, int(255*fade)))
    rl.draw_ring(center, 42*scale, 44*scale, 0, 360, 64, rl.Color(110, 40, 40, int(180*fade)))
    draw_glow(center, 35*scale, rl.Color(255, 65, 25, int(255*fade)), rl.Color(75, 0, 0, int(255*fade)))
    draw_glow(center, 16*scale, rl.Color(255, 190, 130, int(255*fade)), rl.Color(220, 0, 0, int(120*fade)))
    # Three reflected lights on either side of the curved glass. At the display
    # center the pairs are mirror images; subtle parallax follows screen position
    # rather than jumping with the unsmoothed camera motion coordinates.
    shift_x = (.5-(center.x-rect.x)/max(rect.width, 1))*8
    shift_y = (.5-(center.y-rect.y)/max(rect.height, 1))*8
    for side in (-1, 1):
      for dx, dy, rx, ry in ((26, -19, 4.4, 2.2), (31, 0, 3.2, 4.), (26, 19, 4.4, 2.2)):
        x = int(center.x+(side*dx+shift_x)*scale)
        y = int(center.y+(dy+shift_y)*scale)
        rl.draw_ellipse(x, y, (rx+1.3)*scale, (ry+1.)*scale, rl.Color(255, 185, 170, int(40*fade)))
        rl.draw_ellipse(x, y, rx*scale, ry*scale, rl.Color(255, 235, 230, int(170*fade)))
