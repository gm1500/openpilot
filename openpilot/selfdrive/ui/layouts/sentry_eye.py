import time
import pyray as rl

from openpilot.selfdrive.ui.ui_state import ui_state, device
from openpilot.system.ui.widgets import Widget


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
    center = rl.Vector2(rect.x+100+self.x*max(0, rect.width-200), rect.y+100+self.y*max(0, rect.height-200))
    fade = max(0., min(1., (device.sentry_motion_until-now)/.5))
    rl.draw_circle_gradient(int(center.x), int(center.y), 95, rl.Color(130, 0, 0, int(110*fade)), rl.BLANK)
    rl.draw_circle_v(center, 44, rl.Color(18, 5, 5, int(255*fade)))
    rl.draw_ring(center, 42, 44, 0, 360, 64, rl.Color(110, 40, 40, int(180*fade)))
    rl.draw_circle_gradient(int(center.x), int(center.y), 35, rl.Color(255, 65, 25, int(255*fade)), rl.Color(75, 0, 0, int(255*fade)))
    rl.draw_circle_gradient(int(center.x), int(center.y), 16, rl.Color(255, 190, 130, int(255*fade)), rl.Color(220, 0, 0, int(120*fade)))
    rl.draw_ellipse(int(center.x-13), int(center.y-18), 11, 5, rl.Color(255, 235, 230, int(160*fade)))
