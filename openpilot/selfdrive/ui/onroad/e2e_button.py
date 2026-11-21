import math
import time
from dataclasses import dataclass

import raylib_nb as rl

from openpilot.selfdrive.ui.onroad.gap_lane_gesture import GapLaneGesture
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget, device


@dataclass(frozen=True)
class E2EButtonState:
  requested: bool = False
  active: bool = False
  lateral_requested: bool = False
  lateral_active: bool = False
  lateral_supported: bool = False
  lateral_available: bool = False
  lateral_suspended: bool = False
  fresh: bool = False

  @property
  def hands_outline(self):
    return (self.lateral_requested and not (self.fresh and self.lateral_suspended) and
            not (self.fresh and self.lateral_supported and self.lateral_available))


def read_button_state(ui, now):
  sm = ui.sm

  def message_fresh(service):
    return (ui.started and sm.valid[service] and sm.recv_frame[service] >= ui.started_frame and
            0 <= now - sm.recv_time[service] <= .5 and 0 <= now - sm.logMonoTime[service] * 1e-9 <= .5)

  fresh = all(message_fresh(service) for service in ('selfdriveState', 'carControl'))
  ss, cc = sm['selfdriveState'], sm['carControl']
  lateral_supported = fresh and bool(getattr(ss, 'alwaysOnLateralSupported', False))
  cs, cp = sm['carState'], ui.CP
  lateral_available = bool(lateral_supported and message_fresh('carState') and cp is not None and math.isfinite(cs.vEgo) and
                           (cp.steerAtStandstill or (not cs.standstill and abs(cs.vEgo) > max(cp.minSteerSpeed, .3))))
  return E2EButtonState(
    requested=ui.experimental_mode,
    active=bool(fresh and ss.experimentalMode and cc.longActive),
    lateral_requested=ui.always_on_lateral,
    lateral_active=bool(lateral_available and ui.always_on_lateral and not getattr(ss, 'alwaysOnLateralSuspended', False) and
                        getattr(ss, 'alwaysOnLateralActive', False) and cc.latActive),
    lateral_supported=lateral_supported,
    lateral_available=lateral_available,
    lateral_suspended=bool(fresh and getattr(ss, 'alwaysOnLateralSuspended', False)),
    fresh=fresh)


def draw_steering_hands(center, scale, color, outline=False):
  """Filled or outlined grips over the original mode icon, with forearms fading to clear."""
  # Paired contour strips form the fingers, hooked thumb, palm, and wrist.
  # Coordinates are in the original 192 px button's space, relative to its center.
  outer = ((61, -37), (66, -38), (71, -31), (73, -23), (76, -17), (78, -5), (78, 3), (86, 17))
  inner = ((61, -32), (62, -25), (61, -18), (57, -18), (52, -27), (48, -27), (49, -19), (74, 26))
  for side in (-1, 1):
    def point(p, side=side):
      return rl.Vector2(center.x + side * p[0] * scale, center.y + p[1] * scale)

    # A subtle dark separation keeps white hands legible over the white rim.
    contour = outer + inner[::-1]
    shadow = rl.Color(0, 0, 0, round(color.a * .75))
    for i, p in enumerate(contour):
      if outline and i == len(outer) - 1:
        continue  # wrist continues into the two fading forearm edges
      rl.draw_line_ex(point(p), point(contour[(i + 1) % len(contour)]), (2 if outline else 3) * scale,
                      color if outline else shadow)
    for i in range(0 if outline else len(outer) - 1):
      a, b, c, d = point(outer[i]), point(inner[i]), point(outer[i + 1]), point(inner[i + 1])
      # Raylib requires counter-clockwise triangles on either mirrored hand.
      rl.draw_triangle_fan([a, c, b], 3, color) if side == -1 else rl.draw_triangle_fan([a, b, c], 3, color)
      rl.draw_triangle_fan([b, c, d], 3, color) if side == -1 else rl.draw_triangle_fan([b, d, c], 3, color)

    # Narrow, diagonal forearms extend from the wrist. Small contiguous bands
    # give a smooth alpha falloff without creating or uploading a texture.
    start_outer, start_inner = outer[-1], inner[-1]
    end_outer, end_inner = (94, 42), (83, 42)
    for i in range(24):
      t0, t1 = i / 24, (i + 1) / 24
      def along(a, b, t, project=point):
        return project((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
      a, b = along(start_outer, end_outer, t0), along(start_inner, end_inner, t0)
      c, d = along(start_outer, end_outer, t1), along(start_inner, end_inner, t1)
      fade = 1 - ((t0 + t1) / 2) ** 1.3
      tint = rl.Color(color.r, color.g, color.b, round(color.a * fade))
      if outline:
        rl.draw_line_ex(a, c, 2 * scale, tint)
        rl.draw_line_ex(b, d, 2 * scale, tint)
      else:
        rl.draw_triangle_fan([a, c, b], 3, tint) if side == -1 else rl.draw_triangle_fan([a, b, c], 3, tint)
        rl.draw_triangle_fan([b, c, d], 3, tint) if side == -1 else rl.draw_triangle_fan([b, d, c], 3, tint)


def button_colors(state, pressed=False):
  alpha = 180 if pressed else 255
  available = state.fresh and state.lateral_supported and state.lateral_available
  active = available and state.lateral_requested and state.lateral_active and not state.lateral_suspended
  background = rl.Color(0, 0, 0, 166)
  if not state.lateral_requested:
    hands = None
  elif state.fresh and state.lateral_suspended:
    hands = rl.Color(255, 152, 0, alpha)
  elif state.hands_outline:
    hands = rl.Color(170, 180, 190, round(alpha * .7))
  elif active:
    hands = rl.Color(13, 248, 122, alpha)
  else:
    hands = rl.Color(255, 255, 255, alpha)
  mode = rl.Color(255, 255, 255, alpha if state.fresh else round(alpha * .6))
  return background, hands, mode


def draw_e2e_button(rect, state, progress=0., pressed=False):
  scale = rect.width / 192.
  background, hands_color, mode_color = button_colors(state, pressed)
  center = rl.Vector2(rect.x + rect.width / 2, rect.y + rect.height / 2)
  rl.draw_circle(int(center.x), int(center.y), rect.width / 2, background)
  # Preserve the original comma button's 192 px circle and 144 px mode asset.
  name = 'experimental.png' if state.requested or state.active else 'chffr_wheel.png'
  icon = gui_app.texture('icons/' + name, round(144 * scale), round(144 * scale))
  rl.draw_texture_ex(icon, rl.Vector2(center.x - icon.width / 2, center.y - icon.height / 2), 0., 1., mode_color)
  if hands_color is not None:
    draw_steering_hands(center, scale, hands_color, outline=state.hands_outline)
  if progress > 0:
    ring = rl.Color(255, 255, 255, 255) if state.lateral_active else rl.Color(13, 248, 122, 255)
    rl.draw_ring(center, rect.width * .47, rect.width * .5, -90, -90 + 360 * progress, 64, ring)


class E2EButton(Widget):
  """Tap selects full E2E; a separate long hold selects always-on lateral."""

  def __init__(self):
    super().__init__()
    self._gesture = GapLaneGesture()
    self._consume_until = 0.
    self._state = E2EButtonState()
    self._drive_frame: int | None = None

  @property
  def is_pressed(self):
    # Consume release as well as press so the camera cannot also open its menu.
    return self._gesture.pressed or time.monotonic() < self._consume_until

  def _reset_touch(self):
    self._gesture.reset()
    self._consume_until = 0.

  def show_event(self):
    self._reset_touch()
    super().show_event()

  def hide_event(self):
    self._reset_touch()
    super().hide_event()

  def _update_state(self):
    if self._drive_frame != ui_state.started_frame:
      self._reset_touch()
      self._drive_frame = ui_state.started_frame
    self._state = read_button_state(ui_state, time.monotonic())
    if not ui_state.started or not device.awake or not self.enabled or not self.is_visible:
      self._reset_touch()

  def _act(self, action):
    if action is None:
      return
    self._consume_until = time.monotonic() + .2
    if action == 'hold':
      ui_state.set_always_on_lateral(not bool(ui_state.params.get('AlwaysOnLateral', return_default=True)))
    elif action == 'tap' and ui_state.map_cruise_supported:
      ui_state.set_experimental_mode(not ui_state.params.get_bool('ExperimentalMode'))

  def _process_mouse_events(self):
    if not ui_state.started or not device.awake or not self.enabled or not self.is_visible or not self._touch_valid():
      self._reset_touch()
      return
    for event in gui_app.mouse_events:
      inside = rl.check_collision_point_rec(rl.Vector2(event.pos.x, event.pos.y), self._hit_rect)
      releasing_contact = self._gesture.pressed and event.left_released
      self._act(self._gesture.event(event, inside))
      if releasing_contact:
        self._consume_until = time.monotonic() + .2
    self._act(self._gesture.advance(time.monotonic()))

  def _render(self, rect):
    draw_e2e_button(rect, self._state, self._gesture.progress(time.monotonic()), self._gesture.pressed)
