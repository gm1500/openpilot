"""Brake and blind-spot feedback for the comma 3/3X driving border."""
import pyray as rl

from openpilot.selfdrive.ui import UI_BORDER_SIZE
from openpilot.selfdrive.ui.onroad.braking import braking_active, fresh_ui_message

BRAKE_COLOR = rl.Color(0xFF, 0x3B, 0x45, 255)
BLINDSPOT_WARNING_COLOR = rl.Color(0xC9, 0x22, 0x31, 255)
BLINDSPOT_COLOR = rl.Color(0xDA, 0x6F, 0x25, 255)
FLASH_HZ = 2.


def border_signals(sm, started_frame, now):
  if not fresh_ui_message(sm, 'carState', started_frame, now):
    return False, False, False, False, False
  car = sm['carState']
  return braking_active(sm, started_frame, now), car.leftBlindspot, car.rightBlindspot, car.leftBlinker, car.rightBlinker


def draw_border_indicators(rect, border_rect, roundness, sm, started_frame, now):
  braking, left, right, left_signal, right_signal = border_signals(sm, started_frame, now)
  thickness = UI_BORDER_SIZE

  def clipped_border(x, y, w, h, color, alpha=1.):
    rl.begin_scissor_mode(round(x), round(y), max(0, round(w)), max(0, round(h)))
    rl.draw_rectangle_rounded_lines_ex(border_rect, roundness, 10, thickness,
                                      rl.Color(color.r, color.g, color.b, round(255 * alpha)))
    rl.end_scissor_mode()

  def horizontal_segment(x, y, width, color, fade_left=True, fade_right=True):
    fade = min(thickness * 1.5, width / 4)
    clipped_border(x + (fade if fade_left else 0), y,
                   width - fade * (int(fade_left) + int(fade_right)), thickness * 2, color)
    # Sample the same rounded outline, so overlays follow its corner geometry.
    for i in range(12):
      alpha = (i + .5) / 12
      if fade_left:
        clipped_border(x + i * fade / 12, y, fade / 12 + 1, thickness * 2, color, alpha)
      if fade_right:
        clipped_border(x + width - (i + 1) * fade / 12, y, fade / 12 + 1, thickness * 2, color, alpha)

  for active, signal, is_left in ((left, left_signal, True), (right, right_signal, False)):
    if not active:
      continue
    # Return to the ordinary border during the off phase, rather than flashing orange.
    if signal and int(now * FLASH_HZ * 2) % 2:
      continue
    color = BLINDSPOT_WARNING_COLOR if signal else BLINDSPOT_COLOR
    side_width = thickness * 2
    x = rect.x if is_left else rect.x + rect.width - side_width
    clipped_border(x, rect.y, side_width, rect.height, color)
    width = max(rect.width * .10, thickness * 3)
    x = rect.x if is_left else rect.x + rect.width - width
    for y in (rect.y, rect.y + rect.height - thickness * 2):
      horizontal_segment(x, y, width, color, fade_left=not is_left, fade_right=is_left)

  if braking:
    horizontal_segment(rect.x + rect.width * .375, rect.y, rect.width * .25, BRAKE_COLOR)
