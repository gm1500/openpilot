import pyray as rl


LANE_GREEN = rl.Color(13, 248, 122, 255)
ICON_BACKGROUND = rl.Color(0, 0, 0, 166)


def draw_lane_icon_frame(rect: rl.Rectangle, color: rl.Color) -> None:
  center_x = int(rect.x + rect.width / 2)
  center_y = int(rect.y + rect.height / 2)
  rl.draw_circle(center_x, center_y, rect.width / 2, ICON_BACKGROUND)
  top = center_y - rect.height * .27
  bottom = center_y + rect.height * .27
  for side in (-1, 1):
    rl.draw_line_ex(rl.Vector2(center_x + side * rect.width * .28, top),
                    rl.Vector2(center_x + side * rect.width * .34, bottom), rect.width * .035, color)
