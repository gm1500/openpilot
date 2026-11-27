import math
import raylib_nb as rl
from dataclasses import dataclass
from openpilot.common.constants import CV
from openpilot.selfdrive.ui.mici.onroad import DIMMED_ICON_ALPHA, glow_highlight_texture
from openpilot.selfdrive.ui.mici.onroad.status import StatusFade, lerp_color
from openpilot.selfdrive.ui.mici.onroad.torque_bar import TorqueBar
from openpilot.selfdrive.ui.onroad.gap_lane_button import GapLaneButton
from openpilot.selfdrive.ui.onroad.e2e_button import E2EButton
from openpilot.selfdrive.ui.onroad.speed_limit import SpeedLimitButton
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus, ChestnutState
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from openpilot.common.filter_simple import FirstOrderFilter


# Constants
SET_SPEED_NA = 255
KM_TO_MILE = 0.621371

SET_SPEED_PERSISTENCE = 2.5  # seconds
WHEEL_SIZE = 52
WHEEL_WHITE = rl.Color(255, 255, 255, 255)
WHEEL_GREEN = rl.Color(0, 255, 64, 255)
WHEEL_ORANGE = rl.Color(255, 115, 0, 255)


@dataclass(frozen=True)
class FontSizes:
  current_speed: int = 176
  speed_unit: int = 66


@dataclass(frozen=True)
class Colors:
  WHITE = rl.WHITE
  WHITE_TRANSLUCENT = rl.Color(255, 255, 255, 200)


FONT_SIZES = FontSizes()
COLORS = Colors()


class HudRenderer(Widget):
  def __init__(self):
    super().__init__()
    """Initialize the HUD renderer."""
    self.is_cruise_set: bool = False
    self.set_speed: float = SET_SPEED_NA
    self.speed: float = 0.0
    self.v_ego_cluster_seen: bool = False
    self._engaged: bool = False
    self._chestnut_fade_time: float = 0

    self._can_draw_top_icons = True
    self._show_wheel_critical = False

    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._font_medium: rl.Font = gui_app.font(FontWeight.MEDIUM)

    self._torque_bar = TorqueBar()
    self._speed_limit_button = self._child(SpeedLimitButton())
    self._gap_lane_button = self._child(GapLaneButton())
    self._e2e_button = self._child(E2EButton())

    self._txt_wheel: rl.Texture = gui_app.texture('icons_mici/wheel.png', WHEEL_SIZE, WHEEL_SIZE)
    self._txt_wheel_highlight = glow_highlight_texture('icons_mici/wheel.png', WHEEL_SIZE)
    self._txt_chestnut: rl.Texture = gui_app.texture('icons_mici/chestnut.png', 48, 36)
    self._txt_chestnut_green: rl.Texture = gui_app.texture('icons_mici/chestnut_green.png', 48, 36)
    self._txt_chestnut_orange: rl.Texture = gui_app.texture('icons_mici/chestnut_orange.png', 61, 36)
    self._chestnut_icon: rl.Texture | None = None
    self._status = StatusFade(0.1)
    self._wheel_critical_filter = FirstOrderFilter(0, 0.1, 1 / gui_app.target_fps)
    self._wheel_angle_filter = FirstOrderFilter(0, 0.1, 1 / gui_app.target_fps)

    self._chestnut_alpha_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)

  def set_wheel_critical_icon(self, critical: bool):
    """Set the wheel icon to critical or normal state."""
    self._show_wheel_critical = critical

  def set_torque_bar_hidden(self, hidden: bool):
    self._torque_bar.set_hidden(hidden)

  def set_can_draw_top_icons(self, can_draw_top_icons: bool, *, personality_visible: bool = False):
    """Set whether to draw the top part of the HUD."""
    # The personality overlay covers the left sign, not the right E2E/AOL control.
    self._can_draw_top_icons = can_draw_top_icons and not personality_visible
    self._speed_limit_button.show_sign = self._can_draw_top_icons
    self._e2e_button.set_visible(can_draw_top_icons)

  def drawing_top_icons(self) -> bool:
    return self._can_draw_top_icons

  def _update_state(self) -> None:
    """Update HUD state based on car state and controls state."""
    sm = ui_state.sm
    if not ui_state.received_since_started("carState"):
      self.is_cruise_set = False
      self.set_speed = SET_SPEED_NA
      self.speed = 0.0
      return

    car_state = sm['carState']

    set_speed = car_state.vCruiseCluster
    engaged = sm['selfdriveState'].enabled
    if engaged != self._engaged:
      self._chestnut_fade_time = rl.get_time() if engaged else 0
    self._engaged = engaged
    self.set_speed = set_speed
    self.is_cruise_set = 0 < self.set_speed < SET_SPEED_NA

    v_ego_cluster = car_state.vEgoCluster
    self.v_ego_cluster_seen = self.v_ego_cluster_seen or v_ego_cluster != 0.0
    v_ego = v_ego_cluster if self.v_ego_cluster_seen else car_state.vEgo
    speed_conversion = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    self.speed = max(0.0, v_ego * speed_conversion)

  def _render(self, rect: rl.Rectangle) -> None:
    """Render HUD elements to the screen."""

    self._torque_bar.render(rect)

    self._draw_set_speed(rect)

    self._draw_model_source(rect)
    self._e2e_button.render(rl.Rectangle(rect.x + rect.width - 94, rect.y + 12, 82, 82))

  def render_steering_wheel(self, rect: rl.Rectangle) -> None:
    """Draw the steering wheel centered at the bottom of rect, after render() has updated state."""
    self._draw_steering_wheel(rect)
    self._gap_lane_button.render(rl.Rectangle(rect.x + rect.width - 94, rect.y + rect.height - 174, 82, 82))

  def user_interacting(self) -> bool:
    return self._gap_lane_button.is_pressed or self._speed_limit_button.is_pressed or self._e2e_button.is_pressed

  def _draw_model_source(self, rect: rl.Rectangle) -> None:
    loading = ui_state.chestnut_state == ChestnutState.LOADING
    if loading:
      icon = self._txt_chestnut
      opacity = 0.35 + 0.65 * (0.5 - 0.5 * math.cos(rl.get_time() * 6.0))
    elif ui_state.chestnut_state in (ChestnutState.UNCOMPILED, ChestnutState.FAILED):
      icon = self._txt_chestnut_orange
      opacity = 1.0
    elif ui_state.chestnut_state == ChestnutState.ACTIVE:
      icon = self._txt_chestnut_green
      opacity = 1.0
    else:
      return

    if icon is not self._chestnut_icon:
      self._chestnut_fade_time = rl.get_time()
      self._chestnut_icon = icon
    visible = loading or rl.get_time() - self._chestnut_fade_time < SET_SPEED_PERSISTENCE
    alpha = self._chestnut_alpha_filter.update(visible)
    if alpha < 1e-2:
      return

    pos = rl.Vector2(rect.x + 14, rect.y + rect.height - 10 - icon.height)
    rl.draw_texture_ex(icon, pos, 0.0, 1.0, rl.Color(255, 255, 255, int(255 * opacity * alpha)))

  def _draw_steering_wheel(self, rect: rl.Rectangle) -> None:
    engaged = self._status.weight(UIStatus.ENGAGED)
    critical = self._wheel_critical_filter.update(self._show_wheel_critical) * engaged
    status_color = self._status.color({UIStatus.ENGAGED: WHEEL_GREEN, UIStatus.OVERRIDE: WHEEL_WHITE, UIStatus.DISENGAGED: WHEEL_WHITE})
    wheel_color = lerp_color(status_color, WHEEL_ORANGE, critical)
    wheel_txt = self._txt_wheel

    # pos
    pos_x = int(rect.x + rect.width / 2)
    pos_y = int(rect.y + rect.height - 10 - wheel_txt.height / 2)
    # ease between the live angle and rest on ignition changes; carState keeps its last angle once stopped
    rotation = -ui_state.sm['carState'].steeringAngleDeg * self._wheel_angle_filter.update(ui_state.started and ui_state.received_since_started('carState'))

    alpha = 255 * (1 - (1 - DIMMED_ICON_ALPHA) * self._status.weight(UIStatus.DISENGAGED))
    rl.draw_texture_pro(wheel_txt, rl.Rectangle(0, 0, wheel_txt.width, wheel_txt.height),
                        rl.Rectangle(pos_x, pos_y, wheel_txt.width, wheel_txt.height),
                        rl.Vector2(wheel_txt.width / 2, wheel_txt.height / 2), rotation, rl.Color(wheel_color.r, wheel_color.g, wheel_color.b, int(alpha)))
    rl.draw_texture_pro(self._txt_wheel_highlight, rl.Rectangle(0, 0, wheel_txt.width, wheel_txt.height),
                        rl.Rectangle(pos_x, pos_y, wheel_txt.width, wheel_txt.height),
                        rl.Vector2(wheel_txt.width / 2, wheel_txt.height / 2), rotation, rl.Color(255, 255, 255, int(alpha * engaged)))

  def _draw_set_speed(self, rect: rl.Rectangle) -> None:
    set_speed = self.set_speed * (1. if ui_state.is_metric else KM_TO_MILE) if self.is_cruise_set else None
    set_color = rl.Color(128, 216, 166, 255) if ui_state.status == UIStatus.ENGAGED else rl.WHITE
    sign = rl.Rectangle(rect.x + 16, rect.y + 12, 96, 204 * 96 / 180)
    self._speed_limit_button.set_speed = set_speed
    self._speed_limit_button.set_color = set_color
    self._speed_limit_button.render(sign)

  def _draw_current_speed(self, rect: rl.Rectangle) -> None:
    """Draw the current vehicle speed and unit."""
    speed_text = str(round(self.speed))
    speed_text_size = measure_text_cached(self._font_bold, speed_text, FONT_SIZES.current_speed)
    speed_pos = rl.Vector2(rect.x + rect.width / 2 - speed_text_size.x / 2, 180 - speed_text_size.y / 2)
    rl.draw_text_ex(self._font_bold, speed_text, speed_pos, FONT_SIZES.current_speed, 0, COLORS.WHITE)

    unit_text = tr("km/h") if ui_state.is_metric else tr("mph")
    unit_text_size = measure_text_cached(self._font_medium, unit_text, FONT_SIZES.speed_unit)
    unit_pos = rl.Vector2(rect.x + rect.width / 2 - unit_text_size.x / 2, 290 - unit_text_size.y / 2)
    rl.draw_text_ex(self._font_medium, unit_text, unit_pos, FONT_SIZES.speed_unit, 0, COLORS.WHITE_TRANSLUCENT)
