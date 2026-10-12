"""The HUD mode order; full ExperimentalMode retains its existing meaning."""
from enum import IntEnum

from openpilot.selfdrive.car.map_cruise import map_cruise_supported


class LongitudinalMode(IntEnum):
  voacc = 0
  conditional = 1
  experimental = 2


def selected_mode(experimental: bool, conditional: bool) -> LongitudinalMode:
  return LongitudinalMode.experimental if experimental else LongitudinalMode.conditional if conditional else LongitudinalMode.voacc


def automatic_e2e_selected(params, CP, experimental: bool) -> bool:
  # Selection is independent of SLC; card requires SLC for automatic activation.
  return (map_cruise_supported(CP) and not experimental and params.get_bool('ExperimentalModeConfirmed') and
          params.get_bool('ConditionalExperimentalMode'))


def set_longitudinal_mode(params, mode: LongitudinalMode):
  # Mode selection never changes the separate SLC toggle. Clear the previous
  # mode first so intermediate reads cannot accidentally select full E2E.
  if mode == LongitudinalMode.experimental:
    params.put_bool('ConditionalExperimentalMode', False, block=True)
    params.put_bool('ExperimentalMode', True, block=True)
  else:
    params.put_bool('ExperimentalMode', False, block=True)
    params.put_bool('ConditionalExperimentalMode', mode == LongitudinalMode.conditional, block=True)
