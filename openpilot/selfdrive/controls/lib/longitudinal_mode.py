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
  return (map_cruise_supported(CP) and not experimental and params.get_bool('ExperimentalModeConfirmed') and
          params.get('MapCruiseEnabled', return_default=True))


def set_longitudinal_mode(params, mode: LongitudinalMode):
  # SLC owns automatic E2E. Keep the legacy conditional parameter coherent for
  # older checkouts, while full ExperimentalMode remains an explicit override.
  if mode == LongitudinalMode.experimental:
    params.put_bool('ConditionalExperimentalMode', False, block=True)
    params.put_bool('ExperimentalMode', True, block=True)
  else:
    if mode == LongitudinalMode.voacc:
      params.put_bool('MapCruiseEnabled', False, block=True)
    params.put_bool('ExperimentalMode', False, block=True)
    params.put_bool('ConditionalExperimentalMode', mode == LongitudinalMode.conditional, block=True)
    if mode == LongitudinalMode.conditional:
      params.put_bool('MapCruiseEnabled', True, block=True)
