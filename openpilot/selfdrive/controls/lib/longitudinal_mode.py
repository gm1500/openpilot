"""The HUD mode order; full ExperimentalMode retains its existing meaning."""
from enum import IntEnum


class LongitudinalMode(IntEnum):
  voacc = 0
  conditional = 1
  experimental = 2


def selected_mode(experimental: bool, conditional: bool) -> LongitudinalMode:
  return LongitudinalMode.experimental if experimental else LongitudinalMode.conditional if conditional else LongitudinalMode.voacc


def set_longitudinal_mode(params, mode: LongitudinalMode):
  # Clear the previous mode before setting the next. Intermediate observations
  # can only see ordinary mode; they can never accidentally enable full E2E.
  if mode == LongitudinalMode.experimental:
    params.put_bool('ConditionalExperimentalMode', False, block=True)
    params.put_bool('ExperimentalMode', True, block=True)
  else:
    params.put_bool('ExperimentalMode', False, block=True)
    params.put_bool('ConditionalExperimentalMode', mode == LongitudinalMode.conditional, block=True)
