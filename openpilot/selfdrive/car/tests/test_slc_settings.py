from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.car.cruise import VCruiseHelper, ButtonType
from openpilot.selfdrive.car.slc_settings import SLCSettings
from opendbc.car.structs import car


@pytest.mark.parametrize('settings,metric,limit,expected', [
  (SLCSettings(), True, 50, 50),
  (SLCSettings(offset_value=5), True, 50, 55),
  (SLCSettings(offset_value=5), False, 50, 58.047),
  (SLCSettings(offset_type=1, offset_value=10), True, 50, 55),
  (SLCSettings(offset_type=1, offset_value=-10), False, 100, 90),
  (SLCSettings(offset_value=30), True, 140, 145),
  (SLCSettings(offset_value=-30), True, 30, 8),
])
def test_offset_units_percent_and_cruise_bounds(settings, metric, limit, expected):
  assert settings.target(limit, metric) == expected


def test_preferences_reject_invalid_values():
  assert SLCSettings.read(NS(get=lambda key, **kw: None)) == SLCSettings()
  assert SLCSettings.read(NS(get=lambda key, **kw: 'bad')) == SLCSettings()
  assert SLCSettings.read(NS(get=lambda key, **kw: 100)).offset_value == 30


class Drive:
  def __init__(self, settings):
    self.helper = VCruiseHelper(car.CarParams(openpilotLongitudinalControl=True))
    self.cs = car.CarState(vEgo=40/3.6, cruiseState={'available': True})
    self.helper.initialize_v_cruise(self.cs, False)
    self.now = 100.
    self.settings = settings

  def tick(self, limit, count=220, events=()):
    self.cs.buttonEvents = events
    for _ in range(count):
      self.now += .01
      self.helper.update_v_cruise(self.cs, True, True, map_enabled=True,
                                 map_speed=limit/3.6 if limit is not None else None,
                                 map_gps_time=self.now, now=self.now, slc_settings=self.settings)
    return self.helper.v_cruise_kph

  def accept(self, limit):
    return self.tick(limit, 1, [car.CarState.ButtonEvent(type=ButtonType.setCruise, pressed=False)])


def test_default_accepts_new_qualified_limits_with_offset():
  d = Drive(SLCSettings(offset_value=5))
  assert d.tick(50) == 55
  assert d.tick(30) == 35
  assert d.helper.map_cruise._candidate == 30


def test_lower_only_does_not_raise_set_even_after_recovery():
  d = Drive(SLCSettings(change_policy=1))
  assert d.tick(50) == 40
  assert d.tick(30) == 30
  assert d.tick(None) == 30
  assert d.tick(80) == 30
  assert d.accept(80) == 80
  assert d.tick(80) == 80
  assert d.tick(100) == 80


def test_accept_mode_keeps_driver_target_until_explicit_set():
  d = Drive(SLCSettings(change_policy=2))
  assert d.tick(50) == 40
  assert d.accept(50) == 50
  assert d.tick(50) == 50
  assert d.tick(30) == 50
  d.cs.vEgo = 25/3.6
  assert d.accept(30) == 30


@pytest.mark.parametrize('resume,expected', [(0, 80), (1, 49)])
def test_manual_override_resume_policy(resume, expected):
  d = Drive(SLCSettings(resume_policy=resume))
  assert d.tick(50) == 50
  d.tick(50, 1, [car.CarState.ButtonEvent(type=ButtonType.decelCruise, pressed=True)])
  assert d.tick(50, 1, [car.CarState.ButtonEvent(type=ButtonType.decelCruise, pressed=False)]) == 49
  assert d.tick(80) == expected
  assert d.accept(80) == 80


def test_changing_preferences_cannot_raise_lower_only_target_or_resume_manual_override():
  d = Drive(SLCSettings())
  assert d.tick(50) == 50
  d.settings = SLCSettings(change_policy=1, offset_value=10)
  assert d.tick(50) == 50
  assert d.accept(50) == 60
  d.helper.map_cruise.pause()
  d.settings = SLCSettings(offset_value=20)
  assert d.tick(50) == 60


def test_setting_change_during_same_limit_requalification_is_applied():
  d = Drive(SLCSettings())
  assert d.tick(50) == 50
  d.tick(None, 1)
  d.settings = SLCSettings(change_policy=1, offset_value=10)
  assert d.tick(50) == 50


def test_set_only_resume_does_not_block_lower_limits_without_manual_override():
  d = Drive(SLCSettings(change_policy=1, resume_policy=1))
  assert d.tick(80) == 40
  assert d.tick(30) == 30
