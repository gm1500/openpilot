from dataclasses import replace

import pytest

from opendbc.car.structs import car
from openpilot.cereal import log
from openpilot.selfdrive.car.cruise import VCruiseHelper, ButtonType
from openpilot.selfdrive.car.map_cruise import MapPreview, read_map_preview, read_map_speed
from openpilot.selfdrive.car.slc_settings import SLCSettings


class Drive:
  def __init__(self, **settings):
    self.helper = VCruiseHelper(car.CarParams(openpilotLongitudinalControl=True))
    self.cs = car.CarState(vEgo=20., cruiseState={'available': True})
    self.helper.initialize_v_cruise(self.cs, False)
    self.now = 100.
    self.settings = SLCSettings(advance=True, **settings)
    self.speed, self.way = 100., 1
    self.preview = None
    self.automatic = False
    self.enabled = True
    self.gps = None
    self.tick(220)

  def tick(self, count=1):
    for _ in range(count):
      self.now += .01
      self.helper.update_v_cruise(self.cs, self.enabled, True, map_enabled=True,
        map_speed=self.speed / 3.6 if self.speed is not None else None,
        map_gps_time=self.now if self.gps is None else self.gps, now=self.now,
        automatic_e2e=self.automatic, e2e_ready=True, slc_settings=self.settings,
        map_preview=self.preview, map_way_id=self.way)

  def approach(self, speed=70.):
    self.preview = MapPreview(speed / 3.6, 100., 2)
    self.tick(220)

  @property
  def target(self):
    return self.helper.v_cruise_kph


def test_lower_limit_starts_at_two_seconds_and_keeps_posted_limit_distinct():
  d = Drive()
  d.approach()
  assert d.target == 100
  d.preview = replace(d.preview, distance=40.1)
  d.tick()
  assert d.target == 100
  d.preview = replace(d.preview, distance=40.)
  d.tick()
  assert d.target == 70
  assert d.helper.map_cruise.preview_active
  d.speed, d.way, d.preview = 70., 2, None
  d.tick()
  assert d.target == 70
  assert not d.helper.map_cruise.preview_active


def test_higher_limit_waits_for_entry_without_second_qualification_delay():
  d = Drive()
  d.approach(110.)
  d.preview = replace(d.preview, distance=1.)
  d.tick()
  assert d.target == 100
  d.speed, d.way = 110., 2
  # The following boundary can appear in the same map packet as this entry.
  d.preview = MapPreview(120 / 3.6, 100., 3)
  d.tick()
  assert d.target == 110


@pytest.mark.parametrize('issue', ['off', 'frozen', 'changed_way', 'stale', 'reversed'])
def test_unqualified_preview_does_not_bypass_normal_delay(issue):
  d = Drive()
  if issue == 'off':
    d.settings = replace(d.settings, advance=False)
  if issue == 'frozen':
    d.gps = d.now
  d.approach(110.)
  if issue == 'changed_way':
    d.preview = replace(d.preview, way_id=3)
    d.tick()
  elif issue == 'stale':
    d.now += 3.1
  elif issue == 'reversed':
    d.gps = d.now - 1.
    d.tick()
  d.speed, d.way, d.preview = 110., 2, None
  d.tick()
  assert d.target == 100


def test_short_boundary_gap_preserves_only_exact_way_entry_qualification():
  d = Drive()
  d.approach(110.)
  d.speed, d.preview = None, None
  d.tick(180)
  assert d.target == 100
  assert not d.helper.map_cruise.preview_active
  d.speed, d.way = 110., 2
  d.tick()
  assert d.target == 110


@pytest.mark.parametrize('intervention', ['gasPressed', 'brakePressed', 'disabled', 'manual', 'accept'])
def test_early_target_respects_overrides(intervention):
  d = Drive(resume_policy=1, change_policy=2 if intervention == 'accept' else 0)
  d.approach()
  original = d.target
  if intervention in ('gasPressed', 'brakePressed'):
    setattr(d.cs, intervention, True)
  elif intervention == 'disabled':
    d.enabled = False
  elif intervention == 'manual':
    d.cs.buttonEvents = [car.CarState.ButtonEvent(type=ButtonType.resumeCruise, pressed=True)]
    d.tick()
    d.cs.buttonEvents = []
  d.preview = replace(d.preview, distance=20.)
  d.tick()
  assert d.target == original


def test_lower_only_policy_still_blocks_higher_limit_on_entry():
  d = Drive(change_policy=1)
  d.approach(110.)
  d.speed, d.way, d.preview = 110., 2, None
  d.tick()
  assert d.target <= 100


def test_uninitialized_set_cannot_treat_a_future_limit_as_an_early_reduction():
  d = Drive()
  d.speed = None
  d.approach(110.)
  d.helper.v_cruise_kph = 255.
  d.preview = replace(d.preview, distance=20.)
  d.tick()
  assert not d.helper.map_cruise.preview_active
  assert d.helper.map_cruise.target_kph is None


def test_preview_does_not_raise_lower_manual_target_or_release_no_speed_e2e():
  d = Drive()
  d.helper.v_cruise_kph = 60.
  d.approach(70.)
  d.preview = replace(d.preview, distance=20.)
  d.tick()
  # Restore the manual SET after the normal current limit was processed.
  d.helper.v_cruise_kph = 60.
  d.tick()
  assert not d.helper.map_cruise.preview_active
  d = Drive()
  d.speed, d.automatic = None, True
  d.tick(110)
  assert d.helper.map_cruise.e2e_fallback
  d.approach(70.)
  d.preview = replace(d.preview, distance=20.)
  d.tick()
  assert d.target == 70
  assert d.helper.map_cruise.e2e_fallback
  d.speed, d.way, d.preview = 70., 2, None
  d.tick()
  assert not d.helper.map_cruise.e2e_fallback


@pytest.mark.parametrize('speed', [8., 70., 145.])
def test_wire_fields_can_preview_unknown_current_road_without_changing_display(speed):
  packet = log.Event.new_message(valid=True, logMonoTime=int(100e9))
  msg = packet.init('mapSpeedLimit')
  msg.gpsMonoTime = int(100e9)
  msg.headingValid = True
  msg.nextSpeedLimitValid = True
  msg.nextSpeedLimit = speed / 3.6
  msg.nextSpeedLimitDistance = 80
  msg.nextSpeedLimitWayId = 2
  msg.matchedWayId = 1
  class SM(dict):
    pass
  with log.Event.from_bytes(packet.to_bytes()) as decoded:
    sm = SM(mapSpeedLimit=decoded.mapSpeedLimit)
    sm.valid = {'mapSpeedLimit': True}
    sm.logMonoTime = {'mapSpeedLimit': int(100e9)}
    sm.recv_time = {'mapSpeedLimit': 100.}
    assert read_map_speed(sm, 100.)[0] is None
    assert read_map_preview(sm, 100.).way_id == 2
    assert read_map_preview(sm, 101.) is None
    sm.valid['mapSpeedLimit'] = False
    assert read_map_preview(sm, 100.) is None


@pytest.mark.parametrize('issue', ['default_fields', 'wrong_heading', 'unknown_way', 'nan_speed', 'nan_distance',
                                   'far_distance', 'stale_gps', 'future_gps'])
def test_preview_reader_rejects_unusable_inputs(issue):
  packet = log.Event.new_message(valid=True, logMonoTime=int(100e9))
  m = packet.init('mapSpeedLimit')
  m.gpsMonoTime = m.positionMonoTime = int(100e9)
  m.headingValid = m.positionEstimated = m.nextSpeedLimitValid = True
  m.nextSpeedLimit = 70 / 3.6
  m.nextSpeedLimitDistance, m.nextSpeedLimitWayId = 40., 2
  field, value = {
    'default_fields': ('nextSpeedLimitValid', False),
    'wrong_heading': ('headingValid', False),
    'unknown_way': ('nextSpeedLimitWayId', 0),
    'nan_speed': ('nextSpeedLimit', float('nan')),
    'nan_distance': ('nextSpeedLimitDistance', float('nan')),
    'far_distance': ('nextSpeedLimitDistance', 301.),
    'stale_gps': ('gpsMonoTime', int(96e9)),
    'future_gps': ('gpsMonoTime', int(101e9)),
  }[issue]
  setattr(m, field, value)
  class SM(dict):
    pass
  sm = SM(mapSpeedLimit=m)
  sm.valid = {'mapSpeedLimit': True}
  sm.logMonoTime = {'mapSpeedLimit': int(100e9)}
  sm.recv_time = {'mapSpeedLimit': 100.}
  assert read_map_preview(sm, 100.) is None
