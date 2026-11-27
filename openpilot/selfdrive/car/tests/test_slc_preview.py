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
    self.helper.initialize_v_cruise(self.cs, 40)
    self.now = 100.
    self.settings = SLCSettings(advance=True, **settings)
    self.speed = 100.
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
        map_preview=self.preview)

  def approach(self, speed=70.):
    self.preview = MapPreview(speed / 3.6, 100., 2)
    self.tick(220)

  @property
  def target(self):
    return self.helper.v_cruise_kph


def test_lower_limit_uses_braking_distance_and_keeps_posted_limit_distinct():
  d = Drive()
  d.approach()
  assert d.target == 100
  d.preview = replace(d.preview, distance=51.1)
  d.tick()
  assert d.target == 100
  d.preview = replace(d.preview, distance=50.)
  d.tick()
  assert d.target == 70
  assert d.helper.map_cruise.preview_active
  d.speed, d.preview = 70., None
  d.tick()
  assert d.target == 70
  assert not d.helper.map_cruise.preview_active


def test_higher_limit_waits_for_entry_then_applies_immediately():
  d = Drive()
  d.approach(110.)
  d.preview = replace(d.preview, distance=1.)
  d.tick()
  assert d.target == 100
  d.speed = 110.
  # The following boundary can appear in the same map packet as this entry.
  d.preview = MapPreview(120 / 3.6, 100., 3)
  d.tick()
  assert d.target == 110


def test_upcoming_legal_limit_display_keeps_current_ramp_advisory_target():
  d = Drive()
  d.speed = 40.  # read_map_speed supplies the current advisory when legal speed is absent
  d.preview = MapPreview(100 / 3.6, 80., 2, advisory=False, merge=False)
  d.tick()
  assert d.target == 40
  assert d.helper.map_cruise.preview_speed_ms == pytest.approx(100 / 3.6)
  assert not d.helper.map_cruise.preview_active
  d.preview = replace(d.preview, distance=1.)
  d.tick()
  assert d.target == 40
  d.speed, d.preview = 100., None
  d.tick()
  assert d.target == 100


@pytest.mark.parametrize('issue', ['off', 'invalid_way', 'stale', 'reversed', 'missing'])
def test_unusable_preview_does_not_apply_an_early_lower_limit(issue):
  d = Drive()
  d.preview = MapPreview(70 / 3.6, 20., 2)
  if issue == 'off':
    d.settings = replace(d.settings, advance=False)
  elif issue == 'invalid_way':
    d.preview = replace(d.preview, way_id=0)
  elif issue == 'stale':
    d.gps = d.now - 3.1
  elif issue == 'reversed':
    d.gps = d.now - .1
  else:
    d.preview = None
  d.tick()
  assert d.target == 100
  assert not d.helper.map_cruise.preview_active
  # A fresh current-road match does not depend on earlier preview history.
  d.speed, d.preview, d.gps = 70., None, None
  d.tick()
  assert d.target == 70


def test_fresh_lower_preview_in_range_applies_on_first_frame_with_unchanged_gps():
  d = Drive()
  d.gps = d.now
  d.preview = MapPreview(70 / 3.6, 20., 2)
  d.tick()
  assert d.target == 70
  assert d.helper.map_cruise.preview_active


def test_boundary_gap_never_blocks_new_fresh_current_limit():
  d = Drive()
  d.approach(110.)
  d.speed, d.preview = None, None
  d.tick(180)
  assert d.target == 100
  assert not d.helper.map_cruise.preview_active
  d.speed = 110.
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
  d.speed, d.preview = 110., None
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
  d.speed, d.preview = 70., None
  d.tick()
  assert not d.helper.map_cruise.e2e_fallback


@pytest.mark.parametrize('resume_policy', [0, 1])
@pytest.mark.parametrize('next_limit', ['preview_entry', 'different_current', 'different_preview'])
@pytest.mark.parametrize('button,manual_speed', [(ButtonType.decelCruise, 69), (ButtonType.accelCruise, 71)])
def test_manual_speed_survives_same_preview_source_switches_and_gaps(resume_policy, next_limit, button, manual_speed):
  d = Drive(resume_policy=resume_policy)
  preview = MapPreview(70 / 3.6, 20., 2)
  d.preview = preview
  d.tick()
  assert d.target == 70
  for pressed in (True, False):
    d.cs.buttonEvents = [car.CarState.ButtonEvent(type=button, pressed=pressed)]
    d.tick()
  d.cs.buttonEvents = []
  assert d.target == manual_speed
  assert not d.helper.map_cruise.tracking

  # Neither withdrawal, reappearance, nor brief invalid GPS is a new speed zone.
  for upcoming, stale in ((preview, False), (None, False), (preview, False),
                          (replace(preview, way_id=99), False), (preview, True),
                          (None, False), (preview, False)):
    d.preview = upcoming
    d.gps = d.now - 3.1 if stale else None
    for _ in range(5):
      d.tick()
      assert d.target == manual_speed
      assert not d.helper.map_cruise.tracking

  if next_limit == 'preview_entry':
    d.speed, d.preview = 70., None
    expected = 70
  elif next_limit == 'different_current':
    d.speed, d.preview = 60., None
    expected = 60
  else:
    d.preview = MapPreview(60 / 3.6, 20., 3)
    expected = 60
  d.tick()
  assert d.target == (expected if resume_policy == 0 else manual_speed)
  assert d.helper.map_cruise.tracking == (resume_policy == 0)


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
    'far_distance': ('nextSpeedLimitDistance', 1001.),
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


def test_confirmed_unrestricted_merge_raises_target_before_highway_entry():
  d = Drive()
  d.speed = None
  d.preview = MapPreview(100 / 3.6, 150., 2, merge=True)
  d.helper.v_cruise_kph = 70.
  d.tick(220)
  assert d.target == 100 and d.helper.map_cruise.preview_active
  assert d.speed is None


@pytest.mark.parametrize('issue', ['restriction', 'advisory', 'ordinary', 'too_far', 'lower_only', 'manual'])
def test_merge_target_keeps_restrictions_and_user_controls(issue):
  d = Drive(change_policy=1 if issue == 'lower_only' else 0, resume_policy=1)
  d.speed = 40. if issue == 'restriction' else None
  d.preview = MapPreview(100 / 3.6, 170. if issue == 'too_far' else 150., 2,
                         advisory=issue == 'advisory', merge=issue != 'ordinary')
  d.helper.v_cruise_kph = 70.
  if issue == 'manual':
    d.helper.map_cruise.pause()
  d.tick(220)
  assert d.target <= 70.


def test_confirmed_merge_applies_immediately_without_requalifying_geometry():
  d = Drive()
  d.speed = None
  d.preview = MapPreview(110 / 3.6, 100., 2)
  d.tick(220)
  d.preview = replace(d.preview, merge=True)
  d.tick()
  assert d.helper.map_cruise.preview_active and d.target == 110


def test_advisory_fifty_metres_ahead_is_an_early_lower_target():
  d = Drive()
  d.preview = MapPreview(40 / 3.6, 50., 2, advisory=True)
  d.tick(220)
  assert d.target == 40 and d.helper.map_cruise.preview_active


def test_distant_speed_is_visible_without_being_applied():
  d = Drive()
  d.speed = None
  d.preview = MapPreview(60 / 3.6, 800., 2)
  d.tick(220)
  assert d.target == 100 and not d.helper.map_cruise.preview_active


@pytest.mark.parametrize('speed,advisory', [(70., False), (110., False), (60., True)])
def test_new_preview_is_available_immediately_without_changing_set(speed, advisory):
  d = Drive(offset_value=5)
  d.preview = MapPreview(speed / 3.6, 800., 2, advisory=advisory)
  d.tick()
  map_cruise = d.helper.map_cruise
  assert map_cruise.preview_speed_ms == speed / 3.6
  assert map_cruise.preview_advisory == advisory
  assert not map_cruise.preview_active
  assert d.target == 105


@pytest.mark.parametrize('issue', ['missing', 'stale', 'invalid_speed', 'invalid_way', 'same_limit', 'disabled'])
def test_unusable_or_unchanged_preview_clears_display_candidate(issue):
  d = Drive()
  d.preview = MapPreview(70 / 3.6, 800., 2)
  d.tick()
  assert d.helper.map_cruise.preview_speed_ms == 70 / 3.6
  if issue == 'missing':
    d.preview = None
  elif issue == 'stale':
    d.gps = d.now - 4
  elif issue == 'invalid_speed':
    d.preview = replace(d.preview, speed=float('nan'))
  elif issue == 'invalid_way':
    d.preview = replace(d.preview, way_id=0)
  elif issue == 'same_limit':
    d.preview = replace(d.preview, speed=100 / 3.6)
  else:
    d.settings = replace(d.settings, advance=False)
  d.tick()
  assert d.helper.map_cruise.preview_speed_ms is None
  assert not d.helper.map_cruise.preview_advisory
  assert d.target == 100


def test_preview_wire_keeps_future_posted_speed_distinct_from_actual_adjustment():
  packet = log.Event.new_message(valid=True, logMonoTime=int(100e9))
  state = packet.init('mapCruiseState')
  assert not state.previewValid
  assert state.previewSpeed == 0
  d = Drive(offset_value=5)
  d.preview = MapPreview(70 / 3.6, 800., 2, advisory=True)
  d.tick()
  state.previewValid = d.helper.map_cruise.preview_speed_ms is not None
  state.previewSpeed = d.helper.map_cruise.preview_speed_ms
  state.previewAdvisory = d.helper.map_cruise.preview_advisory
  state.adjustingSpeed = d.helper.map_pulse.target_kph / 3.6
  state.setSpeed = d.target / 3.6
  with log.Event.from_bytes(packet.to_bytes()) as decoded:
    assert decoded.mapCruiseState.previewValid
    assert decoded.mapCruiseState.previewAdvisory
    assert decoded.mapCruiseState.previewSpeed * 3.6 == pytest.approx(70)
    assert decoded.mapCruiseState.adjustingSpeed * 3.6 == pytest.approx(105)
    assert decoded.mapCruiseState.setSpeed * 3.6 == pytest.approx(105)
