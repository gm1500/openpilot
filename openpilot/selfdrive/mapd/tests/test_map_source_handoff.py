from dataclasses import replace
from datetime import UTC, datetime
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from openpilot.selfdrive.mapd import osm_speed_limit as osm
from openpilot.selfdrive.mapd.tests.test_osm_speed_limit import point, polyline


def test_same_way_handoff_keeps_live_speed_until_expiry():
  offline = replace(polyline(((0, -2000), (0, 2000)), '120', way_id=1), node_ids=())
  live = replace(offline, tags={**offline.tags, 'maxspeed': '110'})
  service = osm.OSMSpeedLimit()
  service._thread = Mock()
  service._offline = SimpleNamespace(update=lambda fix, now: (offline,))
  fix = osm.GpsFix(*point(0, -100), 0., 3., 100., speed=30.)
  assert service.update(fix, 100.)[1] == pytest.approx(120/3.6)
  assert service.speed_source == 'offline'
  assert service.speed_match.road is offline
  service._cache = (live,), osm.GpsFix(*point(0, -1500), 0., 3., 101.), 101.
  assert service.update(replace(fix, timestamp=101.), 101.)[1] == pytest.approx(110/3.6)
  assert service.speed_source == 'live'
  assert service.speed_match.road is live
  # Leaving the live circle must not resurrect the conflicting offline tag.
  moved = osm.GpsFix(*point(0, -40), 0., 3., 103., speed=30.)
  assert service.update(moved, 103.)[1] == pytest.approx(110/3.6)
  assert service.speed_source == 'offline+live'
  assert service.speed_match.road.geometry == offline.geometry
  assert service.speed_match.road.node_ids == ()
  assert offline.tags['maxspeed'] == '120'
  assert service.control_match is None
  assert service.update(None, 104.) is None
  assert service.speed_source == 'none' and service.speed_match is None
  assert service.update(replace(moved, timestamp=700.), 700.)[1] == pytest.approx(110/3.6)
  assert service.update(replace(moved, timestamp=701.), 701.)[1] == pytest.approx(120/3.6)
  assert service.speed_source == 'offline'


def fallback_service(offline, live, **tags):
  live = replace(live, tags={**live.tags, **tags})
  service = osm.OSMSpeedLimit()
  service._thread = Mock()
  service._offline = SimpleNamespace(update=lambda fix, now: (offline,))
  service._cache = (live,), osm.GpsFix(*point(0, -1500), 0., 3., 100.), 100.
  return service, osm.GpsFix(*point(0, 0), 0., 3., 101., speed=30.)


@pytest.mark.parametrize('change', ['id', 'geometry', 'direction', 'zero_id'])
def test_live_speed_never_crosses_way_geometry_or_direction_boundaries(change):
  offline = replace(polyline(((0, -2000), (0, 2000)), '120', way_id=1), node_ids=())
  live = replace(offline, tags={'maxspeed': '110'})
  if change == 'id':
    live = replace(live, way_id=2)
  elif change == 'geometry':
    live = replace(live, geometry=tuple(point(1, y) for y in (-2000, 2000)))
  elif change == 'direction':
    live = replace(live, tags={**live.tags, 'oneway': 'yes'})
  else:
    offline, live = replace(offline, way_id=0), replace(live, way_id=0)
  service, fix = fallback_service(offline, live)
  assert service.update(fix, fix.timestamp)[1] == pytest.approx(120/3.6)
  assert service.speed_source == 'offline'


def test_live_tag_replacement_and_float_noise_do_not_pin_a_numeric_limit():
  offline = replace(polyline(((0, -2000), (0, 2000)), '120', way_id=1), node_ids=())
  live = replace(offline, geometry=tuple((lat + 1e-14, lon - 1e-14) for lat, lon in offline.geometry),
                 tags={'maxspeed': '110'})
  service, fix = fallback_service(offline, live)
  assert service.update(fix, 101.)[1] == pytest.approx(110/3.6)
  centre = service._cache[1]
  service._cache = (replace(live, tags={'maxspeed': '130'}),), centre, 102.
  assert service.update(replace(fix, timestamp=102.), 102.)[1] == pytest.approx(130/3.6)
  # A new response without this way cannot extend the prior correction.
  service._cache = (), centre, 103.
  assert service.update(replace(fix, timestamp=103.), 103.)[1] == pytest.approx(120/3.6)
  service._offline = SimpleNamespace(update=lambda fix, now: None)
  assert service.update(replace(fix, timestamp=104.), 104.) is None


def test_fallback_rechecks_conditions_and_direction_instead_of_freezing_speed():
  offline = replace(polyline(((0, -2000), (0, 2000)), '120', way_id=1), node_ids=())
  live = replace(offline, tags={'maxspeed': '110', 'maxspeed:backward': '80',
                               'maxspeed:forward:conditional': '30 @ (Mo-Fr 08:00-09:00)', 'school_zone': 'yes'})
  service, fix = fallback_service(offline, live)
  clock = [datetime(2026, 10, 16, 7, 59, tzinfo=UTC)]
  service.clock.local_time = lambda fix, now: clock[0]
  assert service.update(fix, 101.)[1] == pytest.approx(110/3.6)
  clock[0] = clock[0].replace(hour=8, minute=0)
  assert service.update(fix, 101.2)[1] == pytest.approx(30/3.6)
  assert service.zone_type == 'school'
  service._offline_tracker.reset()
  assert service.update(replace(fix, bearing=180., timestamp=102.), 102.)[1] == pytest.approx(80/3.6)


@pytest.mark.parametrize('tags,expected', [({}, None), ({'maxspeed': '90'}, 90),
  ({'maxspeed': '110', 'maxspeed:conditional': '30 @ wet', 'maxspeed:advisory': '80'}, None)])
def test_removed_and_unresolved_tags_do_not_revive_offline_speeds(tags, expected):
  offline = replace(polyline(((0, -2000), (0, 2000)), '120', way_id=1,
                            **{'maxspeed:conditional': '100 @ wet', 'maxspeed:advisory': '110'}), node_ids=())
  live = replace(offline, tags=tags)
  service, fix = fallback_service(offline, live)
  service.clock.local_time = lambda fix, now: datetime(2026, 10, 16, 8, tzinfo=UTC)
  sample = service.update(fix, 101.)
  assert sample[1] == (pytest.approx(expected/3.6) if expected else None)
  assert sample[2] is None
  assert service.speed_source == 'offline+live'


def test_next_way_keeps_its_own_limit():
  first = replace(polyline(((0, -2000), (0, 10)), '120', way_id=1), node_ids=())
  following = replace(polyline(((0, 10), (0, 2000)), '60', way_id=2), node_ids=())
  live = replace(first, tags={'maxspeed': '110'})
  service, fix = fallback_service(first, live)
  roads = first, following
  service._offline = SimpleNamespace(update=lambda fix, now: roads)
  assert service.update(fix, 101.)[1] == pytest.approx(110/3.6)
  for t, y in ((102., 30.), (103., 60.), (104., 90.)):
    result = service.update(replace(fix, latitude=point(0, y)[0], timestamp=t), t)
  assert result[1] == pytest.approx(60/3.6)
  assert service.speed_match.road.way_id == 2 and service.speed_source == 'offline'


def test_unmatched_and_untagged_roads_remain_distinguishable():
  road = polyline(((0, -100), (0, 100)), None, way_id=2)
  service = osm.OSMSpeedLimit()
  service._thread = Mock()
  service._offline = SimpleNamespace(update=lambda fix, now: None)
  fix = osm.GpsFix(0., 0., 0., 3., 100.)
  service._cache = (road,), fix, 100.
  assert service.update(fix, 100.)[1:3] == (None, None)
  assert service.speed_source == 'live' and service.speed_match.road.way_id == 2
  assert service.update(replace(fix, bearing=90., timestamp=101.), 101.)[1:3] == (None, None)
  assert service.speed_source == 'none' and service.speed_match is None


@pytest.mark.parametrize('to_offline', [False, True])
def test_source_handoff_preserves_mainline_at_incoming_parallel_ramp(to_offline):
  main = polyline(((0, -200), (0, 300), (0, 600)), '100', way_id=1, highway='motorway', oneway='yes')
  ramp = polyline(((-30, -100), (0, 300)), None, way_id=2, highway='motorway_link',
                  oneway='yes', **{'maxspeed:advisory': '70'})
  live = main, ramp
  offline = tuple(replace(r, node_ids=(), geometry=tuple((lat + 1e-14, lon - 1e-14) for lat, lon in r.geometry)) for r in live)
  available = [offline[:1]]
  service = osm.OSMSpeedLimit()
  service._thread = Mock()
  service._offline = SimpleNamespace(update=lambda fix, now: available[0])
  fix = osm.GpsFix(*point(0, 40), 6., 15., 100., speed=20.)
  if to_offline:
    service._cache = live[:1], fix, 100.
  assert service.update(fix, 100.)[1] == pytest.approx(100/3.6)

  available[0] = offline
  service._cache = ((), None, -float('inf')) if to_offline else (live, fix, 100.1)
  # A source switch is not a new GPS observation, and must not re-acquire the
  # nearby incoming arm. Its more favourable heading is deliberately tempting.
  for step in range(5):
    moved = replace(fix, latitude=point(0, 40 + 20 * step)[0], timestamp=100. + step)
    sample = service.update(moved, moved.timestamp + .1)
    assert sample[1:3] == (pytest.approx(100/3.6), None)
    assert service.speed_match.road.way_id == 1
    assert service.speed_source == ('offline' if to_offline else 'live')
    assert (service.control_match is None) == to_offline
    assert bool(service.speed_match.road.node_ids) != to_offline


def test_handoff_keeps_a_real_outgoing_ramp_reachable():
  main = polyline(((0, -100), (0, 0), (0, 200)), '100', way_id=1, highway='motorway', oneway='yes')
  ramp = polyline(((0, 0), (40, 100)), None, way_id=2, highway='motorway_link',
                  oneway='yes', **{'maxspeed:advisory': '70'})
  service = osm.OSMSpeedLimit()
  service._thread = Mock()
  service._offline = SimpleNamespace(update=lambda fix, now: (replace(main, node_ids=()),))
  fix = osm.GpsFix(*point(0, -20), 0., 3., 100., speed=20.)
  assert service.update(fix, 100.)[1] == pytest.approx(100/3.6)
  service._cache = (main, ramp), fix, 100.1
  assert service.update(fix, 100.1)[1] == pytest.approx(100/3.6)
  for step in range(1, 5):
    moved = osm.GpsFix(*point(8 * step, 20 * step), 21.8, 3., 100. + step, speed=22.)
    sample = service.update(moved, moved.timestamp)
  assert sample[1:3] == (None, pytest.approx(70/3.6))
  assert service.speed_match.road.way_id == 2


@pytest.mark.parametrize('change', ['id', 'geometry', 'direction', 'zero_id', 'duplicate', 'stale'])
def test_handoff_does_not_transfer_history_between_unverified_roads(change):
  road = polyline(((0, -100), (0, 100)), '100', way_id=1, oneway='yes')
  fix = osm.GpsFix(*point(0, 0), 0., 3., 100., speed=10.)
  previous, current = osm.RoadTracker(), osm.RoadTracker()
  assert previous.update((road,), fix) is not None
  destination = replace(road, tags={**road.tags, 'maxspeed': '80'}, node_ids=())
  if change == 'id':
    destination = replace(destination, way_id=2)
  elif change == 'geometry':
    destination = replace(destination, geometry=tuple(point(1, y) for y in (-100, 100)))
  elif change == 'direction':
    destination = replace(destination, tags={**destination.tags, 'oneway': '-1'})
  elif change == 'zero_id':
    road, destination = replace(road, way_id=0), replace(destination, way_id=0)
    previous.reset()
    previous.update((road,), fix)
  elif change == 'stale':
    fix = replace(fix, timestamp=104.)
  roads = (destination, destination) if change == 'duplicate' else (destination,)
  current.inherit(previous, roads, fix, None)
  assert current.match is None and current._paths == [] and current._result is None


def test_handoff_preserves_ambiguity_and_uses_new_speed_tags():
  main = polyline(((0, -100), (0, 100)), '100', way_id=1, oneway='yes')
  parallel = polyline(((2, -100), (2, 100)), '100', way_id=2, oneway='yes')
  previous, current = osm.RoadTracker(), osm.RoadTracker()
  fix = osm.GpsFix(*point(1, 0), 0., 15., 100., speed=20.)
  assert previous.update((main, parallel), fix) is not None
  changed = replace(parallel, tags={**parallel.tags, 'maxspeed': '70'}, node_ids=())
  roads = replace(main, node_ids=()), changed
  current.inherit(previous, roads, fix, None)
  assert len(current._paths) == 2
  assert current.update(roads, fix) is None  # repeated fix cannot qualify the new conflict
  assert current.update(roads, replace(fix, latitude=point(1, 20)[0], timestamp=101.)) is None


def test_position_loss_clears_both_source_histories():
  road = polyline(((0, -100), (0, 100)), '100', way_id=1, oneway='yes')
  service = osm.OSMSpeedLimit()
  service._thread = Mock()
  service._offline = SimpleNamespace(update=lambda fix, now: (road,))
  fix = osm.GpsFix(*point(0, 0), 0., 3., 100.)
  assert service.update(fix, 100.)[1] == pytest.approx(100/3.6)
  assert service.update(None, 101.) is None
  assert service._active_tracker is None
  assert service._tracker._paths == service._offline_tracker._paths == []
