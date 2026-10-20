"""Packed-map boundaries must not blank a known road or invent a junction."""
import math
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from openpilot.selfdrive.mapd.osm_speed_limit import GpsFix, OSMSpeedLimit, RoadTracker
from openpilot.selfdrive.mapd.tests.test_osm_speed_limit import point, polyline


def packed_road(points, speed='100', way_id=1, **tags):
  return replace(polyline(points, speed, way_id, highway='motorway', name='Test Highway', ref='216', oneway='yes'),
                 node_ids=(), tags={'highway': 'motorway', 'name': 'Test Highway', 'ref': '216', 'oneway': 'yes',
                                   **({'maxspeed': speed} if speed is not None else {}), **tags})


def fix(x, y, timestamp, bearing=0.):
  return GpsFix(*point(x, y), bearing, 15., timestamp, speed=30., motion_bearing=bearing)


def approach(tracker, roads):
  for i, y in enumerate((-70, -40, -10)):
    result = tracker.update(roads, fix(0, y, 100. + i))
    assert result is not None and result.speed == 100 / 3.6


@pytest.mark.parametrize('speed,tags,expected', [
  ('100', {}, 100 / 3.6), ('80', {}, 80 / 3.6), ('120', {}, 120 / 3.6),
  (None, {}, None), ('100', {'maxspeed:conditional': '30 @ wet'}, None),
  (None, {'maxspeed:advisory': '40'}, None),
])
def test_boundary_uses_new_way_speed_without_blank_or_previous_limit_hold(speed, tags, expected):
  before = packed_road([(0, -500), (0, 0)])
  after = packed_road([(0, 0), (0, 500)], speed, 2, **tags)
  tracker = RoadTracker(offline_continuity=True)
  approach(tracker, (before, after))
  result = tracker.update((before, after), fix(0, 20, 103.))
  assert result is not None and result.road.way_id == 2 and result.speed == expected
  assert result.advisory_speed == (40 / 3.6 if 'maxspeed:advisory' in tags else None)


def test_crosses_short_pieces_between_gps_fixes_in_both_directions():
  roads = tuple(packed_road([(0, a), (0, b)], way_id=i, oneway='no')
                for i, (a, b) in enumerate(((-500, 0), (0, 5), (5, 10), (10, 500))))
  for direction in (1, -1):
    tracker = RoadTracker(offline_continuity=True)
    points = (-40, -10, 20, 50) if direction == 1 else (50, 20, -10, -40)
    for i, y in enumerate(points):
      result = tracker.update(roads, fix(0, y, 100. + i, 0 if direction == 1 else 180))
      assert result is not None and result.speed == 100 / 3.6


@pytest.mark.parametrize('case', ['parallel', 'interior', 'turn', 'reverse', 'name', 'ref', 'class', 'missing_identity', 'real_nodes'])
def test_geometry_does_not_connect_unrelated_or_incompatible_roads(case):
  before = packed_road([(0, -500), (0, 0)])
  after = packed_road([(0, 0), (0, 500)], '40', 2)
  x, y, heading = 0, 20, 0
  if case == 'parallel':
    after = replace(after, geometry=(point(4, 0), point(4, 500)))
    x = 4
  elif case == 'interior':
    after = replace(after, geometry=(point(0, -200), point(0, 500)))
  elif case == 'turn':
    after = replace(after, geometry=(point(0, 0), point(200, 500)))
    x, heading = 8, math.degrees(math.atan2(200, 500))
  elif case == 'reverse':
    after = replace(after, geometry=after.geometry[::-1])
  elif case == 'name':
    after = replace(after, tags={**after.tags, 'name': 'Another highway'})
  elif case == 'ref':
    after = replace(after, tags={**after.tags, 'ref': '2'})
  elif case == 'class':
    after = replace(after, tags={**after.tags, 'highway': 'motorway_link'})
  elif case == 'missing_identity':
    before = replace(before, tags={'highway': 'motorway', 'maxspeed': '100'})
    after = replace(after, tags={'highway': 'motorway', 'maxspeed': '40'})
  elif case == 'real_nodes':
    before = replace(before, node_ids=(1, 2))
    after = replace(after, node_ids=(3, 4))
  tracker = RoadTracker(offline_continuity=True)
  approach(tracker, (before,))
  assert tracker.update((before, after), fix(x, y, 103., heading)) is None


def test_equal_mainline_fork_does_not_infer_a_unique_continuation():
  before = packed_road([(0, -500), (0, 0)])
  left = packed_road([(0, 0), (-20, 500)], '80', 2)
  right = packed_road([(0, 0), (20, 500)], '60', 3)
  roads = before, left, right
  tracker = RoadTracker(offline_continuity=True)
  approach(tracker, roads)
  assert tracker.update(roads, fix(0, 20, 103.)) is None


def test_mainline_continues_beside_ramp_and_releases_on_measured_departure():
  before = packed_road([(0, -500), (0, 0)])
  after = packed_road([(0, 0), (0, 500)], way_id=2)
  ramp = packed_road([(0, 0), (200, 500)], '40', 3, highway='motorway_link')
  roads = before, after, ramp
  tracker = RoadTracker(offline_continuity=True)
  approach(tracker, roads)
  assert tracker.update(roads, fix(2, 20, 103.)).road.way_id == 2
  heading = math.degrees(math.atan2(200, 500))
  for i, y in enumerate((50, 80, 110)):
    result = tracker.update(roads, fix(y * .4, y, 104. + i, heading))
    assert result is None or result.road.way_id == 3
  assert result is not None and result.speed == 40 / 3.6


def test_boundary_does_not_bypass_stale_gps_or_ambiguous_reacquisition():
  before = packed_road([(0, -500), (0, 0)])
  after = packed_road([(0, 0), (0, 500)], way_id=2)
  parallel = packed_road([(1, -500), (1, 500)], '40', 3)
  tracker = RoadTracker(offline_continuity=True)
  approach(tracker, (before, after))
  # History has expired; the nearby different-speed road still makes acquisition ambiguous.
  assert tracker.update((before, after, parallel), fix(.5, 20, 110.)) is None


def test_downloaded_map_continuity_never_publishes_a_junction():
  roads = packed_road([(0, -500), (0, 0)]), packed_road([(0, 0), (0, 500)], '80', 2)
  service = OSMSpeedLimit()
  service._thread = Mock()
  service._offline = SimpleNamespace(update=lambda fix, now: roads)
  for i, y in enumerate((-40, -10, 20, 50)):
    sample = service.update(fix(0, y, 100. + i), 100. + i)
    assert sample[1] == (100 if y < 0 else 80) / 3.6
    assert service.speed_source == 'offline'
    assert service.control_match is None and service.control is None
  assert service.update(fix(0, 50, 104.), 108.) is None


def test_live_tracker_keeps_requiring_shared_node_ids():
  roads = packed_road([(0, -500), (0, 0)]), packed_road([(0, 0), (0, 500)], way_id=2)
  tracker = RoadTracker()
  approach(tracker, roads)
  assert tracker.update(roads, fix(0, 20, 103.)) is None
