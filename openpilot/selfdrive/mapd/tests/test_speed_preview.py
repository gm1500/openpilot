from dataclasses import replace
from types import SimpleNamespace as NS
from unittest.mock import Mock

import numpy as np
import pytest

from openpilot.selfdrive.mapd import osm_speed_limit as osm
from openpilot.selfdrive.mapd.heading import VehicleMotion
from openpilot.selfdrive.mapd.speed_preview import ModelPath, SpeedPreviewer, model_path
from openpilot.selfdrive.mapd.tests.test_osm_speed_limit import point, polyline


def preview(roads, prediction=None, offline=False, **fix_changes):
  fix = osm.GpsFix(*point(0, 0), 0., 3., 100., speed=20.)
  fix = replace(fix, **fix_changes)
  tracker = osm.RoadTracker(offline_continuity=offline)
  match = tracker.update(tuple(roads), fix)
  return SpeedPreviewer().update(tracker, match, fix, prediction)


def pair(speed='100', next_speed='70', **tags):
  return (polyline([(0, -100), (0, 80)], speed, 1, oneway='yes', **tags),
          polyline([(0, 80), (0, 400)], next_speed, 2, oneway='yes', **tags))


def packed(roads):
  return tuple(replace(r, node_ids=()) for r in roads)


def test_directed_boundary_and_packed_continuation():
  for offline in (False, True):
    roads = pair(name='Main Street')
    result = preview(packed(roads) if offline else roads, offline=offline)
    assert result.way_id == 2
    assert result.speed == pytest.approx(70 / 3.6)
    assert result.distance == pytest.approx(80)


def test_untagged_ramp_can_preview_sole_aligned_receiving_road():
  ramp, receiving = pair(speed=None)
  ramp = replace(ramp, tags={**ramp.tags, 'highway': 'primary_link'})
  for offline in (False, True):
    roads = packed((ramp, receiving)) if offline else (ramp, receiving)
    result = preview(roads, offline=offline)
    assert result.way_id == receiving.way_id
    assert result.distance == pytest.approx(80)


def test_packed_oblique_merge_is_preview_only_and_not_a_right_angle_turn():
  ramp = polyline([(0, -100), (0, 60), (10, 80)], None, 1, highway='motorway_link', oneway='yes')
  receiving = polyline([(10, 80), (10, 400)], '70', 2, oneway='yes')
  assert preview(packed((ramp, receiving)), offline=True).way_id == 2
  receiving = replace(receiving, geometry=(point(10, 80), point(400, 80)))
  assert preview(packed((ramp, receiving)), offline=True) is None


@pytest.mark.parametrize('issue', ['nearby', 'fork', 'reverse', 'turn', 'two_way', 'unknown', 'advisory', 'conditional'])
def test_packed_ramp_rejects_inferred_forks_and_restrictions(issue):
  ramp, receiving = pair(speed=None)
  ramp = replace(ramp, tags={**ramp.tags, 'highway': 'primary_link'})
  extra = []
  if issue == 'nearby':
    receiving = replace(receiving, geometry=(point(1, 80), point(1, 400)))
  elif issue == 'fork':
    extra = [polyline([(0, 80), (100, 400)], '40', 3, oneway='yes')]
  elif issue == 'reverse':
    receiving = replace(receiving, tags={**receiving.tags, 'oneway': '-1'})
  elif issue == 'turn':
    receiving = replace(receiving, geometry=(point(0, 80), point(320, 80)))
  elif issue == 'two_way':
    receiving = replace(receiving, tags={**receiving.tags, 'oneway': 'no'})
  elif issue == 'unknown':
    receiving = replace(receiving, tags={'highway': 'primary', 'oneway': 'yes'})
  else:
    tag = 'maxspeed:advisory' if issue == 'advisory' else 'maxspeed:conditional'
    ramp = replace(ramp, tags={**ramp.tags, tag: '40' if issue == 'advisory' else '40 @ wet'})
  assert preview(packed([ramp, receiving, *extra]), offline=True) is None


def fork_path(right=True, heading_error=3.):
  # 40 metres to the fork, then either diagonal arm. Arc length is explicit.
  distances = np.array([60., 90., 120., 150.])
  diagonal = (distances - 40.) / np.sqrt(2)
  points = np.column_stack((diagonal * (1 if right else -1), 40 + diagonal))
  return ModelPath(points, np.ones(4), np.zeros(2), distances, heading_error)


def fork():
  return (polyline([(0, -100), (0, 40)], '100', 1, oneway='yes'),
          polyline([(0, 40), (200, 240)], '70', 2, oneway='yes'),
          polyline([(0, 40), (-200, 240)], '50', 3, oneway='yes'))


def test_model_selects_only_a_distinguishable_connected_fork():
  roads = fork()
  assert preview(roads) is None
  assert preview(roads, fork_path()).way_id == 2
  assert preview(roads, fork_path(False)).way_id == 3
  assert preview(roads, fork_path(heading_error=70)) is None
  assert preview(packed(roads), fork_path(), offline=True) is None
  # A missing speed on the other branch must not count as evidence for this one.
  unknown = replace(roads[2], tags={'highway': 'primary', 'oneway': 'yes'})
  assert preview((roads[0], roads[1], unknown), fork_path()) is None
  # Coincident geometry cannot be disambiguated by a desired speed value.
  duplicate = replace(roads[1], way_id=4, tags={**roads[1].tags, 'maxspeed': '50'})
  assert preview((roads[0], roads[1], duplicate), fork_path()) is None


@pytest.mark.parametrize('change', [{'stationary': True}, {'speed': -10.}, {'bearing': 25.}, {'motion_bearing': 25.},
                                   {'longitude': point(20, 0)[1]}])
def test_motion_and_match_quality_gate(change):
  assert preview(pair(), **change) is None


def test_provider_keeps_actual_limit_separate_and_defaults_to_legacy():
  ramp, receiving = pair(speed=None)
  ramp = replace(ramp, tags={**ramp.tags, 'highway': 'primary_link'})
  fix = osm.GpsFix(*point(0, 0), 0., 3., 100., speed=20.)
  service = osm.OSMSpeedLimit()
  service._thread = Mock()
  service._cache = (ramp, receiving), fix, 100.
  assert service.update(fix, 100.)[1] == pytest.approx(70 / 3.6)
  assert service.preview is None
  sample = service.update(fix, 100., advance=True)
  assert sample[1:3] == (None, None)
  assert service.preview.way_id == 2


class SM(dict):
  pass


def model_fixture():
  t = np.linspace(0, 10, 33)
  sm = SM(modelV2=NS(timestampEof=int(100e9), frameDropPerc=0.,
                    position=NS(x=20 * t, y=np.zeros(33), t=t, yStd=np.ones(33))))
  sm.services = list(sm)
  sm.valid = dict.fromkeys(sm, True)
  sm.recv_time = dict.fromkeys(sm, 100.)
  sm.logMonoTime = dict.fromkeys(sm, int(100e9))
  return sm


def test_model_coordinates_age_and_cumulative_curve_distance():
  fix = osm.GpsFix(0., 0., 90., 3., 99.8, speed=20.)
  motion = VehicleMotion(20., False, 0.)
  sm = model_fixture()
  result = model_path(sm, fix, motion, 100.)
  np.testing.assert_allclose(result.origin, [4., 0.], atol=1e-8)
  np.testing.assert_allclose(result.points[:, 0], result.distances)
  np.testing.assert_allclose(result.points[:, 1], 0., atol=1e-8)
  sm['modelV2'].position.y = np.linspace(0, 50, 33)**2 / 50
  result = model_path(sm, fix, motion, 100.)
  assert np.all(result.distances > np.linalg.norm(result.points, axis=1))
  for stamp in (100.4, 99.6, 102.):
    assert model_path(sm, fix, motion, stamp) is None
  assert model_path(sm, fix, VehicleMotion(20., False, None), 100.) is None
  sm['modelV2'].position.x[10] = np.nan
  assert model_path(sm, fix, motion, 100.) is None
