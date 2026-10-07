import json
import math
import unittest
from dataclasses import replace
from unittest.mock import Mock, patch

from openpilot.selfdrive.mapd import osm_speed_limit as osm


def point(x, y):
  return math.degrees(y / osm.EARTH_RADIUS), math.degrees(x / osm.EARTH_RADIUS)


def road(points, nodes, controls=None, way_id=1, **tags):
  return osm.Road(tuple(point(*p) for p in points), {'highway': 'residential', **tags}, way_id, tuple(nodes), controls or {})


class TestTrafficControls(unittest.TestCase):
  def setUp(self):
    self.fix = osm.GpsFix(*point(0, 0), 0., 3., 100., speed=15.)

  def target(self, roads, fix=None):
    tracker = osm.RoadTracker()
    match = tracker.update(tuple(roads), fix or self.fix)
    return tracker.traffic_control(match) if match else None

  def test_distance_follows_geometry_and_does_not_require_speed_tag(self):
    r = road([(0, -20), (0, 100), (60, 100)], [1, 2, 3], {3: {'highway': 'stop', 'direction': 'forward'}})
    result = self.target([r])
    self.assertEqual(result.kind, 'stopSign')
    self.assertAlmostEqual(result.distance, 160., places=3)
    self.assertIsNone(osm.road_speed(r.tags, True))

  def test_forward_backward_both_and_unsupported_direction(self):
    for direction in ('forward', 'backward', 'both', '90', ''):
      for kind in ('stop', 'traffic_signals'):
        tags = {'highway': kind, f'{kind if kind == "stop" else "traffic_signals"}:direction': direction}
        r = road([(0, -100), (0, 50), (0, 200)], [1, 2, 3], {2: tags})
        self.assertEqual(self.target([r]) is not None, direction in ('forward', 'both'))
        reverse = osm.GpsFix(*point(0, 100), 180., 3., 100., speed=15.)
        self.assertEqual(self.target([r], reverse) is not None, direction in ('backward', 'both'))

  def test_oneway_undirected_and_reverse_oneway(self):
    r = road([(0, -100), (0, 50), (0, 200)], [1, 2, 3], {2: {'highway': 'stop'}}, oneway='yes')
    self.assertIsNotNone(self.target([r]))
    r = replace(r, tags={**r.tags, 'oneway': '-1'})
    self.assertIsNone(self.target([r]))
    reverse = osm.GpsFix(*point(0, 100), 180., 3., 100., speed=15.)
    self.assertIsNotNone(self.target([r], reverse))

  def test_central_signal_and_explicit_all_way_stop(self):
    for tags, expected in (({'highway': 'traffic_signals'}, True), ({'highway': 'stop'}, False),
                           ({'highway': 'stop', 'stop': 'all'}, True), ({'highway': 'stop', 'stop': 'minor'}, False)):
      main = road([(0, -100), (0, 50), (0, 200)], [1, 2, 3], {2: tags})
      side = road([(-100, 50), (0, 50), (100, 50)], [4, 2, 5], {2: tags}, way_id=2)
      self.assertEqual(self.target([main, side]) is not None, expected)

  def test_unique_continuation_walks_to_next_way(self):
    first = road([(0, -100), (0, 50)], [1, 2])
    second = road([(0, 50), (0, 100), (0, 200)], [2, 3, 4], {3: {'highway': 'traffic_signals', 'direction': 'forward'}}, way_id=2)
    result = self.target([first, second])
    self.assertEqual(result.node_id, 3)
    self.assertEqual(result.way_id, 2)
    self.assertAlmostEqual(result.distance, 100., places=3)

  def test_no_target_beyond_unresolved_fork(self):
    first = road([(0, -100), (0, 50)], [1, 2])
    left = road([(0, 50), (-20, 100), (-100, 150)], [2, 3, 4], {3: {'highway': 'stop', 'direction': 'forward'}}, way_id=2)
    right = road([(0, 50), (20, 100), (100, 150)], [2, 5, 6], way_id=3)
    self.assertIsNone(self.target([first, left, right]))

  def test_interior_junction_blocks_target_beyond_it(self):
    main = road([(0, -100), (0, 50), (0, 100), (0, 200)], [1, 2, 3, 4], {3: {'highway': 'stop', 'direction': 'forward'}})
    side = road([(0, 50), (100, 50)], [2, 5], way_id=2)
    self.assertIsNone(self.target([main, side]))

  def test_bridge_and_nearby_side_road_are_not_connected(self):
    main = road([(0, -100), (0, 50), (0, 200)], [1, 2, 3])
    bridge = road([(-100, 50), (0, 50), (100, 50)], [4, 5, 6], {5: {'highway': 'traffic_signals', 'direction': 'both'}}, way_id=2)
    self.assertIsNone(self.target([main, bridge]))

  def test_equal_speed_parallel_roads_need_target_agreement(self):
    main = road([(0, -100), (0, 50), (0, 200)], [1, 2, 3], {2: {'highway': 'stop', 'direction': 'forward'}}, maxspeed='50')
    rival = road([(5, -100), (5, 50), (5, 200)], [4, 5, 6], way_id=2, maxspeed='50')
    self.assertIsNone(self.target([main, rival]))

  def test_near_passed_target_keeps_signed_distance(self):
    r = road([(0, -100), (0, 50), (0, 200)], [1, 2, 3], {2: {'highway': 'stop', 'direction': 'forward'}})
    fix = replace(self.fix, latitude=point(0, 55)[0])
    self.assertAlmostEqual(self.target([r], fix).distance, -5., places=3)
    self.assertIsNone(self.target([r], replace(fix, latitude=point(0, 75)[0])))

  def test_only_requested_control_types(self):
    for tags in ({'highway': 'give_way'}, {'railway': 'level_crossing'}, {'highway': 'crossing', 'crossing': 'traffic_signals'}):
      r = road([(0, -100), (0, 50), (0, 200)], [1, 2, 3], {2: tags}, oneway='yes')
      self.assertIsNone(self.target([r]))

  def test_optional_provider_rejects_stale_cache_and_estimated_position(self):
    r = road([(0, -100), (0, 50), (0, 200)], [1, 2, 3], {2: {'highway': 'stop', 'direction': 'forward'}})
    service = osm.OSMSpeedLimit()
    service._thread = Mock()
    service._cache = ((r,), self.fix, 100.)
    service.update(self.fix, 100.)
    self.assertEqual(service.control.node_id, 2)
    service.update(replace(self.fix, timestamp=100.2, estimated=True), 100.2)
    self.assertIsNone(service.control)
    self.assertIsNone(service.control_match)
    service.update(replace(self.fix, timestamp=100.4, estimated=True, gps_timestamp=100.), 100.4)
    self.assertEqual(service.control.node_id, 2)
    service.update(replace(self.fix, timestamp=103.2, estimated=True, gps_timestamp=100.), 103.2)
    self.assertIsNone(service.control)
    service.update(replace(self.fix, timestamp=800.), 800.)
    self.assertIsNone(service.control)

  def test_fetches_tagged_road_nodes_and_preserves_geometry(self):
    payload = {'elements': [
      {'type': 'way', 'id': 11, 'nodes': [1, 2, 3], 'geometry': [{'lat': y, 'lon': 0} for y in (-.001, .0005, .002)],
       'tags': {'highway': 'residential'}},
      {'type': 'node', 'id': 2, 'lat': .0005, 'lon': 0, 'tags': {'highway': 'stop', 'direction': 'forward'}},
      {'type': 'node', 'id': 999, 'tags': {'highway': 'traffic_signals'}},
    ]}
    response = Mock()
    response.iter_content.return_value = [json.dumps(payload).encode()]
    context = Mock()
    context.__enter__ = Mock(return_value=response)
    context.__exit__ = Mock(return_value=False)
    with patch.object(osm.requests, 'post', return_value=context) as post:
      roads = osm.fetch_roads(self.fix)
    self.assertIn('node(w.roads)', post.call_args.kwargs['data']['data'])
    self.assertEqual(set(roads[0].control_tags), {2})
    self.assertEqual(roads[0].node_ids, (1, 2, 3))
    self.assertIsNotNone(self.target(roads))


if __name__ == '__main__':
  unittest.main()
