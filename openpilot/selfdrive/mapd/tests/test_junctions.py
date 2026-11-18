import json
import math
import unittest
from dataclasses import replace
from unittest.mock import Mock, patch

from openpilot.selfdrive.mapd import osm_speed_limit as osm


def point(x, y):
  return math.degrees(y / osm.EARTH_RADIUS), math.degrees(x / osm.EARTH_RADIUS)


def road(points, nodes, way_id=1, **tags):
  return osm.Road(tuple(point(*p) for p in points), {'highway': 'residential', **tags}, way_id, tuple(nodes))


class TestJunctions(unittest.TestCase):
  def setUp(self):
    self.fix = osm.GpsFix(*point(0, 0), 0., 3., 100., speed=15.)
    self.main = road([(0, -100), (0, 50), (0, 200)], [1, 2, 3])
    self.side = road([(0, 50), (100, 50)], [2, 4], way_id=2)

  def target(self, roads, fix=None):
    tracker = osm.RoadTracker()
    match = tracker.update(tuple(roads), fix or self.fix)
    return tracker.junction_ahead(match) if match else None

  def test_untagged_t_and_cross_intersections_in_either_direction(self):
    for side in (self.side, road([(-100, 50), (0, 50), (100, 50)], [5, 2, 4], way_id=2)):
      for fix in (self.fix, replace(self.fix, latitude=point(0, 100)[0], bearing=180.)):
        with self.subTest(side=side.geometry, bearing=fix.bearing):
          result = self.target([self.main, side], fix)
          self.assertEqual((result.kind, result.node_id), ('junction', 2))
          self.assertAlmostEqual(result.distance, 50., places=3)
          self.assertIsNone(osm.road_speed(self.main.tags, True))

  def test_shallow_split_is_itself_the_target_without_choosing_an_exit(self):
    first = road([(0, -100), (0, 50)], [1, 2], oneway='yes')
    left = road([(0, 50), (-10, 150)], [2, 3], way_id=2, oneway='yes')
    right = road([(0, 50), (10, 150)], [2, 4], way_id=3, oneway='yes')
    result = self.target([first, left, right])
    self.assertEqual(result.node_id, 2)
    self.assertAlmostEqual(result.distance, 50., places=3)
    # Classification/name differences cannot conceal the split.
    for tags in ({'highway': 'motorway_link'}, {'highway': 'secondary_link'}, {'name': 'Another Road'}):
      self.assertEqual(self.target([first, left, replace(right, tags={**right.tags, **tags})]).node_id, 2)

  def test_incoming_merge_is_a_target_even_with_only_one_forward_exit(self):
    main = replace(self.main, tags={**self.main.tags, 'oneway': 'yes'})
    ramp = road([(40, -50), (0, 50)], [4, 2], way_id=2, highway='motorway_link', oneway='yes')
    self.assertEqual(self.target([main, ramp]).node_id, 2)

  def test_t_junction_with_no_straight_continuation_is_a_target(self):
    first = road([(0, -100), (0, 50)], [1, 2])
    cross = road([(-100, 50), (0, 50), (100, 50)], [4, 2, 5], way_id=2)
    self.assertEqual(self.target([first, cross]).node_id, 2)

  def test_plain_way_boundaries_speed_changes_and_bends_are_not_junctions(self):
    first = road([(0, -100), (0, 50)], [1, 2], maxspeed='60')
    for tail in ([(0, 50), (0, 150)], [(0, 50), (100, 50)]):
      second = road(tail, [2, 3], way_id=2, maxspeed='40')
      self.assertIsNone(self.target([first, second]))
    self.assertIsNone(self.target([self.main]))

  def test_distance_follows_connected_geometry_across_way_boundaries(self):
    first = road([(0, -100), (0, 50)], [1, 2])
    middle = road([(0, 50), (60, 50)], [2, 3], way_id=2)
    onward = road([(60, 50), (60, 150), (60, 200)], [3, 4, 5], way_id=3)
    side = road([(60, 150), (160, 150)], [4, 6], way_id=4)
    result = self.target([first, middle, onward, side])
    self.assertEqual((result.node_id, result.way_id), (4, 3))
    self.assertAlmostEqual(result.distance, 210., places=3)

  def test_bridge_coincidence_and_missing_node_ids_are_not_connections(self):
    bridge = road([(-100, 50), (0, 50), (100, 50)], [4, 5, 6], way_id=2)
    self.assertIsNone(self.target([self.main, bridge]))
    for roads in ([replace(self.main, node_ids=()), self.side], [self.main, replace(self.side, node_ids=())]):
      self.assertIsNone(self.target(roads))

  def test_duplicate_way_edges_do_not_add_junction_arms(self):
    tracker = osm.RoadTracker()
    tracker._set_roads((self.main, replace(self.main, way_id=2)))
    self.assertEqual(tracker._junctions, set())
    self.assertIsNone(self.target([self.main, replace(self.main, way_id=2)]))

  def test_nearest_junction_precedes_later_intersection(self):
    main = road([(0, -100), (0, 50), (0, 100), (0, 200)], [1, 2, 3, 4])
    second = road([(0, 100), (100, 100)], [3, 5], way_id=3)
    self.assertEqual(self.target([main, self.side, second]).node_id, 2)
    result = self.target([main, self.side, second], replace(self.fix, latitude=point(0, 80)[0]))
    self.assertEqual(result.node_id, 3)
    self.assertAlmostEqual(result.distance, 20., places=3)

  def test_recently_passed_junction_keeps_signed_distance_for_release(self):
    for y, expected in ((55, -5.), (75, None)):
      result = self.target([self.main, self.side], replace(self.fix, latitude=point(0, y)[0]))
      if expected is None:
        self.assertIsNone(result)
      else:
        self.assertAlmostEqual(result.distance, expected, places=3)

  def test_parallel_road_targets_need_agreement(self):
    rival = road([(5, -100), (5, 200)], [10, 11], way_id=3)
    self.assertIsNone(self.target([self.main, self.side, rival]))

  def test_direction_alignment_distance_and_search_bounds_remain(self):
    main = replace(self.main, tags={**self.main.tags, 'oneway': 'yes'})
    self.assertIsNone(self.target([main, self.side], replace(self.fix, bearing=180.)))
    for fix in (replace(self.fix, bearing=None), replace(self.fix, bearing=25.),
                replace(self.fix, longitude=point(22, 0)[1])):
      self.assertIsNone(self.target([self.main, self.side], fix))
    far = road([(0, -100), (0, 1100), (0, 1200)], [1, 2, 3])
    side = road([(0, 1100), (100, 1100)], [2, 4], way_id=2)
    self.assertIsNone(self.target([far, side]))

  def test_cache_refresh_rebuilds_connectivity(self):
    tracker = osm.RoadTracker()
    match = tracker.update((self.main, self.side), self.fix)
    self.assertEqual(tracker.junction_ahead(match).node_id, 2)
    match = tracker.update((self.main,), self.fix)
    self.assertIsNone(tracker.junction_ahead(match))
    self.assertEqual(tracker.control_reason, 'noJunction')

  def test_provider_requires_real_gps_anchor_and_fresh_cache(self):
    service = osm.OSMSpeedLimit()
    service._thread = Mock()
    service._cache = ((self.main, self.side), self.fix, 100.)
    service.update(self.fix, 100.)
    self.assertEqual(service.control.node_id, 2)
    service.update(replace(self.fix, estimated=True, gps_timestamp=0.), 100.)
    self.assertIsNone(service.control)
    service.update(replace(self.fix, estimated=True, gps_timestamp=99.), 100.)
    self.assertEqual(service.control.node_id, 2)
    service.update(replace(self.fix, timestamp=104., estimated=True, gps_timestamp=100.), 104.)
    self.assertIsNone(service.control)
    self.assertEqual(service.control_reason, 'gpsStale')
    service._cache = ((self.main, self.side), self.fix, -1000.)
    service.update(self.fix, 100.)
    self.assertIsNone(service.control)
    self.assertEqual(service.control_reason, 'cacheUnavailable')

  def test_fetch_needs_only_ways_and_ignores_stop_light_tags(self):
    payload = {'elements': [
      {'type': 'way', 'id': 11, 'nodes': [1, 2, 3], 'geometry': [{'lat': y, 'lon': 0.} for y in (-.001, .0005, .002)],
       'tags': {'highway': 'residential'}},
      {'type': 'way', 'id': 12, 'nodes': [2, 4], 'geometry': [{'lat': .0005, 'lon': x} for x in (0., .001)],
       'tags': {'highway': 'service'}},
      {'type': 'node', 'id': 2, 'tags': {}},
    ]}
    response = Mock()
    context = Mock(__enter__=Mock(return_value=response), __exit__=Mock(return_value=False))
    for tags in ({}, {'highway': 'stop', 'direction': 'backward'}, {'highway': 'traffic_signals'}, {'highway': 'give_way'}):
      with self.subTest(tags=tags):
        payload['elements'][-1]['tags'] = tags
        response.iter_content.return_value = [json.dumps(payload).encode()]
        with patch.object(osm.requests, 'post', return_value=context) as post:
          roads = osm.fetch_roads(self.fix)
        query = post.call_args.kwargs['data']['data']
        self.assertNotIn('node(w.roads)', query)
        self.assertNotIn('traffic_sign', query)
        self.assertEqual(self.target(roads).node_id, 2)
    # Tags alone on a road cannot create an intersection.
    payload['elements'].pop(1)
    response.iter_content.return_value = [json.dumps(payload).encode()]
    with patch.object(osm.requests, 'post', return_value=context):
      self.assertIsNone(self.target(osm.fetch_roads(self.fix)))


if __name__ == '__main__':
  unittest.main()
