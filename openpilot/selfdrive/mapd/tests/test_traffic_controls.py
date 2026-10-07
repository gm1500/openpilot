import json
import math
import unittest
from dataclasses import replace
from unittest.mock import Mock, patch

from openpilot.selfdrive.mapd import osm_speed_limit as osm
from openpilot.selfdrive.mapd.control_tags import COMPASS, SIGN_KEYS, road_controls


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

  def test_side_street_does_not_hide_target_on_clear_straight_road(self):
    main = road([(0, -100), (0, 50), (0, 100), (0, 200)], [1, 2, 3, 4], {3: {'highway': 'stop', 'direction': 'forward'}})
    side = road([(0, 50), (100, 50)], [2, 5], way_id=2)
    result = self.target([main, side])
    self.assertEqual(result.node_id, 3)
    self.assertAlmostEqual(result.distance, 100., places=3)

  def test_straight_continuation_across_way_boundary_and_reverse_direction(self):
    first = road([(0, -100), (0, 50)], [1, 2])
    for reverse in (False, True):
      points, nodes = [(0, 50), (0, 100), (0, 200)], [2, 3, 4]
      if reverse:
        points.reverse()
        nodes.reverse()
      onward = road(points, nodes, {3: {'highway': 'traffic_signals', 'direction': 'backward' if reverse else 'forward'}}, way_id=2)
      cross = road([(-100, 50), (0, 50), (100, 50)], [5, 2, 6], way_id=3)
      result = self.target([first, onward, cross])
      self.assertEqual(result.node_id, 3)
      self.assertAlmostEqual(result.distance, 100., places=3)

  def test_shallow_fork_and_cross_street_target_remain_rejected(self):
    main = road([(0, -100), (0, 50), (0, 200)], [1, 2, 3])
    for x, y in ((25, 150), (100, 50)):
      side = road([(0, 50), (x, y)], [2, 4], {4: {'highway': 'stop', 'direction': 'forward'}}, way_id=2)
      self.assertIsNone(self.target([main, side]))
    # A target on the straight branch cannot break an otherwise shallow tie.
    main = replace(main, control_tags={3: {'highway': 'stop', 'direction': 'forward'}})
    fork = road([(0, 50), (25, 150)], [2, 4], way_id=2)
    tracker = osm.RoadTracker()
    match = tracker.update((main, fork), self.fix)
    self.assertIsNone(tracker.traffic_control(match))
    self.assertEqual(tracker.control_reason, 'ambiguousFork')

  def test_control_rejection_reason_distinguishes_direction_and_missing_data(self):
    tracker = osm.RoadTracker()
    for tags, reason in (({'highway': 'stop'}, 'controlDirection'), ({}, 'noControl')):
      main = road([(0, -100), (0, 50), (0, 200)], [1, 2, 3], {2: tags} if tags else {})
      match = tracker.update((main,), self.fix)
      self.assertIsNone(tracker.traffic_control(match))
      self.assertEqual(tracker.control_reason, reason)

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
    for tags in ({'highway': 'give_way'}, {'railway': 'level_crossing'}, {'highway': 'crossing', 'crossing': 'uncontrolled'},
                 {'highway': 'crossing', 'crossing': 'marked'}, {'highway': 'footway', 'crossing': 'traffic_signals'}):
      r = road([(0, -100), (0, 50), (0, 200)], [1, 2, 3], {2: tags}, oneway='yes')
      self.assertIsNone(self.target([r]))

  def test_signal_controlled_crossing_on_two_way_road(self):
    tags = {'highway': 'crossing', 'crossing': 'traffic_signals', 'crossing:markings': 'zebra'}
    r = road([(0, -100), (0, 50), (0, 200)], [1, 2, 3], {2: tags})
    reverse = replace(self.fix, latitude=point(0, 100)[0], bearing=180.)
    for fix in (self.fix, reverse):
      result = self.target([r], fix)
      self.assertEqual(result.kind, 'trafficLight')
      self.assertEqual(result.node_id, 2)
      self.assertAlmostEqual(result.distance, 50., places=3)
    directed = replace(r, control_tags={2: {**tags, 'traffic_signals:direction': 'backward'}})
    self.assertIsNone(self.target([directed]))
    self.assertIsNotNone(self.target([directed], reverse))

  def test_signal_controlled_crossing_must_belong_to_current_road(self):
    main = road([(0, -100), (0, 50), (0, 200)], [1, 2, 3])
    side = road([(-100, 50), (0, 50), (100, 50)], [4, 2, 5],
                {4: {'highway': 'crossing', 'crossing': 'traffic_signals'}}, way_id=2)
    self.assertIsNone(self.target([main, side]))

  def test_alternate_signal_crossings_and_explicit_no(self):
    variants = [{'crossing:signals': 'yes'}, {'crossing': 'marked', 'crossing:signals': 'yes'},
                {'crossing': 'unmarked', 'crossing:signals': 'yes'}]
    variants += [{'crossing_ref': name} for name in ('pelican', 'puffin', 'toucan', 'pegasus')]
    for tags in variants:
      with self.subTest(tags=tags):
        tags = {'highway': 'crossing', **tags}
        r = road([(0, -100), (0, 50), (0, 200)], [1, 2, 3], {2: tags})
        for fix in (self.fix, replace(self.fix, latitude=point(0, 100)[0], bearing=180.)):
          self.assertEqual(self.target([r], fix).kind, 'trafficLight')
        denied = replace(r, control_tags={2: {**tags, 'crossing:signals': 'no'}})
        self.assertIsNone(self.target([denied]))
    for tags in ({'crossing_ref': 'pxo'}, {'crossing_ref': 'zebra'}, {'crossing_ref': 'tiger'},
                 {'crossing_ref': 'pelican', 'crossing': 'uncontrolled'}):
      self.assertIsNone(osm.control_kind({'highway': 'crossing', **tags}))

  def test_stop_sign_codes_lists_and_no_substring_matches(self):
    values = ('stop', 'US:R1-1', 'DE:206', 'CA:Ra-1', 'CA:AB:RA-1', 'CA:ON:Ra-1',
              'CA:BC:R-001', 'CA:QC:P-010', 'CA:QC:P-010A', 'US:R1-1;US:R1-3P',
              'US:R1-3P,R1-1', 'US:R1-3P;R1-1', 'CA:AB:RA-1-T;RA-1', 'stop;US:R1-3P')
    for key in ('traffic_sign', 'traffic_sign:id'):
      for value in values:
        with self.subTest(key=key, value=value):
          r = road([(0, -100), (0, 50), (0, 200)], [1, 2, 3], {2: {key: value, 'traffic_sign:direction': 'forward'}})
          self.assertEqual(self.target([r]).kind, 'stopSign')
          reverse = replace(self.fix, latitude=point(0, 100)[0], bearing=180.)
          self.assertIsNone(self.target([r], reverse))
    for value in ('stop_ahead', 'bus_stop', 'US:W3-1', 'US:R1-11', 'US:R1-3P', 'DE:205', 'CA:RA-2',
                  'CA:AB:RA-1-T', 'CA:AB:RA-10', 'R1-1', 'ZZ:R1-1', 'US:R1-1[100 m]'):
      with self.subTest(value=value):
        self.assertIsNone(osm.control_kind({'traffic_sign': value}))
    self.assertIsNone(osm.control_kind({'highway': 'give_way', 'traffic_sign': 'US:R1-1'}))

  def test_directional_stop_sign_keys_and_conflicts(self):
    reverse = replace(self.fix, latitude=point(0, 100)[0], bearing=180.)
    cases = [({'traffic_sign:forward': 'stop'}, (True, False)),
             ({'traffic_sign:backward': 'CA:AB:RA-1'}, (False, True)),
             ({'traffic_sign:forward': 'US:R1-1', 'traffic_sign:backward': 'give_way'}, (True, False)),
             ({'traffic_sign:forward': 'stop', 'traffic_sign:backward': 'stop'}, (True, True)),
             ({'traffic_sign': 'stop', 'traffic_sign:forward': 'give_way'}, (False, True)),
             ({'traffic_sign:forward': 'stop', 'direction': 'backward'}, (False, False)),
             ({'highway': 'stop', 'stop:direction': 'forward', 'direction': 'backward'}, (False, False)),
             ({'traffic_sign': 'stop', 'traffic_sign:direction': '90'}, (False, False))]
    for tags, expected in cases:
      with self.subTest(tags=tags):
        r = road([(0, -100), (0, 50), (0, 200)], [1, 2, 3], {2: tags})
        self.assertEqual(tuple(self.target([r], fix) is not None for fix in (self.fix, reverse)), expected)

  def test_all_way_sign_tabs_and_signal_stop_beacons(self):
    for tags in ({'traffic_sign': 'US:R1-1;R1-3P'}, {'traffic_sign': 'CA:AB:RA-1;RA-1-T'},
                 {'highway': 'traffic_signals', 'traffic_signals': 'stop'},
                 {'highway': 'traffic_signals', 'traffic_signals': 'blinker', 'stop': 'all'}):
      with self.subTest(tags=tags):
        main = road([(0, -100), (0, 50), (0, 200)], [1, 2, 3], {2: tags})
        side = road([(-100, 50), (0, 50), (100, 50)], [4, 2, 5], {2: tags}, way_id=2)
        self.assertEqual(self.target([main, side]).kind, 'stopSign')
    tags = {'highway': 'traffic_signals', 'traffic_signals': 'stop', 'traffic_signals:direction': 'backward'}
    main = road([(0, -100), (0, 50), (0, 200)], [1, 2, 3], {2: tags}, oneway='yes')
    self.assertIsNone(self.target([main]))

  def test_special_signal_types_and_inactive_controls(self):
    for subtype in ('signal', 'traffic_lights', 'secondary', 'blink_mode', 'pedestrian_crossing', 'cyclist_crossing', 'hawk', 'emergency'):
      tags = {'highway': 'traffic_signals', 'traffic_signals': subtype, 'direction': 'forward'}
      r = road([(0, -100), (0, 50), (0, 200)], [1, 2, 3], {2: tags})
      self.assertEqual(self.target([r]).kind, 'trafficLight')
    for subtype in ('continuous_green', 'blinker', 'ramp_meter', 'level_crossing', 'train_priority', 'bridge', 'movable_bridge', 'no'):
      self.assertIsNone(osm.control_kind({'highway': 'traffic_signals', 'traffic_signals': subtype}))
    for inactive in ('disused', 'abandoned', 'removed', 'demolished', 'construction', 'proposed'):
      self.assertIsNone(osm.control_kind({'highway': 'stop', 'traffic_sign': 'stop', inactive: 'yes'}))
      self.assertIsNone(osm.control_kind({f'{inactive}:highway': 'traffic_signals'}))

  def test_legacy_compass_stops_select_approach_side_not_travel_direction(self):
    tags = {'highway': 'stop', 'stop': 'E;W'}
    ns = road([(0, -100), (0, 0), (0, 100)], [1, 2, 3], {2: tags})
    ew = road([(-100, 0), (0, 0), (100, 0)], [4, 2, 5], {2: tags}, way_id=2)
    for x, y, heading, expected in ((0, -50, 0, False), (0, 50, 180, False), (-50, 0, 90, True), (50, 0, 270, True)):
      fix = replace(self.fix, latitude=point(x, y)[0], longitude=point(x, y)[1], bearing=heading)
      result = self.target([ns, ew], fix)
      self.assertEqual(result is not None, expected)
      if expected:
        self.assertEqual(result.node_id, 2)
        self.assertAlmostEqual(result.distance, 50., places=3)
    # Each of the 16 cardinal labels means the side the car approaches FROM.
    for name, bearing in COMPASS.items():
      with self.subTest(name=name):
        x, y = 100 * math.sin(math.radians(bearing)), 100 * math.cos(math.radians(bearing))
        r = road([(x, y), (0, 0), (-x, -y)], [1, 2, 3], {2: {'highway': 'stop', 'stop': name}})
        fix = replace(self.fix, latitude=point(x/2, y/2)[0], longitude=point(x/2, y/2)[1], bearing=(bearing+180) % 360)
        self.assertIsNotNone(self.target([r], fix))
        opposite = replace(fix, latitude=-fix.latitude, longitude=-fix.longitude, bearing=bearing)
        self.assertIsNone(self.target([r], opposite))

  def test_compass_stops_reject_ambiguous_arms_and_bad_values(self):
    main = road([(0, -100), (0, 0), (0, 100)], [1, 2, 3], {2: {'highway': 'stop', 'stop': 'S'}})
    shallow = road([(-20, -100), (0, 0)], [4, 2], {2: {'highway': 'stop', 'stop': 'S'}}, way_id=2)
    fix = replace(self.fix, latitude=point(0, -50)[0])
    self.assertIsNone(self.target([main, shallow], fix))
    for value in ('no', 'yes', 'N;invalid', 'E;', 'minor;N'):
      r = replace(main, control_tags={2: {'highway': 'stop', 'stop': value, 'direction': 'both'}})
      self.assertIsNone(self.target([r], fix))

  def test_legacy_way_end_stops_do_not_spread_to_cross_streets(self):
    for value in ('yes', '-1', 'both'):
      with self.subTest(value=value):
        tags = {'highway': 'residential', 'stop': value}
        r = road([(0, -100), (0, 0), (0, 100)], [1, 2, 3])
        r = replace(r, tags=tags, control_tags=road_controls(r.node_ids, tags, {}))
        a = self.target([r], replace(self.fix, latitude=point(0, -50)[0]))
        b = self.target([r], replace(self.fix, latitude=point(0, 50)[0], bearing=180.))
        self.assertEqual(a.node_id if a else None, 3 if value in ('yes', 'both') else None)
        self.assertEqual(b.node_id if b else None, 1 if value in ('-1', 'both') else None)
        side = road([(-100, 100), (0, 100), (100, 100)], [4, 3, 5], way_id=2)
        side_fix = replace(self.fix, latitude=point(-50, 100)[0], longitude=point(-50, 100)[1], bearing=90.)
        self.assertIsNone(self.target([r, side], side_fix))
    self.assertEqual(road_controls((1, 2, 1), {'stop': 'both'}, {}), {})
    self.assertEqual(road_controls((), {'stop': 'both'}, {}), {})
    self.assertEqual(road_controls((1, 2), {'stop': 'yes', 'disused': 'yes'}, {}), {})
    self.assertEqual(road_controls((1, 2), {'stop': 'yes'}, {2: {'highway': 'give_way'}}), {})
    explicit = {2: {'highway': 'stop', 'direction': 'backward'}}
    self.assertEqual(road_controls((1, 2), {'stop': 'yes'}, explicit), explicit)

  def test_optional_provider_rejects_stale_cache_and_estimated_position(self):
    r = road([(0, -100), (0, 50), (0, 200)], [1, 2, 3], {2: {'highway': 'stop', 'direction': 'forward'}})
    service = osm.OSMSpeedLimit()
    service._thread = Mock()
    service._cache = ((r,), self.fix, 100.)
    service.update(self.fix, 100.)
    self.assertEqual(service.control.node_id, 2)
    self.assertEqual(service.control_reason, 'target')
    service.update(replace(self.fix, timestamp=100.2, estimated=True), 100.2)
    self.assertIsNone(service.control)
    self.assertIsNone(service.control_match)
    service.update(replace(self.fix, timestamp=100.4, estimated=True, gps_timestamp=100.), 100.4)
    self.assertEqual(service.control.node_id, 2)
    service.update(replace(self.fix, timestamp=103.2, estimated=True, gps_timestamp=100.), 103.2)
    self.assertIsNone(service.control)
    self.assertEqual(service.control_reason, 'gpsStale')
    service.update(replace(self.fix, timestamp=800.), 800.)
    self.assertIsNone(service.control)
    self.assertEqual(service.control_reason, 'cacheUnavailable')

  def test_fetches_tagged_road_nodes_and_preserves_geometry(self):
    payload = {'elements': [
      {'type': 'way', 'id': 11, 'nodes': [1, 2, 3], 'geometry': [{'lat': y, 'lon': 0} for y in (-.001, .0005, .002)],
       'tags': {'highway': 'residential'}},
      {'type': 'node', 'id': 2, 'lat': .0005, 'lon': 0, 'tags': {'highway': 'stop', 'direction': 'forward'}},
      {'type': 'node', 'id': 999, 'tags': {'highway': 'crossing', 'crossing': 'traffic_signals'}},
    ]}
    response = Mock()
    response.iter_content.return_value = [json.dumps(payload).encode()]
    context = Mock()
    context.__enter__ = Mock(return_value=response)
    context.__exit__ = Mock(return_value=False)
    for tags in ({'highway': 'stop', 'direction': 'forward'}, {'highway': 'crossing', 'crossing': 'traffic_signals'},
                 {'highway': 'crossing', 'crossing:signals': 'yes'}, {'highway': 'crossing', 'crossing_ref': 'pelican'},
                 {'traffic_sign': 'CA:AB:RA-1', 'traffic_sign:direction': 'forward'}, {'traffic_sign:forward': 'US:R1-1'},
                 {'traffic_sign:id': 'DE:206', 'direction': 'forward'}):
      with self.subTest(tags=tags):
        payload['elements'][1]['tags'] = tags
        response.iter_content.return_value = [json.dumps(payload).encode()]
        with patch.object(osm.requests, 'post', return_value=context) as post:
          roads = osm.fetch_roads(self.fix)
        self.assertIn('node(w.roads)["highway"="crossing"]["crossing"="traffic_signals"]', post.call_args.kwargs['data']['data'])
        self.assertIn('node(w.roads)["highway"="crossing"]["crossing:signals"="yes"]', post.call_args.kwargs['data']['data'])
        self.assertIn('["crossing_ref"~"^(pelican|puffin|toucan|pegasus)$"]', post.call_args.kwargs['data']['data'])
        for key in SIGN_KEYS:
          self.assertIn(f'node(w.roads)["{key}"]', post.call_args.kwargs['data']['data'])
        self.assertEqual(set(roads[0].control_tags), {2})
        self.assertEqual(roads[0].node_ids, (1, 2, 3))
        self.assertIsNotNone(self.target(roads))

  def test_fetch_legacy_way_stop_and_conflicting_or_malformed_node(self):
    payload = {'elements': [
      {'type': 'way', 'id': 11, 'nodes': [1, 2], 'geometry': [{'lat': y, 'lon': 0} for y in (-.001, .002)],
       'tags': {'highway': 'residential', 'stop': 'yes'}},
      {'type': 'node', 'id': 2, 'tags': {}},
      {'type': 'node', 'id': 999, 'tags': {'traffic_sign:forward': 'stop'}},
    ]}
    response = Mock()
    context = Mock()
    context.__enter__ = Mock(return_value=response)
    context.__exit__ = Mock(return_value=False)
    for tags, expected in (({}, True), ({'highway': 'give_way'}, False),
                           ({'traffic_sign': 'stop', 'direction': 'backward'}, False)):
      with self.subTest(tags=tags):
        payload['elements'][1]['tags'] = tags
        response.iter_content.return_value = [json.dumps(payload).encode()]
        with patch.object(osm.requests, 'post', return_value=context):
          roads = osm.fetch_roads(self.fix)
        self.assertEqual(self.target(roads) is not None, expected)
        self.assertNotIn(999, roads[0].control_tags)
    payload['elements'][0]['tags'].pop('stop')
    payload['elements'][1]['tags'] = {'traffic_sign': None}
    response.iter_content.return_value = [json.dumps(payload).encode()]
    with patch.object(osm.requests, 'post', return_value=context):
      self.assertEqual(osm.fetch_roads(self.fix)[0].control_tags, {})


if __name__ == '__main__':
  unittest.main()
