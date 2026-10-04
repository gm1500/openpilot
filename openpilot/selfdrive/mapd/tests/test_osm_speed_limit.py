import json
import math
import sys
import unittest
from concurrent.futures import Future
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock, patch

from openpilot.selfdrive.mapd import osm_speed_limit as osm


def road(speed="40", x=0., tags=None):
  # Northbound segment centred on (0, 0), coordinates expressed in metres.
  geometry = ((-.001, math.degrees(x / osm.EARTH_RADIUS)), (.001, math.degrees(x / osm.EARTH_RADIUS)))
  return osm.Road(geometry, {"highway": "residential", **({"maxspeed": speed} if speed is not None else {}), **(tags or {})})


def point(x, y):
  return math.degrees(y / osm.EARTH_RADIUS), math.degrees(x / osm.EARTH_RADIUS)


def polyline(points, speed='60', way_id=0, **tags):
  # Stable synthetic node IDs shared by ways meeting at the same fixture point.
  ids = tuple((round(x * 1000) + 1000000) * 2000001 + round(y * 1000) + 1000001 for x, y in points)
  return osm.Road(tuple(point(x, y) for x, y in points), {'highway': 'primary', **({'maxspeed': speed} if speed else {}), **tags}, way_id, ids)


def tracked_match(roads, fix, previous, previous_fix=None):
  tracker = osm.RoadTracker()
  tracker.update((previous,), previous_fix or replace(fix, timestamp=fix.timestamp - 1))
  return tracker.update(roads, fix)


class TestOSMSpeedLimit(unittest.TestCase):
  def setUp(self):
    self.fix = osm.GpsFix(0., 0., 0., 3., 100.)

  def test_units_and_unknown_values(self):
    for value in ("40", "40 km/h", "40 kmh", "40 kph"):
      self.assertAlmostEqual(osm.parse_speed(value), 40 / 3.6)
    self.assertAlmostEqual(osm.parse_speed("60 mph"), 60 * .44704)
    for value in ("", "none", "signals", "walk", "CA-AB:urban", "40;60", "40 @ wet", "0", "-5", "nan", "99999"):
      self.assertIsNone(osm.parse_speed(value), value)

  def test_tracker_keeps_curved_road_with_lagged_heading_near_other_limit(self):
    radius = 100
    positions = [(radius * (1 - math.cos(math.radians(a))), radius * math.sin(math.radians(a))) for a in range(0, 91, 5)]
    main = polyline([(0, -60), *positions], way_id=1)
    nearby = polyline([(radius - (radius - 12) * math.cos(math.radians(a)), (radius - 12) * math.sin(math.radians(a)))
                       for a in range(0, 91, 5)], '40', way_id=2, highway='primary_link')
    tracker = osm.RoadTracker()
    first = replace(self.fix, latitude=point(0, -30)[0], accuracy=15)
    self.assertEqual(tracker.update((main,), first).speed, 60 / 3.6)
    roads = (main, nearby)
    for i, angle in enumerate(range(0, 81, 5)):
      x, y = positions[i]
      fix = osm.GpsFix(*point(x, y), max(0, angle - 18), 15., 101. + i)
      match = tracker.update(roads, fix)
      self.assertIsNotNone(match, angle)
      self.assertEqual(match.road.way_id, 1)
      self.assertEqual(match.speed, 60 / 3.6)

  def test_tracker_follows_connected_short_ways_and_speed_change(self):
    before = polyline([(0, -60), (0, 0)], way_id=1)
    short = polyline([(0, 0), (0, 5)], way_id=2)
    after = polyline([(0, 5), (10, 30), (30, 60)], '40', way_id=3)
    rival = polyline([(12, -50), (12, 70)], '80', way_id=4)
    tracker = osm.RoadTracker()
    first = osm.GpsFix(*point(0, -5), 0., 15., 100.)
    self.assertIsNotNone(tracker.update((before,), first))
    fix = osm.GpsFix(*point(5, 17.5), 10., 15., 101.)
    match = tracker.update((before, short, after, rival), fix)
    self.assertIsNotNone(match)
    self.assertEqual(match.road.way_id, 3)
    self.assertEqual(match.speed, 40 / 3.6)

  def test_tracker_does_not_lock_to_main_road_after_ramp_departure(self):
    main = polyline([(0, -100), (0, 0), (0, 150)], '110', way_id=1)
    ramp = polyline([(0, 0), (50, 100)], '60', way_id=2, highway='primary_link')
    tracker = osm.RoadTracker()
    first = osm.GpsFix(*point(0, -50), 0., 15., 100.)
    roads = (main, ramp)
    self.assertAlmostEqual(tracker.update(roads, first).speed, 110 / 3.6)
    split = replace(first, latitude=0., timestamp=101.)
    self.assertIsNone(tracker.update(roads, split))
    departed = osm.GpsFix(*point(25, 50), 26.565, 15., 102.)
    self.assertAlmostEqual(tracker.update(roads, departed).speed, 60 / 3.6)
    far = osm.GpsFix(*point(40, 80), 26.565, 15., 103.)
    self.assertEqual(tracker.update(roads, far).speed, 60 / 3.6)

  def test_tracker_releases_on_equal_fork_unknown_road_and_parallel_jump(self):
    main = polyline([(0, -100), (0, 100)], way_id=1)
    rival = polyline([(12, -100), (12, 100)], '40', way_id=2)
    for candidate in (rival, replace(rival, tags={'highway': 'primary'})):
      tracker = osm.RoadTracker()
      first = osm.GpsFix(*point(0, -20), 0., 15., 100.)
      self.assertIsNotNone(tracker.update((main,), first))
      middle = osm.GpsFix(*point(6, -10), 0., 15., 101.)
      self.assertIsNone(tracker.update((main, candidate), middle))
      on_other = osm.GpsFix(*point(12, 0), 0., 15., 102.)
      self.assertIsNone(tracker.update((main, candidate), on_other))  # no connected path from old road

  def test_tracker_history_expires_and_frozen_fixes_cannot_requalify_it(self):
    main = road('60')
    rival = road('40', x=12)
    tracker = osm.RoadTracker()
    first = replace(self.fix, accuracy=15)
    tracker.update((main,), first)
    roads = (main, rival)
    bad = osm.GpsFix(*point(6, 10), 0., 15., 101.)
    for _ in range(50):
      self.assertIsNone(tracker.update(roads, bad))
    self.assertEqual(tracker.fix.timestamp, 100.)
    expired = osm.GpsFix(*point(0, 20), 0., 15., 104.)
    self.assertIsNone(tracker.update(roads, expired))
    self.assertIsNone(tracker.match)

  def test_tracker_refresh_keeps_identity_but_reads_new_tags(self):
    main = polyline([(0, -100), (0, 100)], '60', way_id=1)
    rival = polyline([(12, -100), (12, 100)], '40', way_id=2)
    tracker = osm.RoadTracker()
    first = replace(self.fix, accuracy=15)
    tracker.update((main,), first)
    updated = replace(main, tags={**main.tags, 'maxspeed': '80'})
    moved = osm.GpsFix(*point(0, 10), 18., 15., 101.)
    self.assertEqual(tracker.update((updated, rival), moved).speed, 80 / 3.6)
    moved_again = osm.GpsFix(*point(0, 20), 18., 15., 102.)
    shifted = replace(updated, geometry=(point(4, -100), point(4, 100)))
    self.assertIsNone(tracker.update((shifted, rival), moved_again))

  def test_tracker_directional_and_oneway_limits_survive_topology_tracking(self):
    main = polyline([(0, -100), (0, 100)], '60', way_id=1, **{'maxspeed:backward': '30', 'oneway': 'no'})
    tracker = osm.RoadTracker()
    first = osm.GpsFix(*point(0, 20), 180., 15., 100.)
    self.assertEqual(tracker.update((main,), first).speed, 30 / 3.6)
    moved = osm.GpsFix(*point(0, 10), 162., 15., 101.)
    rival = polyline([(12, -100), (12, 100)], '40')
    self.assertEqual(tracker.update((main, rival), moved).speed, 30 / 3.6)
    wrong_way = replace(main, tags={**main.tags, 'oneway': 'yes'})
    self.assertIsNone(tracker.update((wrong_way,), moved))

  def test_tracker_does_not_connect_coincident_geometry_without_shared_node(self):
    before = polyline([(0, -100), (0, 0)], way_id=1)
    after = polyline([(0, 0), (0, 100)], '40', way_id=2)
    unrelated = replace(after, node_ids=(90001, 90002))
    rival = polyline([(12, -100), (12, 100)], '80', way_id=3)
    for next_road in (unrelated, replace(after, node_ids=())):
      tracker = osm.RoadTracker()
      tracker.update((before,), osm.GpsFix(*point(0, -10), 0., 15., 100.))
      self.assertIsNone(tracker.update((before, next_road, rival), osm.GpsFix(*point(0, 10), 18., 15., 101.)))

  def test_directional_and_conditional_limits(self):
    r = road(tags={"maxspeed:forward": "50", "maxspeed:backward": "30"})
    self.assertAlmostEqual(osm.match_speed((r,), self.fix), 50 / 3.6)
    self.assertAlmostEqual(osm.match_speed((r,), replace(self.fix, bearing=180)), 30 / 3.6)
    self.assertIsNone(osm.match_speed((r,), replace(self.fix, bearing=None)))
    for key in ("maxspeed:conditional", "maxspeed:forward:conditional", "maxspeed:lanes", "maxspeed:variable", "maxspeed:motorcar"):
      self.assertIsNone(osm.match_speed((road(tags={key: "30"}),), self.fix), key)

  def test_advisory_and_legal_limits_are_independent(self):
    tags = {'maxspeed': '100', 'maxspeed:advisory': '50'}
    self.assertAlmostEqual(osm.road_speed(tags, True), 100 / 3.6)
    self.assertAlmostEqual(osm.road_speed(tags, True, advisory=True), 50 / 3.6)
    tags['maxspeed:advisory:conditional'] = '30 @ wet'
    self.assertAlmostEqual(osm.road_speed(tags, True), 100 / 3.6)
    self.assertIsNone(osm.road_speed(tags, True, advisory=True))
    del tags['maxspeed:advisory:conditional']
    tags['maxspeed:conditional'] = '80 @ wet'
    self.assertIsNone(osm.road_speed(tags, True))
    self.assertAlmostEqual(osm.road_speed(tags, True, advisory=True), 50 / 3.6)

  def test_advisory_units_direction_and_unsupported_values(self):
    tags = {'maxspeed:advisory:forward': '30 mph', 'maxspeed:advisory:backward': '40'}
    self.assertAlmostEqual(osm.road_speed(tags, True, advisory=True), 30 * .44704)
    self.assertAlmostEqual(osm.road_speed(tags, False, advisory=True), 40 / 3.6)
    self.assertIsNone(osm.road_speed(tags, None, advisory=True))
    self.assertIsNone(osm.road_speed(tags, True))
    for value in ('signals', '50;60', '30 @ wet', 'none', 'nan'):
      self.assertIsNone(osm.road_speed({'maxspeed:advisory': value}, True, advisory=True))

  def test_advisory_ramp_cannot_leak_onto_equal_limit_mainline(self):
    main = polyline([(0, -100), (0, 0), (0, 150)], '100', way_id=1, oneway='yes')
    ramp = polyline([(0, 0), (50, 100)], '100', way_id=2, oneway='yes',
                    highway='motorway_link', **{'maxspeed:advisory': '50'})
    roads = (main, ramp)
    tracker = osm.RoadTracker()
    tracker.update(roads, osm.GpsFix(*point(0, -30), 0., 15., 100.))
    self.assertIsNone(tracker.update(roads, replace(self.fix, accuracy=15, timestamp=101.)))
    # Same legal limit does not make the ramp advisory applicable to the mainline.
    match = tracker.update(roads, osm.GpsFix(*point(0, 30), 0., 15., 102.))
    self.assertEqual(match.road.way_id, 1)
    self.assertIsNone(match.advisory_speed)
    tracker.reset()
    tracker.update(roads, osm.GpsFix(*point(0, -30), 0., 15., 100.))
    tracker.update(roads, replace(self.fix, accuracy=15, timestamp=101.))
    match = tracker.update(roads, osm.GpsFix(*point(15, 30), 26.565, 15., 102.))
    self.assertAlmostEqual(match.advisory_speed, 50 / 3.6)

  def test_reacquire_ramp_after_fork_outlives_confirmed_history(self):
    main = polyline([(0, -100), (0, 0), (0, 200)], '100', way_id=1, oneway='yes')
    ramp = polyline([(0, 0), (20, 200)], None, way_id=2, oneway='yes',
                    highway='motorway_link', **{'maxspeed:advisory': '50'})
    roads = (main, ramp)
    tracker = osm.RoadTracker()
    tracker.update(roads, osm.GpsFix(*point(0, -30), 0., 15., 100.))
    for t, y in ((101., 0), (102., 10), (103., 20), (104., 30), (105., 70)):
      self.assertIsNone(tracker.update(roads, osm.GpsFix(*point(y / 10, y), 5.71, 15., t)))
    self.assertIsNone(tracker.match)
    # Only the second clear, moving fix re-establishes the road after expiry.
    match = tracker.update(roads, osm.GpsFix(*point(9, 90), 5.71, 15., 106.))
    self.assertEqual(match.road.way_id, 2)
    self.assertIsNone(match.speed)
    self.assertAlmostEqual(match.advisory_speed, 50 / 3.6)

  def test_tentative_ramp_match_requires_motion_freshness_and_connectivity(self):
    roads = (road('100'), road(None, x=12, tags={'highway': 'motorway_link', 'maxspeed:advisory': '50'}))
    first = osm.GpsFix(*point(12, 0), 0., 15., 100.)
    for second in (first, replace(first, timestamp=101.), replace(first, latitude=point(12, 2)[0], timestamp=101.),
                   replace(first, latitude=point(12, 10)[0], timestamp=104.),
                   osm.GpsFix(*point(0, 10), 0., 15., 101.), osm.GpsFix(*point(6, 10), 0., 15., 101.)):
      tracker = osm.RoadTracker()
      self.assertIsNone(tracker.update(roads, first))
      self.assertIsNone(tracker.update(roads, second))

  def test_onramp_advisory_clears_when_merging_to_mainline(self):
    main = polyline([(0, -100), (0, 0), (0, 100)], '100', way_id=1, oneway='yes')
    ramp = polyline([(50, -100), (0, 0)], None, way_id=2, oneway='yes',
                    highway='motorway_link', **{'maxspeed:advisory': '50'})
    roads = (main, ramp)
    tracker = osm.RoadTracker()
    first = tracker.update(roads, osm.GpsFix(*point(40, -80), 333.435, 15., 100.))
    self.assertAlmostEqual(first.advisory_speed, 50 / 3.6)
    merged = tracker.update(roads, osm.GpsFix(*point(0, 40), 0., 15., 102.))
    self.assertEqual(merged.road.way_id, 1)
    self.assertIsNone(merged.advisory_speed)

  def test_ramp_reacquisition_accumulates_motion_with_fast_gps(self):
    roads = (road('100'), road(None, x=12, tags={'maxspeed:advisory': '50'}))
    tracker = osm.RoadTracker()
    for i in range(5):
      self.assertIsNone(tracker.update(roads, osm.GpsFix(*point(12, i * 2), 0., 15., 100. + i / 10)))
    match = tracker.update(roads, osm.GpsFix(*point(12, 10), 0., 15., 100.5))
    self.assertAlmostEqual(match.advisory_speed, 50 / 3.6)

  def test_advisory_display_never_becomes_a_cruise_target(self):
    service = osm.OSMSpeedLimit()
    service._thread = Mock()
    service._result = (self.fix, 100 / 3.6, 50 / 3.6)
    self.assertAlmostEqual(service.update(self.fix, 100.), 50 / 3.6)
    self.assertTrue(service.display_is_advisory)
    self.assertAlmostEqual(service.control_sample[1], 100 / 3.6)
    moved = replace(self.fix, timestamp=101.)
    service._result = (moved, None, 50 / 3.6)
    self.assertAlmostEqual(service.update(moved, 101.), 50 / 3.6)
    self.assertIsNone(service.control_sample[1])
    service._result = (replace(moved, timestamp=102.), None, None)
    self.assertAlmostEqual(service.update(service._result[0], 102.), 50 / 3.6)
    self.assertTrue(service.display_is_advisory)
    self.assertEqual(service.display_timestamp, 101.)
    self.assertIsNone(service.update(service._result[0], 103.1))
    self.assertFalse(service.display_is_advisory)
    service._result = (replace(moved, timestamp=104.), 40 / 3.6, 50 / 3.6)
    self.assertAlmostEqual(service.update(service._result[0], 104.), 40 / 3.6)
    self.assertFalse(service.display_is_advisory)  # legal limit lower than advisory wins

  def test_oneway_heading_and_roundabouts(self):
    for tags in ({"oneway": "yes"}, {"junction": "roundabout"}):
      self.assertIsNone(osm.match_speed((road(tags=tags),), replace(self.fix, bearing=180)))
      self.assertAlmostEqual(osm.match_speed((road(tags=tags),), self.fix), 40 / 3.6)
    self.assertIsNone(osm.match_speed((road(tags={"oneway": "-1"}),), self.fix))
    self.assertIsNone(osm.match_speed((road(),), replace(self.fix, bearing=90)))

  def test_parallel_roads_unknown_roads_and_distance(self):
    self.assertIsNone(osm.match_speed((road(), road("60", x=5)), self.fix))
    self.assertIsNone(osm.match_speed((road(None), road("60", x=15)), self.fix))
    self.assertAlmostEqual(osm.match_speed((road(), road("60", x=20)), self.fix), 40 / 3.6)
    self.assertAlmostEqual(osm.match_speed((road(), road("40", x=5)), self.fix), 40 / 3.6)
    self.assertIsNone(osm.match_speed((road(x=30),), self.fix))
    self.assertIsNone(osm.match_speed((), self.fix))

  def test_speed_change_at_way_boundary_is_not_anticipated(self):
    old = osm.Road(((-.001, 0.), (0., 0.)), {"maxspeed": "60"})
    new = osm.Road(((0., 0.), (.001, 0.)), {"maxspeed": "40"})
    self.assertAlmostEqual(osm.match_speed((old, new), replace(self.fix, latitude=-.0002)), 60 / 3.6)
    self.assertIsNone(osm.match_speed((old, new), self.fix))
    self.assertAlmostEqual(osm.match_speed((old, new), replace(self.fix, latitude=.0002)), 40 / 3.6)

  def test_gps_freshness_and_internal_receiver_fallback(self):
    class SubMaster(dict):
      pass
    gps = SimpleNamespace(latitude=53., longitude=-113., speed=10., bearingDeg=0., bearingAccuracyDeg=5., horizontalAccuracy=3.,
                          hasFix=True, source='ublox')
    sources = ("gpsLocationExternal", "gpsLocation")
    sm = SubMaster({source: SimpleNamespace(**vars(gps)) for source in sources})
    sm.valid = dict.fromkeys(sources, True)
    sm.recv_time = dict.fromkeys(sources, 100.)
    sm.logMonoTime = dict.fromkeys(sources, int(100e9))
    sm.recv_frame = dict.fromkeys(sources, 20)
    sm["gpsLocationExternal"].hasFix = False
    self.assertEqual(osm.gps_fix(sm, 10, 101.).latitude, 53.)
    self.assertIsNone(osm.gps_fix(sm, 21, 101.))
    self.assertIsNone(osm.gps_fix(sm, 10, 104.))
    sm.logMonoTime["gpsLocation"] = int(90e9)
    self.assertIsNone(osm.gps_fix(sm, 10, 101.))  # stale publisher, fresh receive
    sm.logMonoTime["gpsLocation"] = int(100e9)
    for accuracy in (0., 16., math.nan):
      sm["gpsLocation"].horizontalAccuracy = accuracy
      self.assertIsNone(osm.gps_fix(sm, 10, 101.))

    # Internal Qualcomm fixes omit this field; external receivers still need it.
    internal = sm["gpsLocation"]
    internal.source, internal.horizontalAccuracy = 'qcomdiag', 0.
    self.assertEqual(osm.gps_fix(sm, 10, 101.).accuracy, osm.QCOM_UNKNOWN_ACCURACY)
    internal.hasFix = False
    self.assertIsNone(osm.gps_fix(sm, 10, 101.))
    internal.hasFix = True
    self.assertIsNone(osm.gps_fix(sm, 10, 104.))
    internal.horizontalAccuracy = 25.
    self.assertIsNone(osm.gps_fix(sm, 10, 101.))

  def test_display_clears_stale_distant_and_offroad_results(self):
    service = osm.OSMSpeedLimit()
    service._thread = Mock()
    service._result = (self.fix, 40 / 3.6, None)
    self.assertAlmostEqual(service.update(self.fix, 101.), 40 / 3.6)
    self.assertIsNone(service.update(self.fix, 104.))
    self.assertIsNone(service.update(replace(self.fix, latitude=.001), 101.))
    self.assertIsNone(service.update(replace(self.fix, bearing=180), 101.))
    self.assertIsNone(service.update(None, 101.))
    self.assertIsNone(service._result)

  def test_geometry_across_date_line(self):
    r = osm.Road(((0., 179.999), (0., -179.999)), {"maxspeed": "40"})
    fix = replace(self.fix, longitude=180., bearing=90.)
    self.assertAlmostEqual(osm.match_speed((r,), fix), 40 / 3.6)

  def test_highway_gps_step_holds_until_new_match_without_extending_age(self):
    service = osm.OSMSpeedLimit()
    service._thread = Mock()
    service._result = (self.fix, 110 / 3.6, None)
    self.assertAlmostEqual(service.update(self.fix, 100.), 110 / 3.6)
    service._wake.clear()
    moved = replace(self.fix, latitude=math.degrees(31 / osm.EARTH_RADIUS), timestamp=101.)
    self.assertAlmostEqual(service.update(moved, 101.), 110 / 3.6)
    self.assertTrue(service._wake.is_set())  # new fix wakes the matcher immediately
    service._result = (moved, None, None)
    self.assertAlmostEqual(service.update(moved, 101.8), 110 / 3.6)
    self.assertIsNone(service.update(replace(moved, timestamp=102.1), 102.1))

  def test_confirmed_limit_change_is_immediate(self):
    service = osm.OSMSpeedLimit()
    service._thread = Mock()
    service._result = (self.fix, 110 / 3.6, None)
    service.update(self.fix, 100.)
    moved = replace(self.fix, timestamp=101.)
    service._result = (moved, 60 / 3.6, None)
    self.assertAlmostEqual(service.update(moved, 101.), 60 / 3.6)

  def test_display_hold_is_not_a_confirmed_control_sample(self):
    service = osm.OSMSpeedLimit()
    service._thread = Mock()
    service._result = (self.fix, 110 / 3.6, None)
    service.update(self.fix, 100.)
    moved = replace(self.fix, latitude=math.degrees(31 / osm.EARTH_RADIUS), timestamp=101.)
    self.assertAlmostEqual(service.update(moved, 101.), 110 / 3.6)
    self.assertEqual(service.control_sample[0].timestamp, 100.)  # never restamp an older match
    self.assertAlmostEqual(service.update(moved, 101.4), 110 / 3.6)
    self.assertIsNone(service.control_sample)  # 0.3 s processing grace has expired
    service._result = (moved, None, None)  # fresh but ambiguous match, with sign still held
    self.assertAlmostEqual(service.update(moved, 101.5), 110 / 3.6)
    self.assertIsNone(service.control_sample[1])
    self.assertEqual(service.display_timestamp, 100.)
    self.assertIsNone(service.update(None, 101.6))
    self.assertIsNone(service.control_sample)

  def test_control_handoff_rejects_turns_and_position_jumps(self):
    for moved in (replace(self.fix, bearing=21, timestamp=101.), replace(self.fix, latitude=.001, timestamp=101.)):
      service = osm.OSMSpeedLimit()
      service._thread = Mock()
      service._result = (self.fix, 110 / 3.6, None)
      service.update(self.fix, 100.)
      service.update(moved, 101.)
      self.assertIsNone(service.control_sample)

  def test_hold_clears_on_turn_jump_invalid_fix_and_offroad(self):
    for fix, now in ((replace(self.fix, bearing=21, timestamp=101.), 101.),
                     (replace(self.fix, latitude=.001, timestamp=101.), 101.),
                     (self.fix, 104.), (None, 101.)):
      service = osm.OSMSpeedLimit()
      service._thread = Mock()
      service._result = (self.fix, 110 / 3.6, None)
      service.update(self.fix, 100.)
      self.assertIsNone(service.update(fix, now))
      self.assertIsNone(service._display)

  def test_ramp_continuity_requires_the_established_closer_aligned_road(self):
    main = road('110', tags={'highway': 'motorway'})
    ramp = road('80', x=12., tags={'highway': 'motorway_link'})
    fix = replace(self.fix, accuracy=15.)
    self.assertIsNone(osm.match_speed((main, ramp), fix))  # no prior road: keep strict ambiguity
    self.assertAlmostEqual(tracked_match((main, ramp), fix, main).speed, 110 / 3.6)
    self.assertIsNone(tracked_match((main, ramp), fix, ramp))  # cannot stick to the farther road
    on_ramp = replace(fix, longitude=math.degrees(11 / osm.EARTH_RADIUS))
    self.assertIsNone(tracked_match((main, ramp), on_ramp, main))
    self.assertIsNone(tracked_match((main, ramp), replace(fix, bearing=16), main))  # no motion evidence at standstill
    self.assertIsNone(tracked_match((main, road('80', x=2.)), fix, main))
    self.assertIsNone(tracked_match((main, ramp), replace(fix, bearing=None), main))
    self.assertIsNone(tracked_match((road(None), ramp), fix, road(None)).speed)

  def test_continuity_crosses_connected_way_split_but_not_unrelated_roads(self):
    before = polyline([(0, -100), (0, 0)], '110', highway='motorway')
    after = polyline([(0, 0), (0, 100)], '110', highway='motorway')
    ramp = road('80', x=12., tags={'highway': 'motorway_link'})
    fix = replace(self.fix, latitude=.0001, accuracy=15.)
    previous_fix = replace(fix, latitude=-.0001, timestamp=99.)
    self.assertAlmostEqual(tracked_match((before, after, ramp), fix, before, previous_fix).speed, 110 / 3.6)
    unrelated = road('110', x=30., tags={'highway': 'motorway'})
    self.assertIsNone(tracked_match((after, ramp), fix, unrelated, previous_fix))

  def test_continuity_releases_after_taking_ramp(self):
    main = road('110', tags={'highway': 'motorway'})
    ramp = osm.Road(((0., 0.), (.001, .0004)), {'highway': 'motorway_link', 'maxspeed': '80'})
    at_split = replace(self.fix, accuracy=15.)
    self.assertIsNone(tracked_match((main, ramp), at_split, main))
    on_ramp = replace(at_split, latitude=.0009, longitude=.00036, bearing=math.degrees(math.atan2(.4, 1.)))
    self.assertAlmostEqual(tracked_match((main, ramp), on_ramp, main).speed, 80 / 3.6)

  def test_bounded_fetch_and_incomplete_responses(self):
    element = {"type": "way", "tags": {"highway": "residential", "maxspeed": "40"},
               "geometry": [{"lat": -.001, "lon": 0.}, {"lat": .001, "lon": 0.}]}
    response = Mock()
    with patch.object(osm.requests, "post") as post:
      post.return_value.__enter__.return_value = response
      response.iter_content.return_value = [json.dumps({"elements": [element]}).encode()]
      self.assertEqual(osm.fetch_roads(self.fix), (road(),))
      self.assertIn('out body geom;', post.call_args.kwargs['data']['data'])
      element.update(id=123, nodes=[456, 789])
      response.iter_content.return_value = [json.dumps({"elements": [element]}).encode()]
      self.assertEqual(osm.fetch_roads(self.fix)[0].node_ids, (456, 789))
      self.assertEqual(osm.fetch_roads(self.fix)[0].way_id, 123)
      for body in (b'{"elements": [], "remark": "timeout"}', b'null', b'bad json', b'x' * (osm.MAX_RESPONSE_BYTES + 1)):
        response.iter_content.return_value = [body]
        with self.assertRaises(ValueError):
          osm.fetch_roads(self.fix)

  def test_worker_cache_prefetch_expiry_and_error_backoff(self):
    # Fake the network executor and clock; exercise the actual worker scheduling.
    class StopWorker(Exception):
      pass
    for fail in (False, True):
      service = osm.OSMSpeedLimit()
      service._fix = self.fix
      clock = [100.]
      calls = []
      results = []
      steps = iter((101., 129., 131., 161., 191., 400., 701., 702.))

      def submit(fn, fix, calls=calls, clock=clock, fail=fail):
        calls.append(clock[0])
        future = Future()
        if fail:
          future.set_exception(osm.requests.ConnectionError())
        else:
          future.set_result((road(),))
        return future

      def sleep(_, results=results, service=service, clock=clock, steps=steps):
        results.append(service._result)
        try:
          clock[0] = next(steps)
        except StopIteration as error:
          raise StopWorker from error
        service._fix = replace(self.fix, timestamp=clock[0])

      executor = Mock()
      executor.submit.side_effect = submit
      modules = {'openpilot.common.realtime': SimpleNamespace(drop_realtime=lambda: None),
                 'openpilot.common.swaglog': SimpleNamespace(cloudlog=Mock())}
      with patch.dict(sys.modules, modules), patch.object(osm, 'ThreadPoolExecutor') as pool, \
           patch.object(osm.time, 'monotonic', side_effect=lambda clock=clock: clock[0]), patch.object(service._wake, 'wait', side_effect=sleep):
        pool.return_value.__enter__.return_value = executor
        with self.assertRaises(StopWorker):
          service._run()
      if fail:
        self.assertTrue(all(b - a >= 30 for a, b in zip(calls, calls[1:], strict=False)))
        self.assertTrue(all(result[1] is None for result in results))
      else:
        self.assertEqual(calls, [100., 701.])  # stationary cache is reused until TTL
        self.assertAlmostEqual(results[1][1], 40 / 3.6)
        self.assertIsNone(results[-2][1])  # expired map is hidden while refreshing


if __name__ == "__main__":
  unittest.main()
