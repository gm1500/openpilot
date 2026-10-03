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


class TestOSMSpeedLimit(unittest.TestCase):
  def setUp(self):
    self.fix = osm.GpsFix(0., 0., 0., 3., 100.)

  def test_units_and_unknown_values(self):
    for value in ("40", "40 km/h", "40 kmh", "40 kph"):
      self.assertAlmostEqual(osm.parse_speed(value), 40 / 3.6)
    self.assertAlmostEqual(osm.parse_speed("60 mph"), 60 * .44704)
    for value in ("", "none", "signals", "walk", "CA-AB:urban", "40;60", "40 @ wet", "0", "-5", "nan", "99999"):
      self.assertIsNone(osm.parse_speed(value), value)

  def test_directional_and_conditional_limits(self):
    r = road(tags={"maxspeed:forward": "50", "maxspeed:backward": "30"})
    self.assertAlmostEqual(osm.match_speed((r,), self.fix), 50 / 3.6)
    self.assertAlmostEqual(osm.match_speed((r,), replace(self.fix, bearing=180)), 30 / 3.6)
    self.assertIsNone(osm.match_speed((r,), replace(self.fix, bearing=None)))
    for key in ("maxspeed:conditional", "maxspeed:forward:conditional", "maxspeed:lanes", "maxspeed:variable", "maxspeed:motorcar"):
      self.assertIsNone(osm.match_speed((road(tags={key: "30"}),), self.fix), key)

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
    service._result = (self.fix, 40 / 3.6)
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
    service._result = (self.fix, 110 / 3.6)
    self.assertAlmostEqual(service.update(self.fix, 100.), 110 / 3.6)
    service._wake.clear()
    moved = replace(self.fix, latitude=math.degrees(31 / osm.EARTH_RADIUS), timestamp=101.)
    self.assertAlmostEqual(service.update(moved, 101.), 110 / 3.6)
    self.assertTrue(service._wake.is_set())  # new fix wakes the matcher immediately
    service._result = (moved, None)
    self.assertAlmostEqual(service.update(moved, 101.8), 110 / 3.6)
    self.assertIsNone(service.update(replace(moved, timestamp=102.1), 102.1))

  def test_confirmed_limit_change_is_immediate(self):
    service = osm.OSMSpeedLimit()
    service._thread = Mock()
    service._result = (self.fix, 110 / 3.6)
    service.update(self.fix, 100.)
    moved = replace(self.fix, timestamp=101.)
    service._result = (moved, 60 / 3.6)
    self.assertAlmostEqual(service.update(moved, 101.), 60 / 3.6)

  def test_display_hold_is_not_a_confirmed_control_sample(self):
    service = osm.OSMSpeedLimit()
    service._thread = Mock()
    service._result = (self.fix, 110 / 3.6)
    service.update(self.fix, 100.)
    moved = replace(self.fix, latitude=math.degrees(31 / osm.EARTH_RADIUS), timestamp=101.)
    self.assertAlmostEqual(service.update(moved, 101.), 110 / 3.6)
    self.assertEqual(service.control_sample[0].timestamp, 100.)  # never restamp an older match
    self.assertAlmostEqual(service.update(moved, 101.4), 110 / 3.6)
    self.assertIsNone(service.control_sample)  # 0.3 s processing grace has expired
    service._result = (moved, None)  # fresh but ambiguous match, with sign still held
    self.assertAlmostEqual(service.update(moved, 101.5), 110 / 3.6)
    self.assertIsNone(service.control_sample[1])
    self.assertEqual(service.display_timestamp, 100.)
    self.assertIsNone(service.update(None, 101.6))
    self.assertIsNone(service.control_sample)

  def test_control_handoff_rejects_turns_and_position_jumps(self):
    for moved in (replace(self.fix, bearing=21, timestamp=101.), replace(self.fix, latitude=.001, timestamp=101.)):
      service = osm.OSMSpeedLimit()
      service._thread = Mock()
      service._result = (self.fix, 110 / 3.6)
      service.update(self.fix, 100.)
      service.update(moved, 101.)
      self.assertIsNone(service.control_sample)

  def test_hold_clears_on_turn_jump_invalid_fix_and_offroad(self):
    for fix, now in ((replace(self.fix, bearing=21, timestamp=101.), 101.),
                     (replace(self.fix, latitude=.001, timestamp=101.), 101.),
                     (self.fix, 104.), (None, 101.)):
      service = osm.OSMSpeedLimit()
      service._thread = Mock()
      service._result = (self.fix, 110 / 3.6)
      service.update(self.fix, 100.)
      self.assertIsNone(service.update(fix, now))
      self.assertIsNone(service._display)

  def test_ramp_continuity_requires_the_established_closer_aligned_road(self):
    main = road('110', tags={'highway': 'motorway'})
    ramp = road('80', x=12., tags={'highway': 'motorway_link'})
    fix = replace(self.fix, accuracy=15.)
    self.assertIsNone(osm.match_speed((main, ramp), fix))  # no prior road: keep strict ambiguity
    self.assertAlmostEqual(osm.match_road((main, ramp), fix, main).speed, 110 / 3.6)
    self.assertIsNone(osm.match_road((main, ramp), fix, ramp))  # cannot stick to the farther road
    on_ramp = replace(fix, longitude=math.degrees(11 / osm.EARTH_RADIUS))
    self.assertIsNone(osm.match_road((main, ramp), on_ramp, main))
    self.assertIsNone(osm.match_road((main, ramp), replace(fix, bearing=16), main))
    self.assertIsNone(osm.match_road((main, road('80', x=2.)), fix, main))
    self.assertIsNone(osm.match_road((main, ramp), replace(fix, bearing=None), main))
    self.assertIsNone(osm.match_road((road(None), ramp), fix, road(None)))

  def test_continuity_crosses_connected_way_split_but_not_unrelated_roads(self):
    before = osm.Road(((-.001, 0.), (0., 0.)), {'highway': 'motorway', 'maxspeed': '110'})
    after = osm.Road(((0., 0.), (.001, 0.)), dict(before.tags))
    ramp = road('80', x=12., tags={'highway': 'motorway_link'})
    fix = replace(self.fix, latitude=.0001, accuracy=15.)
    self.assertAlmostEqual(osm.match_road((after, ramp), fix, before).speed, 110 / 3.6)
    unrelated = road('110', x=30., tags={'highway': 'motorway'})
    self.assertIsNone(osm.match_road((after, ramp), fix, unrelated))

  def test_continuity_releases_after_taking_ramp(self):
    main = road('110', tags={'highway': 'motorway'})
    ramp = osm.Road(((0., 0.), (.001, .0004)), {'highway': 'motorway_link', 'maxspeed': '80'})
    at_split = replace(self.fix, accuracy=15.)
    self.assertIsNone(osm.match_road((main, ramp), at_split, main))
    on_ramp = replace(at_split, latitude=.0009, longitude=.00036, bearing=math.degrees(math.atan2(.4, 1.)))
    self.assertAlmostEqual(osm.match_road((main, ramp), on_ramp, main).speed, 80 / 3.6)

  def test_bounded_fetch_and_incomplete_responses(self):
    element = {"type": "way", "tags": {"highway": "residential", "maxspeed": "40"},
               "geometry": [{"lat": -.001, "lon": 0.}, {"lat": .001, "lon": 0.}]}
    response = Mock()
    with patch.object(osm.requests, "post") as post:
      post.return_value.__enter__.return_value = response
      response.iter_content.return_value = [json.dumps({"elements": [element]}).encode()]
      self.assertEqual(osm.fetch_roads(self.fix), (road(),))
      self.assertIn('out tags geom;', post.call_args.kwargs['data']['data'])
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
