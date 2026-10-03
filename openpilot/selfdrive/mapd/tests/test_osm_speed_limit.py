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

  def test_matched_fix_and_stale_distant_offroad_results(self):
    service = osm.OSMSpeedLimit()
    service._thread = Mock()
    service._result = (self.fix, 40 / 3.6)
    matched_fix, speed = service.update(replace(self.fix, timestamp=101.), 101.)
    self.assertAlmostEqual(speed, 40 / 3.6)
    self.assertEqual(matched_fix.timestamp, 100.)  # retain the matched fix's age, not the newest input's age
    self.assertIsNone(service.update(self.fix, 104.))
    self.assertIsNone(service.update(replace(self.fix, latitude=.001), 101.))
    self.assertIsNone(service.update(replace(self.fix, bearing=180), 101.))
    self.assertIsNone(service.update(None, 101.))
    self.assertIsNone(service._result)

  def test_geometry_across_date_line(self):
    r = osm.Road(((0., 179.999), (0., -179.999)), {"maxspeed": "40"})
    fix = replace(self.fix, longitude=180., bearing=90.)
    self.assertAlmostEqual(osm.match_speed((r,), fix), 40 / 3.6)

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
           patch.object(osm.time, 'monotonic', side_effect=lambda clock=clock: clock[0]), patch.object(osm.time, 'sleep', side_effect=sleep):
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
