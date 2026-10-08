import json
import math
import sys
import unittest
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock, patch

from openpilot.common.transformations.orientation import rot_from_euler
from openpilot.selfdrive.mapd import osm_speed_limit as osm
from openpilot.selfdrive.mapd.heading import MapHeading, VehicleMotion, vehicle_motion
from openpilot.selfdrive.mapd.position import MapPosition, POSITION_KEY, PARKED_MAX_AGE


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


def matched_speed(roads, fix):
  match = osm.RoadTracker().update(roads, fix)
  return match.speed if match is not None else None


def tracked_match(roads, fix, previous, previous_fix=None):
  tracker = osm.RoadTracker()
  tracker.update((previous,), previous_fix or replace(fix, timestamp=fix.timestamp - 1))
  return tracker.update(roads, fix)


class TestOSMSpeedLimit(unittest.TestCase):
  def setUp(self):
    self.fix = osm.GpsFix(0., 0., 0., 3., 100.)

  def update_service(self, service, fix, now):
    self.sample = service.update(fix, now)
    if self.sample is None or self.sample[0].bearing is None:
      return None
    _, legal, advisory, _ = self.sample
    return legal if legal is not None else advisory

  def cached_service(self, roads=None):
    service = osm.OSMSpeedLimit()
    service._thread = Mock()
    service._cache = (roads or (road(),)), self.fix, 100.
    return service

  def test_units_and_unknown_values(self):
    for value in ("40", "40 km/h", "40 kmh", "40 kph"):
      self.assertAlmostEqual(osm.parse_speed(value), 40 / 3.6)
    self.assertAlmostEqual(osm.parse_speed("60 mph"), 60 * .44704)
    for value in ("", "none", "signals", "walk", "CA-AB:urban", "40;60", "40 @ wet", "0", "-5", "nan", "99999"):
      self.assertIsNone(osm.parse_speed(value), value)

  def test_stop_retains_approach_heading_without_turning_from_steering_at_rest(self):
    h = MapHeading()
    h.update(self.fix, VehicleMotion(3., False, 0.), 100.)
    for i in range(1, 301):
      now = 100. + i * .2
      # Even a noisy GPS course and turned wheels must not rotate a stopped car.
      fix = replace(self.fix, latitude=point(0, 1)[0], bearing=180 if i % 2 else None, timestamp=now)
      result = h.update(fix, VehicleMotion(0., True, .2), now)
      self.assertEqual(result.bearing, 0.)
      self.assertTrue(result.stationary)
      self.assertIsNone(result.motion_bearing)
      self.assertEqual((result.latitude, result.longitude, result.timestamp), (fix.latitude, fix.longitude, fix.timestamp))
    # Pulling away after a long stop retains direction until GPS course returns.
    for i in range(1, 6):
      now = 160. + i * .2
      fix = replace(self.fix, latitude=point(0, 1 + i * .2)[0], bearing=None, timestamp=now)
      result = h.update(fix, VehicleMotion(1., False, 0.), now)
      self.assertEqual(result.bearing, 0.)
      self.assertEqual(result.motion_bearing, 0.)
    self.assertIsNone(h.update(None, VehicleMotion(0., True, 0.), 162.))
    self.assertIsNone(h.update(replace(self.fix, bearing=None, timestamp=163.), VehicleMotion(0., True, 0.), 163.).bearing)

  def test_heading_integrates_calibrated_turn_and_keeps_gps_as_reference(self):
    h = MapHeading()
    motion = VehicleMotion(10., False, math.radians(3) / 10)
    fix = replace(self.fix, bearing=359.)
    for i in range(5):
      h.update(fix, motion, 100. + i * .2)
    moved = osm.GpsFix(*point(0, 10), .5, 3., 101.)
    result = h.update(moved, motion, 101.)
    self.assertAlmostEqual(result.motion_bearing, 2.)
    self.assertEqual(result.bearing, .5)
    for i in range(1, 5):
      h.update(moved, motion, 101. + i * .2)
    inconsistent = h.update(replace(moved, bearing=120., timestamp=102.), motion, 102.)
    self.assertEqual(inconsistent.bearing, 120.)
    self.assertIsNone(inconsistent.motion_bearing)

  def test_heading_hold_rejects_drift_missing_sensors_stale_fixes_and_creep_distance(self):
    for issue in ('drift', 'sensors', 'stale', 'gap'):
      h = MapHeading()
      h.update(self.fix, VehicleMotion(3., False, 0.), 100.)
      fix = replace(self.fix, bearing=None, timestamp=100.2)
      motion = VehicleMotion(0., True, 0.)
      now = 100.2
      if issue == 'drift':
        fix = replace(fix, latitude=point(0, 21)[0])
      elif issue == 'sensors':
        motion = None
      elif issue == 'stale':
        now = 104.
      else:
        fix, now = replace(fix, timestamp=102.), 102.
      result = h.update(fix, motion, now)
      self.assertTrue(result is None or result.bearing is None, issue)
    h = MapHeading()
    h.update(self.fix, VehicleMotion(1.9, False, 0.), 100.)
    for i in range(1, 61):
      now = 100. + i * .2
      result = h.update(replace(self.fix, bearing=None, timestamp=now), VehicleMotion(1.9, False, 0.), now)
    self.assertIsNone(result.bearing)  # stale direction cannot survive indefinite creeping

  def test_vehicle_motion_checks_validity_and_uses_measured_curvature(self):
    class SM(dict):
      pass
    sm = SM(carState=SimpleNamespace(canValid=True, vEgo=10., standstill=False, gearShifter='drive'),
            controlsState=SimpleNamespace(curvature=.003), vehicleParameters=SimpleNamespace(valid=True, sensorValid=True))
    sm.valid = dict.fromkeys(sm, True)
    sm.recv_time = dict.fromkeys(sm, 100.)
    sm.logMonoTime = dict.fromkeys(sm, int(100e9))
    self.assertEqual(vehicle_motion(sm, 100.), VehicleMotion(10., False, .003))
    sm['vehicleParameters'].valid = False
    self.assertIsNone(vehicle_motion(sm, 100.).curvature)
    sm['vehicleParameters'].valid = True
    sm['controlsState'].curvature = math.nan
    self.assertIsNone(vehicle_motion(sm, 100.).curvature)
    sm['carState'].gearShifter = 'reverse'
    self.assertIsNone(vehicle_motion(sm, 100.))
    sm['carState'].gearShifter = 'drive'
    sm['carState'].standstill = True
    sm['carState'].vEgo = -1e-20
    self.assertEqual(vehicle_motion(sm, 100.), VehicleMotion(0., True, None))
    self.assertIsNone(vehicle_motion(sm, 101.))

  def test_signed_wheel_motion_and_calibrated_parking_gyro(self):
    class SM(dict):
      pass
    rpy = [.1, .2, .03]
    device_rate = rot_from_euler(rpy) @ [0., 0., .1]
    rate = SimpleNamespace(x=device_rate[0], y=device_rate[1], z=device_rate[2],
                           xStd=.01, yStd=.01, zStd=.01, valid=True)
    sm = SM(carState=SimpleNamespace(canValid=True, vEgo=2., standstill=False, gearShifter='reverse'),
            controlsState=SimpleNamespace(curvature=-.04), vehicleParameters=SimpleNamespace(valid=True, sensorValid=True),
            deviceMotion=SimpleNamespace(inputsOK=True, sensorsOK=True, timestamp=int(100e9), angularVelocityDevice=rate),
            extrinsicsCalibration=SimpleNamespace(rpyCalib=rpy, calStatus='calibrated'))
    sm.valid = dict.fromkeys(sm, True)
    sm.recv_time = dict.fromkeys(sm, 100.)
    sm.logMonoTime = dict.fromkeys(sm, int(100e9))
    self.assertIsNone(vehicle_motion(sm, 100.))  # unsigned platforms cannot infer a reverse velocity
    forward = vehicle_motion(sm, 100., signed_speed=True)
    self.assertEqual(forward.speed, 2.)  # still rolling forward after R is selected
    self.assertAlmostEqual(forward.turn_rate, .1)
    sm['carState'].vEgo = -2.
    sm['carState'].gearShifter = 'drive'  # still rolling backward after D is selected
    reverse = vehicle_motion(sm, 100., signed_speed=True)
    self.assertEqual(reverse.speed, -2.)
    self.assertAlmostEqual(reverse.turn_rate, .1)  # gyro sign describes actual rotation
    for service in ('deviceMotion', 'extrinsicsCalibration'):
      sm.valid[service] = False
      self.assertAlmostEqual(vehicle_motion(sm, 100., True).turn_rate, .08)
      sm.valid[service] = True
    sm['deviceMotion'].timestamp = int(99e9)
    self.assertIsNone(vehicle_motion(sm, 100., True).yaw_rate)
    sm['deviceMotion'].timestamp = int(100e9)
    rate.z = math.nan
    self.assertIsNone(vehicle_motion(sm, 100., True).yaw_rate)
    sm['carState'].standstill, sm['carState'].vEgo, sm['carState'].gearShifter = True, -.01, 'park'
    parked = vehicle_motion(sm, 100., True)
    self.assertTrue(parked.parked)
    self.assertEqual((parked.speed, parked.turn_rate), (0., 0.))

  def test_reverse_course_is_facing_heading_and_direction_change_rejects_old_course(self):
    h = MapHeading()
    reverse = VehicleMotion(-3., False, 0.)
    result = h.update(replace(self.fix, bearing=180.), reverse, 100.)
    self.assertEqual(result.bearing, 0.)
    h.update(replace(self.fix, bearing=180., timestamp=100.2), VehicleMotion(0., True, 0.), 100.2)
    h.seed(replace(self.fix, timestamp=100.2), VehicleMotion(0., True, 0.), 100.2)
    for i in range(2, 17):
      now = 100. + i * .2
      result = h.update(replace(self.fix, bearing=180., timestamp=now), VehicleMotion(3., False, 0.), now)
      self.assertEqual(result.bearing, 0.)  # discard receiver course spanning R -> stop -> D
    for i in range(17, 21):
      now = 100. + i * .2
      result = h.update(replace(self.fix, bearing=0., timestamp=now), VehicleMotion(3., False, 0.), now)
    self.assertEqual(result.bearing, 0.)

  def test_heading_recovers_from_positions_and_steering_on_a_curve(self):
    h = MapHeading()
    motion = VehicleMotion(10., False, .02)
    fix = None
    for i in range(51):
      elapsed = i / 5
      if i % 5 == 0:
        angle = elapsed * .2
        fix = osm.GpsFix(*point(50 * (1 - math.cos(angle)), 50 * math.sin(angle)), None, 3., 100. + elapsed)
      result = h.update(fix, motion, 100. + elapsed)
      if i >= 20 and i % 5 == 0:
        self.assertAlmostEqual(result.bearing, math.degrees(angle), delta=1.)
        self.assertAlmostEqual(result.odometer, elapsed * 10)
      self.assertEqual((result.latitude, result.longitude, result.timestamp), (fix.latitude, fix.longitude, fix.timestamp))
    # Repeated GPS polls do not advance the published odometry or confidence.
    self.assertEqual(h.update(fix, motion, 110.2), result)
    self.assertIsNone(h.update(None, motion, 110.4))
    restarted = h.update(replace(fix, timestamp=111.), motion, 111.)
    self.assertIsNone(restarted.bearing)
    self.assertNotEqual(restarted.motion_epoch, result.motion_epoch)

  def test_position_heading_requires_odometry_agreement_and_rotation_sensors(self):
    for curvature in (0., None):
      h = MapHeading()
      for i in range(41):
        elapsed = i / 5
        # GPS moves much faster than CAN, or steering evidence is unavailable.
        fix = osm.GpsFix(*point(0, elapsed * (30 if curvature is not None else 10)), None, 3., 100. + elapsed)
        result = h.update(fix, VehicleMotion(10., False, curvature), 100. + elapsed)
        self.assertIsNone(result.bearing)

  def test_steering_bridge_through_turn_is_short_and_needs_fresh_gps(self):
    h = MapHeading()
    motion = VehicleMotion(10., False, .04)
    h.update(self.fix, motion, 100.)
    for i in range(1, 21):
      elapsed = i / 5
      # Unreliable positions cannot establish a replacement absolute course.
      fix = replace(self.fix, bearing=None, timestamp=100. + elapsed)
      result = h.update(fix, motion, 100. + elapsed)
      if i == 14:
        self.assertAlmostEqual(result.bearing, math.degrees(elapsed * .4))
    self.assertIsNone(result.bearing)

  def test_established_road_offset_survives_cache_refresh_but_not_departure(self):
    main = polyline([(0, -100), (0, 300)], '70', way_id=1)
    tracker = osm.RoadTracker()
    roads = (main,)
    for i in range(8):
      fix = osm.GpsFix(*point(5 * i, 20 * i), 0., 15., 100. + i, motion_bearing=0., speed=20.,
                       odometer=20. * i, motion_epoch=100.)
      self.assertAlmostEqual(tracker.update(roads, fix).speed, 70 / 3.6)
    # A new cache must retain the geometry lock and immediately read new tags.
    updated = replace(main, tags={**main.tags, 'maxspeed': '60'})
    self.assertEqual(tracker.update((updated,), fix).speed, 60 / 3.6)
    self.assertIsNone(osm.RoadTracker().update((updated,), fix))  # no wide-radius acquisition
    stopped = replace(fix, stationary=True, speed=0., motion_bearing=None, timestamp=108.)
    self.assertEqual(tracker.update((updated,), stopped).speed, 60 / 3.6)
    self.assertIsNone(tracker.update((updated,), replace(fix, bearing=90., motion_bearing=90., timestamp=109.)))
    self.assertIsNone(tracker.update((updated,), replace(fix, timestamp=113.)))  # old lock expired

  def test_odometry_allows_connected_junction_projection_displacement(self):
    before = polyline([(0, -100), (0, 0)], '70', way_id=1)
    after = polyline([(0, 0), (100, 0)], '50', way_id=2)
    tracker = osm.RoadTracker()
    first = osm.GpsFix(*point(10, -15), 0., 15., 100., motion_bearing=0., speed=8., odometer=0., motion_epoch=100.)
    self.assertAlmostEqual(tracker.update((before, after), first).speed, 70 / 3.6)
    # A rounded physical turn cuts across the sharp mapped centreline corner.
    turned = osm.GpsFix(*point(15, -10), 90., 15., 101., motion_bearing=90., speed=8., odometer=8., motion_epoch=100.)
    self.assertEqual(tracker.update((before, after), turned).speed, 50 / 3.6)

  def test_latched_road_releases_when_measured_turn_conflicts(self):
    main = polyline([(0, -100), (0, 0), (0, 300)], '100', way_id=1)
    ramp = polyline([(0, 0), (20, 100), (100, 200)], '40', way_id=2, highway='primary_link')
    roads = (main, ramp)
    tracker = osm.RoadTracker()
    for i, y in enumerate((-60, -40, -20)):
      fix = osm.GpsFix(*point(0, y), 0., 15., 100. + i, motion_bearing=0., speed=20.,
                       odometer=20. * i, motion_epoch=100.)
      self.assertEqual(tracker.update(roads, fix).speed, 100 / 3.6)
    for i, (x, y) in enumerate(((4, 20), (8, 40), (12, 60), (16, 80)), 3):
      fix = osm.GpsFix(*point(x, y), 11.31, 15., 100. + i, motion_bearing=11.31, speed=20.,
                       odometer=20. * i + 20, motion_epoch=100.)
      result = tracker.update(roads, fix)
      self.assertEqual(result.speed if result else None, 40 / 3.6)

  def test_rejected_fork_cannot_win_later_from_parallel_gps_offset(self):
    main = polyline([(0, -100), (0, 0), (0, 500)], '70', way_id=1)
    ramp = polyline([(0, 0), (8, 22), (13, 240), (90, 400)], None, way_id=2, highway='primary_link')
    roads = (main, ramp)
    tracker = osm.RoadTracker()
    for i in range(13):
      fix = osm.GpsFix(*point(15 + max(0, i - 5) * 2, -50 + i * 25), 0., 15., 100. + i,
                       motion_bearing=0., speed=25., odometer=25. * i, motion_epoch=100.)
      match = tracker.update(roads, fix)
      # The shared fork itself can be ambiguous; thereafter no parallel jump.
      if i != 2:
        self.assertIsNotNone(match, i)
        self.assertEqual(match.road.way_id, 1, i)

  def test_stopped_road_retention_is_bounded_and_releases_on_movement(self):
    main = road('60')
    parallel = road('40', x=12)
    roads = (main, parallel)
    tracker = osm.RoadTracker()
    tracker.update((main,), self.fix)
    for i in range(1, 61):
      fix = osm.GpsFix(*point(7, 0), 0., 15., 100. + i, stationary=True)
      self.assertEqual(tracker.update(roads, fix).speed, 60 / 3.6)
    # Driving toward the other road resumes ordinary ambiguity checks immediately.
    self.assertIsNone(tracker.update(roads, replace(fix, stationary=False, timestamp=161.)))
    tracker = osm.RoadTracker()
    tracker.update((main,), self.fix)
    self.assertIsNone(tracker.update(roads, osm.GpsFix(*point(13, 0), 0., 15., 101., stationary=True)))

  def test_steering_supports_connected_ramp_but_not_equal_fork_or_wrong_turn(self):
    main = polyline([(0, -100), (0, 0), (0, 100)], '100', way_id=1)
    ramp = polyline([(0, 0), (40, 100)], '60', way_id=2)
    first = osm.GpsFix(*point(0, -10), 0., 15., 100.)
    departed = osm.GpsFix(*point(2, 5), 10., 15., 101.)
    for steering, expected in ((None, None), (0., 100 / 3.6), (21.8, 60 / 3.6)):
      tracker = osm.RoadTracker()
      tracker.update((main,), first)
      result = tracker.update((main, ramp), replace(departed, motion_bearing=steering))
      self.assertEqual(result.speed if result else None, expected)
    tracker = osm.RoadTracker()
    tracker.update((main,), first)
    self.assertIsNone(tracker.update((main, ramp), osm.GpsFix(*point(0, 0), 10., 15., 101., motion_bearing=21.8)))
    unconnected = replace(ramp, node_ids=(90001, 90002))
    tracker = osm.RoadTracker()
    tracker.update((main,), first)
    self.assertIsNone(tracker.update((main, unconnected), replace(departed, motion_bearing=21.8)))

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
    self.assertAlmostEqual(matched_speed((r,), self.fix), 50 / 3.6)
    self.assertAlmostEqual(matched_speed((r,), replace(self.fix, bearing=180)), 30 / 3.6)
    self.assertIsNone(matched_speed((r,), replace(self.fix, bearing=None)))
    for key in ("maxspeed:conditional", "maxspeed:forward:conditional", "maxspeed:lanes", "maxspeed:variable", "maxspeed:motorcar"):
      self.assertIsNone(matched_speed((road(tags={key: "30"}),), self.fix), key)

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

  def test_shallow_fork_keeps_paths_until_motion_selects_ramp(self):
    main = polyline([(0, -100), (0, 0), (0, 200)], '100', way_id=1, oneway='yes')
    ramp = polyline([(0, 0), (20, 200)], None, way_id=2, oneway='yes',
                    highway='motorway_link', **{'maxspeed:advisory': '50'})
    roads = (main, ramp)
    tracker = osm.RoadTracker()
    tracker.update(roads, osm.GpsFix(*point(0, -30), 0., 15., 100.))
    for t, y in ((101., 0), (102., 10)):
      self.assertIsNone(tracker.update(roads, osm.GpsFix(*point(y / 10, y), 5.71, 15., t)))
    # Connected history selects the real ramp before waiting for a large
    # centreline separation and reacquiring from scratch.
    early = tracker.update(roads, osm.GpsFix(*point(2, 20), 5.71, 15., 103.))
    self.assertEqual(early.road.way_id, 2)
    for t, y in ((104., 30), (105., 70)):
      self.assertEqual(tracker.update(roads, osm.GpsFix(*point(y / 10, y), 5.71, 15., t)).road.way_id, 2)
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

  def test_connected_mainline_survives_offset_toward_nearer_ramp(self):
    main = polyline([(0, -100), (0, 0), (0, 250)], '100', way_id=1, highway='motorway', oneway='yes')
    ramp = polyline([(0, 0), (30, 250)], None, way_id=2, highway='motorway_link', oneway='yes', **{'maxspeed:advisory': '40'})
    tracker = osm.RoadTracker()
    roads = (main, ramp)
    self.assertEqual(tracker.update(roads, osm.GpsFix(*point(10, -40), 0., 15., 100.)).road.way_id, 1)
    accepted = 0
    for i, y in enumerate(range(-20, 241, 20), 1):
      match = tracker.update(roads, osm.GpsFix(*point(10, y), 0., 15., 100. + i, motion_bearing=0.))
      if match is not None:
        self.assertEqual(match.road.way_id, 1)
        self.assertIsNone(match.advisory_speed)
        accepted += 1
      self.assertLessEqual(len(tracker._paths), 6)
    self.assertGreaterEqual(accepted, 10)

  def test_cache_refresh_does_not_accumulate_a_frozen_fork_fix(self):
    roads = (road('100'), road('40', x=12))
    tracker = osm.RoadTracker()
    fix = osm.GpsFix(*point(12, 0), 0., 15., 100.)
    self.assertIsNone(tracker.update(roads, fix))
    for _ in range(10):
      self.assertIsNone(tracker.update(tuple(replace(r) for r in roads), fix))

  def test_unique_untagged_ramp_looks_ahead_with_time_and_distance_bounds(self):
    ramp = polyline([(0, -400), (0, 0)], None, way_id=1, highway='motorway_link', oneway='yes')
    short = polyline([(0, 0), (0, 10)], None, way_id=2, highway='motorway_link', oneway='yes')
    destination = polyline([(0, 10), (0, 200)], '70', way_id=3, highway='secondary', oneway='yes')
    roads = (ramp, short, destination)
    for y, speed, expected in ((-200, 30., 70 / 3.6), (-250, 40., None), (-200, 20., None), (-20, 0., 70 / 3.6)):
      tracker = osm.RoadTracker()
      fix = osm.GpsFix(*point(0, y), 0., 3., 100., speed=speed)
      match = tracker.update(roads, fix)
      limit, distance = tracker.merge_limit(match, fix)
      if expected is None:
        self.assertIsNone(limit)
      else:
        self.assertAlmostEqual(limit, expected)
      self.assertAlmostEqual(distance, 10 - y if expected is not None else 0.)

  def test_merge_lookahead_stops_at_forks_restrictions_and_missing_nodes(self):
    ramp = polyline([(0, -300), (0, -100), (0, 0)], None, way_id=1, highway='motorway_link', oneway='yes')
    destination = polyline([(0, 0), (0, 200)], '70', way_id=2, highway='secondary', oneway='yes')
    fork = polyline([(0, -100), (80, 50)], '40', way_id=3, highway='motorway_link', oneway='yes')
    at_end = polyline([(0, 0), (100, 200)], '50', way_id=4, highway='motorway_link', oneway='yes')
    cases = ((ramp, destination, fork), (ramp, destination, at_end),
             (replace(ramp, node_ids=()), destination),
             (replace(ramp, tags={**ramp.tags, 'maxspeed:advisory': '40'}), destination),
             (replace(ramp, tags={**ramp.tags, 'maxspeed:conditional': '50 @ wet'}), destination),
             (replace(ramp, tags={**ramp.tags, 'maxspeed': '60'}), destination),
             (replace(ramp, tags={**ramp.tags, 'highway': 'secondary'}), destination),
             (ramp, replace(destination, tags={**destination.tags, 'maxspeed:conditional': '50 @ wet'})),
             (ramp, replace(destination, tags={**destination.tags, 'oneway': '-1'})))
    fix = osm.GpsFix(*point(0, -150), 0., 3., 100., speed=30.)
    for roads in cases:
      tracker = osm.RoadTracker()
      match = tracker.update(roads, fix)
      self.assertIsNotNone(match)
      self.assertEqual(tracker.merge_limit(match, fix), (None, 0.))

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

  def test_early_merge_lane_change_uses_shared_continuation_only(self):
    ramp = polyline([(12, -100), (12, -10), (0, 0)], None, way_id=1, highway='motorway_link', oneway='yes')
    main = polyline([(0, -100), (0, 0)], '70', way_id=2, oneway='yes')
    onward = polyline([(0, 0), (0, 200)], '70', way_id=3, oneway='yes')
    fork = polyline([(0, 0), (80, 100)], '40', way_id=4, oneway='yes')
    first = osm.GpsFix(*point(12, -40), 0., 3., 100., motion_bearing=0., speed=20.)
    merged = replace(first, latitude=point(4, -15)[0], longitude=point(4, -15)[1], timestamp=101.)
    for restricted in (False, True):
      r = replace(ramp, tags={**ramp.tags, **({'maxspeed:advisory': '40'} if restricted else {})})
      tracker = osm.RoadTracker()
      previous = tracker.update((r,), first)
      tracker._set_roads((r, main, onward))
      candidate = osm.road_candidates((main,), merged)[0]
      progress = tracker._progress(previous, candidate, 50.)
      if restricted:
        self.assertIsNone(progress)
      else:
        self.assertIsNotNone(progress)
        self.assertLess(progress, 40.)
        tracker._set_roads((r, main, onward, fork))
        self.assertIsNone(tracker._progress(previous, candidate, 50.))
        tracker._set_roads((r, replace(main, node_ids=()), onward))
        # Use the same road instance as the tracker cache.
        unconnected = osm.road_candidates((tracker._roads[1],), merged)[0]
        self.assertIsNone(tracker._progress(previous, unconnected, 50.))

  def test_ramp_reacquisition_accumulates_motion_with_fast_gps(self):
    roads = (road('100'), road(None, x=12, tags={'maxspeed:advisory': '50'}))
    tracker = osm.RoadTracker()
    for i in range(5):
      self.assertIsNone(tracker.update(roads, osm.GpsFix(*point(12, i * 2), 0., 15., 100. + i / 10)))
    match = tracker.update(roads, osm.GpsFix(*point(12, 10), 0., 15., 100.5))
    self.assertAlmostEqual(match.advisory_speed, 50 / 3.6)

  def test_display_shares_legal_priority_and_freshness_with_control(self):
    service = self.cached_service()
    for stamp, legal, advisory, expected in ((100., '100', '50', 100), (101., None, '50', 50),
                                              (102., None, None, None), (104., '40', '50', 40)):
      service._cache = (replace(road(legal, tags={'maxspeed:advisory': advisory} if advisory else {}), way_id=1),), self.fix, 100.
      fix = replace(self.fix, timestamp=stamp)
      value = self.update_service(service, fix, stamp)
      if expected is None:
        self.assertIsNone(value)
        self.assertEqual(self.sample[1:], (None, None, 0.))
      else:
        self.assertAlmostEqual(value, expected / 3.6)
      self.assertEqual(self.sample[0], fix)

  def test_oneway_heading_and_roundabouts(self):
    for tags in ({"oneway": "yes"}, {"junction": "roundabout"}):
      self.assertIsNone(matched_speed((road(tags=tags),), replace(self.fix, bearing=180)))
      self.assertAlmostEqual(matched_speed((road(tags=tags),), self.fix), 40 / 3.6)
    self.assertIsNone(matched_speed((road(tags={"oneway": "-1"}),), self.fix))
    self.assertIsNone(matched_speed((road(),), replace(self.fix, bearing=90)))

  def test_parallel_roads_unknown_roads_and_distance(self):
    self.assertIsNone(matched_speed((road(), road("60", x=5)), self.fix))
    self.assertIsNone(matched_speed((road(None), road("60", x=15)), self.fix))
    roads = (road(), road("60", x=20))
    tracker = osm.RoadTracker()
    self.assertIsNone(tracker.update(roads, self.fix))
    # The active tracker requires fresh motion to distinguish parallel roads.
    moved = replace(self.fix, latitude=point(0, 10)[0], timestamp=101.)
    self.assertAlmostEqual(tracker.update(roads, moved).speed, 40 / 3.6)
    self.assertAlmostEqual(matched_speed((road(), road("40", x=5)), self.fix), 40 / 3.6)
    self.assertIsNone(matched_speed((road(x=30),), self.fix))
    self.assertIsNone(matched_speed((), self.fix))

  def test_speed_change_at_way_boundary_is_not_anticipated(self):
    old = osm.Road(((-.001, 0.), (0., 0.)), {"maxspeed": "60"})
    new = osm.Road(((0., 0.), (.001, 0.)), {"maxspeed": "40"})
    self.assertAlmostEqual(matched_speed((old, new), replace(self.fix, latitude=-.0002)), 60 / 3.6)
    self.assertIsNone(matched_speed((old, new), self.fix))
    self.assertAlmostEqual(matched_speed((old, new), replace(self.fix, latitude=.0002)), 40 / 3.6)

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
    service = self.cached_service()
    self.assertAlmostEqual(self.update_service(service, self.fix, 101.), 40 / 3.6)
    self.assertIsNone(self.update_service(service, self.fix, 104.))
    self.assertIsNone(service._tracker.match)
    self.assertIsNone(self.update_service(service, replace(self.fix, longitude=.001), 101.))
    self.assertIsNone(self.update_service(service, replace(self.fix, bearing=90), 101.))
    self.assertIsNone(self.update_service(service, None, 101.))
    self.assertIsNone(service._tracker.match)

  def test_geometry_across_date_line(self):
    r = osm.Road(((0., 179.999), (0., -179.999)), {"maxspeed": "40"})
    fix = replace(self.fix, longitude=180., bearing=90.)
    self.assertAlmostEqual(matched_speed((r,), fix), 40 / 3.6)

  def test_current_match_is_published_during_a_turn_without_a_worker_handoff(self):
    curved = polyline([(0, -100), (0, 0), (50, 70), (150, 70)], '70')
    service = self.cached_service((curved,))
    first = replace(self.fix, latitude=point(0, -10)[0], speed=35.)
    self.assertAlmostEqual(self.update_service(service, first, 100.), 70 / 3.6)
    moved = replace(first, latitude=point(14, 20)[0], longitude=point(14, 20)[1], bearing=35., timestamp=101.)
    self.assertAlmostEqual(self.update_service(service, moved, 101.), 70 / 3.6)
    self.assertEqual(self.sample[0], moved)  # uses this fix, never the old road result
    self.assertTrue(service._wake.is_set())
    self.assertIsNone(self.update_service(service, replace(moved, bearing=125., timestamp=102.), 102.))

  def test_confirmed_limit_change_is_immediate_even_during_cache_refresh(self):
    service = self.cached_service((replace(road('110'), way_id=1),))
    self.assertAlmostEqual(self.update_service(service, self.fix, 100.), 110 / 3.6)
    service._cache = (replace(road('60'), way_id=1),), self.fix, 100.1
    self.assertAlmostEqual(self.update_service(service, self.fix, 100.05), 60 / 3.6)  # download completed after `now` was sampled
    self.assertAlmostEqual(self.update_service(service, self.fix, 100.2), 60 / 3.6)

  def test_current_ambiguity_clears_in_the_same_publish_cycle(self):
    service = self.cached_service()
    self.assertAlmostEqual(self.update_service(service, self.fix, 100.), 40 / 3.6)
    service._cache = (road('40'), road('60')), self.fix, 100.
    moved = replace(self.fix, timestamp=101.)
    self.assertIsNone(self.update_service(service, moved, 101.))
    self.assertEqual(self.sample, (moved, None, None, 0.))

  def test_cache_expiry_and_coverage_do_not_reuse_previous_limit(self):
    service = self.cached_service()
    self.assertAlmostEqual(self.update_service(service, self.fix, 100.), 40 / 3.6)
    self.assertIsNone(self.update_service(service, replace(self.fix, timestamp=701.), 701.))
    self.assertIsNone(service._tracker.match)
    service._cache = (road(),), replace(self.fix, latitude=.02), 702.
    self.assertIsNone(self.update_service(service, replace(self.fix, timestamp=702.), 702.))
    self.assertIsNone(service._tracker.match)

  def test_incoming_ramp_merge_does_not_make_established_mainline_ambiguous(self):
    main = polyline([(0, -100), (0, 0)], '100', highway='motorway', oneway='yes')
    after = polyline([(0, 0), (0, 100)], '100', highway='motorway', oneway='yes')
    incoming = polyline([(15, -100), (0, 0)], None, highway='motorway_link', oneway='yes')
    tracker = osm.RoadTracker()
    tracker.update((main,), osm.GpsFix(*point(0, -80), 0., 15., 99., speed=25.))
    roads = (main, after, incoming)
    for i, y in enumerate((-50, -25, -2, 20)):
      fix = osm.GpsFix(*point(0, y), 0., 15., 100. + i, speed=25.)
      match = tracker.update(roads, fix)
      if i:
        self.assertIsNotNone(match)
        self.assertAlmostEqual(match.speed, 100 / 3.6)

  def test_ramp_continuity_requires_the_established_closer_aligned_road(self):
    main = road('110', tags={'highway': 'motorway'})
    ramp = road('80', x=12., tags={'highway': 'motorway_link'})
    fix = replace(self.fix, accuracy=15.)
    self.assertIsNone(matched_speed((main, ramp), fix))  # no prior road: keep strict ambiguity
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
    class StopWorker(Exception):
      pass
    for fail in (False, True):
      service = osm.OSMSpeedLimit()
      service._thread = Mock()
      service._fix = self.fix
      clock = [100.]
      calls = []
      results = []
      steps = iter((101., 129., 131., 161., 191., 400., 701., 702.))

      def fetch(fix, calls=calls, clock=clock, fail=fail):
        calls.append(clock[0])
        if fail:
          raise osm.requests.ConnectionError()
        return (road(),)

      def sleep(_, results=results, service=service, clock=clock, steps=steps):
        results.append(service.update(service._fix, clock[0]))
        try:
          clock[0] = next(steps)
        except StopIteration as error:
          raise StopWorker from error
        service._fix = replace(self.fix, timestamp=clock[0])

      modules = {'openpilot.common.realtime': SimpleNamespace(drop_realtime=lambda: None),
                 'openpilot.common.swaglog': SimpleNamespace(cloudlog=Mock())}
      with patch.dict(sys.modules, modules), patch.object(osm, 'fetch_roads', side_effect=fetch), \
           patch.object(osm.time, 'monotonic', side_effect=lambda clock=clock: clock[0]), patch.object(service._wake, 'wait', side_effect=sleep):
        with self.assertRaises(StopWorker):
          service._run()
      if fail:
        self.assertTrue(all(b - a >= 30 for a, b in zip(calls, calls[1:], strict=False)))
        self.assertTrue(all(result is None for result in results))
      else:
        self.assertEqual(calls, [100., 701.])
        self.assertTrue(all(result[1] == 40 / 3.6 for result in results))



class TestMapPosition(unittest.TestCase):
  def setUp(self):
    self.data = {}
    self.writes = []
    def put(key, value, block=False):
      self.data[key] = value
      self.writes.append(value)
    self.params = SimpleNamespace(get=lambda key: self.data.get(key), put=put, remove=lambda key: self.data.pop(key, None))
    self.saved = {'version': 1, 'vehicle': 'test-car', 'latitude': 0., 'longitude': 0., 'bearing': 0., 'accuracy': 3., 'savedAt': 1000.}
    self.park = VehicleMotion(0., True, None, True)
    self.drive = VehicleMotion(10., False, 0.)

  def position(self, checkpoint=None, wall_time=1100.):
    if checkpoint is not None:
      self.data[POSITION_KEY] = checkpoint
    return MapPosition(self.params, 'test-car', wall_time)

  def test_parked_restart_waits_for_can_and_keeps_real_gps_timestamp_separate(self):
    p = self.position(self.saved)
    self.assertIsNone(p.update(None, None, 10., 1100.))
    self.assertIn(POSITION_KEY, self.data)
    result = p.update(None, self.park, 10.2, 1100.2)
    self.assertTrue(result.estimated)
    self.assertEqual(result.gps_timestamp, 0.)
    self.assertEqual(result.timestamp, 10.2)
    for i in range(1, 501):
      result = p.update(None, self.park, 10.2 + i * .2, 1100.2 + i * .2)
    self.assertEqual(result.latitude, 0.)
    self.assertEqual(result.bearing, 0.)
    self.assertEqual(self.data[POSITION_KEY]['savedAt'], 1000.)
    self.assertEqual(self.writes, [])  # estimates cannot refresh the saved anchor
    for i in range(1, 11):
      result = p.update(None, self.drive, 110.2 + i * .2, 1200.2 + i * .2)
    self.assertGreater(result.latitude, 0.)
    self.assertNotIn(POSITION_KEY, self.data)

  def test_invalid_expired_foreign_or_future_checkpoint_is_ignored(self):
    for change in ({'savedAt':1101.}, {'savedAt':1100. - PARKED_MAX_AGE - 1}, {'vehicle':'other-car'},
                   {'latitude':math.nan}, {'bearing':360.}, {'accuracy':0.}, {'version':2}):
      with self.subTest(change=change):
        p = self.position({**self.saved, **change})
        self.assertIsNone(p.update(None, self.park, 10., 1100.))
    for malformed in ({}, {'version':1}, [], 'invalid'):
      p = self.position(malformed)
      self.assertIsNone(p.update(None, self.park, 10., 1100.))

  def test_no_restore_when_already_moving_or_after_sensor_gap(self):
    p = self.position(self.saved)
    self.assertIsNone(p.update(None, self.drive, 10., 1100.))
    self.assertIsNone(p.update(None, self.park, 10.2, 1100.2))
    for bad, time in ((None, 10.2), (VehicleMotion(10., False, None), 10.2), (self.drive, 11.)):
      p = self.position(self.saved)
      self.assertIsNotNone(p.update(None, self.park, 10., 1100.))
      self.assertIsNone(p.update(None, bad, time, 1101.))
      self.assertIsNone(p.update(None, self.park, time + .2, 1101.2))

  def test_motion_bridge_turns_and_expires_without_self_refresh(self):
    p = self.position()
    motion = VehicleMotion(10., False, .005)
    first = osm.GpsFix(0., 0., 0., 3., 100.)
    self.assertFalse(p.update(first, motion, 100., 1100.).estimated)
    for i in range(1, 51):
      result = p.update(None, motion, 100. + i * .2, 1100. + i * .2)
    x, y = osm.offset_metres(result.latitude, result.longitude, first)
    self.assertAlmostEqual(x, (1 - math.cos(.5)) / .005, delta=.01)
    self.assertAlmostEqual(y, math.sin(.5) / .005, delta=.01)
    self.assertAlmostEqual(result.bearing, math.degrees(.5), places=5)
    self.assertEqual(result.gps_timestamp, 100.)
    for i in range(51, 102):
      result = p.update(None, motion, 100. + i * .2, 1100. + i * .2)
    self.assertIsNone(result)
    self.assertIsNone(p.update(None, self.park, 120.6, 1120.6))
    fresh = replace(first, timestamp=121., bearing=90.)
    result = p.update(fresh, self.drive, 121., 1121.)
    self.assertFalse(result.estimated)
    self.assertEqual(result.bearing, 90.)

  def test_distance_and_stationary_time_limits(self):
    for motion, steps in ((VehicleMotion(30., False, 0.), 44), (self.park, 3002)):
      p = self.position(self.saved)
      p.update(None, self.park, 10., 1100.)
      for i in range(1, steps):
        result = p.update(None, motion, 10. + i * .2, 1100. + i * .2)
      self.assertIsNone(result)

  def test_position_bridge_requires_receiver_course_while_moving(self):
    p = self.position()
    # Fresh positions can still recover a heading for normal map matching.
    for i in range(16):
      now = 100. + i * .2
      result = p.update(osm.GpsFix(*point(0., i * 2.), None, 3., now), self.drive, now, 1100. + i * .2)
    self.assertIsNotNone(result.bearing)
    self.assertFalse(result.estimated)
    self.assertIsNone(p.update(None, self.drive, 103.2, 1103.2))

  def test_fresh_gps_confirms_saved_heading_only_near_saved_position(self):
    for distance, expected in ((5., 0.), (100., None)):
      p = self.position(self.saved)
      fix = osm.GpsFix(*point(0., distance), None, 3., 10.)
      result = p.update(fix, self.park, 10., 1100.)
      self.assertEqual(result.bearing, expected)
      self.assertFalse(result.estimated)
      self.assertEqual(result.latitude, fix.latitude)
      if expected is None:
        self.assertNotIn(POSITION_KEY, self.data)
        restarted = self.position()
        self.assertIsNone(restarted.update(None, self.park, 11., 1101.))

  def test_recent_gps_parked_checkpoints_are_written_and_movement_invalidates(self):
    p = self.position()
    first = osm.GpsFix(0., 0., 0., 3., 100.)
    p.update(first, self.drive, 100., 1100.)
    p.update(replace(first, timestamp=100.2, bearing=None), self.park, 100.2, 1100.2)
    self.assertEqual(len(self.writes), 1)
    # Shutdown immediately after the first Park sample: a fresh process can
    # restore the checkpoint without waiting for another onroad update.
    restarted = MapPosition(self.params, 'test-car', 1101.)
    restored = restarted.update(None, self.park, 10., 1101.)
    self.assertIsNotNone(restored)
    self.assertEqual(restored.bearing, 0.)
    for i in range(2, 51):
      now = 100. + i * .2
      p.update(replace(first, timestamp=now, bearing=None), self.park, now, 1100. + i * .2)
    self.assertEqual(len(self.writes), 1)
    self.assertEqual(self.data[POSITION_KEY]['bearing'], 0.)
    p.update(replace(first, timestamp=110.2), self.drive, 110.2, 1110.2)
    self.assertNotIn(POSITION_KEY, self.data)

  def test_reparking_bypasses_periodic_refresh_delay(self):
    p = self.position()
    first = osm.GpsFix(0., 0., 0., 3., 100.)
    p.update(first, self.drive, 100., 1100.)
    p.update(replace(first, timestamp=100.2), self.park, 100.2, 1100.2)
    p.update(replace(first, timestamp=100.4), self.drive, 100.4, 1100.4)
    self.assertNotIn(POSITION_KEY, self.data)
    p.update(replace(first, timestamp=100.6), self.park, 100.6, 1100.6)
    self.assertEqual(len(self.writes), 2)
    self.assertEqual(self.data[POSITION_KEY]['savedAt'], 1100.6)

  def test_reverse_arc_parking_restart_and_forward_departure(self):
    p = self.position()
    # Facing north while moving south; a positive gyro rate rotates the body
    # clockwise even though translation is backwards.
    reverse = VehicleMotion(-1., False, None, yaw_rate=.1)
    start = osm.GpsFix(0., 0., 180., 3., 100.)
    result = p.update(start, reverse, 100., 1100.)
    self.assertEqual(result.bearing, 0.)
    for i in range(1, 51):
      result = p.update(None, reverse, 100. + i * .2, 1100. + i * .2)
    x, y = osm.offset_metres(result.latitude, result.longitude, start)
    self.assertAlmostEqual(x, -(1 - math.cos(1)) / .1, delta=.001)
    self.assertAlmostEqual(y, -math.sin(1) / .1, delta=.001)
    self.assertAlmostEqual(result.bearing, math.degrees(1), places=5)
    self.assertGreater(result.odometer, 0.)
    parked = p.update(None, self.park, 110.2, 1110.2)
    self.assertEqual(len(self.writes), 1)
    restarted = MapPosition(self.params, 'test-car', 2000.)
    restored = restarted.update(None, self.park, 10., 2000.)
    self.assertEqual(restored.bearing, parked.bearing)
    moved = restarted.update(None, VehicleMotion(1., False, 0.), 10.2, 2000.2)
    x, y = osm.offset_metres(moved.latitude, moved.longitude, restored)
    self.assertGreater(x, 0.)
    self.assertGreater(y, 0.)
    self.assertEqual(moved.bearing, restored.bearing)

  def test_reverse_distance_consumes_bridge_budget(self):
    p = self.position()
    motion = VehicleMotion(-30., False, 0.)
    p.update(osm.GpsFix(0., 0., 180., 3., 100.), motion, 100., 1100.)
    for i in range(1, 46):
      result = p.update(None, motion, 100. + i * .2, 1100. + i * .2)
    self.assertIsNone(result)  # reverse distance must not subtract from the 250 m limit

  def test_park_saves_final_turn_between_gps_packets(self):
    p = self.position()
    fix = osm.GpsFix(0., 0., 180., 3., 100.)
    motion = VehicleMotion(-3., False, None, yaw_rate=.2)
    for i in range(5):
      p.update(fix, motion, 100. + i * .2, 1100. + i * .2)
    p.update(fix, self.park, 100.9, 1100.9)
    self.assertEqual(len(self.writes), 1)
    self.assertAlmostEqual(self.writes[0]['bearing'], math.degrees(.17), places=5)
    self.assertLess(self.writes[0]['latitude'], 0.)

  def test_weak_course_bridge_rejects_conflicting_position(self):
    p = self.position()
    p.update(osm.GpsFix(0., 0., 0., 3., 100.), VehicleMotion(2., False, 0.), 100., 1100.)
    weak = osm.GpsFix(*point(100., 0.), None, 3., 100.2)
    result = p.update(weak, VehicleMotion(-2., False, 0.), 100.2, 1100.2)
    self.assertFalse(result.estimated)
    self.assertIsNone(p.update(None, VehicleMotion(-2., False, 0.), 100.4, 1100.4))

  def test_weak_course_does_not_renew_reverse_projection_budget(self):
    p = self.position()
    motion = VehicleMotion(-2., False, 0.)
    p.update(osm.GpsFix(0., 0., 180., 3., 100.), motion, 100., 1100.)
    estimated_count = 0
    for i in range(1, 102):
      now = 100. + i * .2
      fix = osm.GpsFix(*point(0., -i * .4), None, 3., now)
      result = p.update(fix, motion, now, 1100. + i * .2)
      if result.estimated:
        estimated_count += 1
        self.assertEqual(result.gps_timestamp, 100.)
    self.assertGreater(estimated_count, 90)
    self.assertFalse(result.estimated)  # only independently supported GPS heading can remain

  def test_reverse_clears_previous_map_result(self):
    provider = osm.OSMSpeedLimit()
    provider._thread = Mock()
    previous = osm.GpsFix(0., 0., 0., 3., 100., speed=2.)
    provider._cache = (road(),), previous, 100.
    self.assertIsNotNone(provider.update(previous, 100.)[1])
    self.assertIsNotNone(provider._tracker.match)
    self.assertIsNone(provider.update(replace(previous, speed=-1., timestamp=100.2), 100.2))
    self.assertIsNone(provider._fix)
    self.assertIsNone(provider._tracker.match)

  def test_stop_retains_integrated_heading_when_latest_gps_course_is_weak(self):
    p = self.position()
    motion = VehicleMotion(-2., False, None, yaw_rate=.2)
    p.update(osm.GpsFix(0., 0., 180., 15., 100.), motion, 100., 1100.)
    for i in range(1, 6):
      now = 100. + i * .2
      weak = osm.GpsFix(*point(0, -i * .4), None, 15., now)
      last = p.update(weak, motion, now, 1100. + i * .2)
    stopped = p.update(weak, self.park, 101.1, 1101.1)
    self.assertAlmostEqual(stopped.bearing, last.bearing + math.degrees(.01))
    self.assertAlmostEqual(self.writes[-1]['bearing'], stopped.bearing)

  def test_parking_position_disagreement_uses_bounded_allowances(self):
    for shift, retained in ((22., True), (35., False)):
      p = self.position()
      motion = VehicleMotion(-1., False, None, yaw_rate=.2)
      p.update(osm.GpsFix(0., 0., 180., 15., 100.), motion, 100., 1100.)
      for i in range(1, 31):
        p.update(None, motion, 100. + i * .2, 1100. + i * .2)
      projected = p._position
      weak = replace(projected, longitude=projected.longitude + point(shift, 0)[1],
                     bearing=None, accuracy=15., timestamp=106.2, estimated=False)
      result = p.update(weak, motion, 106.2, 1106.2)
      self.assertEqual(result.estimated, retained)
      if retained:
        self.assertEqual(result.gps_timestamp, 100.)  # weak fixes do not renew the bridge
      else:
        self.assertIsNone(p._position)

  def test_garage_approach_can_save_a_recent_projected_park_but_not_extend_gps_age(self):
    p = self.position()
    p.update(osm.GpsFix(0., 0., 0., 15., 100.), self.drive, 100., 1100.)
    for i in range(1, 26):
      p.update(None, self.drive, 100. + i * .2, 1100. + i * .2)
    for i in range(26, 201):
      p.update(None, self.park, 100. + i * .2, 1100. + i * .2)
    self.assertIn(POSITION_KEY, self.data)
    self.assertGreater(self.data[POSITION_KEY]['latitude'], 0.)
    self.assertLessEqual(self.data[POSITION_KEY]['savedAt'], 1130.)
    restored = MapPosition(self.params, 'test-car', 2000.)
    self.assertIsNotNone(restored.update(None, self.park, 10., 2000.))


if __name__ == "__main__":
  unittest.main()
