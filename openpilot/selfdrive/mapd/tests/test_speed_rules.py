import unittest
from dataclasses import replace
from datetime import UTC, datetime
from unittest.mock import Mock, patch
from zoneinfo import ZoneInfo

from openpilot.selfdrive.mapd.osm_speed_limit import GpsFix, OSMSpeedLimit, Road, RoadTracker
from openpilot.selfdrive.mapd.speed_rules import conditional_speed, road_speed, schedule_active
from openpilot.selfdrive.mapd.zone_time import ZoneClock


def local(value, zone='America/Edmonton'):
  return datetime.fromisoformat(value).replace(tzinfo=ZoneInfo(zone))


class TestSchedules(unittest.TestCase):
  def test_daily_city_schedule_boundaries_and_weekends(self):
    tags = {'maxspeed': '50', 'maxspeed:conditional': '30 @ (07:30-21:00)'}
    for date in ('2026-10-09', '2026-10-10', '2026-12-25'):
      for time, speed in (('07:29:59', 50), ('07:30:00', 30), ('20:59:59', 30), ('21:00:00', 50)):
        with self.subTest(date=date, time=time):
          self.assertAlmostEqual(road_speed(tags, True, local_time=local(f'{date}T{time}')), speed / 3.6)

  def test_weekday_multiple_windows(self):
    schedule = 'Mo-Fr 08:00-09:30,14:30-16:00'
    for stamp, expected in (('2026-10-09T08:00', True), ('2026-10-09T09:30', False),
                            ('2026-10-09T14:30', True), ('2026-10-10T08:30', False)):
      self.assertEqual(schedule_active(schedule, local(stamp)), expected)

  def test_overnight_belongs_to_starting_weekday(self):
    schedule = 'Fr 22:00-06:00'
    for stamp, expected in (('2026-10-09T03:00', False), ('2026-10-09T22:00', True),
                            ('2026-10-10T05:59', True), ('2026-10-10T06:00', False)):
      self.assertEqual(schedule_active(schedule, local(stamp)), expected)

  def test_split_weekly_rules_and_wrap_weekdays(self):
    self.assertTrue(schedule_active('(Mo-Fr 07:00-09:00; Sa-Su 08:00-10:00)', local('2026-10-10T09:30')))
    self.assertTrue(schedule_active('Fr-Mo 08:00-10:00', local('2026-10-12T09:00')))
    self.assertFalse(schedule_active('Mo,We,Fr 08:00-10:00', local('2026-10-10T09:00')))

  def test_temporary_construction_start_end_and_expiry(self):
    tags = {'maxspeed': '100', 'maxspeed:conditional': '50 @ (2026 Oct 10-2026 Oct 12)'}
    for stamp, speed in (('2026-10-09T23:59', 100), ('2026-10-10T00:00', 50),
                         ('2026-10-12T23:59', 50), ('2026-10-13T00:00', 100)):
      self.assertAlmostEqual(road_speed(tags, True, local_time=local(stamp)), speed / 3.6)
    self.assertTrue(schedule_active('2026 Oct 10 22:00-06:00', local('2026-10-11T03:00')))
    self.assertFalse(schedule_active('2026 Oct 10 22:00-06:00', local('2026-10-10T03:00')))

  def test_dated_weekday_work_hours(self):
    expression = '2026 Oct 01-2026 Nov 01 Mo-Fr 08:00-18:00'
    self.assertTrue(schedule_active(expression, local('2026-10-09T10:00')))
    self.assertFalse(schedule_active(expression, local('2026-10-10T10:00')))
    self.assertFalse(schedule_active(expression, local('2026-11-02T10:00')))

  def test_unknown_conditions_and_malformed_rules_never_use_base(self):
    expressions = ['school_days', 'flashing', 'children_present', 'wet', 'sunrise-sunset',
                   'Mo-Fr 08:00-16:00; PH off; SH off', '08:00-08:00', '25:00-26:00',
                   '07:00-24:01', '2026 Feb 30', '2026 Oct 12-2026 Oct 10', '', '(07:30-21:00',
                   '07:30-21:00 OR wet']
    for expression in expressions:
      with self.subTest(expression=expression):
        self.assertIsNone(conditional_speed('30 @ (' + expression + ')', 50 / 3.6, local('2026-10-10T12:00')))
    for clock in (None, datetime(2026, 10, 10, 12)):
      self.assertIsNone(conditional_speed('30 @ (07:30-21:00)', 50 / 3.6, clock))

  def test_unknown_or_conflicting_clauses_invalidate_all(self):
    now = local('2026-10-10T12:00')
    for expression in ('30 @ 07:30-21:00; 40 @ 08:00-16:00', '30 @ 07:30-21:00; 20 @ wet',
                       '30 @ 07:30-21:00;', '30 @ ((07:30-21:00))', '30 @ 24/7 @ 24/7'):
      self.assertIsNone(conditional_speed(expression, 50 / 3.6, now))
    self.assertAlmostEqual(conditional_speed('30 @ 07:30-21:00; 40 @ 21:00-07:30', 50 / 3.6, now), 30 / 3.6)

  def test_mph_direction_and_advisory(self):
    tags = {'maxspeed': '50', 'maxspeed:forward:conditional': '20 mph @ (07:30-21:00)', 'maxspeed:advisory': '15'}
    now = local('2026-10-10T12:00')
    self.assertAlmostEqual(road_speed(tags, True, local_time=now), 20 * .44704)
    self.assertAlmostEqual(road_speed(tags, False, local_time=now), 50 / 3.6)
    self.assertIsNone(road_speed(tags, None, local_time=now))
    self.assertAlmostEqual(road_speed(tags, True, advisory=True, local_time=now), 15 / 3.6)
    tags['maxspeed:forward:conditional'] = '30 @ wet'
    self.assertIsNone(road_speed(tags, True, local_time=now))
    self.assertAlmostEqual(road_speed(tags, False, local_time=now), 50 / 3.6)


class TestZoneClock(unittest.TestCase):
  def setUp(self):
    self.clock = ZoneClock()

  def fix(self, latitude=53.5461, longitude=-113.4938, utc='2026-10-10T14:00:00+00:00'):
    return GpsFix(latitude, longitude, 0., 3., 100., utc_timestamp=datetime.fromisoformat(utc).timestamp())

  def test_global_coordinate_lookup(self):
    # One instant, different civil time in each city; no manually selected city.
    for lat, lon, hour, minute in ((53.5461, -113.4938, 8, 0), (43.6532, -79.3832, 10, 0),
                                   (49.2827, -123.1207, 7, 0), (33.4484, -112.0740, 7, 0),
                                   (51.5074, -.1278, 15, 0), (28.6139, 77.2090, 19, 30),
                                   (-33.8688, 151.2093, 1, 0)):
      fix = self.fix(lat, lon)
      self.clock.observe(fix, 100.)
      dt = self.clock.local_time(fix, 100.)
      self.assertIsNotNone(dt)
      self.assertEqual((dt.hour, dt.minute), (hour, minute))

  def test_dst_and_non_dst_city(self):
    for utc, expected in (('2026-07-10T14:00:00+00:00', 8), ('2026-12-10T14:00:00+00:00', 7)):
      clock = ZoneClock()
      fix = self.fix(utc=utc)
      clock.observe(fix, 100.)
      self.assertEqual(clock.local_time(fix, 100.).hour, expected)
    # Both occurrences of the repeated hour are evaluated in local civil time.
    for utc in ('2026-11-01T07:30:00+00:00', '2026-11-01T08:30:00+00:00'):
      clock = ZoneClock()
      fix = self.fix(utc=utc)
      clock.observe(fix, 100.)
      self.assertTrue(schedule_active('01:00-02:00', clock.local_time(fix, 100.)))

  def test_unknown_clock_stale_fix_and_monotonic_bridge(self):
    fix = self.fix()
    self.assertIsNone(self.clock.local_time(fix, 100.))
    self.clock.observe(replace(fix, estimated=True), 100.)
    self.assertIsNone(self.clock.utc(100.))
    self.clock.observe(fix, 104.)
    self.assertIsNone(self.clock.utc(104.))
    self.clock.observe(fix, 100.)
    self.assertAlmostEqual(self.clock.utc(120.), fix.utc_timestamp + 20)
    self.assertIsNone(self.clock.utc(701.))
    self.assertIsNone(self.clock.utc(99.))

  def test_clock_jump_cannot_be_accepted_by_repolling_same_fix(self):
    fix = self.fix()
    self.clock.observe(fix, 100.)
    bad = replace(fix, timestamp=101., utc_timestamp=fix.utc_timestamp + 3601.)
    for now in (101., 101.2, 101.4):
      self.clock.observe(bad, now)
      self.assertIsNone(self.clock.utc(now))
    self.clock.observe(replace(fix, timestamp=102., utc_timestamp=fix.utc_timestamp + 2.), 102.)
    self.assertIsNotNone(self.clock.utc(102.))

  def test_timezone_border_and_lookup_failure(self):
    fix = self.fix()
    self.clock.observe(fix, 100.)
    with patch.object(self.clock, '_zone', side_effect=['America/Edmonton', 'America/Edmonton', 'America/Vancouver',
                                                       'America/Edmonton', 'America/Edmonton']):
      self.assertIsNone(self.clock.local_time(fix, 100.))
    for error in (ImportError(), ValueError(), OSError()):
      with patch.object(self.clock, '_zone', side_effect=error):
        self.assertIsNone(self.clock.local_time(fix, 100.))


class TestTimedRoadIntegration(unittest.TestCase):
  def setUp(self):
    self.fix = GpsFix(53.5461, -113.4938, 0., 3., 100., stationary=True)
    self.road = Road(((53.5451, -113.4938), (53.5471, -113.4938)),
                     {'highway': 'residential', 'maxspeed': '50', 'maxspeed:conditional': '30 @ (07:30-21:00)'}, 42)

  def test_schedule_boundary_updates_repeated_fix_without_new_motion(self):
    tracker = RoadTracker()
    for stamp, speed in (('2026-10-10T07:29', 50), ('2026-10-10T07:30', 30), ('2026-10-10T21:00', 50)):
      result = tracker.update((self.road,), self.fix, local(stamp))
      self.assertEqual(result.road.way_id, 42)
      self.assertAlmostEqual(result.speed, speed / 3.6)
      self.assertEqual(tracker._paths[0][3], 0.)

  def test_actual_service_uses_location_clock_and_hides_unknown_advisory(self):
    service = OSMSpeedLimit()
    service._thread = Mock()
    service._cache = (self.road,), self.fix, 100.
    fix = replace(self.fix, utc_timestamp=datetime(2026, 10, 10, 14, tzinfo=UTC).timestamp())
    service.clock.observe(fix, 100.)
    result = service.update(fix, 100.)
    self.assertAlmostEqual(result[1], 30 / 3.6)
    service._cache = (replace(self.road, tags={**self.road.tags, 'maxspeed:conditional': '30 @ school_days',
                                             'maxspeed:advisory': '40'}),), self.fix, 100.
    self.assertEqual(service.update(fix, 100.)[1:], (None, None, 0.))

  def test_same_utc_in_new_timezone_uses_new_civil_schedule(self):
    tracker = RoadTracker()
    edt = local('2026-10-10T08:00')
    west = edt.astimezone(ZoneInfo('America/Vancouver'))
    self.assertEqual(edt, west)  # Python compares UTC; schedules must not.
    self.assertAlmostEqual(tracker.update((self.road,), self.fix, edt).speed, 30 / 3.6)
    self.assertAlmostEqual(tracker.update((self.road,), self.fix, west).speed, 50 / 3.6)

  def test_clock_boundary_does_not_qualify_ambiguous_parallel_road(self):
    other = replace(self.road, way_id=43, tags={'highway': 'residential', 'maxspeed': '50'})
    tracker = RoadTracker()
    roads = self.road, other
    self.assertIsNotNone(tracker.update(roads, self.fix, local('2026-10-10T07:29')))
    self.assertIsNone(tracker.update(roads, self.fix, local('2026-10-10T07:30')))

  def test_no_date_or_timezone_does_not_guess_school_speed(self):
    service = OSMSpeedLimit()
    service._thread = Mock()
    service._cache = (self.road,), self.fix, 100.
    self.assertEqual(service.update(self.fix, 100.)[1:], (None, None, 0.))


if __name__ == '__main__':
  unittest.main()
