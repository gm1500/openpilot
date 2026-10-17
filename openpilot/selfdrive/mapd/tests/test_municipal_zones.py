import json
import math
import unittest
from dataclasses import replace
from datetime import datetime
from pathlib import Path
from unittest.mock import Mock, patch
from zoneinfo import ZoneInfo

from openpilot.selfdrive.mapd import municipal_zones as municipal
from openpilot.selfdrive.mapd.osm_speed_limit import GpsFix, OSMSpeedLimit, Road, road_candidates

# Public City of Edmonton and OSM geometry, not a driver's recorded GPS trace.
# https://data.edmonton.ca/resource/6vjs-shqe.json?$where=objectid='430'
# https://www.openstreetmap.org/way/469407361
# https://data.edmonton.ca/resource/6vjs-shqe.json?$where=objectid='1517'
# https://www.openstreetmap.org/way/1375525942
FIXTURE = json.loads((Path(__file__).parent / 'fixtures/edmonton_zones.json').read_text())


def utc(stamp):
  return datetime.fromisoformat(stamp).replace(tzinfo=ZoneInfo('America/Edmonton')).timestamp()


class TestMunicipalZones(unittest.TestCase):
  def setUp(self):
    self.zones = municipal.parse_edmonton(FIXTURE['municipal'])
    self.road = Road(tuple(map(tuple, FIXTURE['osm']['geometry'])), FIXTURE['osm']['tags'], int(FIXTURE['osm']['id']))
    self.fix = GpsFix(*self.road.geometry[3], 185., 3., 100., utc_timestamp=utc('2026-10-10T16:15:30'))
    self.match = road_candidates((self.road,), self.fix)[0]

  def test_published_geometry_replaces_osm_40_with_active_30(self):
    self.assertAlmostEqual(self.match.speed, 40 / 3.6)
    result = municipal.zone_limit(self.zones, self.match, self.fix.utc_timestamp)
    self.assertAlmostEqual(result.speed, 30 / 3.6)
    self.assertEqual(result.kind, 'school')
    self.assertTrue(result.source.startswith('edmonton:'))

  def test_published_divided_road_join_keeps_school_limit(self):
    data = FIXTURE['osm_join']
    road = Road(tuple(map(tuple, data['geometry'])), data['tags'], int(data['id']))
    # The public OSM endpoint falls between the municipal approach and its
    # short median flare. Test positions along the road, not a recorded drive.
    end, previous = road.geometry[:2]
    for bearing in (0., 180.):
      for fraction in (.5, .1, .01, 0.):
        with self.subTest(bearing=bearing, fraction=fraction):
          lat, lon = (a + fraction * (b - a) for a, b in zip(end, previous, strict=True))
          fix = replace(self.fix, latitude=lat, longitude=lon, bearing=bearing)
          match = road_candidates((road,), fix)[0]
          result = municipal.zone_limit(self.zones, match, fix.utc_timestamp)
          self.assertIsNotNone(result)
          self.assertAlmostEqual(result.speed, 30 / 3.6)
          self.assertEqual(result.kind, 'school')

  def test_short_flare_uses_local_direction_without_extending_geometry(self):
    def point(x, y):
      return 53. + math.degrees(y / 6371000.), -113. + math.degrees(x / (6371000. * math.cos(math.radians(53.))))

    zone = replace(self.zones[0], geometry=(point(0., 0.), point(-5., 6.), point(-7., 25.)))
    match = replace(self.match, position=point(-2.5, 3.), bearing=0.)
    for geometry in (zone.geometry, tuple(reversed(zone.geometry))):
      self.assertIsNotNone(municipal.zone_limit((replace(zone, geometry=geometry),), match, self.fix.utc_timestamp))
    # No neighboring shape to support the short diagonal, or genuinely wrong
    # road direction: keep the original alignment rejection.
    self.assertIsNone(municipal.zone_limit((replace(zone, geometry=zone.geometry[:2]),), match, self.fix.utc_timestamp))
    self.assertIsNone(municipal.zone_limit((zone,), replace(match, bearing=90.), self.fix.utc_timestamp))

    for geometry in ((point(0., 0.), point(6., 0.), point(6., 30.)),
                     (point(0., 0.), point(6., 0.), point(0., 0.))):
      with self.subTest(geometry=geometry):
        # A right-angle corner or a folded-back line must not borrow the
        # direction of an arbitrarily long next segment or a zero-length chord.
        self.assertIsNone(municipal.zone_limit((replace(zone, geometry=geometry),),
                                             replace(match, position=point(3., -1.)), self.fix.utc_timestamp))

  def test_local_schedule_boundaries_weekends_and_dst(self):
    for day in ('2026-10-10', '2026-11-01', '2027-03-14'):
      for hour, active in (('07:29:59', False), ('07:30:00', True), ('20:59:59', True), ('21:00:00', False)):
        with self.subTest(day=day, hour=hour):
          self.assertEqual(municipal.zone_limit(self.zones, self.match, utc(f'{day}T{hour}')) is not None, active)

  def test_unknown_clock_schedule_and_expired_feed_suppress_default(self):
    self.assertIsNone(municipal.zone_limit(self.zones, self.match, None).speed)
    self.assertIsNone(municipal.zone_limit(self.zones, self.match, self.fix.utc_timestamp, fresh=False).speed)
    zones = tuple(replace(zone, schedule='school_days') for zone in self.zones)
    self.assertIsNone(municipal.zone_limit(zones, self.match, self.fix.utc_timestamp).speed)

  def test_nearby_crossing_unnamed_and_unknown_heading_are_not_zone_matches(self):
    for match in (replace(self.match, bearing=95.), replace(self.match, bearing=None),
                  replace(self.match, position=(self.match.position[0], self.match.position[1] - .001)),
                  replace(self.match, road=replace(self.road, tags={**self.road.tags, 'name': '114 Street NW'}))):
      self.assertIsNone(municipal.zone_limit(self.zones, match, self.fix.utc_timestamp))

  def test_zone_does_not_extend_beyond_endpoints(self):
    zone = municipal.SpeedZone(((53., -113.), (53.001, -113.)), 'TEST ROAD', 30 / 3.6,
                              '07:30-21:00', 'America/Edmonton', 'school', 'test')
    match = replace(self.match, road=replace(self.road, tags={'name': 'TEST ROAD'}), bearing=0.)
    for latitude in (52.99999, 53.00101):  # about one metre outside each end
      self.assertIsNone(municipal.zone_limit((zone,), replace(match, position=(latitude, -113.)), self.fix.utc_timestamp))

  def test_conflicts_do_not_choose_a_higher_speed(self):
    zones = (*self.zones, *(replace(z, speed=40 / 3.6) for z in self.zones))
    self.assertIsNone(municipal.zone_limit(zones, self.match, self.fix.utc_timestamp).speed)

  def test_service_without_timezonefinder_and_on_clock_boundary(self):
    service = OSMSpeedLimit()
    service._thread = Mock()
    service._cache = (self.road,), self.fix, 100.
    service._zone_cache = self.zones, self.fix, 100.
    with patch.object(service.clock, '_zone', side_effect=AssertionError('municipal zone has its own timezone')):
      service.clock.observe(self.fix, 100.)
      self.assertAlmostEqual(service.update(self.fix, 100.)[1], 30 / 3.6)
      self.assertEqual(service.zone_type, 'school')
      self.assertTrue(service.zone_source.startswith('edmonton:'))
      # A new valid GPS anchor can be initialized after stopping/restarting.
      service.clock._anchor = (utc('2026-10-10T21:00'), 100.)
      self.assertAlmostEqual(service.update(self.fix, 100.)[1], 40 / 3.6)
      self.assertEqual(service.zone_type, 'none')
      self.assertEqual(service.zone_source, '')
      service._zone_cache = self.zones, self.fix, -1000.
      self.assertEqual(service.update(self.fix, 100.)[1:], (None, None, 0.))

  def test_unknown_or_lower_osm_restriction_cannot_be_relaxed(self):
    service = OSMSpeedLimit()
    service._thread = Mock()
    service._zone_cache = self.zones, self.fix, 100.
    service.clock.observe(self.fix, 100.)
    for extra, expected in (({'maxspeed': '20'}, 20 / 3.6), ({'maxspeed:conditional': '20 @ (flashing)'}, None)):
      service._cache = (replace(self.road, tags={**self.road.tags, **extra}),), self.fix, 100.
      with patch.object(service.clock, 'local_time', return_value=None):
        self.assertEqual(service.update(self.fix, 100.)[1], expected)
        self.assertEqual(service.zone_type, 'none')

  def test_source_parser_keeps_unknown_schedule_and_rejects_partial_or_unusable_data(self):
    row = FIXTURE['municipal'][0]
    self.assertEqual(municipal.parse_edmonton([{**row, 'effective_time': 'school days'}])[0].schedule, 'unknown')
    self.assertEqual(municipal.parse_edmonton([{**row, 'type': 'BYLAW DEFAULT'}]), ())
    for rows in (None, [row] * municipal.MAX_ROWS, [{**row, 'speed': 'unknown'}],
                 [{**row, 'geometry_line': {}}], [{**row, 'travel_direction': 'Northbound'}]):
      with self.assertRaises(ValueError):
        municipal.parse_edmonton(rows)

  def test_fetch_is_geographic_bounded_and_does_not_download_outside_coverage(self):
    response = Mock()
    with patch.object(municipal.requests, 'get') as get:
      get.return_value.__enter__.return_value = response
      response.iter_content.return_value = [json.dumps(FIXTURE['municipal']).encode()]
      self.assertEqual(municipal.fetch_zones(self.fix), self.zones)
      self.assertIn('within_box', get.call_args.kwargs['params']['$where'])
      get.reset_mock()
      self.assertEqual(municipal.fetch_zones(replace(self.fix, latitude=49., longitude=-123.)), ())
      get.assert_not_called()
      response.iter_content.return_value = [b'x' * (municipal.MAX_BYTES + 1)]
      with self.assertRaises(ValueError):
        municipal.fetch_zones(self.fix)


if __name__ == '__main__':
  unittest.main()
