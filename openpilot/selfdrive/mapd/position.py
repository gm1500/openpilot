"""Parked startup memory and bounded vehicle-motion bridges for map matching."""
import math
from dataclasses import replace

from openpilot.selfdrive.mapd.heading import MapHeading, VehicleMotion, PARKING_SPEED
from openpilot.selfdrive.mapd.osm_speed_limit import EARTH_RADIUS, GPS_MAX_AGE, GpsFix, offset_metres

POSITION_KEY = 'MapParkedPosition'
PARKED_MAX_AGE = 24 * 3600.
MAX_MOVING_TIME = 20.
MAX_DISTANCE = 250.
MAX_UNCERTAINTY = 30.
MAX_STATIONARY_TIME = 600.


class MapPosition:
  def __init__(self, params, vehicle_id: str, wall_time: float):
    self.params, self.vehicle_id = params, vehicle_id
    self.heading = MapHeading()
    self._saved = self._load(params.get(POSITION_KEY), wall_time)
    self._stored = self._saved is not None
    self._position: GpsFix | None = None
    self._time: float | None = None
    self._motion: VehicleMotion | None = None
    self._gps_time = 0.
    self._distance = self._moving_time = self._turn = 0.
    self._anchor_time = 0.
    self._base_accuracy = 0.
    self._last_write = -math.inf

  def _load(self, data, wall_time):
    try:
      if not isinstance(data, dict) or data['version'] != 1 or data['vehicle'] != self.vehicle_id or not self.vehicle_id:
        return None
      lat, lon, bearing, accuracy, saved_at = (float(data[k]) for k in ('latitude', 'longitude', 'bearing', 'accuracy', 'savedAt'))
      if (not all(math.isfinite(v) for v in (lat, lon, bearing, accuracy, saved_at, wall_time)) or
          not (-85 < lat < 85 and -180 <= lon <= 180 and 0 <= bearing < 360 and 0 < accuracy <= MAX_UNCERTAINTY and
               0 <= wall_time - saved_at <= PARKED_MAX_AGE)):
        return None
      return GpsFix(lat, lon, bearing, accuracy, 0., stationary=True, estimated=True)
    except (KeyError, TypeError, ValueError, OverflowError):
      return None

  def _anchor(self, fix: GpsFix, now: float):
    self._position = replace(fix, timestamp=now, estimated=True, gps_timestamp=self._gps_time)
    self._base_accuracy = fix.accuracy
    self._distance = self._moving_time = self._turn = 0.
    self._anchor_time = now

  def _project(self, motion: VehicleMotion | None, now: float):
    if self._position is None:
      return
    dt = now - self._time if self._time is not None else 0.
    if motion is None or self._motion is None or not 0 <= dt <= .6:
      self._position = None
      return
    if motion.turn_rate is None or self._motion.turn_rate is None:
      self._position = None
      return
    speed = (motion.speed + self._motion.speed) / 2
    step = 0. if motion.stationary and self._motion.stationary else speed * dt
    turn = (motion.turn_rate + self._motion.turn_rate) * dt / 2
    travelled = (abs(motion.speed) + abs(self._motion.speed)) * dt / 2
    self._distance += travelled
    self._moving_time += dt if not motion.stationary else 0.
    self._turn += abs(math.degrees(turn))
    # Conservative growth allowance, not a measured statistical confidence.
    uncertainty = self._base_accuracy + .03 * self._distance + .1 * self._turn
    if (self._distance > MAX_DISTANCE or self._moving_time > MAX_MOVING_TIME or uncertainty > MAX_UNCERTAINTY or
        now - self._anchor_time > MAX_STATIONARY_TIME):
      self._position = None
      return
    p = self._position
    middle = math.radians(p.bearing) + turn / 2
    lat = p.latitude + math.degrees(step * math.cos(middle) / EARTH_RADIUS)
    lon = (p.longitude + math.degrees(step * math.sin(middle) / (EARTH_RADIUS * math.cos(math.radians(p.latitude)))) + 180) % 360 - 180
    self._position = replace(p, latitude=lat, longitude=lon, bearing=(p.bearing + math.degrees(turn)) % 360,
                             accuracy=uncertainty, timestamp=now, stationary=motion.stationary, speed=motion.speed,
                             motion_bearing=None if motion.stationary else (p.bearing + math.degrees(turn)) % 360,
                             odometer=None if p.odometer is None else p.odometer + travelled)

  def update(self, fix: GpsFix | None, motion: VehicleMotion | None, now: float, wall_time: float) -> GpsFix | None:
    if fix is not None and not 0 <= now - fix.timestamp <= GPS_MAX_AGE:
      fix = None
    if self._saved is not None and motion is not None:
      if motion.stationary:
        self._anchor(self._saved, now)
        self._time, self._motion = now, motion
      self._saved = None  # never restore a previous trip after unobserved motion
    self._project(motion, now)
    result = self.heading.update(fix, motion, now)
    if fix is not None:
      course = self.heading.course(fix, motion)
      # Compare two uncertain positions using their bounded allowances. Parking
      # multipath can exceed 20 m without contradicting the wheel/gyro heading.
      tolerance = (min(MAX_UNCERTAINTY, max(20., math.hypot(fix.accuracy, self._position.accuracy)))
                   if self._position is not None and motion is not None and abs(motion.speed) <= PARKING_SPEED and course is None else 20.)
      near_estimate = (self._position is not None and
                       math.hypot(*offset_metres(fix.latitude, fix.longitude, self._position)) <= tolerance)
      if self._stored and self._saved is None and not near_estimate:
        self.params.remove(POSITION_KEY)
        self._stored = False
      if (self._position is not None and motion is not None and motion.stationary and
          near_estimate):
        self.heading.seed(replace(fix, bearing=self._position.bearing), motion, now)
        result = self.heading.update(fix, motion, now)
      if (motion is not None and not motion.stationary and abs(motion.speed) <= PARKING_SPEED and course is None and
          near_estimate):
        # Weak parking GPS course cannot replace a facing heading. Carry the
        # bounded wheel/gyro estimate without renewing its anchor or GPS age.
        result = self._position
      elif fix.timestamp != self._gps_time:
        self._gps_time = fix.timestamp
        self._position = None
        # A heading recovered from a noisy GPS chord can support matching with
        # fresh positions, but must not seed a further position extrapolation.
        # At rest, preserve the last established heading instead of GPS course.
        if result.bearing is not None and motion is not None and (motion.stationary or course is not None):
          self._anchor(result, now)
    else:
      result = self._position
    self._time, self._motion = now, motion

    # Save on the first verified Park sample, before a quick offroad shutdown.
    # A short projected garage approach after signal loss is also eligible.
    # A restored checkpoint can never save itself again:
    # this trip needs a real GPS anchor within 30 seconds of the parked sample.
    if motion is None or not motion.parked:
      if self._stored and self._saved is None:
        self.params.remove(POSITION_KEY)
        self._stored = False
    else:
      # Include motion since the last 1 Hz GPS packet when Park arrives between
      # fixes. The live projection carries the final turn and stop position.
      checkpoint = self._position or result
      if (checkpoint is not None and checkpoint.bearing is not None and self._gps_time > 0 and
          0 <= now - self._gps_time <= 30 and (not self._stored or now - self._last_write >= 10)):
        self.params.put(POSITION_KEY, {'version': 1, 'vehicle': self.vehicle_id, 'latitude': checkpoint.latitude,
                                    'longitude': checkpoint.longitude, 'bearing': checkpoint.bearing, 'accuracy': checkpoint.accuracy,
                                    'savedAt': wall_time}, block=True)
        self._stored, self._last_write = True, now
    return result
