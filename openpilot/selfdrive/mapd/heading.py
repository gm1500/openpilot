"""Fuse fresh GPS motion and calibrated steering for map-matching heading."""
import math
from collections import deque
from dataclasses import dataclass, replace

from openpilot.common.transformations.orientation import rot_from_euler
from openpilot.selfdrive.mapd.osm_speed_limit import GpsFix, GPS_MAX_AGE, offset_metres

PARKING_SPEED = 5.  # m/s; calibrated gyro covers tight, slow parking turns


@dataclass(frozen=True)
class VehicleMotion:
  speed: float  # signed wheel speed when the platform supplies direction
  stationary: bool
  curvature: float | None
  parked: bool = False
  yaw_rate: float | None = None

  @property
  def turn_rate(self) -> float | None:
    if self.stationary:
      return 0.
    if self.yaw_rate is not None:
      return self.yaw_rate  # already a body rotation; never reverse its sign
    return None if self.curvature is None else self.curvature * self.speed


def vehicle_motion(sm, now: float, signed_speed: bool = False) -> VehicleMotion | None:
  def fresh(service, age):
    return (service in sm.valid and sm.valid[service] and 0 <= now - sm.recv_time[service] <= age and
            0 <= now - sm.logMonoTime[service] * 1e-9 <= age)

  cs = sm['carState']
  if (not fresh('carState', .5) or not cs.canValid or not math.isfinite(cs.vEgo) or abs(cs.vEgo) > 75 or
      (not signed_speed and (cs.vEgo < -.1 or str(cs.gearShifter) == 'reverse' and cs.vEgo >= .3))):
    return None
  stationary = cs.standstill and abs(cs.vEgo) < .3
  speed = 0. if stationary else cs.vEgo if signed_speed else max(0., cs.vEgo)
  curvature = None
  params = sm['vehicleParameters']
  if fresh('controlsState', .5) and fresh('vehicleParameters', 1.) and params.valid and params.sensorValid:
    # controlsState.curvature is measured steering converted by the calibrated
    # vehicle model (ratio, offset, stiffness, wheelbase and roll), not desired steering.
    value = sm['controlsState'].curvature
    if math.isfinite(value) and abs(value) <= .2 and abs(value * speed) <= 1.2:
      curvature = value
  yaw_rate = None
  if abs(speed) <= PARKING_SPEED and fresh('deviceMotion', .3) and fresh('extrinsicsCalibration', 2.):
    pose, calib = sm['deviceMotion'], sm['extrinsicsCalibration']
    rate = pose.angularVelocityDevice
    rpy = list(calib.rpyCalib)
    values, stds = (rate.x, rate.y, rate.z), (rate.xStd, rate.yStd, rate.zStd)
    if (pose.inputsOK and pose.sensorsOK and rate.valid and 0 <= now - pose.timestamp * 1e-9 <= .3 and
        str(calib.calStatus) == 'calibrated' and
        len(rpy) == 3 and all(math.isfinite(v) for v in (*rpy, *values, *stds)) and
        all(abs(v) <= .5 for v in rpy) and all(0 < v <= .1 for v in stds)):
      value = float((rot_from_euler(rpy).T @ values)[2])
      if abs(value) <= 1.2:
        yaw_rate = value
  return VehicleMotion(speed, stationary, curvature, stationary and str(cs.gearShifter) == 'park', yaw_rate)


class MapHeading:
  def __init__(self):
    self.reset()

  def reset(self):
    self._time: float | None = None
    self._epoch: float | None = None
    self._rate: float | None = None
    self._speed = 0.
    self._yaw = self._distance = self._moving_time = 0.
    self._east = self._north = 0.
    self._history = deque(maxlen=40)
    self._anchor: GpsFix | None = None
    self._anchor_yaw = self._anchor_distance = self._anchor_moving_time = 0.
    self._processed: GpsFix | None = None
    self._result: GpsFix | None = None
    self._direction = 0
    self._course_after = -math.inf

  def course(self, fix: GpsFix, motion: VehicleMotion | None) -> float | None:
    """Convert travel course to facing heading only with known motion direction."""
    if motion is None or motion.stationary or abs(motion.speed) < .3 or fix.bearing is None or fix.timestamp < self._course_after:
      return None
    return (fix.bearing + (180 if motion.speed < 0 else 0)) % 360

  def seed(self, fix: GpsFix, motion: VehicleMotion, now: float):
    """Restore a saved heading only after a fresh position confirms its vicinity."""
    direction, course_after = self._direction, self._course_after
    self.reset()
    self._direction, self._course_after = direction, course_after
    self._anchor = fix
    self._time = now
    self._rate = motion.turn_rate

  def update(self, fix: GpsFix | None, motion: VehicleMotion | None, now: float) -> GpsFix | None:
    if fix is None or not 0 <= now - fix.timestamp <= GPS_MAX_AGE:
      self.reset()
      return None
    if motion is None:
      self.reset()
      return fix
    dt = 0. if self._time is None else now - self._time
    if not 0 <= dt <= .6:
      self.reset()
      dt = 0.
    direction = 0 if motion.stationary or abs(motion.speed) < .3 else (1 if motion.speed > 0 else -1)
    if direction:
      if self._direction and direction != self._direction:
        self._course_after = now + GPS_MAX_AGE  # discard course spanning a direction reversal
        self._history.clear()
      self._direction = direction
    rate = motion.turn_rate
    if self._epoch is None:
      self._epoch = now
    yaw_before = self._yaw
    if rate is not None and self._rate is not None:
      self._yaw += (rate + self._rate) * dt / 2
    elif not motion.stationary:
      self._anchor = None  # unknown rotation while moving cannot extend an old heading
      self._history.clear()
    step = (motion.speed + self._speed) * dt / 2
    self._distance += (abs(motion.speed) + abs(self._speed)) * dt / 2
    self._east += step * math.sin((yaw_before + self._yaw) / 2)
    self._north += step * math.cos((yaw_before + self._yaw) / 2)
    if not motion.stationary:
      self._moving_time += dt
    self._time, self._rate, self._speed = now, rate, motion.speed
    if fix == self._processed:
      return self._result  # polling/turning the wheel must not create new GPS evidence
    if self._processed is not None and not 0 < fix.timestamp - self._processed.timestamp <= GPS_MAX_AGE:
      self._anchor = None
      self._history.clear()
    self._processed = fix
    prediction = None
    gps_bearing = self.course(fix, motion)
    # Receiver course accuracy can be poor even while positions trace a clear
    # road. Recover absolute heading from a bounded GPS chord, rotating the
    # integrated steering trajectory to that chord. This also handles curves
    # without mistaking their average direction for the current heading.
    if rate is not None and not motion.stationary:
      while self._history and fix.timestamp - self._history[0][0].timestamp > 8:
        self._history.popleft()
      for past, east, north, distance in reversed(self._history):
        if gps_bearing is not None:
          break
        x, y = offset_metres(fix.latitude, fix.longitude, past)
        chord = math.hypot(x, y)
        predicted_chord = math.hypot(self._east - east, self._north - north)
        travelled = self._distance - distance
        if (chord >= max(30., 3 * max(fix.accuracy, past.accuracy)) and predicted_chord >= .8 * travelled and
            abs(chord - predicted_chord) <= max(8., travelled * .2)):
          rotation = math.atan2(x, y) - math.atan2(self._east - east, self._north - north)
          gps_bearing = math.degrees(rotation + self._yaw) % 360
      self._history.append((fix, self._east, self._north, self._distance))
    if self._anchor is not None:
      distance = math.hypot(*offset_metres(fix.latitude, fix.longitude, self._anchor))
      turn = math.degrees(self._yaw - self._anchor_yaw)
      travelled = self._distance - self._anchor_distance
      age = fix.timestamp - self._anchor.timestamp
      # Fresh moving positions permit a short steering-only bridge through a
      # turn. At rest/creep retain the tighter departure/drift allowance.
      moving_bridge = not motion.stationary and abs(motion.speed) >= 2 and age <= GPS_MAX_AGE
      bound = 75 * age + 10 if gps_bearing is not None or moving_bridge else 20
      # Time stopped does not consume the short low-speed departure allowance.
      timely = age <= GPS_MAX_AGE if gps_bearing is not None else self._moving_time - self._anchor_moving_time <= 10
      if distance <= bound and travelled <= bound and abs(turn) <= (90 if moving_bridge else 45) and timely:
        prediction = (self._anchor.bearing + turn) % 360
      else:
        self._anchor = None
    if gps_bearing is not None:
      # Fresh GPS remains the absolute reference. Reject inconsistent or late
      # steering evidence; it must not overrule a conflicting GPS course.
      if prediction is not None and (now - fix.timestamp > .5 or abs((prediction - gps_bearing + 180) % 360 - 180) > 25):
        prediction = None
      self._anchor, self._anchor_yaw, self._anchor_distance = replace(fix, bearing=gps_bearing), self._yaw, self._distance
      self._anchor_moving_time = self._moving_time
      bearing = gps_bearing
    else:
      bearing = prediction
    self._result = replace(fix, bearing=bearing, stationary=motion.stationary,
                           motion_bearing=prediction if not motion.stationary else None, speed=motion.speed,
                           odometer=self._distance, motion_epoch=self._epoch)
    return self._result
