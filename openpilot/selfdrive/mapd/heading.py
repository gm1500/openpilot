"""Bounded steering-assisted heading for map matching; never invent GPS positions."""
import math
from dataclasses import dataclass, replace

from openpilot.selfdrive.mapd.osm_speed_limit import GpsFix, GPS_MAX_AGE, offset_metres


@dataclass(frozen=True)
class VehicleMotion:
  speed: float
  stationary: bool
  curvature: float | None


def vehicle_motion(sm, now: float) -> VehicleMotion | None:
  def fresh(service, age):
    return (sm.valid[service] and 0 <= now - sm.recv_time[service] <= age and
            0 <= now - sm.logMonoTime[service] * 1e-9 <= age)

  cs = sm['carState']
  if (not fresh('carState', .5) or not cs.canValid or not math.isfinite(cs.vEgo) or not -.1 <= cs.vEgo <= 75 or
      str(cs.gearShifter) == 'reverse'):
    return None
  speed = max(0., cs.vEgo)  # filtered ego speed can undershoot zero by tiny amounts at a stop
  curvature = None
  params = sm['vehicleParameters']
  if fresh('controlsState', .5) and fresh('vehicleParameters', 1.) and params.valid and params.sensorValid:
    # controlsState.curvature is measured steering converted by the calibrated
    # vehicle model (ratio, offset, stiffness, wheelbase and roll), not desired steering.
    value = sm['controlsState'].curvature
    if math.isfinite(value) and abs(value) <= .2 and abs(value * speed) <= 1.2:
      curvature = value
  return VehicleMotion(speed, cs.standstill and speed < .3, curvature)


class MapHeading:
  def __init__(self):
    self.reset()

  def reset(self):
    self._time: float | None = None
    self._rate: float | None = None
    self._speed = 0.
    self._yaw = self._distance = self._moving_time = 0.
    self._anchor: GpsFix | None = None
    self._anchor_yaw = self._anchor_distance = self._anchor_moving_time = 0.
    self._processed: GpsFix | None = None
    self._result: GpsFix | None = None

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
    rate = 0. if motion.stationary else None if motion.curvature is None else motion.curvature * motion.speed
    if rate is not None and self._rate is not None:
      self._yaw += (rate + self._rate) * dt / 2
    elif not motion.stationary:
      self._anchor = None  # unknown rotation while moving cannot extend an old heading
    self._distance += (motion.speed + self._speed) * dt / 2
    if not motion.stationary:
      self._moving_time += dt
    self._time, self._rate, self._speed = now, rate, motion.speed
    if fix == self._processed:
      return self._result  # polling/turning the wheel must not create new GPS evidence
    if self._processed is not None and not 0 < fix.timestamp - self._processed.timestamp <= GPS_MAX_AGE:
      self._anchor = None
    self._processed = fix
    prediction = None
    gps_bearing = fix.bearing if not motion.stationary else None
    if self._anchor is not None:
      distance = math.hypot(*offset_metres(fix.latitude, fix.longitude, self._anchor))
      turn = math.degrees(self._yaw - self._anchor_yaw)
      travelled = self._distance - self._anchor_distance
      age = fix.timestamp - self._anchor.timestamp
      bound = 75 * age + 10 if gps_bearing is not None else 20
      # Time stopped does not consume the short low-speed departure allowance.
      timely = age <= GPS_MAX_AGE if gps_bearing is not None else self._moving_time - self._anchor_moving_time <= 10
      if distance <= bound and travelled <= bound and abs(turn) <= 45 and timely:
        prediction = (self._anchor.bearing + turn) % 360
      else:
        self._anchor = None
    if gps_bearing is not None:
      # Fresh GPS remains the absolute reference. Reject inconsistent or late
      # steering evidence; it must not overrule a conflicting GPS course.
      if prediction is not None and (now - fix.timestamp > .5 or abs((prediction - gps_bearing + 180) % 360 - 180) > 25):
        prediction = None
      self._anchor, self._anchor_yaw, self._anchor_distance = fix, self._yaw, self._distance
      self._anchor_moving_time = self._moving_time
      bearing = gps_bearing
    else:
      bearing = prediction
    self._result = replace(fix, bearing=bearing, stationary=motion.stationary,
                           motion_bearing=prediction if not motion.stationary else None)
    return self._result
