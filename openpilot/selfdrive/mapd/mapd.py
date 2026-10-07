"""Optional on-road OSM service, independent of the UI and the control loop."""
import time
import hashlib

from opendbc.car.structs import car
from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.selfdrive.mapd.osm_speed_limit import OSMSpeedLimit, gps_fix
from openpilot.selfdrive.mapd.heading import vehicle_motion
from openpilot.selfdrive.mapd.position import MapPosition


def main():
  sm = messaging.SubMaster(['gpsLocationExternal', 'gpsLocation', 'carState', 'controlsState', 'vehicleParameters',
                           'deviceMotion', 'extrinsicsCalibration'])
  pm = messaging.PubMaster(['mapSpeedLimit', 'mapTrafficControl'])
  provider = OSMSpeedLimit()
  params = Params()
  with car.CarParams.from_bytes(params.get('CarParams', block=True)) as CP:
    vehicle_id = hashlib.sha256(f'{CP.carFingerprint}:{CP.carVin}'.encode()).hexdigest()
    signed_speed = CP.brand == 'gm'  # GM decodes actual wheel direction, independent of the selected gear
  # Wall time only checks checkpoint age across boots; integration uses monotonic time.
  position = MapPosition(params, vehicle_id, time.time())  # noqa: TID251
  rk = Ratekeeper(5, print_delay_threshold=None)
  while True:
    sm.update(0)
    now = time.monotonic()
    fix = position.update(gps_fix(sm, 1, now), vehicle_motion(sm, now, signed_speed), now, time.time())  # noqa: TID251
    sample = provider.update(fix, now)
    msg = messaging.new_message('mapSpeedLimit')
    msg.valid = sample is not None and any(speed is not None for speed in sample[1:3])
    if sample is not None:
      fix, limit, advisory, ahead = sample
      msg.mapSpeedLimit.speedLimit = limit or 0.
      msg.mapSpeedLimit.advisorySpeed = advisory or 0.
      msg.mapSpeedLimit.gpsMonoTime = int((fix.gps_timestamp if fix.estimated else fix.timestamp) * 1e9)
      msg.mapSpeedLimit.positionEstimated = fix.estimated
      msg.mapSpeedLimit.positionMonoTime = int(fix.timestamp * 1e9)
      msg.mapSpeedLimit.headingValid = fix.bearing is not None
      msg.mapSpeedLimit.distanceAhead = ahead
    pm.send('mapSpeedLimit', msg)
    # Independent validity: an untagged speed limit must not hide a well-matched
    # traffic control, and optional map health never disables ordinary control.
    control_msg = messaging.new_message('mapTrafficControl')
    control_msg.valid = sample is not None and provider.control_match is not None
    control_msg.mapTrafficControl.reason = provider.control_reason
    if control_msg.valid:
      control = control_msg.mapTrafficControl
      control.gpsMonoTime = int((fix.gps_timestamp if fix.estimated else fix.timestamp) * 1e9)
      control.positionMonoTime = int(fix.timestamp * 1e9)
      control.positionEstimated = fix.estimated
      control.matchedWayId = provider.control_match.road.way_id
      if provider.control is not None:
        target = provider.control
        control.kind = target.kind
        control.nodeId = target.node_id
        control.wayId = target.way_id
        control.distance = target.distance
    pm.send('mapTrafficControl', control_msg)
    rk.keep_time()


if __name__ == '__main__':
  main()
