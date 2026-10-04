"""Optional on-road OSM service, independent of the UI and the control loop."""
import time

from openpilot.cereal import messaging
from openpilot.common.realtime import Ratekeeper
from openpilot.selfdrive.mapd.osm_speed_limit import OSMSpeedLimit, gps_fix


def main():
  sm = messaging.SubMaster(['gpsLocationExternal', 'gpsLocation'])
  pm = messaging.PubMaster(['mapSpeedLimit'])
  provider = OSMSpeedLimit()
  rk = Ratekeeper(5, print_delay_threshold=None)
  while True:
    sm.update(0)
    now = time.monotonic()
    display_limit = provider.update(gps_fix(sm, 1, now), now)
    sample = provider.control_sample
    msg = messaging.new_message('mapSpeedLimit')
    msg.valid = sample is not None and sample[1] is not None
    if sample is not None:
      fix, limit = sample
      msg.mapSpeedLimit.speedLimit = limit or 0.
      msg.mapSpeedLimit.gpsMonoTime = int(fix.timestamp * 1e9)
      msg.mapSpeedLimit.headingValid = fix.bearing is not None
    msg.mapSpeedLimit.displayValid = display_limit is not None
    msg.mapSpeedLimit.displaySpeedLimit = display_limit or 0.
    msg.mapSpeedLimit.displayGpsMonoTime = int(provider.display_timestamp * 1e9)
    pm.send('mapSpeedLimit', msg)
    rk.keep_time()


if __name__ == '__main__':
  main()
