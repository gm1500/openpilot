"""Take one fresh GPS snapshot for a parked Sentry session."""
import math


def gps_location(gps, mono, now, wall):
  values = (gps.latitude, gps.longitude, gps.horizontalAccuracy, wall)
  if (not gps.hasFix or not 0 <= now-mono <= 3. or not all(math.isfinite(x) for x in values) or
      not (-85 < gps.latitude < 85 and -180 <= gps.longitude <= 180 and 0 < gps.horizontalAccuracy <= 30)):
    return None
  return {'latitude': gps.latitude, 'longitude': gps.longitude, 'accuracy': gps.horizontalAccuracy,
              'savedAt': wall, 'source': 'gpsSnapshot'}
