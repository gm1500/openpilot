"""Validate a vehicle-specific parked position shared by maps and Sentry."""
import hashlib
import math


def vehicle_identity(CP):
  return hashlib.sha256(f'{CP.carFingerprint}:{CP.carVin}'.encode()).hexdigest()


def parked_location(data, vehicle_id, now):
  try:
    if not isinstance(data, dict) or data['version'] != 1 or not vehicle_id or data['vehicle'] != vehicle_id:
      return None
    lat, lon, accuracy, saved = (float(data[k]) for k in ('latitude', 'longitude', 'accuracy', 'savedAt'))
    if not all(math.isfinite(v) for v in (lat, lon, accuracy, saved, now)):
      return None
    if not (-85 < lat < 85 and -180 <= lon <= 180 and 0 < accuracy <= 30 and 0 <= now-saved <= 86400):
      return None
    return {'latitude': lat, 'longitude': lon, 'accuracy': accuracy, 'savedAt': saved, 'source': 'slcParked'}
  except (KeyError, TypeError, ValueError, OverflowError):
    return None
