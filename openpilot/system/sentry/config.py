"""Shared sentry settings and a RAM-only process lease. No camera/control spoofing."""
import json
import math
import os
from pathlib import Path
import time

RUNTIME_PATH = Path('/dev/shm/openpilot_sentry.json')
LEASE_SECONDS = 2.0
DEFAULTS = {
  'SentryEnabled': False, 'SentrySensitivity': 50, 'SentryArmDelay': 60,
  'SentryPreMinutes': 5, 'SentryPostSeconds': 60,
  'SentryNarrowCamera': True, 'SentryWideCamera': True, 'SentryCabinCamera': True,
  'SentryCutoffVoltage': 11.8, 'SentryMaxHours': 4,
  'SentryMaxTemp': 70, 'SentryStorageGB': 2, 'SentryEye': True,
  'SentryWatermark': True,
}
RANGES = {
  'SentrySensitivity': (1, 100), 'SentryArmDelay': (10, 300), 'SentryPreMinutes': (1, 5),
  'SentryPostSeconds': (10, 180), 'SentryCutoffVoltage': (11.0, 11.8),
  'SentryMaxHours': (1, 12), 'SentryMaxTemp': (55, 75),
  'SentryStorageGB': (1, 10),
}


def setting(params, key):
  default = DEFAULTS[key]
  value = params.get(key)
  if value is None:
    return default
  if isinstance(default, bool):
    return bool(value)
  try:
    number = float(value)
    if not math.isfinite(number):
      return default
    low, high = RANGES[key]
    return type(default)(min(high, max(low, number)))
  except (TypeError, ValueError, OverflowError):
    return default


def read_runtime(now=None):
  now = time.monotonic() if now is None else now
  try:
    state = json.loads(RUNTIME_PATH.read_text())
    if 0 <= now - float(state['monoTime']) <= LEASE_SECONDS:
      return state
  except (OSError, ValueError, TypeError, KeyError):
    pass
  return {}


def write_runtime(state):
  # tmpfs only: runtime updates never go through persistent Params.
  path = RUNTIME_PATH.with_suffix('.tmp')
  path.write_text(json.dumps(state, allow_nan=False))
  os.replace(path, RUNTIME_PATH)


def sentry_capture_requested(started, params, CP):
  return not started and bool(read_runtime().get('capture'))


def sentry_gps_requested(started, params, CP):
  return not started and bool(read_runtime().get('gps'))
