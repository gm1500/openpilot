"""Shared sentry settings and a RAM-only process lease. No camera/control spoofing."""
import json
import math
import os
from pathlib import Path
import time

RUNTIME_PATH = Path('/dev/shm/openpilot_sentry.json')
LEASE_SECONDS = 2.0
CAMERA_WARMUP_SECONDS = 10.
RECORDING_CAMERA_MASK = 0b111
MOTION_CAMERAS = (2, 1)  # Front wide, cabin wide (VisionStreamType).
DURATION_PRESETS = (15, 30, 60)
DURATION_KEYS = ('SentryArmDelay', 'SentryPreSeconds', 'SentryPostSeconds')
DEFAULTS = {
  'SentryEnabled': True, 'SentrySensitivity': 75, 'SentryArmDelay': 30,
  'SentryPreSeconds': 60, 'SentryPostSeconds': 60,
  'SentryCutoffVoltage': 11.8, 'SentryMaxHours': 4, 'SentryShutdownOnTimeLimit': True,
  'SentryMaxTemp': 70, 'SentryShutdownOnOverheat': False, 'SentryEye': True,
  'SentryWatermark': True,
}
RANGES = {
  'SentrySensitivity': (1, 100), 'SentryCutoffVoltage': (11.0, 11.8),
  'SentryMaxHours': (1, 12), 'SentryMaxTemp': (55, 75),
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
    if key in DURATION_KEYS:
      return min(DURATION_PRESETS, key=lambda seconds: abs(seconds-number))
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
  return not started and setting(params, 'SentryEnabled') and bool(read_runtime().get('capture'))


def sentry_gps_requested(started, params, CP):
  return not started and setting(params, 'SentryEnabled') and bool(read_runtime().get('gps'))
