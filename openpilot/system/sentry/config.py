"""Shared sentry settings and a RAM-only process lease. No camera/control spoofing."""
import json
import fcntl
import math
import os
from contextlib import contextmanager
from pathlib import Path
import time

from openpilot.cereal import log
from openpilot.system.webrtc.helpers import livestream_video_active

RUNTIME_PATH = Path('/dev/shm/openpilot_sentry.json')
LEASE_SECONDS = 2.0
CAMERA_WARMUP_SECONDS = 10.
RECORDING_CAMERA_MASK = 0b111
MOTION_CAMERAS = (2, 1)  # Front wide, cabin wide (VisionStreamType).
DURATION_PRESETS = (15, 30, 60)
DURATION_KEYS = ('SentryArmDelay', 'SentryPreSeconds', 'SentryPostSeconds')
LOW_RISK, HIGH_RISK = 0, 1
DEFAULTS = {
  'SentryRiskMode': LOW_RISK, 'SentrySkipOnWifi': True,
  'SentryLivestreamOverride': False, 'SentryLivestreamMode': HIGH_RISK,
  'SentryEnabled': True, 'SentrySensitivity': 75, 'SentryAudioSensitivity': 25, 'SentryGyroSensitivity': 50, 'SentryArmDelay': 30,
  'SentryPreSeconds': 60, 'SentryPostSeconds': 60,
  'SentryCutoffVoltage': 11.8, 'SentryMaxHours': 4, 'SentryShutdownOnTimeLimit': True,
  'SentryMaxTemp': 70, 'SentryShutdownOnOverheat': False, 'SentryEye': True,
  'SentryWatermark': True,
}
RANGES = {
  'SentryRiskMode': (LOW_RISK, HIGH_RISK),
  'SentryLivestreamMode': (LOW_RISK, 2),  # 2 skips this parking session.
  'SentrySensitivity': (1, 100), 'SentryAudioSensitivity': (0, 100), 'SentryGyroSensitivity': (0, 100), 'SentryCutoffVoltage': (11.0, 11.8),
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


def parking_mode(params):
  return 'off' if params.get_bool('SentryOffThisDrive') else setting(params, 'SentryRiskMode')


@contextmanager
def parking_mode_lock():
  # UI and controller can make their first choice simultaneously. Serialize the
  # few parameter writes; the lock file lives in RAM and is never written per tick.
  with RUNTIME_PATH.with_suffix('.mode.lock').open('a') as lock:
    fcntl.flock(lock, fcntl.LOCK_EX)
    try:
      yield
    finally:
      fcntl.flock(lock, fcntl.LOCK_UN)


def select_parking_mode(params, mode):
  # Both UIs share one explicit choice; the controller must not replace it with
  # a Wi-Fi default. The master switch remains independent of this parking.
  if mode not in (LOW_RISK, HIGH_RISK, 'off'):
    raise ValueError(f'Invalid Sentry parking mode: {mode}')
  with parking_mode_lock():
    selected_at = time.monotonic()
    params.put('SentryParkingModeSource', 'manual', block=True)
    if mode != 'off':
      params.put('SentryRiskMode', mode, block=True)
    params.put_bool('SentryOffThisDrive', mode == 'off', block=True)
    params.put_bool('SentryModePrompted', True, block=True)
    params.put('SentryArmResetAt', selected_at, block=True)
  return selected_at


def apply_parking_default(params, sm, now):
  # Called by sentryd once fresh parked/network state is available, including
  # while the UI is asleep or the master switch is off. Latch until a new drive.
  if params.get('SentryParkingModeSource'):
    return
  if not params.get_bool('IsOffroad') or any(not sm.valid[s] or not 0 <= now-sm.recv_time[s] < 2.
                                           for s in ('deviceState', 'pandaStates')):
    return
  if sm['deviceState'].started or not sm['pandaStates'] or any(p.ignitionLine or p.ignitionCan for p in sm['pandaStates']):
    return
  with parking_mode_lock():
    if params.get('SentryParkingModeSource'):
      return
    if params.get_bool('SentryOffThisDrive'):
      source = 'manual'  # Retain a choice made before this controller update.
    else:
      wifi = sm['deviceState'].networkType == log.DeviceState.NetworkType.wifi
      source = 'wifi' if setting(params, 'SentrySkipOnWifi') and wifi else 'default'
      if source == 'wifi':
        params.put_bool('SentryOffThisDrive', True, block=True)
    params.put('SentryParkingModeSource', source, block=True)


def apply_livestream_mode(params):
  # Called once when a viewer actually enables video on a connected session,
  # never for Connect's background prewarm. Use the same parking choice as the UI.
  if not setting(params, 'SentryLivestreamOverride'):
    return None
  if not params.get_bool('IsOffroad') or not params.get_bool('IsLiveStreaming'):
    return None
  if any(params.get_bool(k) for k in ('DoShutdown', 'DoReboot', 'DoUninstall')):
    return None
  mode = setting(params, 'SentryLivestreamMode')
  mode = mode if mode < 2 else 'off'
  select_parking_mode(params, mode)
  if mode != 'off':
    params.put_bool('SentryEnabled', True, block=True)
  return mode


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
  return (not started and setting(params, 'SentryEnabled') and not params.get_bool('SentryOffThisDrive') and
          not livestream_video_active(params) and bool(read_runtime().get('capture')))


def sentry_gps_requested(started, params, CP):
  return not started and setting(params, 'SentryEnabled') and not params.get_bool('SentryOffThisDrive') and bool(read_runtime().get('gps'))
