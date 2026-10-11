"""Parked motion controller. Cameras/encoders and GPS are independent of onroad state."""
import time
import math
import numpy as np

from opendbc.car.structs import car
from msgq.visionipc import VisionIpcClient
from openpilot.cereal import messaging, log
from openpilot.common.hardware import HARDWARE
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.system.sentry.config import setting, write_runtime, RUNTIME_PATH, CAMERA_WARMUP_SECONDS
from openpilot.system.sentry.clock import SentryClock
from openpilot.system.sentry.location import vehicle_identity, parked_location, gps_location
from openpilot.system.sentry.motion import MotionDetector, Motion, camera_area_scale


def recent(sm, service, now, timeout=2.):
  return sm.valid[service] and 0 <= now-sm.recv_time[service] < timeout


def inactive_reason(params, sm, now, device_type):
  if not setting(params, 'SentryEnabled'):
    return 'Off'
  if device_type != 'tizi':
    return 'Requires comma 3X'
  if any(params.get_bool(k) for k in ('DoShutdown', 'DoReboot', 'DoUninstall')):
    return 'Shutting down'
  if not recent(sm, 'pandaStates', now) or not sm['pandaStates'] or any(
      p.pandaType == log.PandaState.PandaType.unknown for p in sm['pandaStates']):
    return 'Waiting for panda'
  if any(p.ignitionLine or p.ignitionCan for p in sm['pandaStates']):
    return 'Waiting for ignition off'
  if not recent(sm, 'deviceState', now):
    return 'Waiting for device state'
  if sm['deviceState'].started or not params.get_bool('IsOffroad'):
    return 'Waiting for offroad'
  return ''


class CameraMotion:
  def __init__(self, stream):
    self.stream = stream
    self.size = None
    self.client = VisionIpcClient('camerad', stream, True)
    self.detector = MotionDetector()
    self.last_frame = -1
    self.last_received = 0.

  def sample(self, now, sensitivity, armed):
    if not self.client.is_connected() and not self.client.connect(False):
      return Motion()
    buf = self.client.recv(5)
    if buf is None or self.client.frame_id == self.last_frame or not 0 <= now-self.client.timestamp_eof/1e9 <= .5:
      return Motion()
    self.last_frame = self.client.frame_id
    self.last_received = now
    size = self.client.width, self.client.height
    if size != self.size:
      self.detector = MotionDetector(area_scale=camera_area_scale(self.stream, *size))
      self.size = size
    image = np.frombuffer(buf.data, dtype=np.uint8, count=self.client.stride*self.client.height)
    image = image.reshape(self.client.height, self.client.stride)[:, :self.client.width]
    rows = np.linspace(0, image.shape[0]-1, 80, dtype=int)
    cols = np.linspace(0, image.shape[1]-1, 128, dtype=int)
    return self.detector.update(image[np.ix_(rows, cols)], now, sensitivity, armed=armed)


def main():
  params = Params()
  clock = SentryClock(params)
  sm = messaging.SubMaster(['deviceState', 'pandaStates', 'peripheralState', 'gpsLocation', 'gpsLocationExternal', 'sentryRecorderState'])
  pm = messaging.PubMaster(['sentryState'])
  rk = Ratekeeper(5, print_delay_threshold=None)
  off_since = None
  ready_since = None
  location = None
  cameras = {}
  cooldown = False
  record_until = 0.
  last_camera_mask = 0
  gps_finished = False
  low_since = None
  cutoff_reason = ''
  config_signature = None
  restart_until = 0.
  try:
    while True:
      sm.update(0)
      now, wall = time.monotonic(), time.time()  # noqa: TID251
      phase = inactive_reason(params, sm, now, HARDWARE.get_device_type())
      session = not phase
      capture, gps_request = False, False
      best, motion_camera = Motion(), 2
      triggered = False
      camera_mask = sum(1 << i for i, key in enumerate(('SentryNarrowCamera', 'SentryCabinCamera', 'SentryWideCamera')) if setting(params, key))
      if not session:
        off_since, location, cameras = None, None, {}
        ready_since = None
        record_until, cooldown, gps_finished, low_since = 0., False, False, None
        cutoff_reason = ''
      else:
        if off_since is None:
          off_since = now
          try:
            cp_data = params.get('CarParamsPersistent')
            if cp_data:
              with car.CarParams.from_bytes(cp_data) as CP:
                location = parked_location(params.get('MapParkedPosition'), vehicle_identity(CP), wall)
          except Exception:
            cloudlog.exception('sentry parking checkpoint')
        signature = (camera_mask, setting(params, 'SentryWatermark'))
        if config_signature is not None and signature != config_signature:
          restart_until = now+3.  # manager must observe an off period before reconfiguring encoders
        config_signature = signature
        elapsed = now-off_since
        temp = sm['deviceState'].maxTempC
        limit = setting(params, 'SentryMaxTemp')
        cooldown = not math.isfinite(temp) or (temp >= limit if not cooldown else temp > limit-8)
        voltage = sm['peripheralState'].voltage / 1000.
        voltage_ok = sm.valid['peripheralState'] and now-sm.recv_time['peripheralState'] < 3. and 6 < voltage < 20
        if voltage_ok and voltage < setting(params, 'SentryCutoffVoltage'):
          low_since = now if low_since is None else low_since
        else:
          low_since = None
        low_battery = low_since is not None and now-low_since >= 10
        expired = elapsed >= setting(params, 'SentryMaxHours')*3600
        if not cutoff_reason and (low_battery or expired):
          cutoff_reason = 'Battery cutoff' if low_battery else 'Parking budget reached'
        if cutoff_reason:
          phase = cutoff_reason
        elif not voltage_ok:
          phase = 'Waiting for battery reading'
        elif cooldown:
          phase = 'Cooling down'
        elif now < restart_until:
          phase = 'Applying camera settings'
        elif not camera_mask:
          phase = 'No cameras selected'
        elif params.get_bool('IsDriverViewEnabled') or params.get_bool('IsLiveStreaming'):
          phase = 'Camera preview in use'
        else:
          capture = elapsed >= 2.  # allow driving encoders to finish offroad transition
          phase = 'Starting cameras'
          if location is None and not gps_finished:
            for service in ('gpsLocationExternal', 'gpsLocation'):
              if sm.valid[service]:
                location = gps_location(sm[service], sm.logMonoTime[service]/1e9, now, wall)
                if location is not None:
                  break
            gps_finished = location is not None or elapsed >= 90.
            gps_request = not gps_finished
        if capture:
          if camera_mask != last_camera_mask:
            cameras.clear()
            ready_since = None
          last_camera_mask = camera_mask
          arm_delay = setting(params, 'SentryArmDelay')
          armed = ready_since is not None and now-ready_since >= CAMERA_WARMUP_SECONDS+arm_delay
          for stream in (2, 1, 0):
            if camera_mask & (1 << stream):
              if stream not in cameras:
                cameras[stream] = CameraMotion(stream)
              result = cameras[stream].sample(now, setting(params, 'SentrySensitivity'), armed)
              triggered |= result.triggered
              # Wide cameras drive the eye. Narrow camera can still trigger recording.
              if stream in (1, 2) and result.active and (not best.active or result.score > best.score):
                best, motion_camera = result, stream
          ready = sum(0 <= now-camera.last_received < 1. for camera in cameras.values())
          if ready != len(cameras):
            phase = f'Waiting for cameras ({ready}/{len(cameras)})'
            ready_since = None
          elif not recent(sm, 'sentryRecorderState', now):
            phase = 'Starting recorder'
            ready_since = None
          elif not sm['sentryRecorderState'].bufferBytes:
            phase = 'Waiting for encoded video'
            ready_since = None
          else:
            # Camera exposure gets its own settling period before the user's
            # full arming delay. Both start only after working video arrives.
            if ready_since is None:
              ready_since = now
            warmup_remaining = CAMERA_WARMUP_SECONDS-(now-ready_since)
            if warmup_remaining > 0:
              phase = f'Camera warmup: {math.ceil(warmup_remaining)} s'
            else:
              remaining = max(0., arm_delay+warmup_remaining)
              phase = f'Arming: {math.ceil(remaining)} s' if remaining else 'Watching'
          armed = ready_since is not None and now-ready_since >= CAMERA_WARMUP_SECONDS+arm_delay
          if not armed:
            best = Motion()  # Exposure ramp cannot trigger an event or the eye.
            triggered = False
          manual = params.get_bool('SentryManualTrigger')
          if manual:
            params.remove('SentryManualTrigger')
          if manual or triggered:
            record_until = now+setting(params, 'SentryPostSeconds')
          if sm['sentryRecorderState'].recording and now-sm.recv_time['sentryRecorderState'] < 2.:
            phase = 'Recording'
            if triggered or manual:
              record_until = now+setting(params, 'SentryPostSeconds')
        else:
          cameras.clear()
          ready_since = None
          record_until = 0.
        error = sm['sentryRecorderState'].error if now-sm.recv_time['sentryRecorderState'] < 2. else ''
        if capture and error:
          phase = error
      # Observe GPS that is already running (including onroad) to remember the
      # local timezone for later garage starts. This does not request extra GPS.
      time_location = location
      for service in ('gpsLocationExternal', 'gpsLocation'):
        if sm.valid[service]:
          fix = gps_location(sm[service], sm.logMonoTime[service]/1e9, now, wall)
          if fix is not None:
            time_location = fix
            break
      clock.update(time_location, now)
      zone_name, utc_offset = clock.snapshot(wall)
      write_runtime({'monoTime': now, 'session': bool(session), 'capture': capture, 'gps': gps_request,
                     'location': location, 'phase': phase})
      msg = messaging.new_message('sentryState', valid=True)
      state = msg.sentryState
      state.sessionActive, state.capture, state.phase = bool(session), capture, phase
      state.motion = capture and best.active
      state.motionX, state.motionY, state.motionScore, state.motionCamera = best.x, best.y, best.score, motion_camera
      state.recordUntilMonoTime = int(record_until*1e9)
      state.eyeEnabled, state.cameraMask = setting(params, 'SentryEye'), camera_mask
      state.timeZone, state.utcOffsetSeconds = zone_name, utc_offset
      state.locationValid = location is not None
      if location:
        state.latitude, state.longitude = location['latitude'], location['longitude']
        state.locationAccuracy, state.locationSavedAt = location['accuracy'], location['savedAt']
        state.locationSource = location['source']
      pm.send('sentryState', msg)
      rk.keep_time()
  finally:
    RUNTIME_PATH.unlink(missing_ok=True)


if __name__ == '__main__':
  main()
