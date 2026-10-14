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
from openpilot.system.sentry.config import setting, write_runtime, RUNTIME_PATH, CAMERA_WARMUP_SECONDS, RECORDING_CAMERA_MASK, MOTION_CAMERAS, HIGH_RISK
from openpilot.system.sentry.clock import SentryClock
from openpilot.system.sentry.location import vehicle_identity, parked_location, gps_location
from openpilot.system.sentry.motion import MotionDetector, Motion, PreviewSelector
from openpilot.system.sentry.sensors import SensorActivity


def recent(sm, service, now, timeout=2.):
  return sm.valid[service] and 0 <= now-sm.recv_time[service] < timeout


def inactive_reason(params, sm, now, device_type):
  if not setting(params, 'SentryEnabled'):
    return 'Off'
  if params.get_bool('SentryOffThisDrive'):
    return 'Off for this drive'
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
    self.sample_indices = None
    self.client = VisionIpcClient('camerad', stream, True)
    self.detector = MotionDetector()
    self.last_frame = -1
    self.last_received = 0.
    self.warm_until = 0.

  def sample(self, now, sensitivity, armed, detect=True):
    if not self.client.is_connected():
      if not self.client.connect(False):
        return Motion()
      self.size, self.last_frame = None, -1
    buf = self.client.recv(5)
    if buf is None or self.client.frame_id == self.last_frame or not 0 <= now-self.client.timestamp_eof/1e9 <= .5:
      return Motion()
    size = self.client.width, self.client.height
    if size != self.size or self.client.frame_id < self.last_frame:
      self.detector = MotionDetector.for_camera(self.stream, *size)
      # Average a 4x4 grid inside each output cell. Isolated raw pixels alias
      # high-gain IR noise; blurring the resulting 128x80 image instead erases
      # small, low-contrast subjects. Keep the work bounded for both sensors.
      self.sample_indices = np.ix_(((np.arange(80*4)+.5)*size[1]/(80*4)).astype(int),
                                   ((np.arange(128*4)+.5)*size[0]/(128*4)).astype(int))
      self.size = size
      self.warm_until = now+CAMERA_WARMUP_SECONDS
    self.last_frame = self.client.frame_id
    self.last_received = now
    image = np.frombuffer(buf.data, dtype=np.uint8, count=self.client.stride*self.client.height)
    image = image.reshape(self.client.height, self.client.stride)[:, :self.client.width]
    sample = image[self.sample_indices].reshape(80, 4, 128, 4).mean(axis=(1, 3), dtype=np.float32)
    # Low Risk keeps a small current background while idle, skipping region
    # analysis. A sensor hit can then start tracking on this frame rather than
    # spending another two seconds learning the scene after the event begins.
    return self.detector.update(sample, now, sensitivity, armed=detect and armed and now >= self.warm_until)


def main():
  params = Params()
  clock = SentryClock(params)
  sm = messaging.SubMaster(['deviceState', 'pandaStates', 'peripheralState', 'gpsLocation', 'gpsLocationExternal', 'sentryRecorderState'])
  pm = messaging.PubMaster(['sentryState'])
  rk = Ratekeeper(5, print_delay_threshold=None)
  off_since = None
  parking_since, parking_expired = None, False
  ready_since = None
  arm_complete = False
  location = None
  cameras = {}
  sensors = None
  previous_risk = None
  cooldown = False
  record_until = 0.
  trigger_camera = 2
  preview = PreviewSelector()
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
      driving = (not params.get_bool('IsOffroad') or
                 recent(sm, 'deviceState', now) and sm['deviceState'].started or
                 recent(sm, 'pandaStates', now) and any(p.ignitionLine or p.ignitionCan for p in sm['pandaStates']))
      if driving:
        parking_since, parking_expired = None, False
        cooldown = False
      capture, gps_request = False, False
      best, motion_camera = Motion(), 2
      triggered = False
      if not session:
        sensors = None
        preview.reset()
        off_since, location, cameras = None, None, {}
        ready_since = None
        arm_complete = False
        record_until, gps_finished, low_since = 0., False, None
        cutoff_reason = ''
      else:
        # Cooling, health gaps and enable-switch changes cannot renew an expired
        # parking budget. Only a new ignition/onroad cycle starts another one.
        if parking_since is None:
          parking_since = now
        parking_expired |= now-parking_since >= setting(params, 'SentryMaxHours')*3600
        if off_since is None:
          off_since = now
          try:
            cp_data = params.get('CarParamsPersistent')
            if cp_data:
              with car.CarParams.from_bytes(cp_data) as CP:
                location = parked_location(params.get('MapParkedPosition'), vehicle_identity(CP), wall)
          except Exception:
            cloudlog.exception('sentry parking checkpoint')
        signature = setting(params, 'SentryWatermark')
        if config_signature is not None and signature != config_signature:
          restart_until = now+3.  # manager must observe an off period before reconfiguring encoders
        config_signature = signature
        elapsed = now-off_since
        temp = sm['deviceState'].maxTempC
        limit = setting(params, 'SentryMaxTemp')
        temperature_ok = math.isfinite(temp)
        if temperature_ok:
          cooldown = temp >= limit if not cooldown else temp > limit-8
        voltage = sm['peripheralState'].voltage / 1000.
        voltage_ok = sm.valid['peripheralState'] and now-sm.recv_time['peripheralState'] < 3. and 6 < voltage < 20
        if voltage_ok and voltage < setting(params, 'SentryCutoffVoltage'):
          low_since = now if low_since is None else low_since
        else:
          low_since = None
        low_battery = low_since is not None and now-low_since >= 10
        if low_battery:
          cutoff_reason = 'Battery cutoff'
        elif parking_expired and cutoff_reason != 'Battery cutoff':
          cutoff_reason = 'Parking budget reached'
        if cutoff_reason:
          phase = cutoff_reason
        elif not voltage_ok:
          phase = 'Waiting for battery reading'
        elif not temperature_ok:
          phase = 'Waiting for temperature reading'
        elif cooldown:
          phase = 'Cooling down'
        elif now < restart_until:
          phase = 'Applying camera settings'
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
          risk = setting(params, 'SentryRiskMode')
          if risk != previous_risk:
            cameras.clear()
            preview.reset()
            previous_risk = risk
          if sensors is None:
            sensors = SensorActivity()
          arm_delay = setting(params, 'SentryArmDelay')
          armed = arm_complete or ready_since is not None and now-ready_since >= CAMERA_WARMUP_SECONDS+arm_delay
          sensor_trigger = sensors.sample(now, armed)
          sensors_ready = sensors.ready(now)
          manual = params.get_bool('SentryManualTrigger')
          if manual:
            params.remove('SentryManualTrigger')
          # Sensors start/extend Low Risk events. During those events, camera
          # motion steers the preview and eye but cannot extend the recording.
          track_motion = risk == HIGH_RISK or manual or armed and (sensor_trigger or record_until > now)
          results = {}
          for stream in MOTION_CAMERAS:
            if stream not in cameras:
              cameras[stream] = CameraMotion(stream)
            if track_motion:
              result = cameras[stream].sample(now, setting(params, 'SentrySensitivity'), armed)
            else:
              result = cameras[stream].sample(now, setting(params, 'SentrySensitivity'), armed, detect=False)
            results[stream] = result
            if risk == HIGH_RISK:
              triggered |= result.triggered
            if result.active and (not best.active or result.score > best.score):
              best, motion_camera = result, stream
          ready = sum(0 <= now-camera.last_received < 1. for camera in cameras.values())
          recorder_fresh = recent(sm, 'sentryRecorderState', now)
          recorder_ready = recorder_fresh and sm['sentryRecorderState'].bufferBytes > 0
          if ready != len(cameras):
            phase = f'Waiting for cameras ({ready}/{len(cameras)})'
            if not arm_complete:
              ready_since = None
          elif not sensors_ready and risk != HIGH_RISK:
            phase = 'Waiting for sound / motion sensors'
          elif not recorder_fresh:
            phase = 'Starting recorder'
          elif not recorder_ready:
            phase = 'Waiting for encoded video'
          else:
            # Camera exposure gets its own settling period before the user's
            # full arming delay. Both start only after working video arrives.
            # Keep that clock across recorder-only gaps: saving pre-roll or
            # syncing segments can delay status while cameras stay exposed.
            if ready_since is None:
              ready_since = now
            if arm_complete:
              recovery = max(camera.warm_until-now for camera in cameras.values())
              phase = f'Camera recovering: {math.ceil(recovery)} s' if recovery > 0 else 'Watching'
            else:
              warmup_remaining = CAMERA_WARMUP_SECONDS-(now-ready_since)
              if warmup_remaining > 0:
                phase = f'Camera warmup: {math.ceil(warmup_remaining)} s'
              else:
                remaining = max(0., arm_delay+warmup_remaining)
                phase = f'Arming: {math.ceil(remaining)} s' if remaining else 'Watching'
              arm_complete = now-ready_since >= CAMERA_WARMUP_SECONDS+arm_delay
          # Arming belongs to this capture session, not an event or the eye.
          # Brief frame gaps do not disarm it; CameraMotion independently learns
          # exposure again after an actual camera reconnect/frame-counter reset.
          armed = arm_complete
          if not armed:
            best = Motion()  # Exposure ramp cannot trigger an event or the eye.
          can_trigger = armed and (recorder_ready or record_until > now)
          triggered |= sensor_trigger
          if not can_trigger:
            triggered = False
          if manual or can_trigger:
            trigger_camera = preview.update(results, now, recording=record_until > now, manual=manual, sensor_trigger=sensor_trigger)
          if manual or triggered:
            record_until = now+setting(params, 'SentryPostSeconds')
          if recorder_fresh and sm['sentryRecorderState'].recording:
            phase = 'Recording'
        else:
          cameras.clear()
          sensors = None
          preview.reset()
          ready_since = None
          arm_complete = False
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
                     'location': location, 'phase': phase, 'riskMode': setting(params, 'SentryRiskMode'),
                     'parkingStartedAt': parking_since, 'parkingLimitReached': parking_expired})
      msg = messaging.new_message('sentryState', valid=True)
      state = msg.sentryState
      state.sessionActive, state.capture, state.phase = bool(session), capture, phase
      state.motion = capture and best.active
      state.motionX, state.motionY, state.motionScore, state.motionCamera = best.x, best.y, best.score, motion_camera
      state.recordUntilMonoTime = int(record_until*1e9)
      state.triggerCamera = trigger_camera
      state.eyeEnabled, state.cameraMask = setting(params, 'SentryEye'), RECORDING_CAMERA_MASK
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
