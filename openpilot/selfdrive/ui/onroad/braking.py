"""Fresh driver, vehicle hold and applied openpilot brake feedback for the UI."""
import math


def fresh_ui_message(sm, service, started_frame, now):
  return (service in sm.valid and sm.valid[service] and sm.recv_frame[service] >= started_frame and
          0 <= now - sm.recv_time[service] <= .3 and 0 <= round(now * 1e9) - sm.logMonoTime[service] <= 300_000_000)


def braking_active(sm, started_frame, now):
  if not fresh_ui_message(sm, 'carState', started_frame, now):
    return False
  car = sm['carState']
  braking = bool(getattr(car, 'brakePressed', False) or getattr(car, 'brakeHoldActive', False))
  if (fresh_ui_message(sm, 'carControl', started_frame, now) and
      fresh_ui_message(sm, 'carOutput', started_frame, now) and sm['carControl'].longActive):
    brake = sm['carOutput'].actuatorsOutput.brake
    braking |= math.isfinite(brake) and brake > 0.
  return braking
