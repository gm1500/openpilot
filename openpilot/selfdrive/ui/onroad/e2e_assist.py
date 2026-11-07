import colorsys
import math


def slowing_assist_color(sm, started_frame, now):
  """Colour only a fresh, applied slowing constraint, not the model's proposal."""
  services = ('longitudinalPlan', 'carControl', 'carState', 'selfdriveState')
  for service in services:
    if (not sm.valid[service] or sm.recv_frame[service] < started_frame or
        not 0 <= now - sm.logMonoTime[service] * 1e-9 <= 0.3 or
        not 0 <= now - sm.recv_time[service] <= 0.3):
      return None

  plan = sm['longitudinalPlan']
  cs = sm['carState']
  if (sm['selfdriveState'].experimentalMode or not sm['selfdriveState'].enabled or not sm['carControl'].longActive or
      cs.gasPressed or cs.brakePressed or not plan.e2eAssistActive or
      not math.isfinite(plan.aTarget) or plan.aTarget >= 0):
    return None

  # Same yellow -> orange -> red hue scale as the experimental path. Use a
  # visible saturation even for gentle slowing, and the final requested accel.
  hue = max(60 + plan.aTarget * 35, 0) / 360
  return tuple(int(c * 255) for c in colorsys.hls_to_rgb(hue, 0.62, 1.0))
