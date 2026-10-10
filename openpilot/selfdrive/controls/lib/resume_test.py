"""Driver-held, one-shot Sierra brake-release experiment. No CAN/safety bypass."""
import math

from opendbc.car.gm.values import CAR

MAX_DURATION = 1.0
TEST_ACCEL = 0.3
MAX_SPEED = 0.5
REQUEST_MAX_AGE = 0.20
INPUT_MAX_AGE = 0.30
COOLDOWN = 1.0
SERVICES = ('carState', 'carOutput', 'selfdriveState', 'longitudinalPlan', 'modelV2', 'radarState', 'driverMonitoringState')


def fresh(sm, service, now, max_age=INPUT_MAX_AGE):
  return (sm.valid[service] and sm.seen[service] and
          0 <= now - sm.logMonoTime[service] * 1e-9 <= max_age and
          0 <= now - sm.recv_time[service] <= max_age)


class ResumeTest:
  def __init__(self, CP):
    self.supported = (CP.carFingerprint == CAR.CHEVROLET_SILVERADO and CP.openpilotLongitudinalControl and
                      str(CP.networkLocation) == 'fwdCamera')
    self.ready = self.active = self.latched = False
    self.reason = 'unavailable'
    self.last_id = 0
    self.start = None
    self.cooldown_until = 0.
    self.remaining = 0.

  def update(self, sm, now, long_active, stopping):
    cs, plan = sm['carState'], sm['longitudinalPlan']
    normal = (plan.aTarget, plan.shouldStop)
    request = sm['resumeTestRequest']
    request_fresh = fresh(sm, 'resumeTestRequest', now, REQUEST_MAX_AGE)
    new_press = request_fresh and request.held and request.requestId > self.last_id
    if new_press:
      self.last_id = request.requestId  # rejected presses never become delayed launches
    request_held = request_fresh and request.held and request.requestId == self.last_id
    self.ready = False
    self.remaining = 0.

    # Driver pedals, disengagement and physical Resume leave this isolated test.
    resume = any(str(b.type) == 'accelCruise' and b.pressed for b in cs.buttonEvents)
    if not self.supported or not long_active or cs.gasPressed or cs.brakePressed or resume:
      self.active = self.latched = False
      self.start = None
      self.reason = 'driver' if self.supported else 'unsupported'
      return normal

    valid = all(fresh(sm, service, now) for service in SERVICES)
    values = (cs.vEgo, cs.vEgoRaw, cs.aEgo, cs.vCruise, plan.aTarget)
    valid = valid and all(math.isfinite(x) for x in values)
    hazard = (not valid or not cs.canValid or cs.canTimeout or cs.accFaulted or cs.carNotReady or
              cs.carFaultedNonCritical or cs.doorOpen or cs.seatbeltUnlatched or cs.parkingBrake or
              str(cs.gearShifter) != 'drive' or cs.stockAeb or cs.stockFcw or plan.fcw or
              not sm['selfdriveState'].active or str(sm['selfdriveState'].state) != 'enabled' or
              sm['driverMonitoringState'].noResponseForceDecel or not 0 < cs.vCruise < 255)
    for lead in (sm['radarState'].leadOne, sm['radarState'].leadTwo):
      if lead.present:
        hazard |= (not math.isfinite(lead.dRel) or not math.isfinite(lead.vRel) or
                   lead.dRel < 6. + 2. * max(0., -lead.vRel))

    if self.active:
      elapsed = now - self.start
      reason = ('blocked' if hazard else 'released' if not request_held else
                'speedLimit' if abs(cs.vEgo) >= MAX_SPEED or abs(cs.vEgoRaw) >= MAX_SPEED else
                'timeout' if not 0 <= elapsed < MAX_DURATION else None)
      if reason is not None:
        self.active = False
        self.cooldown_until = now + COOLDOWN
        self.reason = reason

    held_stop = (cs.standstill and abs(cs.vEgo) < .05 and abs(cs.vEgoRaw) < .05 and stopping and
                 sm['carOutput'].actuatorsOutput.brake > 0)
    self.ready = not self.active and not hazard and held_stop and now >= self.cooldown_until
    if new_press and self.ready and 0 <= now - request.requestId * 1e-9 <= REQUEST_MAX_AGE:
      self.active = self.latched = True
      self.start = now
      self.reason = 'pulse'
      self.ready = False

    if self.active:
      self.remaining = max(0., MAX_DURATION - (now - self.start))
      # This explicit manual test overrides the existing model stop for at
      # most one second. Real-lead/FCW/AEB vetoes above remain authoritative.
      return TEST_ACCEL, False
    if self.latched:
      return min(plan.aTarget, 0.) if math.isfinite(plan.aTarget) else 0., True
    self.reason = 'ready' if self.ready else 'blocked'
    return normal

  def apply(self, controller, active, cs, target, should_stop, limits):
    if self.latched and not self.active:
      # Restore full calibrated hold without ramping from the positive pulse.
      controller.last_output_accel = min(controller.last_output_accel, controller.CP.stopAccel)
    accel = controller.update(active, cs, target, should_stop, limits)
    if self.active:
      accel = min(accel, TEST_ACCEL)
      controller.pid.reset()  # no launch integral may accumulate in the test
    return float(accel)
