"""Manual Resume/Set rolling diagnostic with a latched return to brake hold."""
import math

from opendbc.car.gm.resume_set_test import supported, MAX_DURATION, MAX_SPEED, MAX_DISTANCE, TEST_ACCEL, REHOLD_ACCEL, HEARTBEAT_MAX_AGE

INPUT_MAX_AGE = 0.30
COOLDOWN = 1.0
SERVICES = ('carState', 'carOutput', 'selfdriveState', 'longitudinalPlan', 'modelV2', 'radarState', 'driverMonitoringState', 'pandaStates')


def fresh(sm, service, now, max_age=INPUT_MAX_AGE):
  return (sm.valid[service] and sm.seen[service] and
          0 <= now - sm.logMonoTime[service] * 1e-9 <= max_age and
          0 <= now - sm.recv_time[service] <= max_age)


class ResumeSetTest:
  def __init__(self, CP):
    self.supported = supported(CP)
    self.ready = self.active = self.holding = self.recovered = self.brake_held = False
    self.reason = 'idle'
    self.last_id = self.request_id = self.button = 0
    self.saw_release = False  # a held request at startup can never arm the test
    self.start = self.last_update = self.cooldown_until = 0.
    self.remaining = self.distance = 0.
    self.blocked_count_start = 0

  def update(self, sm, now, long_active, stopping):
    cs, plan, output = sm['carState'], sm['longitudinalPlan'], sm['carOutput'].actuatorsOutput
    normal = (plan.aTarget, plan.shouldStop)
    req = sm['resumeSetTestRequest']
    req_fresh = fresh(sm, 'resumeSetTestRequest', now, HEARTBEAT_MAX_AGE)
    new_press = req_fresh and req.held and req.requestId > self.last_id
    released_before_press = self.saw_release
    if req_fresh and not req.held:
      self.saw_release = True
    if new_press:
      self.last_id = req.requestId
      self.saw_release = False  # consume invalid presses as well
    held = (req_fresh and req.held and req.requestId == self.request_id and req.button == self.button)
    self.ready = self.brake_held = False
    self.remaining = 0.

    if not self.supported or not long_active or cs.gasPressed or cs.brakePressed:
      self.active = self.holding = False
      self.reason = 'driver' if self.supported else 'unsupported'
      return normal

    # If controlsd restarts while card still owns a test hold, adopt that hold;
    # never resume a positive request without a release and a fresh new press.
    if fresh(sm, 'carOutput', now) and output.gmResumeSetHolding and not self.holding:
      self.holding = True
      self.reason = 'controllerHold'

    valid = all(fresh(sm, name, now) for name in SERVICES)
    values = (cs.vEgo, cs.vEgoRaw, cs.aEgo, cs.vCruise, plan.aTarget, output.brake)
    finite = all(math.isfinite(v) for v in values)
    panda_ok = (len(sm['pandaStates']) == 1 and str(sm['pandaStates'][0].safetyModel) == 'gm' and
                sm['pandaStates'][0].safetyParam == 11 and sm['pandaStates'][0].controlsAllowed and
                not sm['pandaStates'][0].safetyRxChecksInvalid and not sm['pandaStates'][0].faults)
    blocked_count = sum(p.safetyTxBlocked for p in sm['pandaStates'])
    hazard = (not valid or not finite or not panda_ok or not cs.canValid or cs.canTimeout or cs.accFaulted or
              cs.carNotReady or cs.carFaultedNonCritical or cs.doorOpen or cs.seatbeltUnlatched or cs.parkingBrake or
              str(cs.gearShifter) != 'drive' or cs.stockAeb or cs.stockFcw or plan.fcw or cs.gmCruiseState not in (1, 2, 4) or
              not sm['selfdriveState'].active or str(sm['selfdriveState'].state) != 'enabled' or
              sm['driverMonitoringState'].noResponseForceDecel or not 0 < cs.vCruise < 255)
    for lead in (sm['radarState'].leadOne, sm['radarState'].leadTwo):
      if lead.present:
        hazard |= (not math.isfinite(lead.dRel) or not math.isfinite(lead.vRel) or
                   lead.dRel < 6. + 2. * max(0., -lead.vRel))

    if self.active:
      dt = now - self.last_update
      if finite and 0 <= dt <= HEARTBEAT_MAX_AGE:
        self.distance += max(abs(cs.vEgo), abs(cs.vEgoRaw)) * dt
      self.recovered |= cs.gmCruiseState == 1
      controller_abort = (now - self.start > 0.15 and output.gmResumeSetHolding and not output.gmResumeSetActive)
      reason = ('blocked' if hazard else 'released' if not held else
                'controllerAbort' if controller_abort else 'safetyBlocked' if blocked_count > self.blocked_count_start else
                'controlStale' if not 0 <= dt <= HEARTBEAT_MAX_AGE else
                'reverse' if min(cs.vEgo, cs.vEgoRaw) < -0.05 else
                'speedLimit' if max(abs(cs.vEgo), abs(cs.vEgoRaw)) >= MAX_SPEED else
                'distanceLimit' if self.distance >= MAX_DISTANCE else
                'timeout' if not 0 <= now - self.start < MAX_DURATION else None)
      if reason:
        self.active = False
        self.cooldown_until = now + COOLDOWN
        self.reason = reason

    self.brake_held = (valid and finite and cs.standstill and max(abs(cs.vEgo), abs(cs.vEgoRaw)) < .05 and
                       stopping and output.brake > 0)
    self.ready = (not self.active and not hazard and self.brake_held and cs.gmCruiseState == 4 and now >= self.cooldown_until)
    if (new_press and released_before_press and self.ready and req.button in (2, 3) and
        0 <= now - req.requestId * 1e-9 <= HEARTBEAT_MAX_AGE):
      self.active = self.holding = True
      self.recovered = False
      self.request_id, self.button = req.requestId, req.button
      self.start, self.distance = now, 0.
      self.blocked_count_start = blocked_count
      self.ready = False
      self.reason = 'rolling'

    self.last_update = now
    if self.active:
      self.remaining = max(0., MAX_DURATION - (now - self.start))
      self.reason = 'cruiseActive' if self.recovered else 'rolling'
      return TEST_ACCEL, False
    if self.holding:
      return 0., True
    self.reason = ('ready' if self.ready else 'pandaFault' if any(p.faults for p in sm['pandaStates']) else
                   'pandaNotReady' if not panda_ok else 'inputStale' if not valid else 'blocked')
    return normal

  def apply(self, controller, active, cs, target, stop, limits):
    if self.holding and not self.active:
      controller.last_output_accel = (controller.CP.stopAccel if self.brake_held else REHOLD_ACCEL)
    if self.active:
      controller.pid.reset()
    accel = controller.update(active, cs, target, stop, limits)
    if self.active:
      # Feedforward only for the diagnostic: avoid learning launch torque while
      # the factory standstill state is still refusing propulsion.
      controller.pid.reset()
      accel = min(accel, TEST_ACCEL)
      controller.last_output_accel = accel
    return float(accel)
