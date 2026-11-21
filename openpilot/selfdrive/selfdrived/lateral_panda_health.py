"""AOL health gating with a narrowly qualified historical GM diagnostic fault.

On the verified tres/Silverado camera-long configuration, driving uses CAN0/2.
FDCAN2 is CAN1 regardless of harness orientation. Startup diagnostics can exceed
its IRQ limit; Panda retains that fault bit after diagnostics stop. The fault
is not evidence of a current IRQ storm, but an ongoing storm must still block
steering. Never extend this exception to another fault, board or safety setup.

Only fresh, distinct Panda samples can establish two seconds of quiet CAN1 and
clean driving buses. Any changed counter or unhealthy sample discards that
qualification. SelfdriveD's existing armed-AOL fault latch separately requires
explicit driver rearming after readiness is lost; this helper cannot rearm it.
"""
import math

from opendbc.car.gm.values import CAR, GMSafetyFlags
from opendbc.car.structs import car
from openpilot.cereal import log
from openpilot.selfdrive.selfdrived.always_on_lateral import lateral_safety_supported

QUALIFY_SECONDS = 2.
MAX_SAMPLE_GAP = .3
CAN_INTERRUPT_RATE = 16000  # matching Panda's stm32h7_config.h
GM_AOL_PARAM = int(GMSafetyFlags.HW_CAM | GMSafetyFlags.HW_CAM_LONG |
                   GMSafetyFlags.SILVERADO_TORQUE | GMSafetyFlags.LATERAL_ONLY)
IRQ_FIELDS = ('irq0CallRate', 'irq1CallRate', 'irq2CallRate')
ERROR_COUNTERS = ('receiveErrorCnt', 'transmitErrorCnt', 'totalErrorCnt', 'totalTxLostCnt',
                  'totalRxLostCnt', 'canCoreResetCnt', 'busOffCnt')
QUIET_FIELDS = ERROR_COUNTERS + ('totalTxCnt', 'totalRxCnt', 'totalFwdCnt', 'busOff', 'errorWarning',
                               'errorPassive', 'lastError', 'lastStoredError', 'lastDataError',
                               'lastDataStoredError', 'canSpeed', 'canDataSpeed', 'canfdEnabled',
                               'brsEnabled', 'canfdNonIso')
NO_ERROR = (log.PandaState.PandaCanState.LecErrorCode.noError,
            log.PandaState.PandaCanState.LecErrorCode.noChange)


def _integrity_clear(ps):
  return not (ps.safetyRxChecksInvalid or ps.heartbeatLost or ps.nmiReset or ps.hardfaultReset or
              ps.faultStatus == log.PandaState.FaultStatus.faultPerm)


def _driving_bus_clean(bus):
  return (not (bus.busOff or bus.errorWarning or bus.errorPassive) and bus.totalRxCnt > 0 and
          all(getattr(bus, field) == 0 for field in ERROR_COUNTERS) and
          all(getattr(bus, field) < CAN_INTERRUPT_RATE for field in IRQ_FIELDS) and
          all(getattr(bus, field) in NO_ERROR for field in
              ('lastError', 'lastStoredError', 'lastDataError', 'lastDataStoredError')))


class LateralPandaHealth:
  def __init__(self):
    self._reset()

  def _reset(self):
    self._configuration = None
    self._quiet_since = None
    self._last_sample = None
    self._last_uptime = None
    self._quiet_snapshot = None
    self._qualified = False

  def check(self, CP, pandas, sample_time: float, fresh: bool) -> bool:
    if not fresh or not math.isfinite(sample_time) or sample_time <= 0 or not pandas:
      self._reset()
      return False
    if any(not _integrity_clear(ps) for ps in pandas):
      self._reset()
      return False
    if not any(ps.faults for ps in pandas):
      self._reset()
      return True

    # Only the actual supported single-Panda GM camera-long configuration gets
    # the exception. tres is the H7 hardware observed in the diagnostic replay.
    if (len(pandas) != 1 or len(CP.safetyConfigs) != 1 or
        CP.carFingerprint != CAR.CHEVROLET_SILVERADO or
        CP.networkLocation != car.CarParams.NetworkLocation.fwdCamera or
        not lateral_safety_supported(CP, pandas)):
      self._reset()
      return False
    ps = pandas[0]
    if (ps.pandaType != log.PandaState.PandaType.tres or ps.safetyParam != GM_AOL_PARAM or
        len(ps.faults) != 1 or ps.faults[0] != log.PandaState.FaultType.interruptRateCan2 or
        ps.faultStatus != log.PandaState.FaultStatus.faultTemp or
        ps.harnessStatus not in (log.PandaState.HarnessStatus.normal, log.PandaState.HarnessStatus.flipped) or
        ps.safetyRxInvalid or ps.rxBufferOverflow or ps.txBufferOverflow or ps.spiErrorCount):
      self._reset()
      return False

    diagnostic = ps.canState1
    if (diagnostic.busOff or diagnostic.totalRxCnt or diagnostic.totalFwdCnt or
        any(getattr(diagnostic, field) != 0 for field in IRQ_FIELDS) or
        not _driving_bus_clean(ps.canState0) or not _driving_bus_clean(ps.canState2)):
      self._reset()
      return False

    configuration = (CP.carFingerprint, CP.alternativeExperience, ps.safetyParam,
                     ps.alternativeExperience, ps.pandaType.raw, ps.harnessStatus.raw)
    snapshot = tuple(getattr(diagnostic, field) for field in QUIET_FIELDS)
    if (configuration != self._configuration or self._last_sample is None or
        sample_time < self._last_sample or sample_time - self._last_sample > MAX_SAMPLE_GAP or
        ps.uptime < self._last_uptime or snapshot != self._quiet_snapshot):
      self._reset()
      self._configuration = configuration
      self._quiet_since = sample_time
      self._quiet_snapshot = snapshot

    # A repeated message may keep an already-established result, but cannot
    # count as a new observation or move the beginning of the quiet window.
    if sample_time != self._last_sample:
      self._qualified = sample_time - self._quiet_since >= QUALIFY_SECONDS
    self._last_sample = sample_time
    self._last_uptime = ps.uptime
    return self._qualified
