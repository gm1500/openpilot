"""AOL health gating with a narrowly qualified historical GM diagnostic fault.

On the verified tres/Silverado camera-long configuration, driving uses CAN0/2.
FDCAN2 is CAN1 regardless of harness orientation. Startup diagnostics can exceed
its IRQ limit; Panda retains that fault bit after diagnostics stop. The fault
is not evidence of a current IRQ storm, but an ongoing storm must still block
steering. Never extend this exception to another fault, board or safety setup.

Only fresh, distinct Panda samples can establish two seconds of quiet CAN1 and
clean driving buses. Error-interrupt/RX-loss totals and stored error codes retain
history across drives; they do not describe current bus health. Beyond the
legacy single CAN0 error/loss pair, accept that history only when witnessed
unchanged before GM mode and in verified Park, with both driving buses receiving.
Keep the exact witnessed history, not a list of permitted counts or error codes.
Changed driving-bus history, stale evidence or a bus fault discards this
allowance permanently for the host instance. This establishes preexisting
history, not its original cause.
SelfdriveD's armed-AOL fault latch requires explicit driver rearming after
readiness is lost.
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


def _diagnostic_fault_only(ps):
  return (ps.pandaType == log.PandaState.PandaType.tres and
          len(ps.faults) == 1 and ps.faults[0] == log.PandaState.FaultType.interruptRateCan2 and
          ps.faultStatus == log.PandaState.FaultStatus.faultTemp and
          ps.harnessStatus in (log.PandaState.HarnessStatus.normal, log.PandaState.HarnessStatus.flipped) and
          not (ps.safetyRxInvalid or ps.rxBufferOverflow or ps.txBufferOverflow or ps.spiErrorCount))


def _driving_bus_history(bus):
  return (bus.totalErrorCnt, bus.totalRxLostCnt, bus.lastStoredError.raw, bus.lastDataStoredError.raw)


def _legacy_driving_history(history):
  can0, can2 = history
  return (can0[:2] in ((0, 0), (1, 1)) and can2[:2] == (0, 0) and
          all(error in NO_ERROR for bus in history for error in bus[2:]))


def _driving_bus_clean(bus, *, total_errors=0, rx_lost=0, history=None):
  history_clear = (_driving_bus_history(bus) == history if history is not None else
                   bus.totalErrorCnt == total_errors and bus.totalRxLostCnt == rx_lost and
                   bus.lastStoredError in NO_ERROR and bus.lastDataStoredError in NO_ERROR)
  errors_clear = (history_clear and
                  all(getattr(bus, field) == 0 for field in ERROR_COUNTERS if field not in ('totalErrorCnt', 'totalRxLostCnt')))
  return (not (bus.busOff or bus.errorWarning or bus.errorPassive) and bus.totalRxCnt > 0 and
          errors_clear and
          all(getattr(bus, field) < CAN_INTERRUPT_RATE for field in IRQ_FIELDS) and
          all(getattr(bus, field) in NO_ERROR for field in ('lastError', 'lastDataError')))


class LateralPandaHealth:
  def __init__(self):
    self._startup_seen = False
    self._startup_history = None
    # Once extended history is seen, losing its witness must not fall through
    # to the legacy allowance, even if counters later reset to zero.
    self._startup_history_required = False
    self._startup_harness = None
    self._startup_time = 0.
    self._startup_uptime = 0
    self._startup_rx = (0, 0)
    self._buses_progressing = False
    self._seen_gm = False
    self._parked_in_elm = False
    self._reset()

  def _observe_startup(self, pandas, sample_time, parked):
    if self._startup_seen and self._startup_history is None:
      return
    if len(pandas) != 1:
      self._startup_seen = True
      self._startup_history = None
      return
    ps = pandas[0]
    rx = (ps.canState0.totalRxCnt, ps.canState2.totalRxCnt)
    history = tuple(_driving_bus_history(bus) for bus in (ps.canState0, ps.canState2))
    elm = ps.safetyModel == car.CarParams.SafetyModel.elm327
    if not self._startup_seen:
      self._startup_seen = True
      self._startup_history_required = not _legacy_driving_history(history)
      if not elm or not self._startup_history_required:
        return
      self._startup_history = history
      self._startup_harness = ps.harnessStatus.raw
    elif (not 0 <= sample_time - self._startup_time <= MAX_SAMPLE_GAP or
          not 0 <= ps.uptime - self._startup_uptime <= 1 or
          rx[0] < self._startup_rx[0] or rx[1] < self._startup_rx[1]):
      self._startup_history = None
      return

    if (not _diagnostic_fault_only(ps) or ps.harnessStatus.raw != self._startup_harness or
        not (ps.ignitionCan or ps.ignitionLine) or not _integrity_clear(ps) or
        (elm and (self._seen_gm or ps.safetyParam != 1)) or
        (not elm and (ps.safetyModel != car.CarParams.SafetyModel.gm or ps.safetyParam != GM_AOL_PARAM)) or
        not _driving_bus_clean(ps.canState0, history=self._startup_history[0]) or
        not _driving_bus_clean(ps.canState2, history=self._startup_history[1])):
      self._startup_history = None
      return
    if sample_time != self._startup_time:
      self._buses_progressing = rx[0] > self._startup_rx[0] and rx[1] > self._startup_rx[1]
    self._startup_rx = rx
    self._startup_time = sample_time
    self._startup_uptime = ps.uptime
    self._seen_gm = not elm
    self._parked_in_elm |= elm and parked

  def _reset(self):
    self._configuration = None
    self._quiet_since = None
    self._last_sample = None
    self._last_uptime = None
    self._quiet_snapshot = None
    self._qualified = False

  def check(self, CP, pandas, sample_time: float, fresh: bool, *, parked: bool = False) -> bool:
    if not fresh or not math.isfinite(sample_time) or sample_time <= 0 or not pandas:
      self._startup_history = None
      self._reset()
      return False
    self._observe_startup(pandas, sample_time, parked)
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
    if ps.safetyParam != GM_AOL_PARAM or not _diagnostic_fault_only(ps):
      self._reset()
      return False

    diagnostic = ps.canState1
    history = self._startup_history if self._parked_in_elm else None
    if self._startup_history_required and history is None:
      self._reset()
      return False
    # Preserve the old narrowly defined CAN0 pair when there is no extended
    # startup history. A witnessed history is never recaptured while driving.
    can0_loss = int(ps.canState0.totalErrorCnt == ps.canState0.totalRxLostCnt == 1)
    if (diagnostic.busOff or diagnostic.totalRxCnt or diagnostic.totalFwdCnt or
        any(getattr(diagnostic, field) != 0 for field in IRQ_FIELDS) or
        not _driving_bus_clean(ps.canState0, total_errors=can0_loss, rx_lost=can0_loss,
                               history=history[0] if history is not None else None) or
        not _driving_bus_clean(ps.canState2, history=history[1] if history is not None else None)):
      self._reset()
      return False

    configuration = (CP.carFingerprint, CP.alternativeExperience, ps.safetyParam,
                     ps.alternativeExperience, ps.pandaType.raw, ps.harnessStatus.raw)
    # The CAN0 receive loss was already present before ELM diagnostics in the
    # recorded startup, but selfdrived starts later. Qualify unchanged counters
    # here instead of depending on an unobservable pre-diagnostic sample. A new
    # loss, including 0 -> 1, must immediately withdraw any prior qualification.
    snapshot = (tuple(getattr(diagnostic, field) for field in QUIET_FIELDS) +
                tuple(getattr(bus, field) for bus in (ps.canState0, ps.canState2) for field in ERROR_COUNTERS))
    stalled = history is not None and not self._buses_progressing
    if (stalled or configuration != self._configuration or self._last_sample is None or
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

