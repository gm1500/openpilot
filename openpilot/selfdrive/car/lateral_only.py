"""Steering availability specific to lateral-only assistance."""
import math
import struct

from openpilot.common.constants import CV
from opendbc.car.gm.values import CAR, GMSafetyFlags
from opendbc.car.structs import car


# Match CarState.vEgo's Float32 representation at exactly 10 km/h.
SILVERADO_AOL_MIN_STEER_SPEED = struct.unpack('<f', struct.pack('<f', 10 * CV.KPH_TO_MS))[0]


def lateral_only_speed_available(CP, CS):
  if not math.isfinite(CS.vEgo):
    return False

  gm_aol_flags = GMSafetyFlags.HW_CAM | GMSafetyFlags.HW_CAM_LONG | GMSafetyFlags.SILVERADO_TORQUE | GMSafetyFlags.LATERAL_ONLY
  if (CP.brand == 'gm' and CP.carFingerprint == CAR.CHEVROLET_SILVERADO and
      CP.openpilotLongitudinalControl and not CP.pcmCruise and
      any(config.safetyModel == car.CarParams.SafetyModel.gm and config.safetyParam & gm_aol_flags == gm_aol_flags
          for config in CP.safetyConfigs)):
    # This port keeps minSteerSpeed=-1 to suppress the stock low-speed warning.
    # AOL still observes the camera platform's original 10 km/h steering floor.
    return not CS.standstill and CS.vEgo >= SILVERADO_AOL_MIN_STEER_SPEED

  return CP.steerAtStandstill or (not CS.standstill and abs(CS.vEgo) > max(CP.minSteerSpeed, .3))
