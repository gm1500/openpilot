import pytest

from openpilot.common.constants import CV
from openpilot.selfdrive.car.lateral_only import lateral_only_speed_available
from opendbc.car.gm.values import CAR, GMSafetyFlags
from opendbc.car.structs import car


def silverado_params():
  return car.CarParams.new_message(
    brand='gm', carFingerprint=CAR.CHEVROLET_SILVERADO, openpilotLongitudinalControl=True, minSteerSpeed=-1,
    safetyConfigs=[{'safetyModel': 'gm', 'safetyParam': int(GMSafetyFlags.HW_CAM | GMSafetyFlags.HW_CAM_LONG |
                                                        GMSafetyFlags.SILVERADO_TORQUE | GMSafetyFlags.LATERAL_ONLY)}])


@pytest.mark.parametrize('speed,standstill,available', [
  (0., True, False), (9.99, False, False), (9.99999, False, False), (10., False, True), (10.01, False, True),
  (20., True, False), (-20., False, False), (float('nan'), False, False), (float('inf'), False, False),
])
def test_silverado_aol_observes_physical_floor_without_changing_warning_parameter(speed, standstill, available):
  CP = silverado_params()
  CS = car.CarState.new_message(vEgo=speed * CV.KPH_TO_MS, standstill=standstill)
  assert lateral_only_speed_available(CP, CS) is available
  assert CP.minSteerSpeed == -1


@pytest.mark.parametrize('change', ['brand', 'fingerprint', 'longitudinal', 'pcm', 'safety_mode',
                                   'camera', 'camera_long', 'silverado_torque', 'lateral_only'])
def test_physical_floor_is_limited_to_supported_silverado_aol_configuration(change):
  CP = silverado_params()
  if change == 'brand':
    CP.brand = 'other'
  elif change == 'fingerprint':
    CP.carFingerprint = CAR.CHEVROLET_BOLT_EUV
  elif change == 'longitudinal':
    CP.openpilotLongitudinalControl = False
  elif change == 'pcm':
    CP.pcmCruise = True
  elif change == 'safety_mode':
    CP.safetyConfigs[0].safetyModel = 'noOutput'
  else:
    flag = {'camera': GMSafetyFlags.HW_CAM, 'camera_long': GMSafetyFlags.HW_CAM_LONG,
            'silverado_torque': GMSafetyFlags.SILVERADO_TORQUE, 'lateral_only': GMSafetyFlags.LATERAL_ONLY}[change]
    CP.safetyConfigs[0].safetyParam &= ~int(flag)
  assert lateral_only_speed_available(CP, car.CarState.new_message(vEgo=5 * CV.KPH_TO_MS))
