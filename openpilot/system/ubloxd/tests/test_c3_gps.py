"""Check the retained u-blox path without a connected receiver."""
import struct
from unittest.mock import Mock, patch

import pytest

from openpilot.common.gps import get_gps_location_service
from openpilot.system.manager import process_config
from openpilot.system.ubloxd import pigeond
from openpilot.system.ubloxd.ubloxd import UbloxMsgParser, UbxFramer


@pytest.mark.parametrize('tty,quectel,use_ublox', [(True, False, True), (True, True, False), (False, False, False)])
def test_gps_receiver_selection(tty, quectel, use_ublox):
  paths = {'/dev/ttyHS0': tty, '/persist/comma/use-quectel-gps': quectel}
  params = Mock()
  params.get_bool.return_value = not use_ublox
  with patch.object(process_config.os.path, 'exists', side_effect=paths.__getitem__):
    assert process_config.ublox(True, params, None) is use_ublox
    assert process_config.qcomgps(True, params, None) is not use_ublox
  params.put_bool.assert_called_once_with('UbloxAvailable', use_ublox, block=True)
  params.get_bool.return_value = use_ublox
  assert get_gps_location_service(params) == ('gpsLocationExternal' if use_ublox else 'gpsLocation')


@pytest.mark.parametrize('enabled', [True, False])
def test_ublox_power_pins(enabled):
  with patch.object(pigeond, 'gpio_init') as initialize, patch.object(pigeond, 'gpio_set') as write:
    pigeond.set_power(enabled)
  assert [c.args for c in initialize.call_args_list] == [(33, True), (34, True), (32, True)]
  assert [c.args for c in write.call_args_list] == [(33, True), (34, enabled), (32, enabled)]


def test_nav_pvt_uart_frame_to_gps_location():
  # UBX-NAV-PVT, the 92-byte M8 message configured by pigeond.
  payload = bytearray(92)
  struct.pack_into('<H6B', payload, 4, 2026, 10, 20, 12, 30, 0, 7)
  struct.pack_into('<4B', payload, 20, 3, 1, 0, 12)  # 3D fix, gnssFixOK, 12 satellites
  struct.pack_into('<4i2I', payload, 24, -1221234567, 371234567, 25000, 20000, 1000, 1500)
  struct.pack_into('<5i2I', payload, 48, 1000, 2000, -500, 2236, 6343500, 100, 100000)
  frame = pigeond.add_ubx_checksum(b'\xb5\x62\x01\x07\x5c\x00' + payload)
  framer = UbxFramer()
  assert framer.add_data(1., frame[:17]) == []
  frames = framer.add_data(1.1, frame[17:])
  assert len(frames) == 1
  service, message = UbloxMsgParser().parse_frame(frames[0])
  assert service == 'gpsLocationExternal'
  gps = message.gpsLocationExternal
  assert message.valid and gps.hasFix and gps.source == 'ublox'
  assert gps.latitude == pytest.approx(37.1234567)
  assert gps.longitude == pytest.approx(-122.1234567)
  assert gps.altitude == pytest.approx(25)
  assert gps.vNED == pytest.approx([1, 2, -.5])
  assert gps.satelliteCount == 12
  assert gps.horizontalAccuracy == pytest.approx(1.)
  assert framer.add_data(2., frame[:-1] + bytes([frame[-1] ^ 1])) == []
