"""Host-side checks of the original comma 3's peripheral boundaries."""
import importlib
import json
from collections import defaultdict
from types import SimpleNamespace
from unittest.mock import Mock, call, patch

import pytest

from openpilot.cereal import messaging
from openpilot.common.hardware.comma import amplifier, hardware, modem


@pytest.mark.parametrize('device,volume,mixer,eq_enabled', [('tici', 0x1c, 0x02, True), ('tizi', 0x17, 0x21, False)])
def test_amplifier_programs_device_routing(device, volume, mixer, eq_enabled):
  registers = defaultdict(int)
  bus = Mock()
  bus.read_byte_data.side_effect = lambda address, register, force: registers[register]
  bus.write_byte_data.side_effect = lambda address, register, value, force: registers.__setitem__(register, value)
  with patch.object(amplifier, 'SMBus') as smbus:
    smbus.return_value.__enter__.return_value = bus
    assert amplifier.Amplifier().initialize_configuration(device)
  assert registers[0x3e] & 0x1f == volume
  assert registers[0x22] & 0x33 == mixer
  assert bool(registers[0x49] & 2) == eq_enabled
  assert registers[0x51] & 0x80  # powered after programming
  if device == 'tici':
    assert bytes(registers[i] for i in range(0x84, 0x8e)) == bytes.fromhex('274fc0ff3bf90b3c1656')


@pytest.mark.parametrize('device,commands', [
  ('tici', ['AT+QSIMDET=0', 'AT+QSIMSTAT=0']),
  ('tizi', ['AT+QSIMDET=1,0', 'AT+QSIMSTAT=1']),
])
def test_modem_sim_detection_matches_board_wiring(device, commands):
  instance = object.__new__(modem.Modem)
  instance._at = Mock()
  with patch.object(modem.HARDWARE, 'get_device_type', return_value=device):
    instance._configure_modem('EG25GGBR07A08M2G')
  sim_commands = [c.args[0] for c in instance._at.call_args_list if c.args[0].startswith('AT+QSIM')]
  assert sim_commands == commands
  assert call('AT+QNVW=5280,0,"0102000000000000"') in instance._at.call_args_list
  instance._at.reset_mock()
  instance._configure_modem('EG916')
  instance._at.assert_not_called()


def test_tici_usb_irq_affinity():
  instance = hardware.HardwareComma()
  instance.amplifier = Mock()
  with patch.object(instance, 'get_device_type', return_value='tici'), patch.object(hardware, 'affine_irq') as irq, \
       patch.object(hardware, 'sudo_write'), patch.object(hardware, 'gpio_init'), patch.object(hardware, 'gpio_set'), \
       patch.object(hardware, 'subprocess'):
    instance.initialize_hardware()
  irq.assert_has_calls([call(3, 'xhci-hcd:usb3'), call(3, 'xhci-hcd:usb1')], any_order=True)
  instance.amplifier.initialize_configuration.assert_called_once_with('tici')


def test_networkmanager_wifi_properties():
  # Exercise real Jeepney message construction against the AGNOS 12.8 D-Bus API.
  conn = Mock()
  conn.send_and_get_reply.side_effect = [
    SimpleNamespace(body=('/device/wlan0',)),
    SimpleNamespace(body=(('o', '/ap/1'),)),
    SimpleNamespace(body=(('y', 70),)),
  ]
  with patch('jeepney.io.blocking.open_dbus_connection') as dbus:
    dbus.return_value.__enter__.return_value = conn
    assert hardware.nm_wifi_property('AccessPoint', 'Strength') == 70
  messages = [c.args[0] for c in conn.send_and_get_reply.call_args_list]
  assert [m.body for m in messages] == [
    ('wlan0',), ('org.freedesktop.NetworkManager.Device.Wireless', 'ActiveAccessPoint'),
    ('org.freedesktop.NetworkManager.AccessPoint', 'Strength'),
  ]


@pytest.mark.parametrize('metered,expected', [(1, True), (2, False), (3, True), (4, False)])
def test_tici_wifi_metering_and_strength(metered, expected):
  instance = hardware.HardwareComma()
  with patch.object(instance, 'get_device_type', return_value='tici'), \
       patch.object(hardware, 'wpa_supplicant_cmd') as wpa, \
       patch.object(hardware, 'nm_wifi_property', side_effect=[metered, 70]):
    assert instance.get_network_metered(hardware.NetworkType.wifi) is expected
    assert instance.get_network_strength(hardware.NetworkType.wifi) == hardware.NetworkStrength.good
  wpa.assert_not_called()


def test_nvme_temperatures_roundtrip_and_missing_drive():
  instance = hardware.HardwareComma()
  with patch.object(instance, 'get_device_type', return_value='tici'), patch.object(hardware.Path, 'exists', return_value=True), \
       patch.object(hardware.subprocess, 'check_output', return_value=json.dumps({'nvme_smart_health_information_log': {'temperature_sensors': [41, 43]}})):
    msg = messaging.new_message('deviceState', valid=True)
    msg.deviceState.nvmeTempC = instance.get_nvme_temperatures()
    decoded = messaging.log_from_bytes(msg.to_bytes())
    assert list(decoded.deviceState.nvmeTempC) == [41, 43]
  with patch.object(instance, 'get_device_type', return_value='tici'), patch.object(hardware.Path, 'exists', return_value=False), \
       patch.object(hardware.subprocess, 'check_output') as read:
    assert instance.get_nvme_temperatures() == []
    read.assert_not_called()


@pytest.mark.parametrize('mounted,drive,emmc,alert', [
  (True, True, False, False), (False, True, False, True), (False, False, False, True),
  (False, False, True, False), (False, True, True, True),
])
def test_nvme_mount_alert(mounted, drive, emmc, alert):
  from openpilot.system.hardware import hardwared
  with patch.object(hardwared.HARDWARE, 'get_device_type', return_value='tici'), \
       patch.object(hardwared.Path, 'is_mount', return_value=mounted), patch.object(hardwared.Path, 'exists', return_value=drive), \
       patch.object(hardwared.Path, 'is_file', return_value=emmc), patch.object(hardwared, 'set_offroad_alert_if_changed') as show:
    hardwared.check_storage_mount()
  show.assert_called_once_with('Offroad_StorageMissing', alert)


@pytest.mark.parametrize('device,offset', [('tici', 0), ('tizi', 5), ('mici', 0)])
def test_fan_target_matches_os(device, offset):
  from openpilot.system.hardware import fan_controller
  try:
    with patch.object(fan_controller.HARDWARE, 'get_device_type', return_value=device):
      importlib.reload(fan_controller)
    assert fan_controller.OFFSET == offset
    controller = fan_controller.FanController(2)
    for _ in range(1000):
      duty = controller.update(100, True)
    assert duty == 100
  finally:
    importlib.reload(fan_controller)


@pytest.mark.parametrize('device,wait', [('tici', True), ('tizi', False)])
def test_panda_usb_enumeration_wait(device, wait):
  from openpilot.selfdrive.pandad import pandad
  events = []
  with patch.object(pandad.signal, 'signal'), patch.object(pandad.Panda, 'list', return_value=[]), \
       patch.object(pandad.HARDWARE, 'get_device_type', return_value=device), \
       patch.object(pandad.HARDWARE, 'reset_internal_panda', side_effect=lambda: events.append('reset')), \
       patch.object(pandad.time, 'sleep', side_effect=lambda t: events.append(('wait', t))), \
       patch.object(pandad.PandaDFU, 'list', side_effect=KeyboardInterrupt):
    with pytest.raises(KeyboardInterrupt):
      pandad.main()
  assert events == (['reset', ('wait', 3)] if wait else ['reset'])


def test_modem_corrects_detection_before_requiring_a_sim():
  instance = object.__new__(modem.Modem)
  instance._ppp = Mock()
  instance._init_at_channel = Mock(return_value=True)
  instance._read_identity = Mock(return_value={'modem_version': 'EG25GGBR07A08M2G', 'imei': 'test', 'iccid': ''})
  instance._configure_modem = Mock()
  with patch.object(modem.os.path, 'exists', return_value=True):
    assert instance._do_initializing() == modem.State.INITIALIZING
  instance._configure_modem.assert_called_once_with('EG25GGBR07A08M2G')
