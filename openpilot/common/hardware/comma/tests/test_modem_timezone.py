from unittest.mock import Mock

import pytest

from openpilot.common.hardware.comma import modem


@pytest.mark.parametrize('reply,expected', [
  ('"2026/10/11,03:00:00-24,1"', -21600),
  ('"26/01/11,03:00:00-28",0', -25200),
  ('"2026/10/11,12:00:00+21,0"', 18900),
  ('"2026/10/11,12:00:00+00,0"', 0),
  (None, None), ('""', None), ('ERROR', None),
  ('"1980/01/06,00:00:00+00,0"', None),
  ('"2026/02/31,00:00:00+00,0"', None),
  ('"2026/10/11,25:00:00+00,0"', None),
  ('"2026/10/11,00:00:00+57,0"', None),
  ('"2026/10/11,00:00:00-49,0"', None),
])
def test_network_time_offset_is_validated_without_double_adding_dst(reply, expected):
  assert modem.parse_network_timezone(reply) == expected


def test_network_time_query_uses_existing_owner_and_backs_off(monkeypatch):
  clock = [100.]
  monkeypatch.setattr(modem.time, 'monotonic', lambda: clock[0])
  device = object.__new__(modem.Modem)
  device.S = {'modem_version': 'EG25GGBR07A08M2G'}
  device._next_time_poll = 0.
  device._atv = Mock(return_value='"2026/10/11,03:00:00-24,1"')
  assert device._poll_network_time() == {'network_timezone': -21600, 'network_timezone_mono': 100.}
  device._atv.assert_called_once_with('AT+QLTS=2', '+QLTS:')
  clock[0] = 159.
  assert device._poll_network_time() == {}
  assert device._atv.call_count == 1
  clock[0] = 160.
  device._atv.return_value = '""'
  assert device._poll_network_time() == {}
  clock[0] = 220.
  device._atv.side_effect = OSError('AT port unavailable')
  assert device._poll_network_time() == {}
  clock[0] = 280.
  device.S['modem_version'] = 'other hardware'
  assert device._poll_network_time() == {}
  assert device._atv.call_count == 3
