import fcntl
import os
import select
import threading
import time

import pytest

from openpilot.system.sentry import network_time


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
def test_offset_validated_without_double_adding_dst(reply, expected):
  assert network_time.parse_network_timezone(reply) == expected


@pytest.mark.parametrize('reply,expected', [
  (b'\r\n+QLTS: "2026/10/11,03:00:00-24,1"\r\n\r\nOK\r\n', -21600),
  (b'\r\n+QLTS: ""\r\nOK\r\n', None),
  (b'ERROR\r\n', None),
  (b'', None),
  (b'x'*8192, None),
])
def test_query_is_bounded_and_does_not_change_modem_settings(tmp_path, monkeypatch, reply, expected):
  master, slave = os.openpty()
  monkeypatch.setattr(network_time, 'AT_PORT', os.ttyname(slave))
  monkeypatch.setattr(network_time, 'AT_LOCK', str(tmp_path/'modem.lock'))
  monkeypatch.setattr(network_time, 'QUERY_TIMEOUT', .1)
  command = []

  def modem():
    if select.select([master], [], [], 1)[0]:
      command.append(os.read(master, 1024))
      if reply:
        os.write(master, reply)

  worker = threading.Thread(target=modem)
  worker.start()
  try:
    started = time.monotonic()
    assert network_time.read_network_timezone() == expected
    assert time.monotonic()-started < .5
    worker.join(timeout=1)
    assert not worker.is_alive()
    assert command == [b'AT+QLTS=2\r']
    # The OS owner can acquire its lock immediately after success or failure.
    with open(network_time.AT_LOCK, 'r+') as lock:
      fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
  finally:
    os.close(master)
    os.close(slave)


def test_busy_os_modem_lock_never_opens_serial_port(tmp_path, monkeypatch):
  monkeypatch.setattr(network_time, 'AT_LOCK', str(tmp_path/'modem.lock'))
  monkeypatch.setattr(network_time, 'AT_PORT', str(tmp_path/'must-not-open'))
  real_open = os.open
  opened = []

  def checked_open(path, *args):
    opened.append(path)
    return real_open(path, *args)

  with open(network_time.AT_LOCK, 'w') as owner:
    fcntl.flock(owner.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    monkeypatch.setattr(network_time.os, 'open', checked_open)
    assert network_time.read_network_timezone() is None
    assert opened == [network_time.AT_LOCK]
