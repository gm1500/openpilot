"""Read the EG25 network timezone using AGNOS's shared AT-port lock."""
from datetime import datetime
import fcntl
import os
import re
import select
import termios
import time

AT_PORT = '/dev/modem_at0'
AT_LOCK = '/dev/shm/modem.lock'
QUERY_TIMEOUT = 1.


def parse_network_timezone(value):
  """QLTS=2 reports local network time; its signed quarter-hours include DST."""
  if not value:
    return None
  match = re.fullmatch(r'(\d{2}|\d{4})/(\d{2})/(\d{2}),(\d{2}):(\d{2}):(\d{2})([+-]\d{2}),\s*([012])',
                       value.replace('"', '').strip())
  if match is None:
    return None
  year, month, day, hour, minute, second, quarters, _ = map(int, match.groups())
  if year < 100:
    year += 2000
  try:
    datetime(year, month, day, hour, minute, second)
  except ValueError:
    return None
  if not 2020 <= year < 2100 or not -48 <= quarters <= 56:
    return None
  return quarters*15*60  # Do not add the DST indicator a second time.


def read_network_timezone():
  # AGNOS 20.1 owns the modem daemon. Cooperate with its lock, also used by
  # qcomgpsd and LPA. Never wait for a busy port or alter modem configuration.
  try:
    with os.fdopen(os.open(AT_LOCK, os.O_CREAT | os.O_RDWR, 0o666), 'r+') as lock:
      fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
      fd = os.open(AT_PORT, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
      try:
        attrs = termios.tcgetattr(fd)
        attrs[0] = attrs[1] = attrs[3] = 0
        attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
        attrs[4] = attrs[5] = termios.B9600
        attrs[6][termios.VMIN] = attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIFLUSH)
        command = b'AT+QLTS=2\r'
        if os.write(fd, command) != len(command):
          return None
        deadline, received, offset = time.monotonic()+QUERY_TIMEOUT, b'', None
        while len(received) < 4096 and (remaining := deadline-time.monotonic()) > 0:
          if not select.select([fd], [], [], remaining)[0]:
            break
          chunk = os.read(fd, 4096-len(received))
          if not chunk:
            break
          received += chunk
          for raw in received.split(b'\n')[:-1]:
            line = raw.decode(errors='replace').strip()
            if line == 'OK':
              return offset
            if line == 'ERROR' or line.startswith('+CME ERROR'):
              return None
            if line.startswith('+QLTS:'):
              offset = parse_network_timezone(line.split(':', 1)[1])
      finally:
        os.close(fd)
  except (OSError, termios.error):
    pass
  return None
