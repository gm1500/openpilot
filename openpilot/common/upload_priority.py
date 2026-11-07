"""Yield file-upload bandwidth to active map requests across processes."""
import fcntl
import os
import struct
import time
from collections.abc import Callable
from contextlib import contextmanager

from openpilot.common.hardware.hw import Paths

# Longer than the map response budget plus a final blocked socket read. The
# kernel releases the lock immediately if mapd exits; expiry also covers a hang.
MAP_PRIORITY_TIMEOUT = 120.
POLL_INTERVAL = .2
UPLOAD_CHUNK_SIZE = 16 * 1024


class UploadDeferred(Exception):
  """Keep an upload pending without charging a failed network attempt."""


def _priority_path() -> str:
  return os.path.join(Paths.shm_path(), "map_download_priority" + os.environ.get("OPENPILOT_PREFIX", ""))


@contextmanager
def map_download_priority():
  # RAM only on the device; no Params keys, persistent writes or service needed.
  # This is held only for a fetch, never for cache lookup or retry backoff.
  lease = None
  try:
    lease = open(_priority_path(), 'a+b')
    fcntl.flock(lease, fcntl.LOCK_EX)
    lease.truncate(0)
    lease.write(struct.pack('d', time.monotonic() + MAP_PRIORITY_TIMEOUT))
    lease.flush()
  except OSError:
    # Coordination trouble must not prevent map acquisition.
    if lease is not None:
      lease.close()
    lease = None
  try:
    yield
  finally:
    if lease is not None:
      lease.close()


def map_download_active() -> bool:
  try:
    with open(_priority_path(), 'rb') as lease:
      try:
        fcntl.flock(lease, fcntl.LOCK_SH | fcntl.LOCK_NB)
      except BlockingIOError:
        deadline = lease.read(8)
        if len(deadline) == 8:
          remaining = struct.unpack('d', deadline)[0] - time.monotonic()
          return 0 < remaining <= MAP_PRIORITY_TIMEOUT
  except OSError:
    pass
  return False


def wait_for_map_download(check: Callable[[], None] | None = None) -> None:
  while True:
    # Athena's shutdown/network checks must still run while the body is paused.
    if check is not None:
      check()
    if not map_download_active():
      return
    time.sleep(POLL_INTERVAL)


class MapAwareReader:
  """Pause between small upload chunks, preserving file length and position."""
  def __init__(self, stream, check: Callable[[], None] | None = None):
    self.stream = stream
    self.check = check

  def __getattr__(self, attr):
    return getattr(self.stream, attr)

  def read(self, size: int = -1):
    wait_for_map_download(self.check)
    return self.stream.read(UPLOAD_CHUNK_SIZE if size < 0 else min(size, UPLOAD_CHUNK_SIZE))
