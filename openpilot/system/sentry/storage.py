"""Identify a live recorder lock when the ordinary uploader restarts."""
import os
from pathlib import Path
import time


def live_sentry_lock(directory):
  path = Path(directory)
  if not (path/'.sentry').exists():
    return False
  lock = path/'sentry.lock'
  try:
    pid = int(lock.read_text().strip())
    if pid <= 0:
      return False
    os.kill(pid, 0)
    return True
  except ProcessLookupError:
    return False
  except PermissionError:
    return True
  except ValueError:
    # The file may have just been created and not populated yet.
    try:
      return 0 <= time.time()-lock.stat().st_mtime < 10  # noqa: TID251
    except OSError:
      return False
  except OSError:
    return False
