"""Sentry storage cleanup and live recorder lock detection."""
import os
from pathlib import Path
import time

from openpilot.common.hardware.hw import Paths
from openpilot.common.swaglog import cloudlog


def is_sentry_file(filename: str) -> bool:
  return os.path.isfile(os.path.join(os.path.dirname(filename), '.sentry'))


def remove_legacy_upload_authorization():
  # Automatic full-file uploads and their account setup have been removed.
  # Delete only their obsolete credential file, without reading it.
  path = Path(Paths.persist_root())/'comma'/'sentry_connect.json'
  try:
    path.unlink(missing_ok=True)
  except OSError:
    cloudlog.warning('Could not remove obsolete Sentry upload authorization file: %s', path)


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
