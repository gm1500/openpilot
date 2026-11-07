#!/usr/bin/env python3
"""Delete completed local drive/Sentry recordings, clear Connect requests, and reboot."""
import argparse
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import time


LOG_ROOT = Path('/data/media/0/realdata')
SEGMENT = re.compile(r'(?P<route>(?:[0-9a-f]{8}--[0-9a-f]{10}|\d{4}-\d{2}-\d{2}--\d{2}-\d{2}-\d{2}))--\d+')
ATHENA_MANAGER = 'openpilot.system.athena.manage_athenad'
ATHENA = 'openpilot.system.athena.athenad'


def completed_segments(root: Path, protected_routes: set[str]) -> list[Path]:
  """Only recognized segment directories; preserve every segment of a locked route."""
  routes: dict[str, list[Path]] = {}
  protected = set(protected_routes)
  for path in root.iterdir():
    match = SEGMENT.fullmatch(path.name)
    if match is None or path.is_symlink() or not path.is_dir():
      continue
    route = match['route']
    routes.setdefault(route, []).append(path)
    try:
      if any(child.name.endswith('.lock') for child in path.iterdir()):
        protected.add(route)
    except FileNotFoundError:
      continue  # The normal deleter may run concurrently.
  return sorted(path for route, paths in routes.items() if route not in protected for path in paths)


def delete_segments(root: Path, protected_routes: set[str]) -> int:
  count = 0
  for path in completed_segments(root, protected_routes):
    # Recheck immediately before deletion. Never follow a replaced symlink.
    if path.is_symlink():
      raise RuntimeError('A recording directory changed during cleanup; stopped.')
    try:
      if any(child.name.endswith('.lock') for child in path.iterdir()):
        continue
      shutil.rmtree(path)
      count += 1
    except FileNotFoundError:
      continue
  return count


def parked_routes(params) -> set[str]:
  from openpilot.cereal import messaging

  if not params.get_bool('IsOffroad'):
    raise RuntimeError('Park and wait for offroad mode before running this script.')
  for key in ('ForcePowerDown', 'DoShutdown', 'DoReboot', 'DoUninstall'):
    if params.get_bool(key):
      raise RuntimeError(f'{key} is already set; inspect the shutdown reason first.')
  sm = messaging.SubMaster(['deviceState', 'managerState', 'sentryRecorderState', 'pandaStates'])
  deadline = time.monotonic() + 3
  while time.monotonic() < deadline:
    sm.update(100)
  for service in ('deviceState', 'managerState'):
    if not sm.seen[service] or not sm.valid[service] or time.monotonic() - sm.recv_time[service] > 2:
      raise RuntimeError(f'No fresh {service}; stopped without deleting recordings.')
  if sm['deviceState'].started or any(p.ignitionLine or p.ignitionCan for p in sm['pandaStates']):
    raise RuntimeError('Turn off the ignition before running this script.')
  protected = set()
  if any(p.name == 'sentry_loggerd' and p.running for p in sm['managerState'].processes):
    if not sm.seen['sentryRecorderState'] or time.monotonic() - sm.recv_time['sentryRecorderState'] > 2:
      raise RuntimeError('Cannot identify the active Sentry route; stopped without deleting recordings.')
    if sm['sentryRecorderState'].route:
      protected.add(sm['sentryRecorderState'].route)
  return protected


def module_process(pid: int, module: str) -> bool:
  try:
    argv = Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
  except (FileNotFoundError, ProcessLookupError):
    return False
  encoded = module.encode()
  return argv[0] == encoded or any(a == b'-m' and b == encoded for a, b in zip(argv, argv[1:], strict=False))


def clear_requests_and_reboot(params) -> None:
  # Athena caches its in-memory queue repeatedly. Hold its supervisor and stop
  # the worker before clearing the saved queue so it cannot be written back.
  # Run inside tmux: stopping the worker can disconnect a Prime SSH connection.
  manager_pid = int(params.get('AthenadPid') or 0)
  held = False
  try:
    if manager_pid > 1 and module_process(manager_pid, ATHENA_MANAGER):
      os.kill(manager_pid, signal.SIGSTOP)
      held = True
      deadline = time.monotonic() + 2
      while time.monotonic() < deadline:
        state = Path(f'/proc/{manager_pid}/status').read_text()
        if re.search(r'^State:\s+[Tt]\b', state, re.MULTILINE):
          break
        time.sleep(.05)
      else:
        raise RuntimeError('Could not hold the Connect queue supervisor; stopped.')
    for entry in Path('/proc').iterdir():
      if entry.name.isdigit() and module_process(int(entry.name), ATHENA):
        try:
          os.kill(int(entry.name), signal.SIGKILL)
        except ProcessLookupError:
          pass
    params.put('AthenadUploadQueue', [], block=True)
    print('[QUEUE] Cleared pending Connect upload requests. Rebooting now...', flush=True)
    os.sync()
    subprocess.run(['sudo', '-n', 'reboot'], check=True)
    # Usually the reboot kills this process within a second. Restore the
    # supervisor if a reboot command returns success but shutdown never occurs.
    time.sleep(10)
    raise RuntimeError('Reboot did not complete; Connect is being restored. Reboot manually.')
  finally:
    if held:
      try:
        os.kill(manager_pid, signal.SIGCONT)
      except ProcessLookupError:
        pass


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--delete-completed-and-reboot', action='store_true', required=True)
  parser.parse_args()
  if not os.environ.get('TMUX'):
    raise RuntimeError('Run this script in tmux so it survives Prime SSH disconnecting.')
  from openpilot.common.hardware.hw import Paths
  from openpilot.common.params import Params

  if Path(Paths.log_root()) != LOG_ROOT or LOG_ROOT.resolve() != LOG_ROOT or not LOG_ROOT.is_dir():
    raise RuntimeError('Unexpected recording location; nothing deleted.')
  subprocess.run(['sudo', '-n', 'true'], check=True)  # Verify reboot permission before deleting anything.
  params = Params()
  protected = parked_routes(params)
  print('[SENTRY] Enable setting remains:', params.get_bool('SentryEnabled'), flush=True)
  print('[DELETE] Removing completed local drive/Sentry recordings. Active routes are preserved.', flush=True)
  before = shutil.disk_usage(LOG_ROOT).free
  count = delete_segments(LOG_ROOT, protected)
  freed = (shutil.disk_usage(LOG_ROOT).free - before) / 1024**3
  print(f'[DELETE] Removed {count} segments; approximately {freed:.2f} GiB freed.', flush=True)
  clear_requests_and_reboot(params)


if __name__ == '__main__':
  main()
