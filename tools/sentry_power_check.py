#!/usr/bin/env python3
"""Read current power settings and pre-manager snapshots without changing the device."""
import argparse
import json
from pathlib import Path


POWER_KEYS = (
  'SentryEnabled', 'SentryOffThisDrive', 'SentryRiskMode', 'IsOffroad',
  'ForcePowerDown', 'DisablePowerDown', 'DoShutdown', 'DoReboot', 'DoUninstall',
  'SentryCutoffVoltage', 'SentryMaxHours', 'SentryShutdownOnTimeLimit',
  'SentryMaxTemp', 'SentryShutdownOnOverheat', 'CarBatteryCapacity',
  'LastManagerExitReason', 'LastAgnosPowerMonitorShutdown', 'LastPowerDropDetected',
)
RUNTIME_KEYS = ('monoTime', 'session', 'capture', 'phase', 'parkingStartedAt', 'parkingLimitReached')


def read_current_params(root):
  result = {}
  for key in POWER_KEYS:
    try:
      result[key] = (root/key).read_text(errors='replace')[-2000:].strip()
    except FileNotFoundError:
      result[key] = '(unset)'
    except OSError as exc:
      result[key] = f'(unreadable: {type(exc).__name__})'
  return result


def boot_snapshot(path, reader):
  # manager copies these params before clearing DoShutdown and exit reasons.
  # Keep a strict allowlist: bootlogs also contain account/device credentials.
  for msg in reader(str(path)):
    if msg.which() == 'initData':
      return {entry.key: bytes(entry.value).decode(errors='replace')[-2000:].strip()
              for entry in msg.initData.params.entries if entry.key in POWER_KEYS}
  return {}


def print_report(params_root=Path('/data/params/d'), runtime=Path('/dev/shm/openpilot_sentry.json'),
                 boot_root=Path('/data/media/0/realdata/boot'), count=3):
  print('CURRENT POWER SETTINGS (read-only):')
  print(json.dumps(read_current_params(params_root), indent=2))
  try:
    state = json.loads(runtime.read_text())
    print('SENTRY RUNTIME (timestamps are monotonic; file may be stale):')
    print(json.dumps({key: state[key] for key in RUNTIME_KEYS if key in state}, indent=2))
  except (OSError, ValueError, TypeError):
    print('Sentry runtime: unavailable')

  paths = sorted(boot_root.glob('*.zst'), key=lambda p: p.stat().st_mtime, reverse=True)[:count]
  if not paths:
    print('No saved bootlogs found.')
    return
  try:
    from openpilot.tools.lib.logreader import LogReader
  except Exception as exc:
    print(f'Bootlog reader unavailable: {type(exc).__name__}. Current settings above do not require a working build.')
    print('Latest bootlog files:', ', '.join(p.name for p in paths))
    return
  for path in paths:
    print(f'PRE-MANAGER SNAPSHOT: {path.name}')
    try:
      print(json.dumps(boot_snapshot(path, LogReader), indent=2))
    except Exception as exc:
      print(f'Cannot read snapshot: {type(exc).__name__}')


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--bootlogs', type=int, default=3, choices=range(11), metavar='0-10')
  args = parser.parse_args()
  print_report(count=args.bootlogs)
