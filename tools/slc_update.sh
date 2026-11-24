#!/usr/bin/env bash
# Run in tmux on a parked comma. Sentry and power preferences stay unchanged.
set -eEo pipefail
trap 'echo "[ERROR] SLC update stopped at line $LINENO. Not rebooting; see /data/slc-update.log." >&2; exit 1' ERR

cd /data/openpilot
exec 9>/tmp/sentry-update.lock
if ! flock -n 9; then
  echo "[ERROR] Another update is running. Let it finish first." >&2
  exit 1
fi
exec > >(tee -a /data/slc-update.log) 2>&1
target="${1:-batmobile}"
case "$target" in
  batmobile) ;;
  *) echo "[ERROR] Unsupported update branch: $target" >&2; exit 1 ;;
esac
echo "[START] $(date -Is) - SLC update on $target"
if [[ -n "$(git status --porcelain --untracked-files=no)" ]]; then
  echo "[ERROR] Local changes found. Preserve them before updating." >&2
  exit 1
fi
git submodule foreach --recursive 'test -z "$(git status --porcelain)"'
export GIT_TERMINAL_PROMPT=0
export PYTHONPATH="$PWD"
if [[ -f .venv/bin/activate ]]; then
  source .venv/bin/activate
fi

check_parked() {
  python - <<'PY'
from openpilot.common.params import Params
p = Params()
if not p.get_bool('IsOffroad'):
  raise SystemExit('[ERROR] Park and wait for offroad mode before updating.')
for key in ('ForcePowerDown', 'DoShutdown', 'DoReboot', 'DoUninstall'):
  if p.get_bool(key):
    raise SystemExit(f'[ERROR] {key} is pending. Resolve that condition before updating.')
PY
}
check_parked

echo "[UPDATE] Fetching origin/$target..."
git fetch --no-tags origin "refs/heads/$target:refs/remotes/origin/$target"
if git show-ref --verify --quiet "refs/heads/$target"; then
  git switch "$target"
else
  git switch --track -c "$target" "origin/$target"
fi
git merge --ff-only "origin/$target"
git submodule sync --recursive
git submodule update --init --recursive --jobs 2
git lfs pull
rm -f .overlay_init
git log -1 --oneline
check_parked

# AGNOS does not run uv sync after a branch refresh. Repair only the civil-time
# dependencies in this explicit parked updater, never from launch or mapd.
echo "[DEPS] Checking GPS timezone lookup and civil-time data..."
python - <<'PYDEPS'
import importlib.util
from importlib import metadata
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import tomllib

from openpilot.common.params import Params

required = {'timezonefinder': '8.2.0', 'tzdata': '2026.6'}
verify = r"""
from datetime import datetime, timedelta
from importlib import metadata, resources
from zoneinfo import ZoneInfo
from timezonefinder import TimezoneFinder
if metadata.version('timezonefinder') != '8.2.0':
  raise SystemExit('[ERROR] timezonefinder version mismatch')
if metadata.version('tzdata') != '2026.6':
  raise SystemExit('[ERROR] tzdata version mismatch')
name = TimezoneFinder().timezone_at(lat=53.5461, lng=-113.4938)
if name != 'America/Edmonton':
  raise SystemExit(f'[ERROR] GPS timezone lookup failed: {name!r}')
zones = [ZoneInfo(name)]
with resources.files('tzdata').joinpath('zoneinfo', 'America', 'Edmonton').open('rb') as source:
  zones.append(ZoneInfo.from_file(source, key=name))
for zone in zones:
  if (datetime(2026, 1, 15, tzinfo=zone).utcoffset() != timedelta(hours=-7) or
      datetime(2026, 7, 15, tzinfo=zone).utcoffset() != timedelta(hours=-6)):
    raise SystemExit('[ERROR] Civil-time daylight-saving lookup failed')
print('[DEPS] Verified timezonefinder 8.2.0, tzdata 2026.6 and Edmonton GPS/DST lookup.')
"""
# A fresh interpreter also catches missing native libraries and avoids retaining
# an old imported module after pip replaces an incomplete installation.
if subprocess.run([sys.executable, '-c', verify], check=False).returncode == 0:
  raise SystemExit(0)

normalize = lambda name: re.sub(r'[-_.]+', '-', name).lower()
def installed_versions():
  return {normalize(dist.metadata['Name']): dist.version for dist in metadata.distributions() if dist.metadata['Name']}

before = installed_versions()
packages = {p['name']: p for p in tomllib.loads(Path('uv.lock').read_text())['package']}
for name, version in required.items():
  if packages[name]['version'] != version:
    raise SystemExit(f'[ERROR] {name} repair pin does not match uv.lock. Not installing.')
# Pin any missing transitive dependencies from the committed lockfile. Existing
# packages, including NumPy/cffi, are constrained to their installed versions.
closure = set(required)
pending = list(required)
while pending:
  for dependency in packages[pending.pop()].get('dependencies', []):
    name = dependency['name']
    if name not in closure:
      closure.add(name)
      pending.append(name)
constraints = {name: version for name, version in before.items() if name not in required}
constraints.update({name: packages[name]['version'] for name in closure if name not in before})
requirements = [f'{name}=={version}' for name, version in required.items()]
requirements.extend(f'{name}=={packages[name]["version"]}' for name in sorted(closure - set(before) - set(required)))

p = Params()
if not p.get_bool('IsOffroad') or any(p.get_bool(key) for key in ('ForcePowerDown', 'DoShutdown', 'DoReboot', 'DoUninstall')):
  raise SystemExit('[ERROR] Vehicle is no longer parked and ready. Not installing dependencies.')
with tempfile.TemporaryDirectory(prefix='slc-timezone-') as temporary:
  constraint_file = Path(temporary) / 'constraints.txt'
  constraint_file.write_text(''.join(f'{name}=={version}\n' for name, version in sorted(constraints.items())))
  if importlib.util.find_spec('pip') is not None:
    installer = [sys.executable, '-m', 'pip', 'install', '--disable-pip-version-check']
  elif shutil.which('uv'):
    installer = [shutil.which('uv'), 'pip', 'install', '--python', sys.executable]
  else:
    raise SystemExit('[ERROR] No pip or uv installer is available for the active Python. Not rebooting.')
  print(f'[DEPS] Repairing civil-time packages for {sys.executable}; existing runtime versions are constrained.', flush=True)
  subprocess.check_call([*installer, '--constraint', str(constraint_file), *requirements])
after = installed_versions()
changed = [name for name, version in before.items() if name not in required and after.get(name) != version]
if changed:
  raise SystemExit(f'[ERROR] Existing runtime packages changed unexpectedly: {changed}. Not rebooting.')
subprocess.run([sys.executable, '-c', verify], check=True)
PYDEPS
check_parked

# Disable both caches so the native registry includes the new settings.
echo "[BUILD] Rebuilding the native parameter registry..."
CCACHE_DISABLE=1 python -m SCons --cache-disable -j2 openpilot/common/libparams_c.so
test -s openpilot/common/libparams_c.so
python - <<'PY'
from openpilot.common.params import Params
p = Params()
for key in ('SlcOffsetType', 'SlcOffsetValue', 'SlcChangePolicy', 'SlcResumePolicy', 'SlcAdvancedSpeedLimit', 'InitialCruiseSpeedKph',
            'OfflineMapRegion', 'OfflineMapRequest', 'OfflineMapStatus', 'OfflineMapUpdateSchedule', 'OfflineMapUpdateWeekday'):
  p.check_key(key)
print('[BUILD] All SLC parameter keys verified.')
PY

if [[ -f /AGNOS ]]; then
  installed_version="$(cat /VERSION)"
  required_version="$(unset AGNOS_VERSION; source ./launch_env.sh; printf '%s' "$AGNOS_VERSION")"
  test -n "$required_version"
  if [[ "$installed_version" != "$required_version" ]]; then
    echo "[AGNOS] Staging $installed_version -> $required_version..."
    python openpilot/common/hardware/comma/agnos.py --swap openpilot/system/hardware/comma/agnos.json
  else
    echo "[AGNOS] Already current ($installed_version)."
  fi
fi
check_parked
echo "[SUCCESS] SLC update and rebuild complete. Rebooting..."
sync
sudo reboot
