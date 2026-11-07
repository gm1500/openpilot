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

# Disable both caches so the native registry includes the new settings.
echo "[BUILD] Rebuilding the native parameter registry..."
CCACHE_DISABLE=1 python -m SCons --cache-disable -j2 openpilot/common/libparams_c.so
test -s openpilot/common/libparams_c.so
python - <<'PY'
from openpilot.common.params import Params
p = Params()
for key in ('SlcOffsetType', 'SlcOffsetValue', 'SlcChangePolicy', 'SlcResumePolicy',
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
