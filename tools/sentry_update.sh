#!/usr/bin/env bash
# Run through tmux on a parked comma. Keep the installed Sentry branch.
set -eEo pipefail
trap 'echo "[ERROR] Update stopped at line $LINENO. Not rebooting; Sentry may remain off." >&2; exit 1' ERR

enable_sentry=0
if [[ $# -gt 1 || ( $# -eq 1 && "$1" != "--enable-sentry" ) ]]; then
  echo "Usage: bash tools/sentry_update.sh [--enable-sentry]" >&2
  exit 2
fi
if [[ "${1:-}" == "--enable-sentry" ]]; then
  enable_sentry=1
fi

cd /data/openpilot
exec 9>/tmp/sentry-update.lock
if ! flock -n 9; then
  echo "[ERROR] Another Sentry update is already running." >&2
  exit 1
fi
exec > >(tee -a /data/sentry-update.log) 2>&1
echo "[START] $(date -Is) — update log: /data/sentry-update.log"

branch="$(git symbolic-ref --quiet --short HEAD)"
if [[ "$branch" != "batmobile" && "$branch" != "sentry-session-routes-poc" ]]; then
  echo "[ERROR] This updater requires batmobile or sentry-session-routes-poc; current branch: $branch" >&2
  exit 1
fi
if [[ -n "$(git status --porcelain --untracked-files=no)" ]]; then
  echo "[ERROR] Local changes found. Preserve them before updating." >&2
  exit 1
fi
git submodule foreach --recursive 'test -z "$(git status --porcelain)"'

export PYTHONPATH="$PWD"
export GIT_TERMINAL_PROMPT=0
if [[ -f .venv/bin/activate ]]; then
  source .venv/bin/activate
fi

# Manager maintains IsOffroad as the inverse of deviceState.started. This branch
# has no IsOnroad parameter. A missing IsOffroad value must also stop the update.
sentry_enabled="$(python - <<'PY'
from openpilot.common.params import Params

p = Params()
if not p.get_bool('IsOffroad'):
  raise SystemExit('[ERROR] Park the vehicle and wait for offroad mode before updating.')
previous = p.get_bool('SentryEnabled')
p.put_bool('SentryEnabled', False, block=True)
if p.get_bool('SentryEnabled'):
  raise SystemExit('[ERROR] Could not persist Sentry pause. Not updating.')
print(int(previous))
PY
)"
target_sentry="$sentry_enabled"
if [[ "$enable_sentry" == "1" ]]; then
  target_sentry=1
fi
echo "[SENTRY] Enable setting before update: $sentry_enabled; after success: $target_sentry (0=off, 1=on)."
echo "[SENTRY] Waiting for the recorder to close its files..."
for attempt in {1..30}; do
  if ! pgrep -f '(^|/)sentry_loggerd([[:space:]]|$)' >/dev/null; then
    break
  fi
  sleep 1
done
if pgrep -f '(^|/)sentry_loggerd([[:space:]]|$)' >/dev/null; then
  echo "[ERROR] Recorder is still running. Not updating or rebooting." >&2
  exit 1
fi

echo "[UPDATE] Updating $branch..."
git fetch --no-tags origin "refs/heads/$branch:refs/remotes/origin/$branch"
git merge --ff-only "refs/remotes/origin/$branch"
git submodule sync --recursive
git submodule update --init --recursive --jobs 2
git lfs pull
# Prevent an already-staged overlay update from replacing this manual update.
rm -f .overlay_init
git log -1 --oneline

echo "[BUILD] Rebuilding the Sentry recorder and parameter library..."
rm -f openpilot/system/loggerd/sentry_loggerd openpilot/system/loggerd/sentry_loggerd.o
CCACHE_DISABLE=1 scons --cache-disable -j2 openpilot/common/libparams_c.so openpilot/system/loggerd/sentry_loggerd
test -s openpilot/common/libparams_c.so
test -x openpilot/system/loggerd/sentry_loggerd

# Use the branch's launch requirement, not a nonexistent manifest version field.
# Stage/verify the inactive OS slot only after the recorder builds successfully.
if [[ -f /AGNOS ]]; then
  installed_version="$(cat /VERSION)"
  required_version="$(unset AGNOS_VERSION; source ./launch_env.sh; printf '%s' "$AGNOS_VERSION")"
  test -n "$required_version"
  if [[ "$installed_version" != "$required_version" ]]; then
    echo "[AGNOS] Updating $installed_version to $required_version..."
    python openpilot/common/hardware/comma/agnos.py --swap openpilot/common/hardware/comma/agnos.json
  else
    echo "[AGNOS] Already current ($installed_version)."
  fi
fi

python - "$target_sentry" "$enable_sentry" <<'PY'
import sys
from openpilot.common.params import Params, UnknownKeyName

p = Params()
enabled = sys.argv[1] == '1'
if sys.argv[2] == '1':
  try:
    p.put_bool('SentryOffThisDrive', False, block=True)
    if p.get_bool('SentryOffThisDrive'):
      raise SystemExit('[ERROR] Could not clear the temporary Sentry off setting. Not rebooting.')
  except UnknownKeyName:
    pass  # Older supported branches do not have this temporary override.
p.put_bool('SentryEnabled', enabled, block=True)
if p.get_bool('SentryEnabled') != enabled:
  raise SystemExit('[ERROR] Sentry enable setting did not persist. Not rebooting.')
print('[SENTRY] Verified saved enable setting:', 'ON' if enabled else 'OFF')
PY
echo "[SUCCESS] Updated and rebuilt. Rebooting now..."
sync
sudo reboot
