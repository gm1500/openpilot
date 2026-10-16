#!/usr/bin/env bash
# One-time cleanup of completed recordings; run in tmux while parked.
set -eEuo pipefail
trap 'echo "[ERROR] Cleanup/update stopped at line $LINENO; not rebooting." >&2' ERR
test -n "${TMUX:-}" || { echo 'Run this script inside tmux.' >&2; exit 1; }
cd /data/openpilot
exec 9>/tmp/sentry-update.lock
flock -n 9 || { echo 'Another update is running.' >&2; exit 1; }
exec > >(tee -a /data/sentry-cleanup-update.log) 2>&1
echo "[START] $(date -Is)"
test "$(git symbolic-ref --quiet --short HEAD)" = batmobile
test -z "$(git status --porcelain --untracked-files=no)"
export PYTHONPATH="$PWD"
export GIT_TERMINAL_PROMPT=0
if [[ -f .venv/bin/activate ]]; then source .venv/bin/activate; fi
python - <<'PY'
from openpilot.common.params import Params
p = Params()
if not p.get_bool('IsOffroad'):
  raise SystemExit('Park and wait for offroad mode before updating.')
for key in ('ForcePowerDown', 'DoShutdown', 'DoReboot', 'DoUninstall'):
  if p.get_bool(key):
    raise SystemExit(f'{key} is set; inspect the shutdown reason first.')
PY
git submodule foreach --recursive 'test -z "$(git status --porcelain)"'
git fetch --no-tags origin refs/heads/batmobile:refs/remotes/origin/batmobile
git merge --ff-only refs/remotes/origin/batmobile
git submodule sync --recursive
git submodule update --init --recursive --jobs 2
# Invalidate a staged overlay so startup keeps this manual update.
rm -f .overlay_init
git log -1 --oneline
python - <<'PY'
from openpilot.selfdrive.mapd.osm_speed_limit import QUERY_RADIUS
from openpilot.system.loggerd.uploader import SENTRY_DENIED_RETRY_SECONDS
if QUERY_RADIUS != 3000 or SENTRY_DENIED_RETRY_SECONDS < 300:
  raise SystemExit('Expected 3 km cache/retry fix missing; no recordings deleted.')
print('[VERIFY] 3 km map cache and denied-file retry cooldown are installed.')
PY
python tools/sentry_clear_recordings.py --delete-completed-and-reboot
