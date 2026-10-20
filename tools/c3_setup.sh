#!/usr/bin/env bash
# AGNOS 12.8 supplies the kernel/drivers; uv.lock supplies Batmobile's runtime.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
if ! grep -q '^comma tici' /sys/firmware/devicetree/base/model 2>/dev/null; then
  echo "c3_setup.sh requires an original comma 3" >&2
  exit 1
fi
if [[ "$(cat /VERSION)" != "12.8" ]]; then
  echo "Install AGNOS 12.8 before setting up the comma 3 runtime" >&2
  exit 1
fi

# Reuse a complete environment offline; resync whenever dependency metadata changes.
stamp="$(sha256sum uv.lock pyproject.toml | sha256sum | cut -d ' ' -f 1)"
if [[ ! -x .venv/bin/python || "$(cat .venv/c3-dependencies 2>/dev/null || true)" != "$stamp" ]]; then
  # The image's uv predates this lockfile format. Keep the bootstrap isolated
  # from both the OS environment and openpilot's locked environment.
  if [[ ! -x .venv-c3-bootstrap/bin/uv ]]; then
    uv venv --seed --python /usr/bin/python3 --python-preference only-system .venv-c3-bootstrap
    .venv-c3-bootstrap/bin/python -m pip install --disable-pip-version-check 'uv==0.12.23'
  fi
  # Use the supported system 3.12 interpreter, not the newer .python-version pin.
  UV_PROJECT_ENVIRONMENT="$ROOT/.venv" .venv-c3-bootstrap/bin/uv sync --frozen --extra tools --python /usr/bin/python3 --python-preference only-system
  printf '%s\n' "$stamp" > .venv/c3-dependencies
fi

# Modern modemd uses stable interface aliases absent from AGNOS 12.8. Keep
# these rules in /run so switching back to a legacy branch needs only a reboot.
if ! cmp -s tools/c3/78-modem.rules /run/udev/rules.d/78-batmobile-modem.rules; then
  sudo install -D -m 644 tools/c3/78-modem.rules /run/udev/rules.d/78-batmobile-modem.rules
  sudo udevadm control --reload-rules
  sudo udevadm trigger --subsystem-match=tty --action=add
fi
