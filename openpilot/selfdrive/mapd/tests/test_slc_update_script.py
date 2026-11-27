"""Run the parked updater with isolated package metadata, installs and device commands."""
import importlib.resources
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tomllib

import pytest


SCRIPT = Path(__file__).resolve().parents[4] / 'tools/slc_update.sh'


@pytest.fixture
def updater(tmp_path):
  commands = tmp_path / 'bin'
  commands.mkdir()
  calls = tmp_path / 'calls.jsonl'
  state = tmp_path / 'packages.json'
  fake = commands / 'command'
  fake.write_text(f'#!{sys.executable}\n' + '''
import json, os, pathlib, sys
name, args = pathlib.Path(sys.argv[0]).name, sys.argv[1:]
with open(os.environ['UPDATE_CALLS'], 'a') as output:
  output.write(json.dumps([name, *args]) + '\\n')
stage = os.environ.get('FAIL_STAGE')
if name == 'git':
  if args[0] == 'status' and stage == 'dirty':
    print(' M modified.py')
  if stage == 'fetch' and args[0] == 'fetch':
    sys.exit(1)
elif name == 'python':
  if args[:2] == ['-m', 'SCons']:
    if stage == 'build':
      sys.exit(1)
    pathlib.Path('openpilot/common/libparams_c.so').write_text('rebuilt')
  elif args[0] == '-':
    os.execv(sys.executable, [sys.executable, *args])
  else:
    raise AssertionError(args)
elif name == 'sudo':
  assert args == ['reboot']
''')
  fake.chmod(0o755)
  for name in ('git', 'python', 'sudo', 'sync'):
    (commands / name).symlink_to(fake)

  params = tmp_path / 'openpilot/common/params.py'
  params.parent.mkdir(parents=True)
  params.write_text('''
import os
from pathlib import Path
class Params:
  def get_bool(self, key):
    if key == 'IsOffroad':
      path = Path('offroad_checks')
      checks = int(path.read_text()) + 1 if path.exists() else 1
      path.write_text(str(checks))
      stage = os.environ.get('FAIL_STAGE')
      return not (stage == 'onroad' or (stage == 'onroad_before_install' and checks >= 3) or
                  (stage == 'onroad_after_install' and checks >= 4))
    return os.environ.get('PENDING_POWER_FLAG') == key
  def check_key(self, key):
    pass
''')
  # Keep the real metadata API and fresh Python subprocesses. Only the package
  # inventory and package installer are isolated from the host environment.
  (tmp_path / 'sitecustomize.py').write_text('''
import importlib.metadata as metadata
import importlib.util
import json, os, pathlib, types
def inventory():
  return json.loads(pathlib.Path(os.environ['UPDATE_PACKAGES']).read_text())
def version(name):
  try:
    return inventory()[name.lower().replace('_', '-')]
  except KeyError:
    raise metadata.PackageNotFoundError(name) from None
metadata.version = version
metadata.distributions = lambda: [types.SimpleNamespace(metadata={'Name': name}, version=value)
                                  for name, value in inventory().items()]
if os.environ.get('FAIL_STAGE') == 'no_installer':
  import shutil
  original_find_spec = importlib.util.find_spec
  original_which = shutil.which
  importlib.util.find_spec = lambda name, *args: None if name == 'pip' else original_find_spec(name, *args)
  shutil.which = lambda name, *args: None if name == 'uv' else original_which(name, *args)
''')
  (tmp_path / 'timezonefinder.py').write_text('''
import importlib.metadata as metadata
import json, os
metadata.version('timezonefinder')
class TimezoneFinder:
  def __init__(self, **kwargs):
    pass
  def timezone_at(self, *, lat, lng):
    with open(os.environ['UPDATE_CALLS'], 'a') as output:
      output.write(json.dumps(['timezone', lat, lng]) + '\\n')
    assert (lat, lng) == (53.5461, -113.4938)
    return 'Europe/London' if os.environ.get('FAIL_STAGE') == 'verification' else 'America/Edmonton'
''')
  zone = tmp_path / 'tzdata/zoneinfo/America/Edmonton'
  zone.parent.mkdir(parents=True)
  (tmp_path / 'tzdata/__init__.py').touch()
  zone.write_bytes(importlib.resources.files('tzdata').joinpath('zoneinfo', 'America', 'Edmonton').read_bytes())
  (tmp_path / 'pip.py').write_text('''
import json, os, pathlib, sys
args = sys.argv[1:]
assert args[0] == 'install'
constraint = pathlib.Path(args[args.index('--constraint') + 1]).read_text()
with open(os.environ['UPDATE_CALLS'], 'a') as output:
  output.write(json.dumps(['install', sys.executable, args, constraint]) + '\\n')
if os.environ.get('FAIL_STAGE') == 'install':
  raise SystemExit(1)
state = pathlib.Path(os.environ['UPDATE_PACKAGES'])
packages = json.loads(state.read_text())
for arg in args:
  if '==' in arg:
    name, version = arg.split('==')
    packages[name] = version
if os.environ.get('FAIL_STAGE') == 'runtime_changed':
  packages['numpy'] = '99.0.0'
if os.environ.get('FAIL_STAGE') == 'civil_time':
  pathlib.Path('tzdata/zoneinfo/America/Edmonton').write_bytes(b'invalid timezone data')
state.write_text(json.dumps(packages))
''')
  (tmp_path / 'uv.lock').write_bytes((SCRIPT.parent.parent / 'uv.lock').read_bytes())
  script = SCRIPT.read_text().replace('cd /data/openpilot', f'cd {shlex.quote(str(tmp_path))}')
  for path in ('/tmp/sentry-update.lock', '/AGNOS', '/VERSION', '/data/slc-update.log'):
    script = script.replace(path, str(tmp_path / Path(path).name))
  local = tmp_path / 'update.sh'
  local.write_text(script)

  def run(*args, healthy=False, **variables):
    packages = {'numpy': '2.5.3', 'cffi': '2.0.0', 'tinygrad': '0.14.0', 'existing-package': '1.2.3'}
    if healthy:
      packages.update(timezonefinder='8.2.0', tzdata='2026.6')
    state.write_text(json.dumps(packages))
    calls.write_text('')
    env = dict(os.environ, PATH=str(commands) + ':' + os.environ['PATH'], UPDATE_CALLS=str(calls),
               UPDATE_PACKAGES=str(state), **variables)
    result = subprocess.run(['bash', str(local), *args], env=env, capture_output=True, text=True, timeout=20)
    return result, [json.loads(line) for line in calls.read_text().splitlines()]
  return run


def test_healthy_dependencies_are_verified_without_installing(updater):
  result, calls = updater(healthy=True)
  assert result.returncode == 0, result.stdout + result.stderr
  assert not any(call[0] == 'install' for call in calls)
  assert ['timezone', 53.5461, -113.4938] in calls
  assert calls[-2:] == [['sync'], ['sudo', 'reboot']]


def test_missing_dependencies_are_pinned_without_changing_existing_runtime(updater):
  result, calls = updater()
  assert result.returncode == 0, result.stdout + result.stderr
  install = next(call for call in calls if call[0] == 'install')
  assert install[1] == sys.executable
  assert 'timezonefinder==8.2.0' in install[2]
  assert 'tzdata==2026.6' in install[2]
  assert '--upgrade' not in install[2]
  assert 'numpy==2.5.3' in install[3].splitlines()
  assert 'cffi==2.0.0' in install[3].splitlines()
  assert 'existing-package==1.2.3' in install[3].splitlines()
  assert 'tinygrad==0.14.0' in install[3].splitlines()
  assert not any(arg.startswith(('numpy==', 'cffi==')) for arg in install[2])
  locked = {package['name']: package['version'] for package in tomllib.loads((SCRIPT.parent.parent / 'uv.lock').read_text())['package']}
  for name in ('h3', 'flatbuffers', 'pycparser'):
    assert f'{name}=={locked[name]}' in install[2]
  build = next(i for i, call in enumerate(calls) if call[:3] == ['python', '-m', 'SCons'])
  verify = max(i for i, call in enumerate(calls) if call[0] == 'timezone')
  assert calls.index(install) < verify < build < calls.index(['sudo', 'reboot'])


@pytest.mark.parametrize('stage', ['install', 'verification', 'civil_time', 'runtime_changed', 'onroad_after_install'])
def test_dependency_failures_never_build_or_reboot(updater, stage):
  result, calls = updater(FAIL_STAGE=stage)
  assert result.returncode != 0
  assert any(call[0] == 'install' for call in calls)
  assert not any(call[:3] == ['python', '-m', 'SCons'] for call in calls)
  assert ['sudo', 'reboot'] not in calls
  assert '[SUCCESS]' not in result.stdout


@pytest.mark.parametrize('stage', ['dirty', 'onroad', 'onroad_before_install', 'fetch', 'no_installer'])
def test_unsafe_update_state_prevents_dependency_install(updater, stage):
  result, calls = updater(FAIL_STAGE=stage)
  assert result.returncode != 0
  assert not any(call[0] == 'install' for call in calls)
  assert ['sudo', 'reboot'] not in calls


@pytest.mark.parametrize('flag', ['ForcePowerDown', 'DoShutdown', 'DoReboot', 'DoUninstall'])
def test_pending_power_request_prevents_dependency_install(updater, flag):
  result, calls = updater(PENDING_POWER_FLAG=flag)
  assert result.returncode != 0
  assert flag in result.stdout + result.stderr
  assert not any(call[0] == 'install' for call in calls)
  assert ['sudo', 'reboot'] not in calls
