"""Exercise update/reboot ordering with isolated git, build and device commands."""
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys

import pytest

SCRIPT = Path(__file__).resolve().parents[4]/'tools/sentry_update.sh'


@pytest.fixture
def updater(tmp_path):
  commands = tmp_path/'bin'
  commands.mkdir()
  calls = tmp_path/'calls.jsonl'
  fake = commands/'command'
  fake.write_text(f'#!{sys.executable}\n'+'''
import json, os, pathlib, sys

name, args = pathlib.Path(sys.argv[0]).name, sys.argv[1:]
def record(value):
  with open(os.environ['UPDATE_CALLS'], 'a') as log:
    log.write(json.dumps(value)+'\\n')
record([name, *args])
stage = os.environ.get('FAIL_STAGE')
if name == 'git':
  command = ' '.join(args)
  if args[0] == 'symbolic-ref':
    print('unsupported-branch' if stage == 'branch' else os.environ.get('INSTALLED_BRANCH', 'sentry-session-routes-poc'))
  elif args[0] == 'status' and stage == 'dirty':
    print(' M modified.cc')
  if ((stage == 'fetch' and args[0] == 'fetch') or
      (stage == 'merge' and args[0] == 'merge') or
      (stage == 'submodules' and command.startswith('submodule update')) or
      (stage == 'lfs' and command == 'lfs pull')):
    sys.exit(1)
elif name == 'pgrep':
  sys.exit(1)
elif name == 'scons':
  if stage == 'build':
    sys.exit(1)
  for arg in args:
    if not arg.startswith('openpilot/') or (stage == 'params_build' and arg.endswith('libparams_c.so')):
      continue
    target = pathlib.Path(arg)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text('rebuilt')
    target.chmod(0o755)
elif name == 'python':
  if args[0] == '-':
    os.execv(sys.executable, [sys.executable, *args])
  assert args[0].endswith('/agnos.py') and '--swap' in args
  if stage == 'agnos':
    sys.exit(1)
elif name == 'sudo':
  assert args == ['reboot']
''')
  fake.chmod(0o755)
  for name in ('git', 'scons', 'pgrep', 'python', 'sudo', 'sync'):
    (commands/name).symlink_to(fake)
  params = tmp_path/'openpilot/common/params.py'
  params.parent.mkdir(parents=True)
  params.write_text('''
import json, os
class Params:
  def get_bool(self, key):
    # Only model the branch's registered keys; unknown names must fail.
    return {'IsOffroad': os.environ.get('FAIL_STAGE') not in ('onroad', 'missing_offroad'),
            'SentryEnabled': os.environ.get('PREVIOUS_SENTRY', '1') == '1'}[key]
  def put_bool(self, key, value):
    with open(os.environ['UPDATE_CALLS'], 'a') as log:
      log.write(json.dumps(['param', key, value])+'\\n')
''')
  (tmp_path/'launch_env.sh').write_text('export AGNOS_VERSION="19.9"\n')
  (tmp_path/'AGNOS').touch()
  (tmp_path/'VERSION').write_text('19.8\n')
  script = SCRIPT.read_text().replace('cd /data/openpilot', f'cd {shlex.quote(str(tmp_path))}')
  for path in ('/tmp/sentry-update.lock', '/AGNOS', '/VERSION'):
    script = script.replace(path, str(tmp_path/Path(path).name))
  local = tmp_path/'update.sh'
  local.write_text(script)

  def run(**variables):
    env = dict(os.environ, PATH=str(commands)+':'+os.environ['PATH'], UPDATE_CALLS=str(calls), **variables)
    result = subprocess.run(['bash', str(local)], env=env, capture_output=True, text=True, timeout=10)
    return result, [json.loads(line) for line in calls.read_text().splitlines()]
  return run, tmp_path


@pytest.mark.parametrize('stage', ['branch', 'dirty', 'onroad', 'missing_offroad', 'fetch', 'merge', 'submodules', 'lfs', 'build', 'params_build', 'agnos'])
def test_update_failure_never_reboots(updater, stage):
  run, _ = updater
  result, calls = run(FAIL_STAGE=stage)
  assert result.returncode != 0
  assert ['sudo', 'reboot'] not in calls
  assert ['param', 'SentryEnabled', True] not in calls
  if stage != 'agnos':
    assert not any(c[0] == 'python' and c[1].endswith('/agnos.py') for c in calls)
  if stage in ('branch', 'dirty', 'onroad', 'missing_offroad'):
    assert not any(c[0] in ('scons', 'param') for c in calls)


@pytest.mark.parametrize('current_os,enabled', [('19.9', '0'), ('19.8', '1')])
@pytest.mark.parametrize('branch', ['batmobile', 'sentry-session-routes-poc'])
def test_success_builds_before_os_swap_and_reboot(updater, current_os, enabled, branch):
  run, root = updater
  (root/'VERSION').write_text(current_os+'\n')
  result, calls = run(PREVIOUS_SENTRY=enabled, INSTALLED_BRANCH=branch)
  assert result.returncode == 0, result.stderr
  assert ['git', 'fetch', '--no-tags', 'origin', f'refs/heads/{branch}:refs/remotes/origin/{branch}'] in calls
  assert ['git', 'merge', '--ff-only', f'refs/remotes/origin/{branch}'] in calls
  build = next(i for i, c in enumerate(calls) if c[0] == 'scons')
  assert 'openpilot/common/libparams_c.so' in calls[build]
  assert 'openpilot/system/loggerd/sentry_loggerd' in calls[build]
  restore = max(i for i, c in enumerate(calls) if c[0] == 'param')
  assert calls[restore] == ['param', 'SentryEnabled', enabled == '1']
  assert build < restore < calls.index(['sudo', 'reboot'])
  assert calls[-2:] == [['sync'], ['sudo', 'reboot']]
  agnos = [i for i, c in enumerate(calls) if c[0] == 'python' and c[1].endswith('/agnos.py')]
  if current_os == '19.8':
    assert len(agnos) == 1 and build < agnos[0] < restore
  else:
    assert not agnos
