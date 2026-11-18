"""Check the compiled registry used by native processes, not Python's service list."""
import os
import shlex
import shutil
import signal
import subprocess
from pathlib import Path

import capnproto
import json11
import pytest
import zeromq

from openpilot.common.prefix import OpenpilotPrefix


@pytest.fixture(scope='module')
def services_binary(tmp_path_factory):
  root = Path(__file__).resolve().parents[4]
  libraries = [root/'openpilot/cereal/libsocketmaster.a', root/'msgq_repo/libmsgq.a', root/'openpilot/common/libcommon.a']
  if not all(path.exists() for path in libraries):
    pytest.skip('build native messaging libraries first')
  compiler = shlex.split(os.environ.get('CXX', 'c++'))
  if not compiler or not shutil.which(compiler[0]):
    pytest.skip('native service check requires a C++ compiler')
  binary = tmp_path_factory.mktemp('native_services')/'test_services'
  subprocess.run([*compiler, '-std=c++17', '-I'+str(root), '-I'+str(root/'openpilot'), '-I'+str(root/'msgq_repo'),
                  '-I'+str(root/'openpilot/cereal/gen/cpp'), '-I'+capnproto.INCLUDE_DIR,
                  str(Path(__file__).with_suffix('.cc')), *map(str, libraries),
                  '-L'+capnproto.LIB_DIR, '-lcapnp', '-lkj', '-L'+json11.LIB_DIR, '-ljson11',
                  '-L'+zeromq.LIB_DIR, '-lzmq', '-pthread', '-o', str(binary)], check=True)
  return binary


@pytest.mark.parametrize('mode,names', [
  ('sub', ['selfdriveState', 'deviceState', 'cabinCameraState']),
  ('pub', ['can', 'pandaStates', 'peripheralState']),
  ('sub', ['sentryState', 'pandaStates', 'deviceState', 'sentryNarrowEncodeData',
           'sentryWideEncodeData', 'sentryCabinEncodeData', 'sentryQcamEncodeData', 'sentryWideQcamEncodeData', 'sentryCabinQcamEncodeData']),
  ('pub', ['sentryRecorderState']),
  ('pub', ['sentryNarrowEncodeData', 'sentryWideEncodeData', 'sentryCabinEncodeData',
           'sentryQcamEncodeData', 'sentryWideQcamEncodeData', 'sentryCabinQcamEncodeData']),
])
def test_native_services_start_without_publishers(services_binary, mode, names):
  # Match the native panda, recorder and encoder subscriptions/publications.
  # A stale libsocketmaster.a must fail here even when Python knows the names.
  with OpenpilotPrefix():
    result = subprocess.run([str(services_binary), mode, *names], capture_output=True, text=True, timeout=10)
  assert result.returncode == 0, result.stderr


@pytest.mark.parametrize('mode', ['sub', 'pub'])
def test_unknown_native_service_names_the_missing_service(services_binary, mode):
  with OpenpilotPrefix():
    result = subprocess.run([str(services_binary), mode, 'missingSentryService'], capture_output=True, text=True, timeout=10)
  assert result.returncode == -signal.SIGABRT
  assert "Unknown messaging service 'missingSentryService'" in result.stderr
