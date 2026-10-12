"""Compile the shared native startup guard without requiring panda hardware."""
import os
import shlex
import shutil
import subprocess
from pathlib import Path

import capnproto
import pytest


def test_native_state_before_first_message(tmp_path):
  compiler = shlex.split(os.environ.get('CXX', 'c++'))
  if not compiler or not shutil.which(compiler[0]):
    pytest.skip('native startup check requires a C++ compiler')
  root = Path(__file__).resolve().parents[4]
  binary = tmp_path/'test_state'
  subprocess.run([*compiler, '-std=c++17', '-I'+str(root), '-I'+str(root/'openpilot'),
                  '-I'+str(root/'openpilot/cereal/gen/cpp'), '-I'+capnproto.INCLUDE_DIR,
                  str(Path(__file__).with_name('test_state.cc')), '-L'+capnproto.LIB_DIR,
                  '-lcapnp', '-lkj', '-pthread', '-o', str(binary)], check=True)
  subprocess.run([str(binary)], check=True)
