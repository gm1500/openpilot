"""Host checks for device-specific OS selection; never write block devices."""
import json
import os
from pathlib import Path
import subprocess
from unittest.mock import patch

import pytest

from openpilot.common.basedir import BASEDIR

ROOT = Path(BASEDIR)


@pytest.mark.parametrize('model,version,manifest', [
  ('comma tici', '12.8', 'agnos-tici.json'),
  ('comma tizi', '19.9', 'agnos.json'),
  ('comma mici', '19.9', 'agnos.json'),
])
def test_launch_selects_device_manifest(tmp_path, model, version, manifest):
  device = tmp_path/'model'
  device.write_bytes(model.encode() + b'\0')
  script = (ROOT/'launch_env.sh').read_text().replace('/sys/firmware/devicetree/base/model', str(device))
  result = subprocess.check_output(['bash', '-c', script + '\nprintf "%s\\n%s" "$AGNOS_VERSION" "$AGNOS_MANIFEST"'],
                                   env={**os.environ, 'AGNOS_VERSION': '19.9'}, text=True)
  selected_version, selected_manifest = result.splitlines()
  assert selected_version == version
  assert Path(selected_manifest).name == manifest
  partitions = json.loads((ROOT/selected_manifest).read_text())
  assert {p['name'] for p in partitions} == {'xbl', 'xbl_config', 'abl', 'aop', 'devcfg', 'boot', 'system'}
  assert all(p['has_ab'] and len(p['hash']) == 64 and p['size'] > 0 for p in partitions)


def test_updater_rejects_os_without_c3_support():
  from openpilot.system.updated import updated
  with patch.object(updated, 'HARDWARE') as hardware, patch.object(updated, 'run', return_value='19.9'), \
       patch('openpilot.common.hardware.comma.agnos.flash_agnos_update') as flash:
    hardware.get_device_type.return_value = 'tici'
    hardware.get_os_version.return_value = '12.8'
    with pytest.raises(RuntimeError, match='without original comma 3 support'):
      updated.handle_agnos_update()
    flash.assert_not_called()


def test_updater_flashes_c3_manifest(tmp_path):
  from openpilot.system.updated import updated
  manifest = tmp_path/'openpilot/common/hardware/comma/agnos-tici.json'
  manifest.parent.mkdir(parents=True)
  manifest.write_text('[]')
  with patch.object(updated, 'HARDWARE') as hardware, patch.object(updated, 'run', return_value='12.8'), \
       patch.object(updated, 'OVERLAY_MERGED', str(tmp_path)), patch.object(updated, 'set_consistent_flag') as consistent, \
       patch('openpilot.common.hardware.comma.agnos.get_target_slot_number', return_value=1), \
       patch('openpilot.common.hardware.comma.agnos.flash_agnos_update') as flash:
    hardware.get_device_type.return_value = 'tici'
    hardware.get_os_version.return_value = '12.7'
    updated.handle_agnos_update()
    consistent.assert_called_once_with(False)
    flash.assert_called_once_with(str(manifest), 1, updated.cloudlog)
