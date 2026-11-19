"""Run real UI startup and ignition transitions with native headless drawing.

Only unavailable visual assets are substituted. Schema messages, UI state updates,
layout constructors, projection, widgets and draw calls use their real code.
"""

import os
from pathlib import Path
import subprocess
import sys


def test_tici_startup_and_ignition_transitions():
  root = Path(__file__).resolve().parents[4]
  env = dict(os.environ, RAYLIB_BACKEND='headless', BIG='1', SCALE='1')
  env['PYTHONPATH'] = str(root) + os.pathsep + env.get('PYTHONPATH', '')
  result = subprocess.run([sys.executable, str(Path(__file__).resolve())], cwd=root, env=env, text=True, capture_output=True, timeout=30)
  assert result.returncode == 0, result.stdout + result.stderr
  assert 'onroad transitions rendered' in result.stdout


def run_startup_smoke():
  import os
  import time
  from types import SimpleNamespace
  from openpilot.common.swaglog import cloudlog

  cloudlog.handlers.clear()
  from openpilot.common.prefix import OpenpilotPrefix

  with OpenpilotPrefix():
    from openpilot.selfdrive.ui.ui import gui_app, MainLayout, MiciMainLayout, ui_state
    from openpilot.selfdrive.ui.layouts.onboarding import terms_version, training_version
    from openpilot.system.ui.lib.application import FontWeight
    from openpilot.cereal import messaging
    import raylib_nb as rl
    from openpilot.system.ui.lib import emoji

    emoji._archive = lambda: SimpleNamespace(files=[])
    gui_app.init_window('Full startup smoke')
    font = rl.get_font_default()
    gui_app._fonts = dict.fromkeys(FontWeight, font)

    def placeholder(asset_path, width=None, height=None, *args, **kwargs):
      return rl.gen_image_color(max(1, round(width or 64)), max(1, round(height or 64)), rl.Color(180, 180, 180, 255))

    gui_app._load_image_from_path = placeholder
    ui_state.params.put('HasAcceptedTerms', terms_version, block=True)
    ui_state.params.put('CompletedTrainingVersion', training_version, block=True)
    layout = MainLayout() if gui_app.big_ui() else MiciMainLayout()
    print('constructed', os.environ['BIG'], flush=True)
    rect = rl.Rectangle(0, 0, gui_app.width, gui_app.height)
    ui_state.sm.frame = 1
    for _ in range(3):
      layout.render(rect)
    print('offroad rendered', flush=True)
    msgs = {}
    for service in (
      'deviceState',
      'carState',
      'carParams',
      'selfdriveState',
      'driverStateV2',
      'driverMonitoringState',
      'carControl',
      'carOutput',
      'radarState',
      'extrinsicsCalibration',
      'modelV2',
      'longitudinalPlan',
      'mapCruiseState',
      'mapSpeedLimit',
    ):
      msgs[service] = messaging.new_message(service, valid=True)
    msgs['pandaStates'] = messaging.new_message('pandaStates', 1, valid=True)
    msgs['pandaStates'].pandaStates[0].pandaType = 'uno'
    msgs['deviceState'].deviceState.started = True
    cs = msgs['carState'].carState
    cs.vEgo = 15.0
    cs.vEgoCluster = 15.0
    cs.vCruise = 100.0
    cs.vCruiseCluster = 100.0
    cs.gearShifter = 'drive'
    cs.cruiseState.available = True
    msgs['carParams'].carParams.openpilotLongitudinalControl = True
    ss = msgs['selfdriveState'].selfdriveState
    ss.enabled = True
    ss.state = 'enabled'
    ss.personality = 'standard'
    dm = msgs['driverStateV2'].driverStateV2
    dm.leftDriverData.faceOrientation = [0.0, 0.0, 0.0]
    dm.rightDriverData.faceOrientation = [0.0, 0.0, 0.0]
    msgs['driverMonitoringState'].driverMonitoringState.activePolicy = 'vision'
    msgs['carControl'].carControl.latActive = True
    msgs['carControl'].carControl.longActive = True
    cal = msgs['extrinsicsCalibration'].extrinsicsCalibration
    cal.calStatus = 'calibrated'
    cal.height = [1.2]
    cal.rpyCalib = [0.0, 0.0, 0.0]
    cal.wideFromDeviceEuler = [0.0, 0.0, 0.0]
    model = msgs['modelV2'].modelV2
    for key in ('position', 'velocity', 'acceleration', 'orientation'):
      v = getattr(model, key)
      v.x = [float(i * 3) if key == 'position' else 15.0 if key == 'velocity' else 0.0 for i in range(33)]
      v.y = [0.0] * 33
      v.z = [0.0] * 33
    for key, n in (('laneLines', 4), ('roadEdges', 2)):
      for i, line in enumerate(model.init(key, n)):
        line.x = [float(i * 3) for i in range(33)]
        line.y = [(i - (n - 1) / 2) * 3.5] * 33
        line.z = [0.0] * 33
    model.laneLineProbs = [0.9] * 4
    model.roadEdgeStds = [0.2, 0.2]
    model.lanePolicy.valid = True
    model.lanePolicy.enabled = True
    model.lanePolicy.active = True
    model.lanePolicy.leftSource = 'painted'
    model.lanePolicy.rightSource = 'painted'

    def receive(_timeout):
      now = time.monotonic()
      for m in msgs.values():
        m.logMonoTime = int(now * 1e9)
      ui_state.sm.update_msgs(now, [m.as_reader() for m in msgs.values()])

    ui_state.sm.update = receive
    for _cycle in range(3):
      msgs['deviceState'].deviceState.started = False
      msgs['pandaStates'].pandaStates[0].ignitionLine = False
      ui_state.update()
      assert not ui_state.started
      layout.render(rect)
      msgs['deviceState'].deviceState.started = True
      msgs['pandaStates'].pandaStates[0].ignitionLine = True
      for _frame in range(3):
        ui_state.update()
        assert ui_state.started
        if gui_app.big_ui():
          road = layout._layouts[2]
        else:
          road = layout._car_onroad_layout
        road._content_rect = rect
        road._calc_frame_matrix(rect)
        road.render(rect)
        layout.render(rect)
    print('onroad transitions rendered', flush=True)
    gui_app.close()


if __name__ == '__main__':
  run_startup_smoke()
