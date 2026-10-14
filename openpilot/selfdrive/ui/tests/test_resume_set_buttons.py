"""Exercise the actual HUD publisher with simulated application/input surfaces."""
import importlib.util
import sys
import unittest
from pathlib import Path
from types import ModuleType, SimpleNamespace as NS
from unittest.mock import patch

import pyray as rl
from openpilot.cereal import log


class TestResumeSetButtons(unittest.TestCase):
  def setUp(self):
    class SM(dict):
      pass
    self.sm = SM({name: log.Event.new_message().init(name) for name in ('controlsState', 'carState')})
    self.sm.valid = self.sm.seen = dict.fromkeys(self.sm, True)
    self.sm.logMonoTime = dict.fromkeys(self.sm, 100_000_000_000)
    self.sm.recv_time = dict.fromkeys(self.sm, 100.)
    self.state = self.sm['controlsState'].resumeSetTest
    self.state.supported = self.state.ready = True
    self.state.cruiseState = 4
    self.sent = []
    sent = self.sent

    class Publisher:
      def __init__(self, services):
        self.services = services

      def send(self, service, msg):
        sent.append((service, msg.to_dict()))

    class Widget:
      def __init__(self):
        self.rect = rl.Rectangle(0, 0, 820, 210)

      def set_enabled(self, enabled):
        self.enabled = enabled

    def new_message(service):
      msg = log.Event.new_message(logMonoTime=100_000_000_000)
      msg.init(service)
      return msg

    self.input = NS(poll=100., held=(False, False))
    self.app = NS(font=lambda _: None, mouse_events=[], touch_hold_snapshot=lambda: (self.input.poll, self.input.held))
    self.ui = NS(started=True, sm=self.sm)
    self.device = NS(awake=True)
    surfaces = {
      'openpilot.cereal.messaging': {'PubMaster': Publisher, 'new_message': new_message},
      'openpilot.selfdrive.ui.ui_state': {'ui_state': self.ui, 'device': self.device},
      'openpilot.system.ui.lib.application': {'gui_app': self.app, 'FontWeight': NS(BOLD=1)},
      'openpilot.system.ui.lib.text_measure': {'measure_text_cached': lambda *_: rl.Vector2(0, 0)},
      'openpilot.system.ui.widgets': {'Widget': Widget},
    }
    modules = {}
    for name, values in surfaces.items():
      modules[name] = ModuleType(name)
      modules[name].__dict__.update(values)
    self.patch = patch.dict(sys.modules, modules)
    self.patch.start()
    self.addCleanup(self.patch.stop)
    path = Path(__file__).parents[1] / 'onroad' / 'resume_set_buttons.py'
    spec = importlib.util.spec_from_file_location('hud_resume_set_under_test', path)
    self.module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(self.module)
    self.panel = self.module.ResumeSetButtons()
    self.panel._update_state()  # released startup heartbeat

  def update(self, slot=0, x=100, pressed=False, released=False, held=(True, False)):
    self.input.held = held
    self.app.mouse_events = [NS(slot=slot, pos=rl.Vector2(x, 30), left_pressed=pressed, left_released=released)]
    with patch.object(self.module.time, 'monotonic', return_value=100.), patch.object(self.module.time, 'monotonic_ns', return_value=100_000_000_000):
      self.panel._update_state()
    return self.sent[-1][1]['resumeSetTestRequest']

  def test_each_button_publishes_selected_hold_and_release(self):
    for x, button in ((100, 2), (600, 3)):
      request = self.update(x=x, pressed=True)
      self.assertTrue(request['held'])
      self.assertEqual(request['button'], button)
      self.assertEqual(request['requestId'], 100_000_000_000)
      request = self.update(x=x, released=True, held=(False, False))
      self.assertFalse(request['held'])

  def test_active_hold_survives_ready_becoming_false(self):
    self.update(pressed=True)
    self.state.ready, self.state.active, self.state.button = False, True, 2
    self.update()
    self.assertEqual(self.panel.gesture.button, 2)
    self.state.active = False
    self.update()
    self.assertEqual(self.panel.gesture.button, 0)

  def test_input_failure_or_offroad_publishes_release(self):
    self.update(pressed=True)
    self.input.poll = 99.
    self.assertFalse(self.update()['held'])
    self.update(released=True, held=(False, False))
    self.input.poll = 100.
    self.update(pressed=True)
    self.ui.started = False
    self.assertFalse(self.update()['held'])

  def test_slide_to_other_button_and_second_finger_cannot_select_it(self):
    self.update(pressed=True)
    self.assertFalse(self.update(x=600)['held'])
    self.update(released=True, held=(False, False))
    self.update(pressed=True)
    self.assertFalse(self.update(slot=1, x=600, pressed=True, held=(True, True))['held'])


if __name__ == '__main__':
  unittest.main()
