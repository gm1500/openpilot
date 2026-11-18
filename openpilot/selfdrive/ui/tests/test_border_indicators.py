import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

import pyray as rl

from openpilot.selfdrive.controls.tests.test_conditional_experimental import FakeSM
from openpilot.selfdrive.ui.onroad import border_indicators as border


class TestBorderIndicators(unittest.TestCase):
  def setUp(self):
    self.car = NS(brakePressed=False, leftBlindspot=False, rightBlindspot=False, leftBlinker=False, rightBlinker=False)
    self.sm = FakeSM(carState=self.car, carControl=NS(longActive=False), carOutput=NS(actuatorsOutput=NS(brake=0.)))

  def test_manual_and_applied_braking(self):
    self.car.brakePressed = True
    self.assertTrue(border.border_signals(self.sm, 1, 100.)[0])
    self.car.brakePressed = False
    self.sm['carControl'].longActive = True
    self.sm['carOutput'].actuatorsOutput.brake = 25.
    self.assertTrue(border.border_signals(self.sm, 1, 100.)[0])
    self.sm['carOutput'].actuatorsOutput.brake = 0.
    self.assertFalse(border.border_signals(self.sm, 1, 100.)[0])

  def test_stale_outputs_and_prior_drive_do_not_light_border(self):
    self.sm['carControl'].longActive = True
    self.sm['carOutput'].actuatorsOutput.brake = 25.
    self.sm.valid['carOutput'] = False
    self.assertFalse(border.border_signals(self.sm, 1, 100.)[0])
    self.car.brakePressed = self.car.leftBlindspot = True
    self.assertEqual(border.border_signals(self.sm, 11, 100.), (False,) * 5)
    self.assertEqual(border.border_signals(self.sm, 1, 100.5), (False,) * 5)

  def draw(self, now):
    for name in self.sm.services:
      self.sm.recv_time[name] = now
      self.sm.logMonoTime[name] = int(now * 1e9)
    with patch.object(rl, 'begin_scissor_mode'), patch.object(rl, 'end_scissor_mode'), \
         patch.object(rl, 'draw_rectangle_rounded_lines_ex') as draw:
      border.draw_border_indicators(rl.Rectangle(0, 0, 2160, 1080), rl.Rectangle(30, 30, 2100, 1020), .12, self.sm, 1, now)
    return [call.args[-1] for call in draw.call_args_list]

  def test_matching_signal_flashes_red_and_other_side_remains_orange(self):
    self.car.leftBlindspot = self.car.rightBlindspot = self.car.leftBlinker = True
    colors = self.draw(100.)
    self.assertTrue(any(c.r == border.BLINDSPOT_WARNING_COLOR.r for c in colors))
    self.assertTrue(any(c.r == border.BLINDSPOT_COLOR.r for c in colors))
    colors = self.draw(100.25)
    self.assertTrue(colors)
    self.assertTrue(all(c.r == border.BLINDSPOT_COLOR.r for c in colors))
    self.car.rightBlinker = True
    self.assertEqual(self.draw(100.25), [])

  def test_braking_remains_visible_during_bsm_off_phase(self):
    self.car.brakePressed = self.car.leftBlindspot = self.car.leftBlinker = True
    colors = self.draw(100.25)
    self.assertTrue(colors)
    self.assertTrue(all(c.r == border.BRAKE_COLOR.r for c in colors))
