import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, call, patch

import pyray as rl

from openpilot.selfdrive.controls.tests.test_conditional_experimental import FakeSM
from openpilot.selfdrive.ui.onroad import gap_button as gap


class TestGapButton(unittest.TestCase):
  def setUp(self):
    self.sm = FakeSM(selfdriveState=NS(personality=NS(raw=1)))
    self.state = NS(sm=self.sm, started_frame=1, has_longitudinal_control=True, params=Mock())
    self.addCleanup(patch.stopall)
    patch.object(gap, 'ui_state', self.state).start()
    self.clock = patch.object(gap.time, 'monotonic', return_value=100.).start()
    self.button = gap.GapButton()
    self.button._update_state()

  def tap(self):
    self.button._handle_mouse_release(rl.Vector2(0, 0))

  def colors(self):
    with patch.object(gap, 'draw_lane_icon_frame'), patch.object(rl, 'draw_rectangle_rounded') as draw:
      self.button._render(rl.Rectangle(0, 0, 192, 192))
    return [(c.args[-1].r, c.args[-1].g, c.args[-1].b) for c in draw.call_args_list]

  def test_confirmed_personalities_fill_from_bottom(self):
    green, grey = (13, 248, 122), (110, 110, 110)
    for personality, colors in ((0, [grey, grey, green]), (1, [grey, green, green]), (2, [green] * 3)):
      with self.subTest(personality=personality):
        self.sm['selfdriveState'].personality.raw = personality
        self.button._update_state()
        self.assertEqual(self.colors(), colors)

  def test_rapid_taps_cycle_without_faking_reported_state(self):
    before = self.colors()
    for _ in range(3):
      self.tap()
    self.assertEqual(self.state.params.put.call_args_list,
                     [call('LongitudinalPersonality', p) for p in (0, 2, 1)])
    self.assertEqual(self.colors(), before)

  def test_stock_button_change_updates_display_and_next_tap(self):
    self.tap()  # request aggressive, still awaiting acknowledgement
    self.sm['selfdriveState'].personality.raw = 2  # a stock button changes the actual state
    self.button._update_state()
    self.assertEqual(self.colors(), [(13, 248, 122)] * 3)
    self.tap()
    self.state.params.put.assert_called_with('LongitudinalPersonality', 1)

  def test_missing_acknowledgement_expires(self):
    self.tap()
    self.clock.return_value = 100.6
    self.sm.recv_time['selfdriveState'] = 100.6
    self.tap()
    self.assertEqual(self.state.params.put.call_args_list, [call('LongitudinalPersonality', 0)] * 2)

  def test_stale_invalid_and_previous_drive_state_disable_taps(self):
    for field, value in (('recv_time', 99.), ('valid', False), ('recv_frame', 0)):
      with self.subTest(field=field):
        original = getattr(self.sm, field)['selfdriveState']
        getattr(self.sm, field)['selfdriveState'] = value
        self.tap()
        self.assertFalse(self.button.enabled)
        self.assertEqual(self.colors(), [(110, 110, 110)] * 3)
        getattr(self.sm, field)['selfdriveState'] = original
    self.state.params.put.assert_not_called()

  def test_stock_acc_hides_and_disables_button(self):
    self.state.has_longitudinal_control = False
    self.tap()
    self.assertFalse(self.button.is_visible)
    self.assertFalse(self.button.enabled)
    self.state.params.put.assert_not_called()
