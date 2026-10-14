import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import pyray as rl

from openpilot.selfdrive.ui.onroad.gap_lane_gesture import GapLaneGesture
from openpilot.selfdrive.ui.onroad import gap_lane_button as button
from openpilot.system.ui.lib.application import MouseEvent, MousePos


def event(t, kind='down', slot=0, inside=True):
  return MouseEvent(MousePos(40, 40) if inside else MousePos(400, 400), slot,
                    kind == 'press', kind == 'release', kind != 'release', t)


class TestGapLaneGesture(unittest.TestCase):
  def setUp(self):
    self.gesture = GapLaneGesture()

  def send(self, t, kind='down', slot=0, inside=True):
    return self.gesture.event(event(t, kind, slot, inside), inside)

  def test_tap_and_following_hold_are_distinct(self):
    self.send(10, 'press')
    self.assertEqual(self.send(10.2, 'release'), 'tap')
    self.send(11, 'press')
    self.assertEqual(self.gesture.advance(11.7), 'hold')
    self.assertTrue(self.gesture.pressed)
    self.assertIsNone(self.gesture.advance(15))
    self.assertIsNone(self.send(15.1, 'release'))
    self.assertFalse(self.gesture.pressed)

  def test_queued_long_press_release_is_only_hold(self):
    self.send(10, 'press')
    self.assertEqual(self.send(11, 'release'), 'hold')
    self.assertIsNone(self.gesture.advance(12))

  def test_short_queued_press_does_not_become_hold_after_render_stall(self):
    self.send(10, 'press')
    self.assertEqual(self.send(10.1, 'release'), 'tap')
    self.assertIsNone(self.gesture.advance(15))

  def test_exit_and_reentry_cannot_tap_or_hold(self):
    self.send(10, 'press')
    self.send(10.1, inside=False)
    self.send(10.2)
    self.assertIsNone(self.gesture.advance(12))
    self.assertIsNone(self.send(12.1, 'release'))

  def test_second_touch_cancels_until_all_contacts_release(self):
    self.send(10, 'press')
    self.send(10.1, 'press', slot=1)
    self.send(10.2, 'release', slot=1)
    self.assertIsNone(self.gesture.advance(12))
    self.assertIsNone(self.send(12.1, 'release'))
    self.send(13, 'press')
    self.assertEqual(self.send(13.1, 'release'), 'tap')

  def test_show_hide_or_down_without_press_cannot_fire(self):
    self.send(10, 'press')
    self.gesture.reset()
    self.send(10.5)
    self.assertIsNone(self.gesture.advance(12))
    self.assertIsNone(self.send(12.1, 'release'))


class TestGapLaneButton(unittest.TestCase):
  def setUp(self):
    self.now = 100.
    self.sm = {'selfdriveState': NS(personality=NS(raw=1))}
    self.sm = type('SM', (dict,), {})(self.sm)
    self.sm.valid = {'selfdriveState': True}
    self.sm.recv_frame = {'selfdriveState': 10}
    self.sm.recv_time = {'selfdriveState': self.now}
    self.ui = NS(sm=self.sm, started=True, started_frame=1, has_longitudinal_control=True,
                 lane_policy_enabled=True, params=Mock(), set_lane_policy_enabled=Mock())
    self.ui.params.get.return_value = True
    self.app = NS(mouse_events=[])
    for target, value in (('ui_state', self.ui), ('gui_app', self.app), ('device', NS(awake=True))):
      p = patch.object(button, target, value)
      p.start()
      self.addCleanup(p.stop)
    p = patch.object(button.time, 'monotonic', side_effect=lambda: self.now)
    p.start()
    self.addCleanup(p.stop)
    self.widget = button.GapLaneButton()
    self.widget.set_rect(rl.Rectangle(0, 0, 192, 192))
    self.widget._update_state()

  def frame(self, now, *events):
    self.now = now
    self.app.mouse_events = list(events)
    self.widget._update_state()
    self.widget._process_mouse_events()

  def test_long_hold_toggles_only_once_and_release_does_not_change_gap(self):
    self.frame(100., event(100., 'press'))
    self.frame(100.7)
    self.frame(102.)
    self.frame(102.1, event(102.1, 'release'))
    self.ui.set_lane_policy_enabled.assert_called_once_with(False)
    self.ui.params.put.assert_not_called()

  def test_batch_long_release_does_not_change_gap(self):
    self.frame(101., event(100., 'press'), event(100.8, 'release'))
    self.ui.set_lane_policy_enabled.assert_called_once_with(False)
    self.ui.params.put.assert_not_called()

  def test_three_quick_taps_cycle_pending_requests_without_faking_display(self):
    for i, expected in enumerate((0, 2, 1)):
      self.frame(100. + i * .1, event(100. + i * .1, 'press'), event(100.05 + i * .1, 'release'))
      self.ui.params.put.assert_called_with('LongitudinalPersonality', expected)
      self.assertEqual(self.widget._personality, 1)
    self.ui.set_lane_policy_enabled.assert_not_called()

  def test_stale_or_unavailable_gap_still_allows_lane_hold(self):
    self.ui.has_longitudinal_control = False
    self.frame(100.1, event(100., 'press'), event(100.1, 'release'))
    self.ui.params.put.assert_not_called()
    self.frame(101., event(100.2, 'press'), event(101., 'release'))
    self.ui.set_lane_policy_enabled.assert_called_once_with(False)

  def test_physical_gap_change_clears_pending_and_display_follows_ack(self):
    self.frame(100.1, event(100., 'press'), event(100.1, 'release'))
    self.sm['selfdriveState'].personality.raw = 2
    self.frame(100.2)
    self.assertEqual(self.widget._personality, 2)
    self.assertIsNone(self.widget._requested_personality)
    self.frame(100.3, event(100.2, 'press'), event(100.3, 'release'))
    self.ui.params.put.assert_called_with('LongitudinalPersonality', 1)

  def test_hide_and_offroad_cancel_contact(self):
    self.frame(100., event(100., 'press'))
    self.widget.hide_event()
    self.frame(100.7, event(100.7, 'release'))
    self.frame(100.8, event(100.8, 'press'))
    self.ui.started = False
    self.frame(102.)
    self.ui.set_lane_policy_enabled.assert_not_called()
    self.ui.params.put.assert_not_called()

  def test_bar_count_lane_colour_and_thicker_geometry(self):
    for personality in (0, 1, 2, None):
      for lane in (False, True):
        with patch.object(rl, 'draw_circle'), patch.object(rl, 'draw_line_ex') as lines, \
             patch.object(rl, 'draw_rectangle_rounded') as bars:
          button.draw_gap_lane_icon(self.widget.rect, personality, lane)
        self.assertEqual(len(lines.call_args_list), 2)
        self.assertTrue(all(c.args[-1].g == (248 if lane else 110) for c in lines.call_args_list))
        self.assertEqual(sum(c.args[-1].g == 248 for c in bars.call_args_list), 0 if personality is None else personality + 1)
        self.assertTrue(all(c.args[0].height > 192 * .065 for c in bars.call_args_list))
