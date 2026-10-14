import unittest

from openpilot.selfdrive.ui.onroad.resume_set_gesture import ResumeSetGesture


class TestResumeSetGesture(unittest.TestCase):
  def test_release_and_slide_do_not_restart_on_reentry(self):
    for button in (2, 3):
      g = ResumeSetGesture()
      g.event(0, button, True, False, 100, True)
      self.assertEqual(g.button, button)
      g.event(0, 0, False, False, 110, True)
      self.assertEqual(g.button, 0)
      g.event(0, button, False, False, 120, True)
      self.assertEqual(g.button, 0)
      g.event(0, 0, False, True, 130, True)
      g.event(0, button, True, False, 140, True)
      self.assertEqual(g.button, button)
      self.assertEqual(g.request_id, 140)

  def test_switch_and_multitouch_cancel(self):
    g = ResumeSetGesture()
    g.event(0, 2, True, False, 100, True)
    g.event(0, 3, False, False, 110, True)
    self.assertEqual(g.button, 0)
    g.event(1, 3, True, False, 120, True)
    self.assertEqual(g.button, 0)
    g.event(0, 0, False, True, 130, True)
    g.event(1, 3, False, False, 140, True)
    self.assertEqual(g.button, 0)

  def test_press_when_blocked_never_arms_later(self):
    g = ResumeSetGesture()
    g.event(0, 2, True, False, 100, False)
    g.event(0, 2, False, False, 110, True)
    self.assertEqual(g.button, 0)

  def test_input_thread_stall_release_and_second_touch_cancel_even_without_events(self):
    for now, poll, held in ((1., .8, (True, False)), (1., 1.1, (True, False)),
                            (1., 1., (False, False)), (1., 1., (True, True))):
      g = ResumeSetGesture()
      g.event(0, 2, True, False, 100, True)
      g.confirm_hold(now, poll, held)
      self.assertEqual(g.button, 0)
    g = ResumeSetGesture()
    g.event(0, 2, True, False, 100, True)
    g.confirm_hold(1., 1., (True, False))
    self.assertEqual(g.button, 2)


if __name__ == '__main__':
  unittest.main()
