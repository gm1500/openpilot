import math
import unittest
from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.e2e_slowing import E2ESlowingAssist, RELEASE_JERK
from openpilot.selfdrive.ui.onroad.e2e_assist import slowing_assist_color


class TestE2ESlowingAssist(unittest.TestCase):
  def setUp(self):
    self.assist = E2ESlowingAssist(0.05)

  def step(self, **kwargs):
    args = {'eligible': True, 'v_ego': 40 / 3.6, 'regular_accel': 0.5, 'e2e_accel': -0.5, 'e2e_stop': False}
    args.update(kwargs)
    return self.assist.update(**args)

  def enter(self, **kwargs):
    for _ in range(4):
      result = self.step(**kwargs)
    return result

  def test_entry_needs_sustained_slowing(self):
    self.assertEqual(self.step(), (0.5, False, False))
    self.assertEqual(self.step(e2e_accel=0), (0.5, False, False))
    for _ in range(100):
      self.assertEqual(self.step(e2e_accel=-0.1), (0.5, False, False))
    self.assertEqual(self.enter(), (-0.5, False, True))

  def test_regular_braking_always_wins(self):
    self.enter()
    self.assertEqual(self.step(regular_accel=-3), (-3, False, False))

  def test_no_constraint_when_regular_always_stronger(self):
    self.enter(regular_accel=-2)
    self.assertEqual(self.step(regular_accel=-0.3), (-0.5, False, True))

  def test_stronger_e2e_braking_is_immediate_after_entry(self):
    self.enter()
    self.assertEqual(self.step(e2e_accel=-2), (-2, False, True))

  def test_positive_e2e_does_not_cap_normal_catchup(self):
    for _ in range(100):
      self.assertEqual(self.step(e2e_accel=0.2), (0.5, False, False))

  def test_release_confirmation_and_bounded_handoff(self):
    previous, _, _ = self.enter()
    for i in range(40):
      accel, stop, _ = self.step(e2e_accel=0.3)
      self.assertLessEqual(accel - previous, RELEASE_JERK * 0.05 + 1e-8)
      self.assertFalse(stop)
      if i < 3:
        self.assertTrue(self.assist.braking)
        self.assertLessEqual(accel, 0)
      previous = accel
    self.assertFalse(self.assist.braking)
    self.assertEqual(self.step(e2e_accel=0.3), (0.5, False, False))

  def test_one_positive_frame_does_not_release(self):
    self.enter()
    self.step(e2e_accel=0.3)
    self.step(e2e_accel=-0.3)
    self.assertTrue(self.assist.braking)

  def test_new_braking_interrupts_handoff(self):
    self.enter()
    for _ in range(5):
      self.step(e2e_accel=0.3)
    self.assertFalse(self.assist.braking)
    self.assertEqual(self.enter(e2e_accel=-2), (-2, False, True))

  def test_stop_request_does_not_release_or_launch(self):
    self.enter()
    for _ in range(20):
      accel, stop, _ = self.step(v_ego=0, e2e_accel=0.05, e2e_stop=True)
      self.assertLessEqual(accel, 0)
      self.assertTrue(stop)
    self.assertEqual(self.step(eligible=False), (0.5, False, False))

  def test_override_or_invalid_input_discards_latch(self):
    for invalid in ({'eligible': False}, {'v_ego': math.nan}, {'e2e_accel': math.nan}, {'e2e_accel': math.inf}):
      self.enter()
      self.assertEqual(self.step(**invalid), (0.5, False, False))
      self.assertFalse(self.assist.braking)
      self.assertIsNone(self.assist.accel_limit)

  def test_high_speed_and_boundary_hysteresis(self):
    self.assertEqual(self.enter(v_ego=66 / 3.6), (0.5, False, False))
    self.assertEqual(self.enter(v_ego=64 / 3.6), (0.5, False, False))
    self.assertEqual(self.enter(v_ego=62 / 3.6), (-0.5, False, True))
    previous = -0.5
    for _ in range(30):
      accel, _, _ = self.step(v_ego=65 / 3.6)
      self.assertLessEqual(accel - previous, RELEASE_JERK * 0.05 + 1e-8)
      previous = accel
    self.assertEqual(self.step(v_ego=65 / 3.6), (0.5, False, False))


class TestE2EAssistColor(unittest.TestCase):
  def setUp(self):
    class SM(dict):
      pass
    self.sm = SM(longitudinalPlan=SimpleNamespace(e2eAssistActive=True, aTarget=-0.3),
                 carControl=SimpleNamespace(longActive=True),
                 carState=SimpleNamespace(gasPressed=False, brakePressed=False),
                 selfdriveState=SimpleNamespace(enabled=True, experimentalMode=False))
    self.sm.valid = dict.fromkeys(self.sm, True)
    self.sm.recv_frame = dict.fromkeys(self.sm, 20)
    self.sm.recv_time = dict.fromkeys(self.sm, 100.)
    self.sm.logMonoTime = dict.fromkeys(self.sm, int(100e9))

  def color(self, now=100.):
    return slowing_assist_color(self.sm, 10, now)

  def test_deceleration_colours(self):
    yellow = self.color()
    self.sm['longitudinalPlan'].aTarget = -1
    orange = self.color()
    self.sm['longitudinalPlan'].aTarget = -2
    red = self.color()
    self.assertEqual(yellow[0], orange[0])
    self.assertGreater(yellow[1], orange[1])
    self.assertGreater(orange[1], red[1])
    self.assertEqual(red[1], red[2])

  def test_clear_when_not_actually_slowing(self):
    cases = [('carControl', 'longActive', False), ('carState', 'gasPressed', True), ('carState', 'brakePressed', True),
             ('selfdriveState', 'enabled', False), ('selfdriveState', 'experimentalMode', True),
             ('longitudinalPlan', 'e2eAssistActive', False), ('longitudinalPlan', 'aTarget', 0),
             ('longitudinalPlan', 'aTarget', math.nan)]
    for service, field, value in cases:
      old = getattr(self.sm[service], field)
      setattr(self.sm[service], field, value)
      self.assertIsNone(self.color(), (service, field, value))
      setattr(self.sm[service], field, old)

  def test_stale_invalid_and_previous_drive_messages(self):
    self.assertIsNone(self.color(100.31))
    self.assertIsNone(self.color(99.9))
    for service in self.sm:
      for field, value in [('valid', False), ('recv_frame', 9), ('recv_time', 99.), ('logMonoTime', int(99e9))]:
        data = getattr(self.sm, field)
        old = data[service]
        data[service] = value
        self.assertIsNone(self.color(), (service, field))
        data[service] = old


if __name__ == '__main__':
  unittest.main()
