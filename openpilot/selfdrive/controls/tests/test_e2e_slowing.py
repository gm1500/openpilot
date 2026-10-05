import math
import unittest
from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.e2e_slowing import E2ESlowingAssist, RELEASE_JERK, get_model_lead_speed
from openpilot.selfdrive.ui.onroad.e2e_assist import slowing_assist_color


class TestE2ESlowingAssist(unittest.TestCase):
  def setUp(self):
    self.assist = E2ESlowingAssist(0.05)

  def step(self, **kwargs):
    args = {'eligible': True, 'v_ego': 40 / 3.6, 'model_lead_speed': 30 / 3.6,
            'regular_accel': 0.5, 'e2e_accel': -0.5, 'e2e_stop': False}
    args['lead'] = SimpleNamespace(present=True, dRel=20., vLead=40 / 3.6, vRel=0.)
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
    invalid_lead = SimpleNamespace(present=True, dRel=math.nan, vLead=10., vRel=0.)
    for invalid in ({'eligible': False}, {'v_ego': math.nan}, {'e2e_accel': math.nan}, {'e2e_accel': math.inf}, {'lead': invalid_lead}):
      self.enter()
      self.assertEqual(self.step(**invalid), (0.5, False, False))
      self.assertFalse(self.assist.braking)
      self.assertIsNone(self.assist.accel_limit)

  def test_gate_uses_model_lead_speed_instead_of_ego_or_derived_speed(self):
    self.assertEqual(self.enter(v_ego=20 / 3.6, model_lead_speed=60 / 3.6), (0.5, False, False))
    self.assertEqual(self.enter(model_lead_speed=55 / 3.6), (0.5, False, False))
    self.assertEqual(self.enter(v_ego=80 / 3.6, model_lead_speed=54.9 / 3.6), (-0.5, False, True))

  def test_lead_speed_crossing_keeps_latch_until_go_then_requires_new_qualification(self):
    self.enter(model_lead_speed=54.9 / 3.6)
    for speed in (55., 54., 60., 54., 70.):
      self.assertEqual(self.step(model_lead_speed=speed / 3.6), (-0.5, False, True))
      self.assertTrue(self.assist.braking)
    self.assertEqual(self.step(model_lead_speed=60 / 3.6, regular_accel=-3.), (-3., False, False))
    self.assertTrue(self.assist.braking)
    previous = -3.
    for _ in range(80):
      accel, _, _ = self.step(model_lead_speed=60 / 3.6, e2e_accel=.3)
      self.assertLessEqual(accel - previous, RELEASE_JERK * 0.05 + 1e-8)
      previous = accel
    self.assertFalse(self.assist.braking)
    for _ in range(10):
      self.assertEqual(self.step(model_lead_speed=60 / 3.6), (0.5, False, False))
    self.assertEqual(self.step(model_lead_speed=55 / 3.6), (0.5, False, False))
    self.assertEqual(self.step(model_lead_speed=54.9 / 3.6), (0.5, False, False))
    self.assertEqual(self.step(model_lead_speed=55 / 3.6), (0.5, False, False))
    self.assertEqual(self.enter(model_lead_speed=54.9 / 3.6), (-0.5, False, True))

  def test_missing_or_nonfinite_model_speed_cannot_enter_and_releases_existing_cap(self):
    for speed in (None, math.nan, math.inf, -math.inf):
      self.assist.reset()
      self.assertEqual(self.enter(model_lead_speed=speed), (0.5, False, False))
      self.enter()
      accel, stop, _ = self.step(model_lead_speed=speed)
      self.assertAlmostEqual(accel, -0.45)
      self.assertFalse(stop)
      self.assertFalse(self.assist.braking)

  def test_far_paced_lead_does_not_inherit_scene_braking(self):
    lead = SimpleNamespace(present=True, dRel=90., vLead=60 / 3.6, vRel=0.)
    for _ in range(100):
      self.assertEqual(self.step(lead=lead, v_ego=60 / 3.6, e2e_accel=-2), (0.5, False, False))

  def test_far_stopped_lead_retains_full_assistance(self):
    lead = SimpleNamespace(present=True, dRel=100., vLead=0., vRel=-60 / 3.6)
    self.assertEqual(self.enter(lead=lead, v_ego=60 / 3.6, e2e_accel=-2), (-2., False, True))

  def test_gap_weight_varies_continuously(self):
    results = []
    for gap in (35., 40., 44.):
      self.assist.reset()
      lead = SimpleNamespace(present=True, dRel=gap, vLead=40 / 3.6, vRel=0.)
      results.append(self.enter(lead=lead)[0])
    self.assertLess(results[0], results[1])
    self.assertLess(results[1], results[2])
    self.assertGreater(results[0], -0.5)
    self.assertLess(results[2], 0.5)

  def test_relevance_loss_releases_gradually(self):
    previous, _, _ = self.enter(e2e_accel=-2)
    lead = SimpleNamespace(present=True, dRel=90., vLead=40 / 3.6, vRel=0.)
    for _ in range(100):
      current, _, _ = self.step(lead=lead, e2e_accel=-2)
      self.assertLessEqual(current - previous, RELEASE_JERK * .05 + 1e-8)
      previous = current
    self.assertEqual(current, 0.)  # no propulsion while the model still requests strong slowing
    for _ in range(30):
      current, _, _ = self.step(lead=lead, e2e_accel=-.05)
    self.assertEqual(current, .5)

  def test_confirmed_pull_away_releases_near_zero_e2e(self):
    self.enter()
    for i in range(25):
      lead = SimpleNamespace(present=True, dRel=25. + .1 * i, vLead=40 / 3.6 + 2., vRel=2.)
      self.step(lead=lead, model_lead_speed=60 / 3.6, e2e_accel=.05)
    self.assertFalse(self.assist.braking)
    self.assertGreater(self.step(lead=lead, e2e_accel=.05)[0], .1)

  def test_no_early_release_from_velocity_alone_or_range_jump(self):
    for jump in (False, True):
      self.assist.reset()
      self.enter()
      for i in range(30):
        lead = SimpleNamespace(present=True, dRel=25. + (8. if jump and i >= 15 else 0.), vLead=14., vRel=2.)
        self.step(lead=lead, e2e_accel=.05)
      self.assertTrue(self.assist.braking)

  def test_lead_loss_handoff_and_immediate_override(self):
    absent = SimpleNamespace(present=False)
    previous, _, _ = self.enter(e2e_accel=-1.)
    for _ in range(10):
      current, stop, active = self.step(lead=absent, regular_accel=1.5, e2e_accel=-3.)
      self.assertAlmostEqual(current - previous, RELEASE_JERK * .05)
      self.assertFalse(stop)
      self.assertTrue(active)
      previous = current
    self.assertEqual(self.step(eligible=False, lead=absent, regular_accel=1.5), (1.5, False, False))
    self.assertEqual(self.step(lead=absent, regular_accel=1.5), (1.5, False, False))

  def test_new_close_lead_interrupts_lead_loss_release(self):
    self.enter(e2e_accel=-1.)
    self.step(lead=SimpleNamespace(present=False), regular_accel=1.5)
    self.assertEqual(self.enter(e2e_accel=-3.), (-3., False, True))
    self.assertEqual(self.step(lead=SimpleNamespace(present=False), regular_accel=-3.5), (-3.5, False, False))

  def test_lead_loss_at_rest_preserves_fresh_latched_stop(self):
    self.enter(v_ego=0., e2e_stop=True)
    for _ in range(20):
      accel, stop, _ = self.step(lead=SimpleNamespace(present=False), v_ego=0., e2e_accel=.05, e2e_stop=True)
      self.assertLessEqual(accel, 0.)
      self.assertTrue(stop)
    self.assertEqual(self.step(eligible=False, v_ego=0., e2e_stop=True), (.5, False, False))


class TestModelLeadSpeed(unittest.TestCase):
  def test_ego_speed_correction_and_missing_predictions(self):
    model = SimpleNamespace(leadsV3=[SimpleNamespace(prob=.9, v=[8.])], velocity=SimpleNamespace(x=[10.]))
    self.assertEqual(get_model_lead_speed(model, 12.), 10.)
    model.leadsV3[0].v = [-3.]
    self.assertEqual(get_model_lead_speed(model, 10.), 0.)
    for values in ([], [math.nan], [math.inf]):
      model.leadsV3[0].v = values
      self.assertIsNone(get_model_lead_speed(model, 12.))
    model.leadsV3[0].v = [8.]
    for probability in (0., .5, math.nan, math.inf):
      model.leadsV3[0].prob = probability
      self.assertIsNone(get_model_lead_speed(model, 12.))
    model.leadsV3[0].prob = .9
    for values in ([], [math.nan], [math.inf]):
      model.velocity.x = values
      self.assertIsNone(get_model_lead_speed(model, 12.))
    model.velocity.x = [10.]
    self.assertIsNone(get_model_lead_speed(model, math.nan))
    model.leadsV3 = []
    self.assertIsNone(get_model_lead_speed(model, 12.))


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
