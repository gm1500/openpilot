import unittest
from types import SimpleNamespace as NS

import numpy as np
from openpilot.cereal import log
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
  LongitudinalMpc, STOP_DISTANCE, LEAD_DANGER_FACTOR, T_IDXS,
)
from openpilot.selfdrive.controls.lib.drive_helpers import get_accel_from_plan


def radar(distance=None, second=False):
  def lead(d):
    return NS(present=d is not None, dRel=d or 0., vLead=0., aLeadK=0., aLeadTau=1.5, modelProb=.99 if d else 0.)
  return NS(leadOne=lead(None if second else distance), leadTwo=lead(distance if second else None))


class TestStopTargetMpc(unittest.TestCase):
  def test_gap_conversion_and_stop_constraint(self):
    mpc = LongitudinalMpc()
    mpc.set_cur_state(5., 0.)
    mpc.update(radar(), stop_distance=15.)
    np.testing.assert_allclose(mpc.params[:, 2], 15. + STOP_DISTANCE)
    np.testing.assert_allclose(mpc.params[:, 6], 15. + STOP_DISTANCE)
    np.testing.assert_allclose(mpc.params[:, 5], LEAD_DANGER_FACTOR)
    self.assertEqual(mpc.source, log.LongitudinalPlan.LongitudinalPlanSource.stopTarget)
    self.assertEqual(mpc.solution_status, 0)
    self.assertEqual(mpc.crash_cnt, 0)  # virtual target cannot invent FCW

  def test_closer_real_leads_keep_their_gap_policy(self):
    for second in (False, True):
      ordinary, target = LongitudinalMpc(), LongitudinalMpc()
      for mpc in (ordinary, target):
        mpc.set_cur_state(3., 0.)
      ordinary.update(radar(10., second))
      target.update(radar(10., second), stop_distance=20.)
      np.testing.assert_allclose(target.params[:, :6], ordinary.params[:, :6])
      # Compare the applied action; SQP's distant horizon can differ slightly
      # with an additional inactive inequality and a fixed iteration budget.
      target_action = get_accel_from_plan(target.v_solution, target.a_solution, T_IDXS, action_t=.55)
      ordinary_action = get_accel_from_plan(ordinary.v_solution, ordinary.a_solution, T_IDXS, action_t=.55)
      self.assertAlmostEqual(target_action, ordinary_action, delta=1e-4)
      self.assertEqual(target.source, ordinary.source)
      np.testing.assert_allclose(target.params[:, 5], LEAD_DANGER_FACTOR)

  def test_target_release_removes_constraint_and_preserves_real_lead(self):
    mpc = LongitudinalMpc()
    mpc.set_cur_state(4., 0.)
    mpc.update(radar(30.), stop_distance=5.)
    self.assertEqual(mpc.source, log.LongitudinalPlan.LongitudinalPlanSource.stopTarget)
    mpc.update(radar(30.))
    np.testing.assert_allclose(mpc.params[:, 2], 30.)
    np.testing.assert_allclose(mpc.params[:, 5], LEAD_DANGER_FACTOR)
    np.testing.assert_allclose(mpc.params[:, 6], 1e4)
    self.assertEqual(mpc.source, log.LongitudinalPlan.LongitudinalPlanSource.lead0)

  def test_real_lead_selection_cannot_remove_stop_constraint(self):
    mpc = LongitudinalMpc()
    mpc.set_cur_state(2., 0.)
    mpc.update(radar(10.), stop_distance=4.5)
    self.assertEqual(mpc.source, log.LongitudinalPlan.LongitudinalPlanSource.lead0)
    np.testing.assert_allclose(mpc.params[:, 2], 10.)
    np.testing.assert_allclose(mpc.params[:, 6], 10.5)

  def test_invalid_target_does_not_change_ordinary_plan(self):
    for distance in (None, -1., float('nan'), float('inf')):
      mpc = LongitudinalMpc()
      mpc.set_cur_state(3., 0.)
      mpc.update(radar(15.), stop_distance=distance)
      np.testing.assert_allclose(mpc.params[:, 2], 15.)
      np.testing.assert_allclose(mpc.params[:, 5], LEAD_DANGER_FACTOR)

  def test_closed_loop_stop_and_departure(self):
    # Ideal acceleration tracking isolates gap bookkeeping and stop/release.
    for speed, distance in ((5.9, 17.2), (8., 25.), (15., 60.), (0., 0.)):
      with self.subTest(speed=speed, distance=distance):
        mpc = LongitudinalMpc()
        travel, accel = 0., 0.
        for _ in range(600):
          mpc.set_weights(speed > .05)
          mpc.set_cur_state(speed, accel)
          mpc.update(radar(), stop_distance=max(0., distance - travel))
          self.assertEqual(mpc.solution_status, 0)
          accel = float(np.clip(get_accel_from_plan(mpc.v_solution, mpc.a_solution, T_IDXS, action_t=.55), -3.5, 2.))
          next_speed = max(0., speed + accel * .05)
          travel += .5 * (speed + next_speed) * .05
          speed = next_speed
        self.assertLess(speed, .1)
        self.assertLess(abs(distance - travel), 1.)
        # Removing the target must permit launch; no fake lead remains.
        for _ in range(40):
          mpc.set_weights(False)
          mpc.set_cur_state(speed, accel)
          mpc.update(radar())
          accel = float(np.clip(get_accel_from_plan(mpc.v_solution, mpc.a_solution, T_IDXS, action_t=.55), -3.5, 2.))
          speed = max(0., speed + accel * .05)
        self.assertGreater(speed, 1.)


if __name__ == '__main__':
  unittest.main()
