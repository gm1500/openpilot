"""Cruise limits, source handoffs, and delayed speed-control regression tests."""
import math
import unittest
from collections import deque
from unittest.mock import patch

import numpy as np

from openpilot.selfdrive.controls.lib.drive_helpers import smooth_value
from openpilot.selfdrive.controls.lib.longitudinal_planner import (
  A_CRUISE_MIN, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS, J_CRUISE_BP, J_CRUISE_VALS, get_cruise_accel,
)
from openpilot.selfdrive.controls.tests import test_longitudinal_modes as modes


def native_cruise_accel(target, speed, previous, selected, dt, **_):
  """Cheddar's 2 m/s², one-second-filter reference before Batmobile changes."""
  raw = np.clip(smooth_value(target - speed, previous, tau=1., dt=dt), A_CRUISE_MIN, 2.)
  jerk = np.interp(speed, J_CRUISE_BP, J_CRUISE_VALS)
  return float(np.clip(raw, min(previous, selected) - jerk * dt, previous + jerk * dt))


def mapped_cruise_accel(target, speed, previous, selected, dt):
  return get_cruise_accel(target, speed, previous, selected, dt,
                          max_accel=float(np.interp(speed, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)))


def simulate(policy, target_kph, *, start_kph=None, initial_accel=0., delay=.5, duration=45.,
             roadload=0., speed_noise=0.):
  """20 Hz planner and 100 Hz delayed acceleration plant with 150 ms lag.

  This checks cruise arrival and stability, not vehicle torque calibration or
  road performance. Changing the policy also changes the simulated speed.
  """
  dt = .01
  speed = (target_kph - 25. if start_kph is None else start_kph) / 3.6
  accel = selected = cruise = initial_accel
  delayed = deque([initial_accel] * round(delay / dt))
  rows = []
  for i in range(round(duration / dt)):
    now = i * dt
    if i % 5 == 0:
      measured_speed = speed + speed_noise * math.sin(2. * math.pi * 1.7 * now)
      cruise = policy(target_kph / 3.6, measured_speed, cruise, selected, .05)
      selected = cruise
    delayed.append(selected)
    accel += (delayed.popleft() - accel) * (1. - math.exp(-dt / .15))
    disturbance = roadload * math.sin(2. * math.pi * now / 12.)
    speed = max(0., speed + (accel + disturbance) * dt)
    if i % 5 == 0:
      rows.append((i * dt, speed * 3.6, selected))
  return np.array(rows)


class TestCruiseApproach(unittest.TestCase):
  def harness(self, speed, *, fallback=False):
    h = modes.TestLongitudinalModes()
    h.setUp()
    h.sm['carState'].vEgo = speed / 3.6
    h.sm['carState'].vCruise = min(speed + 30., 145.)
    h.set_model_speed(speed / 3.6)
    h.sm['modelV2'].action.desiredAcceleration = 2.
    h.sm['mapCruiseState'].e2eFallback = fallback
    h.planner.v_desired_filter.x = speed / 3.6
    h.planner.a_cruise = h.planner.output_a_target = 2.
    # Give the real MPC a consistent previous trajectory before testing source
    # handoffs; its cold solver state otherwise dominates the first plan.
    for _ in range(80):
      h.tick()
    return h

  def test_original_chill_limits_across_speeds_and_native_e2e_ceiling(self):
    # Exact archived curve: 1.6/1.2/.8/.6 at 0/10/25/40 m/s.
    for speed, limit in ((0., 1.6), (36., 1.2), (60., 1.0222222222), (90., .8), (120., .6888888889), (144., .6)):
      for fallback in (False, True):
        with self.subTest(speed=speed, fallback=fallback):
          h = self.harness(speed, fallback=fallback)
          plan = h.tick()
          cap = 2. if fallback else limit
          self.assertAlmostEqual(float(np.interp(speed / 3.6, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)), limit)
          self.assertLessEqual(plan.aTarget, cap + 1e-6)
          self.assertLessEqual(h.planner.a_cruise, cap + 1e-6)
          if 0. < speed < 140.:
            self.assertAlmostEqual(plan.aTarget, cap, places=5)

  def test_recovery_cannot_expose_hidden_positive_cruise_history(self):
    h = self.harness(82., fallback=True)
    h.sm['carState'].vCruise = 105.
    h.sm['modelV2'].action.desiredAcceleration = .264
    previous = h.tick().aTarget
    self.assertAlmostEqual(previous, .314, places=6)  # Standard E2E adds its native 0.05 boost
    h.sm['carState'].vCruise = 110.
    h.sm['mapCruiseState'].e2eFallback = False
    plan = h.tick()
    jerk = np.interp(82. / 3.6, J_CRUISE_BP, J_CRUISE_VALS)
    self.assertGreater(plan.aTarget, previous)
    self.assertLessEqual(plan.aTarget - previous, jerk * .05 + 1e-6)
    self.assertFalse(plan.slcE2eFallback)

  def test_recovery_applies_lower_ceiling_even_with_large_previous_output(self):
    h = self.harness(60., fallback=True)
    self.assertAlmostEqual(h.tick().aTarget, 2., places=6)
    h.sm['mapCruiseState'].e2eFallback = False
    plan = h.tick()
    self.assertAlmostEqual(plan.aTarget, 1.0222222222, places=5)
    self.assertLessEqual(h.planner.a_cruise, 1.022223)

  def test_map_loss_raises_available_ceiling_without_acceleration_step(self):
    h = self.harness(80.)
    previous = h.tick().aTarget
    h.sm['carState'].vCruise = 105.
    h.sm['mapCruiseState'].e2eFallback = True
    plan = h.tick()
    jerk = np.interp(80. / 3.6, J_CRUISE_BP, J_CRUISE_VALS)
    self.assertTrue(plan.slcE2eFallback)
    self.assertGreater(plan.aTarget, previous)
    self.assertLessEqual(plan.aTarget - previous, jerk * .05 + 1e-6)

  def test_final_mode_ceiling_also_covers_a_positive_mpc_selection(self):
    for fallback, expected in ((False, 1.0222222222), (True, 1.5)):
      with self.subTest(fallback=fallback):
        h = self.harness(60., fallback=fallback)
        # Isolate candidate arbitration from cruise's own ceiling: a future
        # cruise change must not allow another positive source to evade it.
        with patch('openpilot.selfdrive.controls.lib.longitudinal_planner.get_cruise_accel', return_value=2.), \
             patch('openpilot.selfdrive.controls.lib.longitudinal_planner.get_accel_from_plan', return_value=1.5):
          plan = h.tick()
        self.assertAlmostEqual(plan.aTarget, expected, places=5)

  def test_engagement_and_pedal_release_cannot_leave_an_above_limit_state(self):
    h = self.harness(60.)
    for gas_pressed in (True, False):
      h.sm['carState'].gasPressed = gas_pressed
      h.sm['carState'].aEgo = 2.
      h.sm['controlsState'].longControlState = 'off' if gas_pressed else 'pid'
      h.planner.a_cruise = h.planner.output_a_target = 2.
      with self.subTest(gas_pressed=gas_pressed):
        plan = h.tick()
        self.assertLessEqual(plan.aTarget, 1.022223)
        self.assertLessEqual(h.planner.a_cruise, 1.022223)

  def test_clear_lane_release_does_not_copy_previous_lead_braking(self):
    # A vanished lead must not leave its -2 m/s² command in the cruise history.
    accel = mapped_cruise_accel(60. / 3.6, 50. / 3.6, 2., -2., .05)
    self.assertGreaterEqual(accel, 0.)
    self.assertLessEqual(accel, .06)
    # Genuine cruise overspeed continues to request its own deceleration.
    for _ in range(20):
      accel = mapped_cruise_accel(40. / 3.6, 60. / 3.6, accel, accel, .05)
    self.assertAlmostEqual(accel, A_CRUISE_MIN)

  def test_normal_positive_rise_remains_jerk_limited(self):
    for speed in (10., 20., 30.):
      for selected in (0., .2, .8):
        with self.subTest(speed=speed, selected=selected):
          accel = get_cruise_accel(speed + 20., speed, 2., selected, .05)
          jerk = np.interp(speed, J_CRUISE_BP, J_CRUISE_VALS)
          self.assertLessEqual(accel - selected, jerk * .05 + 1e-9)

  def test_delayed_arrivals_across_city_and_highway_speeds(self):
    for target in (40., 60., 80., 100., 120.):
      for delay in (.3, .5, .8):
        with self.subTest(target=target, delay=delay):
          before = simulate(native_cruise_accel, target, delay=delay)
          after = simulate(mapped_cruise_accel, target, delay=delay)
          self.assertLess(np.max(after[:, 1]), np.max(before[:, 1]))
          self.assertLess(np.max(after[:, 1]) - target, 1.5 if delay <= .5 else 2.5)
          self.assertLess(np.max(np.abs(after[-200:, 1] - target)), .1)
          self.assertLess(np.ptp(after[-200:, 2]), .02)

  def test_varying_roadload_and_speed_noise_do_not_create_growing_oscillation(self):
    for target in (60., 120.):
      with self.subTest(target=target):
        rows = simulate(mapped_cruise_accel, target, start_kph=target, duration=60.,
                        roadload=.15, speed_noise=.03)
        # The disturbance is intentionally not pre-compensated. The controller
        # must make real corrections while keeping speed and demand bounded.
        final = rows[rows[:, 0] >= 30.]
        self.assertGreater(np.ptp(final[:, 1]), .1)
        self.assertLess(np.ptp(final[:, 1]), 2.)
        self.assertLess(np.ptp(final[:, 2]), .7)
        self.assertLess(np.max(np.abs(final[:, 1] - target)), 1.)

  def test_late_lower_set_cannot_erase_physical_acceleration(self):
    rows = simulate(mapped_cruise_accel, 60., start_kph=59., initial_accel=2.)
    self.assertLessEqual(rows[0, 2], 1.03)
    self.assertGreater(np.max(rows[:, 1]), 61.)
    self.assertLess(np.max(np.abs(rows[-200:, 1] - 60.)), .1)

  def test_real_lead_and_e2e_braking_remain_stronger_than_cruise_ceiling(self):
    for source in ('lead', 'e2e'):
      plans = []
      for policy in (native_cruise_accel, get_cruise_accel):
        h = self.harness(60., fallback=source == 'e2e')
        if source == 'lead':
          lead = h.sm['radarState'].leadOne
          lead.present, lead.dRel, lead.vLead, lead.vRel = True, 12., 2., 2. - 60. / 3.6
          lead.aLeadTau, lead.modelProb = 1.5, .99
        else:
          h.sm['modelV2'].action.desiredAcceleration = -2.
        with patch('openpilot.selfdrive.controls.lib.longitudinal_planner.get_cruise_accel', policy):
          plans.append([h.tick() for _ in range(6)])
      with self.subTest(source=source):
        self.assertLess(plans[1][-1].aTarget, -.5)
        np.testing.assert_allclose([p.aTarget for p in plans[1]], [p.aTarget for p in plans[0]], atol=1e-6)
        self.assertEqual([p.longitudinalPlanSource for p in plans[1]], [p.longitudinalPlanSource for p in plans[0]])


if __name__ == '__main__':
  unittest.main()
