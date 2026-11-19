"""Cruise arrival regression: delayed acceleration must taper before SET."""
import math
import unittest
from collections import deque
from unittest.mock import patch

import numpy as np

from openpilot.selfdrive.controls.lib.drive_helpers import smooth_value
from openpilot.selfdrive.controls.lib.longitudinal_planner import (
  A_CRUISE_MIN, J_CRUISE_BP, J_CRUISE_VALS, get_cruise_accel,
)
from openpilot.selfdrive.controls.tests import test_longitudinal_modes as modes


def native_cruise_accel(target, speed, previous, selected, dt, **_):
  """Pre-fix Cheddar reference, retained to quantify the arrival regression."""
  raw = np.clip(smooth_value(target - speed, previous, tau=1., dt=dt), A_CRUISE_MIN, 2.)
  jerk = np.interp(speed, J_CRUISE_BP, J_CRUISE_VALS)
  return float(np.clip(raw, min(previous, selected) - jerk * dt, previous + jerk * dt))


def simulate(policy, target_kph, *, start_kph=None, initial_accel=0., delay=.5, duration=35.,
             grade=0., noise=0., lead_until=0., initial_cruise=None):
  """20 Hz planner, 100 Hz delayed plant with 150 ms actuator lag.

  A compensated constant grade has nonzero actuator effort but zero net aEgo.
  This isolates the cruise approach policy from the car-specific PID gains.
  """
  dt = .01
  speed = (target_kph - 25. if start_kph is None else start_kph) / 3.6
  accel = selected = initial_accel
  cruise = initial_accel if initial_cruise is None else initial_cruise
  plant_accel = initial_accel - grade
  delayed = deque([plant_accel] * round(delay / dt))
  rows = []
  for i in range(round(duration / dt)):
    now = i * dt
    if i % 5 == 0:
      measured_accel = accel + noise * math.sin(2. * math.pi * 2. * now)
      cruise = policy(target_kph / 3.6, speed, cruise, selected, .05,
                      a_ego=measured_accel, actuator_delay=.5)
      selected = min(cruise, -.3) if now < lead_until else cruise
    delayed.append(selected - grade)
    command = delayed.popleft()
    plant_accel += (command - plant_accel) * (1. - math.exp(-dt / .15))
    accel = plant_accel + grade
    speed = max(0., speed + accel * dt)
    if i % 5 == 0:
      rows.append((now, speed * 3.6, selected, cruise))
  return np.array(rows)


class TestCruiseApproach(unittest.TestCase):
  def test_far_from_set_retains_native_acceleration_and_jerk(self):
    for speed in (0., 10., 20., 30.):
      for previous in (0., 1., 2.):
        with self.subTest(speed=speed, previous=previous):
          args = (speed + 15., speed, previous, previous, .05)
          self.assertEqual(get_cruise_accel(*args, a_ego=1., actuator_delay=.5), native_cruise_accel(*args))

  def test_negative_cruise_target_and_nonpositive_ego_acceleration_are_unchanged(self):
    for measured in (-2., 0., 2.):
      args = (60. / 3.6, 61. / 3.6, -.2, -.2, .05)
      with self.subTest(measured=measured):
        self.assertEqual(get_cruise_accel(*args, a_ego=measured, actuator_delay=.5), native_cruise_accel(*args))
    args = (60. / 3.6, 55. / 3.6, .2, .2, .05)
    self.assertEqual(get_cruise_accel(*args, a_ego=-1., actuator_delay=.5), native_cruise_accel(*args))

  def test_guard_does_not_invent_braking_or_command_steps_at_set(self):
    for measured in (-.2, 0., .2, 2., float('nan')):
      self.assertEqual(get_cruise_accel(60. / 3.6, 60. / 3.6, 0., 0., .05,
                                       a_ego=measured, actuator_delay=.5), 0.)
    jerk = np.interp(60. / 3.6, J_CRUISE_BP, J_CRUISE_VALS)
    value = get_cruise_accel(60. / 3.6, 60. / 3.6, 2., 2., .05, a_ego=2., actuator_delay=.5)
    self.assertAlmostEqual(value, 2. - jerk * .05)

  def test_approaches_across_speeds_and_actuator_delays(self):
    for target in (60., 80., 100., 120.):
      for delay in (.3, .5, .8):
        with self.subTest(target=target, delay=delay):
          before = simulate(native_cruise_accel, target, initial_accel=1.5, delay=delay)
          after = simulate(get_cruise_accel, target, initial_accel=1.5, delay=delay)
          self.assertGreater(np.max(before[:, 1]) - target, 3.)
          self.assertLess(np.max(after[:, 1]) - target, .2)
          self.assertLess(np.max(np.abs(after[-200:, 1] - target)), .1)
          self.assertLess(np.ptp(after[-200:, 2]), .01)
          # Retain the speed-dependent jerk bound, including during the taper.
          jerks = np.abs(np.diff(after[:, 2])) / .05
          limits = np.interp(after[1:, 1] / 3.6, J_CRUISE_BP, J_CRUISE_VALS)
          self.assertTrue(np.all(jerks <= limits + .005))

  def test_launch_warm_cruise_grade_and_measurement_noise(self):
    for target in (60., 100.):
      with self.subTest(target=target):
        launch = simulate(get_cruise_accel, target, start_kph=0., duration=50.)
        self.assertLess(np.max(launch[:, 1]) - target, .2)
        warm = simulate(get_cruise_accel, target, start_kph=target - 5., initial_accel=.3, noise=.2, delay=.8)
        self.assertLess(np.max(warm[:, 1]) - target, .2)
        self.assertLess(np.max(np.abs(warm[-200:, 1] - target)), .1)
        self.assertLess(np.ptp(warm[-200:, 2]), .02)
        steady = simulate(get_cruise_accel, target, start_kph=target, grade=.3, noise=.2)
        np.testing.assert_allclose(steady[:, 1], target, atol=1e-8)
        np.testing.assert_allclose(steady[:, 2], 0., atol=1e-8)

  def test_late_set_change_cannot_remove_existing_acceleration_instantly(self):
    before = simulate(native_cruise_accel, 60., start_kph=59., initial_accel=2.)
    after = simulate(get_cruise_accel, 60., start_kph=59., initial_accel=2.)
    self.assertAlmostEqual(after[0, 2], before[0, 2])
    self.assertGreater(np.max(after[:, 1]), 65.)
    self.assertLessEqual(np.max(after[:, 1]), np.max(before[:, 1]) + .01)
    self.assertLess(np.max(np.abs(after[-200:, 1] - 60.)), .1)

  def test_remembered_acceleration_under_a_lead_tapers_after_release(self):
    rows = simulate(get_cruise_accel, 60., start_kph=52., initial_accel=-.3,
                    initial_cruise=2., lead_until=2.)
    before = simulate(native_cruise_accel, 60., start_kph=52., initial_accel=-.3,
                      initial_cruise=2., lead_until=2.)
    self.assertTrue(np.all(rows[rows[:, 0] < 2., 2] <= -.3))
    self.assertGreater(np.max(before[:, 1]), 65.)
    self.assertLess(np.max(rows[:, 1]), 60.5)
    self.assertLess(np.ptp(rows[-200:, 2]), .01)

  def test_real_lead_and_e2e_braking_still_win(self):
    for source in ('lead', 'e2e'):
      plans = []
      for policy in (native_cruise_accel, get_cruise_accel):
        h = modes.TestLongitudinalModes()
        h.setUp()
        h.sm['carState'].vEgo = h.sm['carState'].vCruise / 3.6
        h.sm['carState'].aEgo = 1.
        h.planner.a_cruise = 2.
        if source == 'lead':
          lead = h.sm['radarState'].leadOne
          lead.present, lead.dRel, lead.vLead, lead.vRel = True, 12., 2., -23.
          lead.aLeadTau, lead.modelProb = 1.5, .99
        else:
          h.sm['mapCruiseState'].e2eFallback = True
          h.sm['modelV2'].action.desiredAcceleration = -2.
        with patch('openpilot.selfdrive.controls.lib.longitudinal_planner.get_cruise_accel', policy):
          plans.append(h.tick())
      with self.subTest(source=source):
        self.assertLess(plans[1].aTarget, -.5)
        self.assertAlmostEqual(plans[1].aTarget, plans[0].aTarget)
        self.assertEqual(plans[1].longitudinalPlanSource, plans[0].longitudinalPlanSource)


if __name__ == '__main__':
  unittest.main()
