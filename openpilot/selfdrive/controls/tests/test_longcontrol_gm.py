"""Controller regressions for the Silverado low-speed feedback tune.

The simple delayed plant checks feedback behavior, not vehicle stopping distance.
"""

from collections import deque
from types import SimpleNamespace
import unittest

import numpy as np

from opendbc.car import gen_empty_fingerprint
from opendbc.car.gm.interface import CarInterface
from opendbc.car.gm.values import CAR
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState


class TestGMLongControl(unittest.TestCase):
  @staticmethod
  def controller(previous=False):
    cp = CarInterface.get_params(CAR.CHEVROLET_SILVERADO, gen_empty_fingerprint(), [], True, False, False)
    if previous:
      cp.longitudinalTuning.kiBP = [5., 35.]
      cp.longitudinalTuning.kiV = [.05, .05]
    return LongControl(cp)

  @staticmethod
  def car(speed=1., accel=0., brake=False, cruise_standstill=False):
    return SimpleNamespace(vEgo=speed, aEgo=accel, brakePressed=brake,
                           cruiseState=SimpleNamespace(standstill=cruise_standstill))

  def response(self, previous, target, disturbance, lag, gain):
    controller = self.controller(previous)
    controller.pid.i = .08
    state = self.car(accel=target + .08 + disturbance)
    delay = deque([target + .08] * (round(lag / .01) + 1))
    values = []
    for _ in range(1500):
      command = controller.update(True, state, target, False, (-4., 2.))
      delay.append(command)
      state.aEgo += .01 / .3 * (gain * delay.popleft() + disturbance - state.aEgo)
      values.append((command, state.aEgo))
    return np.array(values)

  def test_feedback_corrects_under_and_over_braking(self):
    for disturbance in [-.2, .1]:
      for lag in [0., .3, .6]:
        for gain in [.8, 1., 1.2]:
          with self.subTest(disturbance=disturbance, lag=lag, gain=gain):
            before = self.response(True, -.4, disturbance, lag, gain)
            after = self.response(False, -.4, disturbance, lag, gain)
            old_error = np.mean(np.abs(before[-100:, 1] + .4))
            new_error = np.mean(np.abs(after[-100:, 1] + .4))
            self.assertLess(new_error, old_error)
            self.assertLess(new_error, .03)
            self.assertLess(np.ptp(after[-100:, 1]), .01)
            self.assertTrue(np.all(after[:, 0] < 0.))

  def test_launch_feedback_settles_without_braking(self):
    before = self.response(True, .5, -.1, .3, 1.)
    after = self.response(False, .5, -.1, .3, 1.)
    self.assertLess(abs(after[-1, 1] - .5), abs(before[-1, 1] - .5))
    self.assertTrue(np.all((after[:, 0] > 0.) & (after[:, 0] < 1.)))

  def test_moving_pid_preserves_configured_speed_dependent_feedback(self):
    # Compare against the current Sierra gains, not the superseded constant Ki.
    for speed, gain in [(5., .02), (10., .005), (15., .005), (25., .0025), (40., .0025)]:
      for target in [-1., -.2, 0., .5]:
        controller = self.controller()
        controller.pid.i = integral = .08
        state = self.car(speed, target + .2)
        for _ in range(50):
          integral += gain * (target - state.aEgo) * .01
          self.assertAlmostEqual(controller.update(True, state, target, False, (-4., 2.)),
                                 target + integral, places=8)
          self.assertEqual(controller.launch_integrator_hold, 0.0)

  def test_gain_blend_does_not_introduce_a_command_step(self):
    for speed in [2., 5.]:
      before, after = self.controller(), self.controller()
      a = before.update(True, self.car(speed - 1e-6), -.4, False, (-4., 2.))
      b = after.update(True, self.car(speed + 1e-6), -.4, False, (-4., 2.))
      self.assertLess(abs(a - b), 1e-8)

  def test_launch_integrator_waits_for_configured_actuator_delay(self):
    controller = self.controller()
    state = self.car(0.0, 0.0)

    # Establish the normal stopping state first, then release into PID.
    controller.update(True, state, 1.6, True, (-4., 2.))
    self.assertEqual(controller.long_control_state, LongCtrlState.stopping)

    first = controller.update(True, state, 1.6, False, (-4., 2.))
    self.assertEqual(controller.long_control_state, LongCtrlState.pid)
    self.assertAlmostEqual(first, 1.6)
    self.assertAlmostEqual(controller.pid.i, 0.0)

    held_ticks = round(controller.CP.longitudinalActuatorDelay / .01)
    for _ in range(held_ticks - 1):
      controller.update(True, state, 1.6, False, (-4., 2.))
      self.assertAlmostEqual(controller.pid.i, 0.0)

    controller.update(True, state, 1.6, False, (-4., 2.))
    self.assertGreater(controller.pid.i, 0.0)

  def test_launch_integrator_hold_does_not_delay_feedforward_or_moving_pid(self):
    controller = self.controller()
    state = self.car(0.0, 0.0)
    controller.update(True, state, 1.2, True, (-4., 2.))
    for _ in range(10):
      self.assertAlmostEqual(controller.update(True, state, 1.2, False, (-4., 2.)), 1.2)

    moving = self.controller()
    moving_state = self.car(5.0, 0.0)
    moving.update(True, moving_state, .5, False, (-4., 2.))
    self.assertGreater(moving.pid.i, 0.0)
    self.assertEqual(moving.launch_integrator_hold, 0.0)

  def test_stop_hold_and_disengagement_preserve_existing_behavior(self):
    before, after = self.controller(True), self.controller()
    before.last_output_accel = after.last_output_accel = -.25
    for active, stop, state in [(True, True, self.car(.2)), (True, True, self.car(0.)),
                                (True, False, self.car(0., brake=True)),
                                (False, False, self.car(0.))]:
      for _ in range(100):
        self.assertEqual(before.update(active, state, -.2, stop, (-4., 2.)),
                         after.update(active, state, -.2, stop, (-4., 2.)))
        self.assertEqual(before.long_control_state, after.long_control_state)
    self.assertEqual(after.long_control_state, LongCtrlState.off)
    self.assertEqual(after.pid.i, 0.)


if __name__ == '__main__':
  unittest.main()
