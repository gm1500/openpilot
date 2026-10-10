from types import SimpleNamespace as NS

from opendbc.car.gm.values import CAR as GM_CAR
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState, long_control_state_trans


class TestLongControlStateTransition(OpenpilotTestCase):

  def test_sierra_direct_control_releases_its_brake_without_waiting_for_stock_standstill(self):
    for fingerprint, direct, stop, pedal, active, expected in (
      (GM_CAR.CHEVROLET_SILVERADO, True, False, False, True, LongCtrlState.pid),
      (GM_CAR.CHEVROLET_SILVERADO, True, True, False, True, LongCtrlState.stopping),
      (GM_CAR.CHEVROLET_SILVERADO, True, False, True, True, LongCtrlState.stopping),
      (GM_CAR.CHEVROLET_SILVERADO, True, False, False, False, LongCtrlState.off),
      (GM_CAR.CHEVROLET_SILVERADO, False, False, False, True, LongCtrlState.stopping),
      ('OTHER', True, False, False, True, LongCtrlState.stopping),
    ):
      with self.subTest(fingerprint=fingerprint, direct=direct, stop=stop, pedal=pedal, active=active):
        cp = NS(carFingerprint=fingerprint, openpilotLongitudinalControl=direct,
                longitudinalTuning=NS(kiBP=[0.], kiV=[.1]), longitudinalActuatorDelay=.5, stopAccel=-.5)
        cs = NS(vEgo=0., aEgo=0., brakePressed=pedal, cruiseState=NS(standstill=True))
        control = LongControl(cp)
        control.long_control_state = LongCtrlState.stopping
        control.last_output_accel = -.5
        accel = control.update(active, cs, 1., stop, (-3.5, 2.))
        self.assertEqual(control.long_control_state, expected)
        if expected == LongCtrlState.pid:
          self.assertGreater(accel, 0.)
        elif expected == LongCtrlState.stopping:
          self.assertLess(accel, 0.)

  def test_stay_stopped(self):
    active = True
    current_state = LongCtrlState.stopping
    next_state = long_control_state_trans(active, current_state,
                             should_stop=True, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(active, current_state,
                             should_stop=False, brake_pressed=True, cruise_standstill=False)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(active, current_state,
                             should_stop=False, brake_pressed=False, cruise_standstill=True)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(active, current_state,
                             should_stop=False, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.pid
    active = False
    next_state = long_control_state_trans(active, current_state,
                             should_stop=False, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.off

  def test_engage(self):
    active = True
    current_state = LongCtrlState.off
    next_state = long_control_state_trans(active, current_state,
                             should_stop=True, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(active, current_state,
                             should_stop=False, brake_pressed=True, cruise_standstill=False)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(active, current_state,
                             should_stop=False, brake_pressed=False, cruise_standstill=True)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(active, current_state,
                             should_stop=False, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.pid
