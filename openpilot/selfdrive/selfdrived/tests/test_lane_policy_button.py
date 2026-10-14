import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from openpilot.selfdrive.selfdrived.lane_policy_button import LanePolicyButton


class TestLanePolicyButton(unittest.TestCase):
  def setUp(self):
    self.button = LanePolicyButton()
    self.params = Mock()
    self.cs = SimpleNamespace(canValid=True, canTimeout=False, gmLkaButton=False, gmLkaButtonValid=True)

  def update(self, state=False, **values):
    self.cs.gmLkaButton = state
    for name, value in values.items():
      setattr(self.cs, name, value)
    return self.button.update(self.cs, self.params)

  def test_first_valid_observation_only_establishes_baseline(self):
    for state in (False, True):
      with self.subTest(state=state):
        self.button = LanePolicyButton()
        self.assertIsNone(self.update(state))
        self.params.put_bool.assert_not_called()

  def test_constant_zero_does_not_turn_off_the_policy(self):
    # The 34-second Sierra recording contains fresh frames but no bit change.
    for _ in range(3500):
      self.assertIsNone(self.update(False))
    self.params.put_bool.assert_not_called()

  def test_both_transitions_after_baseline_select_policy(self):
    self.update(False)
    for state in (True, False, True):
      self.assertEqual(self.update(state), state)
      self.params.put_bool.assert_called_with('LanePolicyEnabled', state)

  def test_repeated_state_does_not_overwrite_hud_override(self):
    self.update(True)
    self.params.put_bool('LanePolicyEnabled', False)  # HUD override
    self.params.reset_mock()
    for _ in range(100):
      self.assertIsNone(self.update(True))
    self.params.put_bool.assert_not_called()

  def test_missing_invalid_and_timed_out_can_do_not_select_off(self):
    for field, invalid in (('canValid', False), ('canTimeout', True), ('gmLkaButtonValid', False)):
      with self.subTest(field=field):
        old = getattr(self.cs, field)
        self.assertIsNone(self.update(False, **{field: invalid}))
        self.params.put_bool.assert_not_called()
        setattr(self.cs, field, old)

  def test_recovery_respects_override_unless_physical_state_changed(self):
    self.update(True)
    self.params.reset_mock()
    self.update(False, gmLkaButtonValid=False)
    self.assertIsNone(self.update(True, gmLkaButtonValid=True))
    self.params.put_bool.assert_not_called()
    self.update(False, gmLkaButtonValid=False)
    self.assertFalse(self.update(False, gmLkaButtonValid=True))
    self.params.put_bool.assert_called_once_with('LanePolicyEnabled', False)
