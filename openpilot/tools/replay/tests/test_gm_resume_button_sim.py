import unittest

from opendbc.can import CANPacker
from opendbc.car.gm.gmcan import create_buttons
from opendbc.car.gm.values import CruiseButtons
from openpilot.tools.replay.gm_resume_button_sim import Attempt, ButtonFrame, DBC, check_current_safety, simulate


class TestGMResumeButtonSimulation(unittest.TestCase):
  def setUp(self):
    self.packer = CANPacker(DBC)
    self.start = 100_000_000_000
    self.attempt = Attempt(self.start - 10_000_000, self.start, self.start + 1_000_000_000)

  def frame(self, delay_ms, counter, button=CruiseButtons.UNPRESS):
    return ButtonFrame(self.start + delay_ms * 1_000_000, create_buttons(self.packer, 0, counter, button)[1])

  def test_press_release_encoding_and_counter_wrap(self):
    for counter in range(4):
      with self.subTest(counter=counter):
        native = [self.frame(10, counter), self.frame(40, (counter + 1) % 4)]
        before = list(native)
        result = simulate(native, [self.attempt])[0]
        self.assertEqual(result['status'], 'modeled_replacement')
        self.assertEqual(result['press_duration_ms'], 30)
        press, release = result['frames']
        self.assertEqual(press['modeled_data'], create_buttons(self.packer, 0, counter, CruiseButtons.RES_ACCEL)[1].hex())
        self.assertEqual(release['modeled_data'], native[1].data.hex())
        self.assertEqual(native, before)
        # Independent checksum check against the recorded neutral packet:
        # the Resume value differs by 1 and subtracts 16 from the checksum.
        p = bytes.fromhex(press['modeled_data'])
        n = native[0].data
        self.assertEqual(p[:5], n[:5])
        self.assertEqual((p[5] >> 4) & 7, 2)
        self.assertEqual(((p[5] & 15) << 8) | p[6], ((((n[5] & 15) << 8) | n[6]) - 16) & 0xfff)

  def test_one_pair_per_attempt_no_repeat(self):
    native = [self.frame(i * 30, i % 4) for i in range(40)]
    results = simulate(native, [self.attempt, self.attempt])
    self.assertEqual([len(r['frames']) for r in results], [2, 0])
    self.assertEqual(simulate(native, []), [])

  def test_physical_buttons_take_priority(self):
    for button in (CruiseButtons.CANCEL, CruiseButtons.RES_ACCEL, CruiseButtons.DECEL_SET):
      for slot in (0, 1):
        with self.subTest(button=button, slot=slot):
          native = [self.frame(10, 0), self.frame(40, 1)]
          native[slot] = self.frame(10 + slot * 30, slot, button)
          self.assertEqual(simulate(native, [self.attempt])[0]['frames'], [])

  def test_missing_stale_or_broken_counter_slots_skip(self):
    for native in ([], [self.frame(10, 0)], [self.frame(110, 0), self.frame(140, 1)],
                   [self.frame(10, 0), self.frame(140, 1)], [self.frame(10, 0), self.frame(40, 2)]):
      with self.subTest(native=native):
        self.assertEqual(simulate(native, [self.attempt])[0]['frames'], [])

  def test_unknown_fields_bad_checksum_and_short_attempt_skip(self):
    frame = self.frame(10, 0)
    for index in (0, 2, 6):
      data = bytearray(frame.data)
      data[index] ^= 1
      result = simulate([ButtonFrame(frame.time_ns, bytes(data)), self.frame(40, 1)], [self.attempt])[0]
      self.assertEqual(result['frames'], [])
    short = Attempt(self.attempt.request_id, self.start, self.start + 20_000_000)
    self.assertEqual(simulate([self.frame(10, 0), self.frame(40, 1)], [short])[0]['frames'], [])

  def test_current_panda_policy_rejects_both_candidates(self):
    results = simulate([self.frame(10, 0), self.frame(40, 1)], [self.attempt])
    check_current_safety(results)
    self.assertEqual([f['current_panda_tx_allowed'] for f in results[0]['frames']], [False, False])


if __name__ == '__main__':
  unittest.main()
