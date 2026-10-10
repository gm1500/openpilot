"""Offline GM Resume-button replacement model. Never transmits CAN.

Models a press and release in two existing native button-message slots at the
start of each recorded TEST RESUME attempt. This assumes ownership of those
slots, which the current camera harness does not provide on vehicle bus 0.
It is deliberately not an additive injection schedule or an ECU simulator.
"""
import argparse
import bisect
import json
from dataclasses import dataclass
from pathlib import Path

from opendbc.can import CANPacker
from opendbc.car.gm.gmcan import create_buttons
from opendbc.car.gm.values import CruiseButtons

BUTTON_ADDR = 0x1E1
MAX_SLOT_DELAY_NS = 100_000_000
DBC = 'gm_global_a_powertrain_generated'


@dataclass(frozen=True)
class ButtonFrame:
  time_ns: int
  data: bytes


@dataclass(frozen=True)
class Attempt:
  request_id: int
  start_ns: int
  end_ns: int


def simulate(buttons: list[ButtonFrame], attempts: list[Attempt]) -> list[dict]:
  """Return JSON-only candidate replacements; no sockets, Panda or sendcan API."""
  packer = CANPacker(DBC)
  buttons = sorted(buttons, key=lambda b: b.time_ns)
  times = [b.time_ns for b in buttons]
  results = []
  seen = set()
  for attempt in attempts:
    item = {'request_id': attempt.request_id, 'start_ns': attempt.start_ns, 'status': 'skipped', 'frames': []}
    results.append(item)
    if attempt.request_id <= 0 or attempt.request_id in seen:
      item['reason'] = 'missing_or_repeated_request'
      continue
    seen.add(attempt.request_id)
    if not 0 <= attempt.start_ns - attempt.request_id <= 200_000_000 or attempt.end_ns <= attempt.start_ns:
      item['reason'] = 'invalid_attempt_timing'
      continue
    i = bisect.bisect_left(times, attempt.start_ns)
    pair = buttons[i:i + 2]
    if (len(pair) != 2 or pair[0].time_ns - attempt.start_ns > MAX_SLOT_DELAY_NS or
        not 0 < pair[1].time_ns - pair[0].time_ns <= MAX_SLOT_DELAY_NS or pair[1].time_ns >= attempt.end_ns):
      item['reason'] = 'missing_fresh_press_and_release_slots'
      continue
    counters = []
    for frame in pair:
      if len(frame.data) != 7 or ((frame.data[5] >> 4) & 7) != CruiseButtons.UNPRESS:
        item['reason'] = 'native_button_not_idle'
        break
      counter = frame.data[4] & 3
      # Reject unfamiliar fields/checksums instead of zeroing driver data.
      if frame.data != create_buttons(packer, 0, counter, CruiseButtons.UNPRESS)[1]:
        item['reason'] = 'unrecognized_native_frame'
        break
      counters.append(counter)
    else:
      if counters[1] != (counters[0] + 1) % 4:
        item['reason'] = 'native_counter_discontinuity'
        continue
      item['status'] = 'modeled_replacement'
      item['reason'] = 'offline_only_requires_native_slot_ownership'
      item['press_duration_ms'] = (pair[1].time_ns - pair[0].time_ns) / 1e6
      for frame, counter, button in zip(pair, counters, (CruiseButtons.RES_ACCEL, CruiseButtons.UNPRESS), strict=True):
        address, data, bus = create_buttons(packer, 0, counter, button)
        item['frames'].append({'time_ns': frame.time_ns, 'address': address, 'bus': bus,
                               'button': int(button), 'counter': counter,
                               'native_data': frame.data.hex(), 'modeled_data': data.hex()})
  return results


def read_recording(path: Path):
  import zstandard
  from openpilot.cereal import log

  if path.suffix != '.zst':
    raise ValueError('Expected a local rlog.zst file')
  with path.open('rb') as source, zstandard.ZstdDecompressor().stream_reader(source) as reader:
    raw = reader.read()
  buttons, states = [], []
  build = {}
  for event in log.Event.read_multiple_bytes(raw):
    kind = event.which()
    if kind == 'initData':
      build = {'commit': event.initData.gitCommit, 'branch': event.initData.gitBranch}
    elif kind == 'can':
      buttons.extend(ButtonFrame(event.logMonoTime, bytes(c.dat)) for c in event.can if c.address == BUTTON_ADDR and c.src == 0)
    elif kind == 'controlsState':
      state = event.controlsState.resumeTest
      states.append((event.logMonoTime, state.active, state.requestId))

  attempts = []
  previous = None
  pending = None
  for time_ns, active, request_id in sorted(states):
    # An already-active first sample is a truncated attempt, not a new press.
    if active and previous is False:
      pending = (request_id, time_ns)
    elif not active and pending is not None:
      attempts.append(Attempt(*pending, time_ns))
      pending = None
    previous = active
  return build, buttons, attempts


def check_current_safety(results: list[dict]):
  """Exercise compiled hooks in host memory; this never opens a device."""
  from opendbc.car.structs import CarParams
  from opendbc.safety.tests.libsafety import libsafety_py

  safety = libsafety_py.libsafety
  for item in results:
    for frame in item['frames']:
      assert safety.set_safety_hooks(CarParams.SafetyModel.gm, 3) == 0
      safety.set_controls_allowed(True)
      packet = libsafety_py.make_CANPacket(frame['address'], frame['bus'], bytes.fromhex(frame['modeled_data']))
      frame['current_panda_tx_allowed'] = bool(safety.safety_tx_hook(packet))


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('rlog', type=Path)
  parser.add_argument('--check-safety', action='store_true', help='Compile and evaluate host-only Panda safety hooks')
  args = parser.parse_args()
  build, buttons, attempts = read_recording(args.rlog)
  results = simulate(buttons, attempts)
  if args.check_safety:
    check_current_safety(results)
  print(json.dumps({'mode': 'offline_replacement_model', 'live_transmission': False,
                    'vehicle_response_simulated': False, 'recorded_build': build,
                    'note': 'Native vehicle-bus button slots are not intercepted by the current harness; do not inject these frames.',
                    'attempts': results}, indent=2))


if __name__ == '__main__':
  main()
