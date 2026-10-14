"""One physical touch, one attempt. Leaving a button permanently cancels that hold."""


class ResumeSetGesture:
  def __init__(self):
    self.button = 0
    self.request_id = 0
    self.slot = None
    self.down = set()

  def cancel(self):
    self.button = 0
    self.slot = None

  def confirm_hold(self, now, poll_time, held_slots):
    if self.button and (not 0 <= now - poll_time <= .1 or self.slot is None or
                        not held_slots[self.slot] or sum(held_slots) != 1):
      self.cancel()

  def event(self, slot, button, pressed, released, now_nanos, ready):
    if pressed:
      self.down.add(slot)
      if self.button or len(self.down) != 1:
        self.cancel()
      elif ready and button in (2, 3):
        self.button, self.slot, self.request_id = button, slot, now_nanos
    elif released:
      self.down.discard(slot)
      if slot == self.slot:
        self.cancel()
    elif slot == self.slot and button != self.button:
      self.cancel()
