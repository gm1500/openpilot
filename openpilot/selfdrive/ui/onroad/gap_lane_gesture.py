HOLD_SECONDS = .6


class GapLaneGesture:
  """One tap OR one hold per contact, using input timestamps for queued events."""

  def __init__(self):
    self.reset()

  def reset(self):
    self.started: float | None = None
    self.held = False
    self.blocked = False
    self.down: set[int] = set()

  @property
  def pressed(self):
    return self.started is not None

  def cancel(self):
    self.started = None
    self.held = False
    self.blocked = bool(self.down)

  def advance(self, now):
    if self.started is not None and not self.held and now - self.started >= HOLD_SECONDS:
      self.held = True
      return 'hold'
    return None

  def event(self, event, inside):
    if event.left_down or event.left_pressed:
      self.down.add(event.slot)
    if event.left_released or not (event.left_down or event.left_pressed):
      self.down.discard(event.slot)

    action = None
    if self.down - {0}:
      self.cancel()
    elif event.slot == 0:
      if event.left_pressed and inside and not self.blocked:
        self.started, self.held = event.t, False
      if self.started is not None:
        if not inside:
          self.cancel()  # sliding back in must not resurrect a tap
        elif event.left_released:
          # A press and release can arrive in the same render frame. Decide
          # from their timestamps before clearing the contact, not wall time.
          action = self.advance(event.t) or (None if self.held else 'tap')
          self.started = None
          self.held = False
        elif not event.left_down:
          self.cancel()

    if not self.down:
      self.blocked = False
    return action
