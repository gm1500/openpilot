class LanePolicyButton:
  """Follow raw GM bit transitions for this POC; never infer state at startup."""

  def __init__(self):
    self.last_state: bool | None = None

  def update(self, CS, params) -> bool | None:
    if not CS.canValid or CS.canTimeout or not CS.gmLkaButtonValid:
      return None
    state = bool(CS.gmLkaButton)
    if self.last_state is None:
      # A fresh frame only establishes a baseline. On the Sierra recording
      # this bit stays zero even while the dash switch changes, so receiving
      # zero must not turn off the independently selected lane policy.
      self.last_state = state
      return None
    if state == self.last_state:
      return None
    self.last_state = state
    # Write only on an observed transition, not the first valid observation.
    # Repeated CAN frames must not overwrite a user's HUD long-press override.
    params.put_bool('LanePolicyEnabled', state)
    return state
