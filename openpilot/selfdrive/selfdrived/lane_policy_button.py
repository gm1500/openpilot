class LanePolicyButton:
  """Apply observed GM switch state changes; allow HUD overrides between them."""

  def __init__(self):
    self.last_state: bool | None = None

  def update(self, CS, params) -> bool | None:
    if not CS.canValid or CS.canTimeout or not CS.gmLkaButtonValid:
      return None
    state = bool(CS.gmLkaButton)
    if state == self.last_state:
      return None
    self.last_state = state
    # Write only on the first valid observation or a physical transition.
    # Repeated CAN frames must not overwrite a user's HUD long-press override.
    params.put_bool('LanePolicyEnabled', state)
    return state
