"""Painted lane colours only; the UI never invents a road boundary."""
PAINT_BLUE = (97, 183, 230)
PAINT_GREEN = (128, 216, 166)


def boundary_colors(state, side, controlling):
  if not state.valid or not state.enabled:
    return None, None
  source = str(state.leftSource if side == 0 else state.rightSource)
  paint = (PAINT_GREEN if state.active and controlling else PAINT_BLUE) if source == 'painted' else None
  return paint, None
