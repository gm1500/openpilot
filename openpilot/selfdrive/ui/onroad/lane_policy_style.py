"""Per-side lane source colours; no steering or detection decisions in the UI."""
PAINT_BLUE = (97, 183, 230)
PAINT_GREEN = (128, 216, 166)
EDGE_ORANGE = (255, 149, 0)
EDGE_YELLOW = (255, 230, 0)


def boundary_colors(state, side, controlling):
  if not state.valid or not state.enabled:
    return None, None
  source = str(state.leftSource if side == 0 else state.rightSource)
  active = state.active and controlling
  paint = (PAINT_GREEN if active else PAINT_BLUE) if source == 'painted' else None
  edge = None
  if len(state.edges) == 2 and state.edges[side].available and source != 'painted':
    edge = EDGE_YELLOW if active and source == 'roadEdge' else EDGE_ORANGE
  return paint, edge
