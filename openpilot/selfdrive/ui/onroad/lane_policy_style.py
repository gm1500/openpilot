"""Per-side lane source colours; no steering or detection decisions in the UI."""
PAINT_BLUE = (97, 183, 230)
PAINT_GREEN = (128, 216, 166)
EDGE_YELLOW = (255, 230, 0)
EDGE_ORANGE = (255, 149, 0)


def active_edge(state, side, controlling):
  source = str(state.leftSource if side == 0 else state.rightSource)
  return (state.valid and state.enabled and state.active and controlling and source == 'roadEdge' and
          len(state.edges) == 2 and state.edges[side].available)


def boundary_colors(state, side, controlling):
  if not state.valid or not state.enabled:
    return None, None
  source = str(state.leftSource if side == 0 else state.rightSource)
  active = state.active and controlling
  paint = (PAINT_GREEN if active else PAINT_BLUE) if source == 'painted' else None
  edge = None
  if len(state.edges) == 2 and state.edges[side].available and source != 'painted':
    # Held geometry is a width hypothesis anchored to current paint. Keep it
    # visibly distinct even when that hypothesis is actively steering.
    edge = EDGE_ORANGE if state.edges[side].held else PAINT_GREEN if active_edge(state, side, controlling) else EDGE_YELLOW
  return paint, edge
