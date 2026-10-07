"""Normalize OSM stopping controls without inferring their road from proximity."""
import re

COMPASS = {name: i * 22.5 for i, name in enumerate(
  ('N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE', 'S', 'SSW', 'SW', 'WSW', 'W', 'WNW', 'NW', 'NNW'))}
SIGN_KEYS = ('traffic_sign', 'traffic_sign:id', 'traffic_sign:forward', 'traffic_sign:backward')
SIGNAL_CROSSINGS = ('pelican', 'puffin', 'toucan', 'pegasus')
# Exact sign IDs only: stop-ahead, school/crosswalk STOP instructions and tabs
# alone must not become unconditional stop-sign targets. See the tag references
# in docs/conditional-experimental.md for this deliberately explicit code list.
STOP_CODES = {'US:R1-1', 'DE:206', 'CA:BC:R-001', 'CA:QC:P-010', 'CA:QC:P-010A'}
CANADIAN_STOP = re.compile(r'CA(?::(?:AB|BC|MB|NB|NL|NS|NT|NU|ON|PE|QC|SK|YT))?:RA-1')
INACTIVE = ('disused', 'abandoned', 'removed', 'demolished', 'construction', 'proposed')
EXCLUDED_SIGNALS = {'no', 'none', 'continuous_green', 'blinker', 'ramp_meter', 'level_crossing',
                    'train_priority', 'bridge', 'movable_bridge'}


def sign_codes(value: str) -> set[str]:
  """Read comma/semicolon lists, including an omitted repeated country prefix."""
  result, prefix = set(), ''
  for item in re.split('[;,]', value):
    item = item.strip().upper()
    if ':' in item:
      prefix, code = item.rsplit(':', 1)
      prefix += ':'
    else:
      code = item
    # Human-readable values do not inherit a preceding country's namespace.
    result.add(code if code == 'STOP' else prefix + code)
  return result


def has_stop_sign(value: str) -> bool:
  return any(code == 'STOP' or code in STOP_CODES or CANADIAN_STOP.fullmatch(code) is not None for code in sign_codes(value))


def signal_crossing(tags: dict[str, str]) -> bool:
  if tags.get('highway') != 'crossing' or tags.get('crossing:signals') == 'no':
    return False
  return (tags.get('crossing:signals') == 'yes' or tags.get('crossing') == 'traffic_signals' or
          tags.get('crossing_ref') in SIGNAL_CROSSINGS and tags.get('crossing') not in ('no', 'uncontrolled'))


def all_way_stop(tags: dict[str, str]) -> bool:
  if tags.get('stop') == 'all' or tags.get('traffic_signals') == 'stop':
    return True
  codes = sign_codes(tags.get('traffic_sign', '')) | sign_codes(tags.get('traffic_sign:id', ''))
  return ('US:R1-3P' in codes or 'CA:AB:RA-1-T' in codes) and any(has_stop_sign(tags.get(k, '')) for k in SIGN_KEYS[:2])


def control_kind(tags: dict[str, str]) -> str | None:
  if any(tags.get(k) == 'yes' for k in INACTIVE) or tags.get('railway') in ('crossing', 'level_crossing'):
    return None
  highway = tags.get('highway')
  if highway == 'give_way':
    return None  # a conflicting sign alias must not override an explicit yield
  if highway == 'stop' or any(has_stop_sign(tags.get(k, '')) for k in SIGN_KEYS):
    return 'stopSign'
  if highway == 'traffic_signals':
    if tags.get('traffic_signals') == 'stop' or tags.get('traffic_signals') == 'blinker' and tags.get('stop') == 'all':
      return 'stopSign'
    return None if tags.get('traffic_signals') in EXCLUDED_SIGNALS else 'trafficLight'
  return 'trafficLight' if signal_crossing(tags) else None


def stop_sign_direction(tags: dict[str, str], forward: bool) -> bool | None:
  """Directional sign keys replace the generic sign for the indicated direction."""
  keys = ('traffic_sign:forward', 'traffic_sign:backward')
  if not any(k in tags for k in keys) or not any(has_stop_sign(tags.get(k, '')) for k in SIGN_KEYS):
    return None
  key = keys[0 if forward else 1]
  if key in tags:
    return has_stop_sign(tags[key])
  # A generic stop may still apply in the direction without a more specific tag.
  return any(has_stop_sign(tags.get(k, '')) for k in SIGN_KEYS[:2])


def road_controls(node_ids: tuple[int, ...], tags: dict[str, str], nodes: dict[int, dict[str, str]]) -> dict[int, dict[str, str]]:
  controls = {node: nodes[node] for node in node_ids if node in nodes and control_kind(nodes[node]) is not None}
  # Legacy stop tags on a WAY locate a stop at its end(s). Synthesize tags only
  # for that way, never for the other roads sharing the intersection node.
  if len(node_ids) < 2 or node_ids[0] == node_ids[-1] or any(tags.get(k) == 'yes' for k in INACTIVE):
    return controls
  stop = tags.get('stop')
  for node, direction, applies in ((node_ids[-1], 'forward', stop in ('yes', 'both')),
                                   (node_ids[0], 'backward', stop in ('-1', 'both'))):
    explicit = nodes.get(node, {})
    if applies and not any(k in explicit for k in ('highway', 'stop', *SIGN_KEYS)):
      controls[node] = {'highway': 'stop', 'stop:direction': direction}
  return controls
