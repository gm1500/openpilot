"""Strict, location-independent subset of OSM conditional speed restrictions.

Unsupported conditions stay unknown, never silently falling back to a higher
base speed. Local civil time must be supplied by the caller, with a timezone.
"""
import re
from datetime import date, datetime, timedelta
from functools import lru_cache

DAYS = ('Mo', 'Tu', 'We', 'Th', 'Fr', 'Sa', 'Su')
MONTHS = ('Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun', 'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec')


def parse_speed(value: str) -> float | None:
  match = re.fullmatch(r'([0-9]+(?:\.[0-9]+)?)\s*(km/h|kmh|kph|mph)?', value.strip().lower())
  if match is None:
    return None
  speed = float(match[1]) * (0.44704 if match[2] == 'mph' else 1 / 3.6)
  return speed if 0 < speed <= 300 / 3.6 else None


def _split(value: str) -> list[str] | None:
  """Split clauses, preserving semicolons inside a parenthesized schedule."""
  depth, start = 0, 0
  clauses = []
  for i, char in enumerate(value):
    if char == '(':
      depth += 1
      if depth > 1:
        return None
    elif char == ')':
      depth -= 1
      if depth < 0:
        return None
    elif char == ';' and depth == 0:
      clauses.append(value[start:i].strip())
      start = i + 1
  clauses.append(value[start:].strip())
  return clauses if depth == 0 and all(clauses) else None


def _days(value: str) -> frozenset[int]:
  days = set()
  for item in value.split(','):
    ends = item.split('-')
    if len(ends) > 2:
      raise ValueError('weekday range')
    start, end = DAYS.index(ends[0]), DAYS.index(ends[-1])
    days.update((start + i) % 7 for i in range((end - start) % 7 + 1))
  return frozenset(days)


def _minute(value: str, end: bool = False) -> int:
  h, m = map(int, value.split(':'))
  if not (0 <= h <= 23 and 0 <= m <= 59 or end and h == 24 and m == 0):
    raise ValueError('clock time')
  return h * 60 + m


@lru_cache(maxsize=512)
def _schedule(expression: str) -> tuple | None:
  """Weekly windows and explicit year/month/day date ranges; no calendar guesses."""
  expression = expression.strip()
  if expression.startswith('(') and expression.endswith(')'):
    expression = expression[1:-1].strip()
  rules = []
  try:
    for text in expression.split(';'):
      text = text.strip()
      first, last = date.min, date.max
      dated = re.match(r'^(\d{4}) ([A-Z][a-z]{2}) (\d{2})(?:\s*-\s*(\d{4}) ([A-Z][a-z]{2}) (\d{2}))?(?:\s+|$)', text)
      if dated:
        first = date(int(dated[1]), MONTHS.index(dated[2]) + 1, int(dated[3]))
        last = date(int(dated[4]), MONTHS.index(dated[5]) + 1, int(dated[6])) if dated[4] else first
        if last < first:
          return None
        text = text[dated.end():]
      days = frozenset(range(7))
      weekly = re.match(r'^((?:Mo|Tu|We|Th|Fr|Sa|Su)(?:-(?:Mo|Tu|We|Th|Fr|Sa|Su))?(?:,(?:Mo|Tu|We|Th|Fr|Sa|Su)(?:-(?:Mo|Tu|We|Th|Fr|Sa|Su))?)*)(?:\s+|$)', text)
      if weekly:
        days = _days(weekly[1])
        text = text[weekly.end():]
      if text == '24/7' or not text and (dated or weekly):
        windows = ((0, 1440),)
      elif re.fullmatch(r'\d{2}:\d{2}-\d{2}:\d{2}(?:\s*,\s*\d{2}:\d{2}-\d{2}:\d{2})*', text):
        windows = tuple((_minute(a), _minute(b, end=True)) for a, b in (part.strip().split('-') for part in text.split(',')))
        if any(a == b for a, b in windows):
          return None
      else:
        return None  # PH/SH, school_days, flashing, weather, sunrise etc.
      rules.append((first, last, days, windows))
  except (ValueError, TypeError):
    return None
  return tuple(rules)


def schedule_active(expression: str, local_time: datetime | None) -> bool | None:
  rules = _schedule(expression)
  if rules is None or local_time is None or local_time.tzinfo is None or local_time.utcoffset() is None:
    return None
  minute = local_time.hour * 60 + local_time.minute
  for first, last, days, windows in rules:
    for start, end in windows:
      # The after-midnight part belongs to the weekday/date on which it began.
      day = local_time.date() - timedelta(days=int(start > end and minute < end))
      if first <= day <= last and day.weekday() in days:
        if start <= minute < end if start < end else minute >= start or minute < end:
          return True
  return False


def conditional_speed(value: str, base: float | None, local_time: datetime | None) -> float | None:
  if len(value) > 1024:
    return None
  clauses = _split(value)
  if clauses is None:
    return None
  result = base
  active_values = []
  for clause in clauses:
    parts = clause.split('@')
    if len(parts) != 2:
      return None
    speed = parse_speed(parts[0])
    active = schedule_active(parts[1], local_time)
    if speed is None or active is None:
      return None
    if active:
      active_values.append(speed)
  # Overlapping contradictory clauses are unresolved; don't choose a higher one.
  if active_values:
    result = active_values[0] if all(v == active_values[0] for v in active_values) else None
  return result


def road_speed(tags: dict[str, str], forward: bool | None, advisory: bool = False,
               local_time: datetime | None = None) -> float | None:
  prefix = 'maxspeed:advisory' if advisory else 'maxspeed'
  supported = {prefix, f'{prefix}:forward', f'{prefix}:backward', f'{prefix}:type',
               f'{prefix}:conditional', f'{prefix}:forward:conditional', f'{prefix}:backward:conditional'}
  qualified = (key for key in tags if key.startswith(f'{prefix}:'))
  if not advisory:
    qualified = (key for key in qualified if key != 'maxspeed:advisory' and not key.startswith('maxspeed:advisory:'))
  if any(key not in supported for key in qualified):
    return None

  def directional(direction):
    base = parse_speed(tags.get(f'{prefix}:{direction}', tags.get(prefix, '')))
    # Direction-specific tags override their undirected counterpart.
    condition = tags.get(f'{prefix}:{direction}:conditional', tags.get(f'{prefix}:conditional'))
    return conditional_speed(condition, base, local_time) if condition is not None else base

  if forward is None:
    ahead, behind = directional('forward'), directional('backward')
    return ahead if ahead == behind else None
  return directional('forward' if forward else 'backward')


def road_zone(tags: dict[str, str], forward: bool | None, local_time: datetime | None = None) -> str:
  """Classify only a resolved, currently active restriction on the matched way."""
  if road_speed(tags, forward, local_time=local_time) is None:
    return 'none'
  construction = tags.get('maxspeed:type') == 'construction'
  school = tags.get('school_zone') == 'yes' or tags.get('hazard') == 'school_zone'
  if construction == school:
    return 'none'  # absent or conflicting classification

  def active(direction):
    condition = tags.get(f'maxspeed:{direction}:conditional', tags.get('maxspeed:conditional'))
    if condition is None:
      return True
    clauses = _split(condition)
    return clauses is not None and any(schedule_active(clause.split('@')[1], local_time) is True for clause in clauses)

  directions = ('forward', 'backward') if forward is None else ('forward' if forward else 'backward',)
  return ('construction' if construction else 'school') if all(active(direction) for direction in directions) else 'none'
