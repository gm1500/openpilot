"""Calendar periods for refreshing installed maps at the next parked Wi-Fi opportunity."""
from datetime import UTC, datetime, timedelta
from zoneinfo import ZoneInfo, ZoneInfoNotFoundError

from openpilot.common.time_helpers import MIN_DATE, MAX_DATE

WEEKDAYS = ('Monday', 'Tuesday', 'Wednesday', 'Thursday', 'Friday', 'Saturday', 'Sunday')


def schedule_period(params, wall):
  schedule = params.get('OfflineMapUpdateSchedule', return_default=True)
  if schedule not in (1, 2) or not MIN_DATE.replace(tzinfo=UTC).timestamp() < wall < MAX_DATE.replace(tzinfo=UTC).timestamp():
    return None
  try:
    zone = ZoneInfo(params.get('SentryTimeZone') or 'UTC')
  except (ValueError, ZoneInfoNotFoundError):
    zone = UTC
  day = datetime.fromtimestamp(wall, zone).replace(hour=0, minute=0, second=0, microsecond=0)
  if schedule == 2:
    weekday = params.get('OfflineMapUpdateWeekday', return_default=True)
    weekday = weekday if weekday in range(7) else 0
    day -= timedelta(days=(day.weekday()-weekday) % 7)
  return day.timestamp()
