"""Persistent SLC preferences; posted limits remain separate from cruise offsets."""
import math
from dataclasses import dataclass

from openpilot.common.constants import CV


@dataclass(frozen=True)
class SLCSettings:
  offset_type: int = 0  # 0 = display units, 1 = percent
  offset_value: int = 0
  change_policy: int = 0  # 0 = automatic, 1 = lower only, 2 = explicit SET
  resume_policy: int = 0  # 0 = next changed limit, 1 = explicit SET
  advance: bool = False  # qualify ahead; lower targets may start up to two seconds early

  @classmethod
  def read(cls, params):
    def integer(key, default, low, high):
      try:
        value = params.get(key, return_default=True)
        return max(low, min(high, int(value))) if value is not None else default
      except (ValueError, TypeError):
        return default
    return cls(integer('SlcOffsetType', 0, 0, 1), integer('SlcOffsetValue', 0, -30, 30),
               integer('SlcChangePolicy', 0, 0, 2), integer('SlcResumePolicy', 0, 0, 1),
               bool(integer('SlcAdvancedSpeedLimit', 0, 0, 1)))

  def target(self, posted_kph: float, metric: bool) -> float:
    offset = self.offset_value
    adjusted = posted_kph * (1 + offset / 100) if self.offset_type else posted_kph + offset * (1 if metric else CV.MPH_TO_KPH)
    return round(max(8., min(145., adjusted)), 3) if math.isfinite(adjusted) else posted_kph

  def accepts(self, target_kph: float, current_kph: float) -> bool:
    return self.change_policy == 0 or (self.change_policy == 1 and target_kph <= current_kph)
