import math
from collections import deque

import numpy as np

from openpilot.common.constants import CV

ASSIST_MAX_LEAD_SPEED = 55 / CV.MS_TO_KPH
BRAKE_THRESHOLD = -0.2  # m/s^2; ignore near-zero model fluctuations before entry
BRAKE_CONFIRM_TIME = 0.15
GO_THRESHOLD = 0.1  # m/s^2
GO_CONFIRM_TIME = 0.2
RELEASE_JERK = 1.0  # m/s^3; stronger braking is never slew-limited
RELEVANCE_RELEASE_TIME = 1.0  # s; fast entry, gradual reduction in influence
PULL_AWAY_WINDOW = 0.5  # s of continuous accepted range observations


def get_model_lead_speed(model, v_ego):
  """Use the vision fallback's ego-speed correction, not differentiated range."""
  if not model.leadsV3 or not model.velocity.x:
    return None
  lead = model.leadsV3[0]
  if not lead.v or not math.isfinite(lead.prob) or lead.prob <= 0.5:
    return None
  values = (v_ego, lead.v[0], model.velocity.x[0])
  if not all(math.isfinite(x) for x in values):
    return None
  return max(0.0, v_ego + lead.v[0] - model.velocity.x[0])


class E2ESlowingAssist:
  """A deceleration constraint alongside ordinary cruise/lead control, never a mode switch."""

  def __init__(self, dt):
    self.dt = dt
    self.reset()

  def reset(self):
    self.braking = False
    self.brake_time = 0.0
    self.go_time = 0.0
    self.accel_limit = None
    self.influence = 0.0
    self.range_history = deque(maxlen=round(PULL_AWAY_WINDOW / self.dt) + 1)

  def update(self, *, eligible, lead, model_lead_speed, v_ego, regular_accel, e2e_accel, e2e_stop):
    # Driver override, disengagement and unhealthy data discard all history.
    # A healthy message with no accepted lead instead releases the previous cap.
    valid_lead = not lead.present or (all(math.isfinite(x) for x in (lead.dRel, lead.vLead, lead.vRel)) and lead.dRel > 0)
    if not eligible or not valid_lead or not all(math.isfinite(x) for x in (v_ego, regular_accel, e2e_accel)):
      self.reset()
      return regular_accel, False, False

    slow_lead = (model_lead_speed is not None and math.isfinite(model_lead_speed) and
                 model_lead_speed < ASSIST_MAX_LEAD_SPEED)
    holding_stop = self.braking and e2e_stop and v_ego < 0.3
    if not slow_lead or not lead.present:
      # A disappearing lead cannot create a new stop request, or cancel a
      # previously latched stop that the fresh model still requests at rest.
      # A lead reaching 55 km/h also releases through the bounded handoff.
      self.braking = holding_stop
      self.brake_time = self.go_time = 0.0
      self.influence = 1.0 if holding_stop else 0.0
      self.range_history.clear()
    else:
      # Gap provides proximity; relative kinetic energy provides closing demand.
      # A far stopped/slower lead can therefore retain full influence. This is
      # only an assist weight, not a replacement for the MPC's following policy.
      speed = max(v_ego, 0.0)
      closing_demand = max(speed**2 - max(lead.vLead, 0.0)**2, 0.0) / (2.0 * max(lead.dRel - 6.0, 1.0))
      proximity = float(np.interp(lead.dRel, [6.0 + 2.0 * speed, 6.0 + 3.5 * speed], [1.0, 0.0]))
      relevance = max(proximity, float(np.interp(closing_demand, [0.2, 0.6], [0.0, 1.0])))
      if e2e_stop:
        relevance = 1.0
      self.influence = max(relevance, self.influence - self.dt / RELEVANCE_RELEASE_TIME)

      if self.range_history and abs(lead.dRel - self.range_history[-1][0] - lead.vRel * self.dt) > 3.0:
        self.range_history.clear()
      self.range_history.append((lead.dRel, lead.vRel))
      pulling_away = (len(self.range_history) == self.range_history.maxlen and speed > 2.0 and
                      min(point[1] for point in self.range_history) > 0.5 and
                      lead.dRel > self.range_history[0][0] + 0.25 and lead.dRel > 6.0 + 1.5 * speed and
                      e2e_accel > -0.1 and regular_accel > 0.1)

      self.brake_time = self.brake_time + self.dt if e2e_accel <= BRAKE_THRESHOLD and self.influence > 0.0 else 0.0
      if self.brake_time >= BRAKE_CONFIRM_TIME:
        self.braking = True

      go = e2e_accel > GO_THRESHOLD or pulling_away or (self.influence == 0.0 and e2e_accel > -0.1)
      self.go_time = self.go_time + self.dt if go and not e2e_stop else 0.0
      if self.go_time >= GO_CONFIRM_TIME:
        self.braking = False

    if self.braking:
      # Blend braking toward coasting. A noisy lead-speed rise alone must not
      # restore positive propulsion while the model still requests a slowdown.
      target = self.influence * min(e2e_accel, 0.0)
    elif self.accel_limit is not None:
      target = regular_accel
    else:
      return regular_accel, False, False

    # Only soften the return toward acceleration. Keep the previous selected
    # command, including stronger regular braking, as the handoff starting point.
    if self.accel_limit is not None:
      target = min(target, self.accel_limit + RELEASE_JERK * self.dt)
    selected = min(regular_accel, target)
    contributing = selected < regular_accel
    self.accel_limit = selected if contributing or (self.braking and self.accel_limit is not None) else None
    return selected, self.braking and e2e_stop, contributing
