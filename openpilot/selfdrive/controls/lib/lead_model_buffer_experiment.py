"""Offline-only small long-range model anchor on the existing lead estimator.

No new range tracker, history retention, state feedback, or covariance update.
The model contribution is an absolute share of each current estimate, not a
recursive blend that silently accumulates toward the model at camera frequency.
"""

import math
import numpy as np

from openpilot.selfdrive.controls.lib.vision_lead_tracker import VisionLeadTracker


class LeadModelBufferExperiment:
  requires_tracks = True
  MAX_OFFSET = .5  # m/s, 1.8 km/h
  OFFSET_RATE = .2  # m/s per second; corrections revoke immediately when needed

  def __init__(self, maximum_weight=.10):
    self.maximum_weight = maximum_weight
    self.reset()

  def reset(self):
    self.reason = 'reset'
    self.model_weight = self.future_weight = self.delta = 0.
    self.range_speed = None
    self.time = self.age = None
    self.offsets = [0., 0.]

  def update(self, models, observations, raw_leads, baseline, timestamp, ego, yaw_rate, valid=True, tracks=None, **_kwargs):
    dt = timestamp - self.time if self.time is not None else .05
    self.model_weight = self.future_weight = self.delta = 0.
    self.range_speed = None
    if (not valid or not all(math.isfinite(v) for v in (timestamp, ego))
        or yaw_rate is None or not math.isfinite(yaw_rate) or abs(yaw_rate) > .1
        or not .01 <= dt <= .1 + 1e-6 or not 15. < ego < 50.
        or tracks is None or not len(tracks) == len(models) == len(observations) == len(raw_leads) == len(baseline) == 2):
      self.reset()
      return baseline
    groups = VisionLeadTracker._groups(observations)
    if len(groups) != 1:
      self.reset()
      return baseline
    indices, observation = groups[0]
    accepted = [i for i in indices if raw_leads[i].get('present', False) and baseline[i].get('present', False)]
    if not accepted or not 80. < observation.distance < 150. or observation.probability < .9:
      self.reset()
      return baseline
    if any(tracks[i] is None or not tracks[i].ready or tracks[i].mode != 'distance'
           or raw_leads[i].get('radar', False) or baseline[i].get('radar', False) for i in accepted):
      self.reset()
      return baseline
    age = min(tracks[i].age for i in accepted)
    if self.age is not None and age < self.age:
      self.offsets = [0., 0.]
    self.time, self.age = timestamp, age
    nominal = self.maximum_weight * float(np.interp(observation.distance, [80., 100., 140.], [0., .5, 1.]))
    output = list(baseline)
    for i in accepted:
      raw, current = raw_leads[i], baseline[i]
      if (not len(models[i].vStd) or not all(math.isfinite(v) for v in (models[i].vStd[0], raw['vLead'], current['vLead'], current['aLeadK']))
          or models[i].vStd[0] <= 0. or current['aLeadK'] < -.5):
        self.offsets[i] = 0.
        continue
      quality = float(np.interp(models[i].vStd[0], [1., 5.], [1., 0.]))
      weight = nominal * quality
      desired = float(np.clip(weight * (raw['vLead'] - current['vLead']), -self.MAX_OFFSET, self.MAX_OFFSET))
      closing = max(ego - current['vLead'], ego - raw['vLead'], 0.)
      if desired > 0. and observation.distance < 8. * closing:
        desired = 0.
      previous = self.offsets[i] if self.offsets[i] * desired > 0. else 0.
      offset = math.copysign(min(abs(desired), abs(previous) + self.OFFSET_RATE * dt), desired)
      self.offsets[i] = offset
      speed = max(0., current['vLead'] + offset)
      output[i] = dict(current, vLead=speed, vLeadK=speed, vRel=speed - ego)
      self.model_weight = max(self.model_weight, weight)
    self.reason = 'buffer'
    self.range_speed = baseline[accepted[0]]['vLead']
    self.delta = output[accepted[0]]['vLead'] - baseline[accepted[0]]['vLead']
    return output
