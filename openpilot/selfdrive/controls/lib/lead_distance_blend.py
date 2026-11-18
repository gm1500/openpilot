"""Range-scheduled blend base for direct model and derived lead velocity.

The distance state is a private copy of the production Kalman equations, with
the previous experiment's brief-outlier continuity. Runtime use goes through
the guarded LeadResponsiveMotion subclass. All inputs remain correlated camera
estimates, not independent sensors.
"""

import math

import numpy as np

from openpilot.selfdrive.controls.lib.lead_future_motion import FutureMotion
from openpilot.selfdrive.controls.lib.lead_kalman_motion import KalmanLeadMotion
from openpilot.selfdrive.controls.lib.vision_lead_tracker import VisionLeadTracker


class LeadDistanceBlend:
  DISTANCES = [30., 60., 100., 140.]
  MODEL_WEIGHTS = {
    'early': [0., .10, .35, .65],
    'balanced': [0., .25, .60, .85],
    'model_heavy': [.10, .50, .80, .95],
    'buffer_0': [0., 0., 0., 0.],
    'buffer_5': [0., 0., .025, .05],
    'buffer_10': [0., 0., .05, .10],
    'buffer_15': [0., 0., .075, .15],
  }

  def __init__(self, schedule='balanced', use_future=False, future_share=None, closing_guard=False):
    self.schedule = schedule
    self.use_future = use_future
    self.future_share = future_share
    self.closing_guard = closing_guard
    self.motion = KalmanLeadMotion(use_future=False)
    self.reset()

  def reset(self):
    self.motion.reset()
    self.reason = 'reset'
    self.model_weight = 1.
    self.future_weight = 0.
    self.range_speed = None
    self.delta = 0.

  def update(self, models, observations, raw_leads, baseline, timestamp, ego, yaw_rate, valid=True, advance=True):
    if advance:
      self.motion.update(models, observations, raw_leads, baseline, timestamp, ego, yaw_rate, valid)
    self.reason = self.motion.reason
    self.model_weight, self.future_weight, self.range_speed, self.delta = 1., 0., None, 0.
    kf, fit = self.motion.kalman, self.motion.fit
    # Presence, target changes, urgent braking and low-speed behavior retain
    # the current pipeline. No private history is allowed to create a lead.
    if kf is None or fit is None or self.motion.reason in ('range_quarantine', 'uncertain_motion', 'range_uncertainty'):
      return baseline
    groups = VisionLeadTracker._groups(observations)
    if len(groups) != 1:
      return baseline
    indices, observation = groups[0]
    accepted = [i for i in indices if raw_leads[i].get('present', False) and baseline[i].get('present', False)]
    if not accepted or observation.probability < .9:
      return baseline
    if any(not len(models[i].vStd) or not math.isfinite(models[i].vStd[0]) or models[i].vStd[0] <= 0. for i in accepted):
      return baseline
    if max(raw_leads[i]['vLead'] for i in accepted) - min(raw_leads[i]['vLead'] for i in accepted) > 3.:
      self.reason = 'velocity_disagreement'
      return baseline

    # Nominal distance schedule; uncertain range velocity moves the weight
    # toward the direct model, without shrinking covariance through averaging.
    quality = float(np.interp(fit.error, [1., 3.], [1., 0.]))
    nominal = float(np.interp(observation.distance, self.DISTANCES, self.MODEL_WEIGHTS[self.schedule]))
    weight = 1. - (1. - nominal) * quality
    small_buffer = self.schedule.startswith('buffer_')
    if small_buffer:
      nominal = float(np.interp(observation.distance, [30., 80., 100., 140.], self.MODEL_WEIGHTS[self.schedule]))
      model_quality = float(np.interp(max(models[i].vStd[0] for i in accepted), [1., 5.], [1., 0.]))
      weight = nominal * model_quality
    derived = float(getattr(self.motion, 'output_speed', kf.x[1]))
    if not 0. <= derived <= 70.:
      return baseline
    motions = [FutureMotion.from_model(models[i], max_speed_std=24.) for i in accepted] if self.use_future else []
    valid_future = (bool(motions) and all(m is not None for m in motions)
                    and max(m.speed for m in motions) - min(m.speed for m in motions) < 2.)
    output = list(baseline)
    for j, i in enumerate(accepted):
      model = raw_leads[i]['vLead']
      if valid_future:
        # Future positions provide a bounded prior on current velocity; do
        # not substitute velocity at t+2 seconds for velocity at the present.
        variance = float(models[i].vStd[0])**2
        future_weight = min(.20, variance / (variance + motions[j].speed_std**2))
        if self.future_share is not None:
          future_weight = float(np.clip(self.future_share, 0., 1.))
        model = (1. - future_weight) * model + future_weight * motions[j].speed
        self.future_weight = max(self.future_weight, weight * future_weight)
      speed = float(np.clip(weight * model + (1. - weight) * derived, 0., 70.))
      if small_buffer:
        speed = derived + float(np.clip(speed - derived, -.5, .5))
      supported_closing = ego - derived - 2. * fit.error
      if self.closing_guard and supported_closing > .5 and observation.distance < 12. * supported_closing:
        # An optimistic model must not undo an existing slower-lead estimate
        # when the current range history supports closing within this horizon.
        speed = min(speed, baseline[i]['vLead'])
      output[i] = dict(baseline[i], vLead=speed, vLeadK=speed, vRel=speed - ego)
    self.reason, self.model_weight, self.range_speed = 'blend', weight, derived
    self.delta = output[accepted[0]]['vLead'] - baseline[accepted[0]]['vLead']
    return output
