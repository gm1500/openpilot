#!/usr/bin/env python3
import math
import numpy as np
from collections import deque
from typing import Any

import capnp
from openpilot.cereal import messaging, log
from opendbc.car.structs import car
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL, Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.common.simple_kalman import KF1D
from openpilot.common.transformations.orientation import rot_from_euler
from openpilot.selfdrive.controls.lib.vision_lead_confidence import LeadUncertainty, VisionLeadConfidenceFilter
from openpilot.selfdrive.controls.lib.vision_lead_tracker import VisionLeadObservation, VisionLeadTracker
from openpilot.selfdrive.controls.lib.lead_transition import LeadTransitionRecovery
from openpilot.selfdrive.controls.lib.lead_reacquisition import LeadReacquisitionShadow
from openpilot.selfdrive.controls.lib.lead_future_motion import LeadFutureMotion
from openpilot.selfdrive.controls.lib.lead_responsive_motion import LeadResponsiveMotion
from openpilot.selfdrive.controls.lib.lead_departure_shadow import DepartureHead, LeadDepartureShadow


# Default lead acceleration decay set to 50% at 1s
_LEAD_ACCEL_TAU = 1.5

# radar tracks
SPEED, ACCEL = 0, 1     # Kalman filter states enum

# stationary qualification parameters
V_EGO_STATIONARY = 4.   # no stationary object flag below this speed

RADAR_TO_CAMERA = 1.52  # RADAR is ~ 1.5m ahead from center of mesh frame


class KalmanParams:
  def __init__(self, dt: float):
    # Lead Kalman Filter params, calculating K from A, C, Q, R requires the control library.
    # hardcoding a lookup table to compute K for values of radar_ts between 0.01s and 0.2s
    assert dt > .01 and dt < .2, "Radar time step must be between .01s and 0.2s"
    self.A = [[1.0, dt], [0.0, 1.0]]
    self.C = [1.0, 0.0]
    #Q = np.matrix([[10., 0.0], [0.0, 100.]])
    #R = 1e3
    #K = np.matrix([[ 0.05705578], [ 0.03073241]])
    dts = [i * 0.01 for i in range(1, 21)]
    K0 = [0.12287673, 0.14556536, 0.16522756, 0.18281627, 0.1988689,  0.21372394,
          0.22761098, 0.24069424, 0.253096,   0.26491023, 0.27621103, 0.28705801,
          0.29750003, 0.30757767, 0.31732515, 0.32677158, 0.33594201, 0.34485814,
          0.35353899, 0.36200124]
    K1 = [0.29666309, 0.29330885, 0.29042818, 0.28787125, 0.28555364, 0.28342219,
          0.28144091, 0.27958406, 0.27783249, 0.27617149, 0.27458948, 0.27307714,
          0.27162685, 0.27023228, 0.26888809, 0.26758976, 0.26633338, 0.26511557,
          0.26393339, 0.26278425]
    self.K = [[np.interp(dt, dts, K0)], [np.interp(dt, dts, K1)]]


class Track:
  def __init__(self, identifier: int, v_lead: float, kalman_params: KalmanParams):
    self.identifier = identifier
    self.cnt = 0
    self.aLeadTau = FirstOrderFilter(_LEAD_ACCEL_TAU, 0.45, DT_MDL)
    self.K_A = kalman_params.A
    self.K_C = kalman_params.C
    self.K_K = kalman_params.K
    self.kf = KF1D([[v_lead], [0.0]], self.K_A, self.K_C, self.K_K)

  def update(self, d_rel: float, y_rel: float, v_rel: float, v_lead: float):
    # relative values, copy
    self.dRel = d_rel   # LONG_DIST
    self.yRel = y_rel   # -LAT_DIST
    self.vRel = v_rel   # REL_SPEED
    self.vLead = v_lead

    # computed velocity and accelerations
    if self.cnt > 0:
      self.kf.update(self.vLead)

    self.vLeadK = float(self.kf.x[SPEED][0])
    self.aLeadK = float(self.kf.x[ACCEL][0])

    # Learn if constant acceleration
    if abs(self.aLeadK) < 0.5:
      self.aLeadTau.x = _LEAD_ACCEL_TAU
    else:
      self.aLeadTau.update(0.0)

    self.cnt += 1

  def get_RadarState(self, model_prob: float = 0.0):
    return {
      "dRel": float(self.dRel),
      "yRel": float(self.yRel),
      "vRel": float(self.vRel),
      "vLead": float(self.vLead),
      "vLeadK": float(self.vLeadK),
      "aLeadK": float(self.aLeadK),
      "aLeadTau": float(self.aLeadTau.x),
      "present": True,
      "modelProb": model_prob,
      "radar": True,
      "radarTrackId": self.identifier,
    }

  def potential_low_speed_lead(self, v_ego: float):
    # stop for stuff in front of you and low speed, even without model confirmation
    # Radar points closer than 0.75, are almost always glitches on toyota radars
    return abs(self.yRel) < 1.0 and (v_ego < V_EGO_STATIONARY) and (0.75 < self.dRel < 25)

  def __str__(self):
    ret = f"x: {self.dRel:4.1f}  y: {self.yRel:4.1f}  v: {self.vRel:4.1f}  a: {self.aLeadK:4.1f}"
    return ret


def laplacian_pdf(x: float, mu: float, b: float):
  b = max(b, 1e-4)
  return math.exp(-abs(x-mu)/b)


def match_vision_to_track(v_ego: float, lead: capnp._DynamicStructReader, tracks: dict[int, Track]):
  offset_vision_dist = lead.x[0] - RADAR_TO_CAMERA

  def prob(c):
    prob_d = laplacian_pdf(c.dRel, offset_vision_dist, lead.xStd[0])
    prob_y = laplacian_pdf(c.yRel, -lead.y[0], lead.yStd[0])
    prob_v = laplacian_pdf(c.vRel + v_ego, lead.v[0], lead.vStd[0])

    # This isn't exactly right, but it's a good heuristic
    return prob_d * prob_y * prob_v

  track = max(tracks.values(), key=prob)

  # if no 'sane' match is found return -1
  # stationary radar points can be false positives
  dist_sane = abs(track.dRel - offset_vision_dist) < max([(offset_vision_dist)*.25, 5.0])
  vel_sane = (abs(track.vRel + v_ego - lead.v[0]) < 10) or (v_ego + track.vRel > 3)
  if dist_sane and vel_sane:
    return track
  else:
    return None


def get_RadarState_from_vision(lead_msg: capnp._DynamicStructReader, v_ego: float, model_v_ego: float, lead_prob: float):
  lead_v_rel_pred = lead_msg.v[0] - model_v_ego
  return {
    "dRel": float(lead_msg.x[0] - RADAR_TO_CAMERA),
    "yRel": float(-lead_msg.y[0]),
    "vRel": float(lead_v_rel_pred),
    "vLead": float(v_ego + lead_v_rel_pred),
    "vLeadK": float(v_ego + lead_v_rel_pred),
    "aLeadK": float(lead_msg.a[0]),
    "aLeadTau": 0.3,
    "modelProb": float(lead_prob),
    "present": True,
    "radar": False,
    "radarTrackId": -1,
  }


def get_lead(v_ego: float, ready: bool, tracks: dict[int, Track], lead_msg: capnp._DynamicStructReader,
             model_v_ego: float, lead_prob: float, low_speed_override: bool = True) -> dict[str, Any]:
  # Determine leads, this is where the essential logic happens
  if len(tracks) > 0 and ready and lead_prob > .5:
    track = match_vision_to_track(v_ego, lead_msg, tracks)
  else:
    track = None

  lead_dict = {'present': False}
  if track is not None:
    lead_dict = track.get_RadarState(lead_prob)
  elif (track is None) and ready and (lead_prob > .5):
    lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego, lead_prob)

  if low_speed_override:
    low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]
    if len(low_speed_tracks) > 0:
      closest_track = min(low_speed_tracks, key=lambda c: c.dRel)

      # Only choose new track if it is actually closer than the previous one
      if (not lead_dict['present']) or (closest_track.dRel < lead_dict['dRel']):
        lead_dict = closest_track.get_RadarState()

  return lead_dict


class RadarD:
  def __init__(self, delay: float = 0.0):
    self.tracks: dict[int, Track] = {}
    self.kalman_params = KalmanParams(DT_MDL)
    self.lead_prob_filters = [FirstOrderFilter(0.0, 0.2, DT_MDL) for _ in range(2)]
    self.vision_lead_tracker = VisionLeadTracker()
    self.vision_confidence_filter = VisionLeadConfidenceFilter()
    self.lead_transition_recovery = LeadTransitionRecovery()
    self.lead_future_motion = LeadFutureMotion()
    self.lead_responsive_motion = LeadResponsiveMotion()
    self.lead_reacquisition_shadow = LeadReacquisitionShadow()
    self.lead_departure_shadow = LeadDepartureShadow()
    self.departure_ego_samples = deque(maxlen=4)
    self.departure_motion_samples = deque(maxlen=4)
    self.departure_calibration_samples = deque(maxlen=4)
    self.last_departure_log = -math.inf
    self.last_departure_reason = None
    self.departure_was_active = False
    self.last_reacquisition_log = -math.inf
    self.reacquisition_was_active = False
    self.last_vision_log = 0.0

    self.v_ego = 0.0
    self.v_ego_hist = deque([0.0], maxlen=int(round(delay / DT_MDL))+1)
    self.last_v_ego_frame = -1

    self.radar_state: capnp._DynamicStructBuilder | None = None
    self.radar_state_valid = False

    self.ready = False

  @staticmethod
  def _association_yaw_rate(sm, timestamp):
    """Optional measured yaw for association; never a radar health dependency."""
    try:
      names = ('deviceMotion', 'extrinsicsCalibration')
      if not all(sm.seen.get(name, False) and sm.valid[name] for name in names):
        return None
      motion, calibration = (sm[name] for name in names)
      angular = motion.angularVelocityDevice
      values = tuple(float(v) for v in (angular.x, angular.y, angular.z))
      stds = (angular.xStd, angular.yStd, angular.zStd)
      rpy = tuple(float(v) for v in calibration.rpyCalib)
      measured = motion.timestamp * 1e-9
      published = sm.logMonoTime.get('deviceMotion', 0) * 1e-9
      calibrated = sm.logMonoTime.get('extrinsicsCalibration', 0) * 1e-9
      if not (all(math.isfinite(v) for v in (timestamp, measured, published, calibrated, *values, *rpy))
              and len(rpy) == 3 and all(math.isfinite(v) and v >= 0. for v in stds)
              and motion.inputsOK and motion.sensorsOK and motion.posenetOK and angular.valid
              and str(calibration.calStatus) == 'calibrated'
              and 0. <= measured <= timestamp and measured <= published <= timestamp + .1
              and timestamp - measured <= .1 + 1e-6
              and 0. <= calibrated <= measured and timestamp - calibrated <= 2. + 1e-6):
        return None
      # Device -> calibrated FRD, matching PoseCalibrator. Future model yaw
      # remains a prediction and is deliberately not substituted here.
      yaw = float((rot_from_euler(rpy).T @ values)[2])
      return yaw if math.isfinite(yaw) else None
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
      return None

  def _observe_departure(self, sm, model, heads, timestamp, model_v_ego, valid):
    if self.lead_departure_shadow is None:
      return
    try:
      # Optional measured sources are excluded from radarState health checks.
      # carControl can republish a cached pose, so read the original motion and
      # its filter timestamp. Keep a few copied samples when the newest source
      # measurement postdates this model. No predicted orientation rate is used.
      selected = []
      for name, samples in (('carState', self.departure_ego_samples), ('deviceMotion', self.departure_motion_samples),
                            ('extrinsicsCalibration', self.departure_calibration_samples)):
        if sm.seen.get(name, False):
          message = sm[name]
          published = sm.logMonoTime.get(name, 0) * 1e-9
          source_time, source_valid = published, bool(sm.valid[name])
          if name == 'carState':
            value = float(message.vEgo)
          elif name == 'deviceMotion':
            angular = message.angularVelocityDevice
            value = tuple(float(v) for v in (angular.x, angular.y, angular.z))
            stds = (angular.xStd, angular.yStd, angular.zStd)
            source_time = message.timestamp * 1e-9  # locationd's filter measurement time
            source_valid &= (message.inputsOK and message.sensorsOK and message.posenetOK and angular.valid
                             and all(math.isfinite(v) for v in value) and all(math.isfinite(v) and v >= 0. for v in stds))
          else:
            value = tuple(float(v) for v in message.rpyCalib)
            source_valid &= str(message.calStatus) == 'calibrated' and len(value) == 3 and all(math.isfinite(v) for v in value)
          sample = (published, source_time, source_valid, value)
          if samples and sample[0] < samples[-1][0]:
            samples.clear()
          if samples and sample[0] == samples[-1][0]:
            samples[-1] = sample
          else:
            samples.append(sample)
        else:
          samples.clear()
        # Calibration must already be available at the motion measurement time.
        limit = selected[1][1] if name == 'extrinsicsCalibration' and selected[1] is not None else timestamp
        time_index = 1 if name == 'deviceMotion' else 0
        causal = next((s for s in reversed(samples) if s[time_index] <= limit), None)
        max_age = 2. if name == 'extrinsicsCalibration' else .1
        # A motion measurement before model time may be published afterward.
        # It is causal at this call because it has already been received; retain
        # a bounded publication delay and never use a future filter timestamp.
        latest_publication = timestamp + .1 if name == 'deviceMotion' else timestamp
        selected.append(causal if (causal is not None and causal[2] and 0. <= causal[1] <= causal[0] <= latest_publication
                                   and timestamp - causal[1] <= max_age + 1e-6) else None)
      ego_sample, motion, calibration = selected
      ego = ego_sample[3] if ego_sample is not None else None
      # Match PoseCalibrator's device -> calibrated FRD transform exactly.
      yaw = float((rot_from_euler(calibration[3]).T @ motion[3])[2]) if motion is not None and calibration is not None else None
      context_valid = valid and ego is not None and yaw is not None
      copied = tuple(DepartureHead(float(h.x[0]), float(h.y[0]), (ego if ego is not None else self.v_ego) + float(h.v[0]) - model_v_ego,
                                   float(h.prob), float(h.xStd[0]) if len(h.xStd) else math.nan,
                                   float(h.yStd[0]) if len(h.yStd) else math.nan) for h in heads)
      self.lead_departure_shadow.update(copied, timestamp, ego, yaw, str(model.meta.laneChangeState),
                                       str(model.meta.laneChangeDirection), context_valid)
      status = self.lead_departure_shadow.status
      changed = status.reason != self.last_departure_reason or status.active != self.departure_was_active
      if (changed or status.active) and timestamp - self.last_departure_log >= .2:
        cloudlog.info(" ".join((
          f"lead-departure-shadow: reason={status.reason} active={int(status.active)} near={status.near_head}",
          f"age={status.age:.3f} residual={status.range_residual:.3f} dx={status.range_separation:.3f}",
          f"dv={status.speed_separation:.3f} dy={status.lateral_separation:.3f} can_ignore_lead=0",
        )))
        self.last_departure_log = timestamp
        self.last_departure_reason = status.reason
        self.departure_was_active = status.active
    except Exception:
      # No observer failure is allowed to replace or prevent publishing leads.
      cloudlog.exception("lead-departure-shadow disabled after observer failure")
      self.lead_departure_shadow = None

  def update(self, sm: messaging.SubMaster, rr: car.RadarData):
    self.ready = sm.seen['modelV2']

    if sm.recv_frame['carState'] != self.last_v_ego_frame:
      self.v_ego = sm['carState'].vEgo
      self.v_ego_hist.append(self.v_ego)
      self.last_v_ego_frame = sm.recv_frame['carState']

    ar_pts = {pt.trackId: [pt.dRel, pt.yRel, pt.vRel] for pt in rr.points}

    # *** remove missing points from meta data ***
    for ids in list(self.tracks.keys()):
      if ids not in ar_pts:
        self.tracks.pop(ids, None)

    # *** compute the tracks ***
    for ids in ar_pts:
      rpt = ar_pts[ids]

      # align v_ego by a fixed time to align it with the radar measurement
      v_lead = rpt[2] + self.v_ego_hist[0]

      # create the track if it doesn't exist or it's a new track
      if ids not in self.tracks:
        self.tracks[ids] = Track(ids, v_lead, self.kalman_params)
      self.tracks[ids].update(rpt[0], rpt[1], rpt[2], v_lead)

    # *** publish radarState ***
    self.radar_state_valid = sm.all_checks()
    self.radar_state = log.RadarState.new_message()
    self.radar_state.mdMonoTime = sm.logMonoTime['modelV2']
    self.radar_state.radarErrors = rr.errors

    if len(sm['modelV2'].velocity.x):
      model_v_ego = sm['modelV2'].velocity.x[0]
    else:
      model_v_ego = self.v_ego
    leads_v3 = sm['modelV2'].leadsV3
    if len(leads_v3) > 1:
      # Every motion stage must receive the same two slots as the observations
      # and accepted leads, even when the model publishes additional predictions.
      leads_v3 = [leads_v3[i] for i in range(2)]
      for i in range(2):
        # Asymmetric filter on lead prob to keep lead when uncertain
        lead_prob = leads_v3[i].prob
        if lead_prob > self.lead_prob_filters[i].x:
          self.lead_prob_filters[i].x = lead_prob
        else:
          self.lead_prob_filters[i].update(lead_prob)

      timestamp = self.radar_state.mdMonoTime * 1e-9
      observations, leads, uncertainties = [], [], []
      for i in range(2):
        lead_msg = leads_v3[i]
        leads.append(get_lead(self.v_ego, self.ready, self.tracks, lead_msg, model_v_ego, self.lead_prob_filters[i].x,
                              low_speed_override=(i == 0)))
        # Observe below control acceptance; raw confidence weights private history.
        observations.append(VisionLeadObservation(float(lead_msg.x[0] - RADAR_TO_CAMERA), float(-lead_msg.y[0]),
                                                  float(lead_msg.xStd[0]) if len(lead_msg.xStd) else float('nan'), float(lead_msg.prob)))
        uncertainties.append(LeadUncertainty(
          float(lead_msg.prob), observations[-1].std,
          float(lead_msg.vStd[0]) if len(lead_msg.vStd) else float('nan'),
          float(lead_msg.aStd[0]) if len(lead_msg.aStd) else float('nan'),
        ))
      valid = self.radar_state_valid and self.ready
      raw_leads = leads
      model = sm['modelV2']
      yaw_rate = float(model.orientationRate.z[0]) if len(model.orientationRate.z) else None
      baseline = self.vision_lead_tracker.update(observations, raw_leads, timestamp, self.v_ego, valid, yaw_rate=yaw_rate,
                                                association_yaw_rate=self._association_yaw_rate(sm, timestamp))
      leads = self.vision_confidence_filter.update(raw_leads, baseline, uncertainties, self.vision_lead_tracker.slots,
                                                  timestamp, self.v_ego, valid)
      leads = self.lead_transition_recovery.update(
        observations, raw_leads, leads, [uncertainty.speed_std for uncertainty in uncertainties],
        timestamp, self.v_ego, yaw_rate, valid, tracks=self.vision_lead_tracker.slots)
      leads = self.lead_future_motion.update(leads_v3, raw_leads, leads, self.vision_lead_tracker.slots,
                                            timestamp, self.v_ego, valid)
      if self.lead_responsive_motion is not None:
        try:
          leads = self.lead_responsive_motion.update(leads_v3, observations, raw_leads, leads,
                                                    timestamp, self.v_ego, yaw_rate, valid, tracks=self.vision_lead_tracker.slots)
        except Exception:
          # A failed POC must leave the already accepted lead output intact.
          cloudlog.exception("responsive lead motion disabled after estimator failure")
          self.lead_responsive_motion = None
      # Read-only experiment: its candidates never replace `leads`. A failure
      # disables only the observer for this process lifetime, not lead control.
      if self.lead_reacquisition_shadow is not None:
        try:
          shadow = self.lead_reacquisition_shadow
          shadow.update(observations, raw_leads, leads, self.vision_lead_tracker.slots,
                        timestamp, self.v_ego, yaw_rate, valid)
          active = any(status.active for status in shadow.status)
          if active != self.reacquisition_was_active or (active and timestamp - self.last_reacquisition_log >= .1):
            for i, status in enumerate(shadow.status):
              cloudlog.info(" ".join((
                f"lead-reacquire-shadow: slot={i} reason={status.reason} active={int(status.active)}",
                f"age={status.age:.3f} dr={status.delta_distance:.3f} dv={status.delta_speed:.3f}",
                f"std_ratio={status.std_ratio:.2f} frames={shadow.bridge_frames[i]}",
              )))
            self.last_reacquisition_log = timestamp
            self.reacquisition_was_active = active
        except Exception:
          cloudlog.exception("lead-reacquire-shadow disabled after observer failure")
          self.lead_reacquisition_shadow = None
      self._observe_departure(sm, model, leads_v3, timestamp, model_v_ego, valid)
      for i, field in enumerate(('leadOne', 'leadTwo')):
        setattr(self.radar_state, field, leads[i])
        if timestamp - self.last_vision_log >= 1.0:
          track = self.vision_lead_tracker.slots[i]
          confidence = self.vision_confidence_filter.status[i]
          future = self.lead_future_motion.status[i]
          responsive = self.lead_responsive_motion
          shadow = self.lead_reacquisition_shadow
          cloudlog.info(" ".join((
            f"lead-pre: slot={i} mode={track.mode if track else 'fallback'}",
            f"track_id={track.track_id if track else -1} generation={track.generation if track else -1}",
            f"association={self.vision_lead_tracker.association_reasons[i]}",
            f"age={track.age if track else 0.0:.2f} ready={int(track.ready) if track else 0}",
            f"continuity={track.continuity.reason if track else 'missing'} single_dv={track.single_closing_offset if track else 0.0:.2f}",
            f"v={leads[i].get('vLead', 0.0):.2f}",
            f"confidence={confidence.reason} gains={confidence.range_gain:.3f},{confidence.accel_gain:.3f}",
            f"conf_dr={leads[i].get('dRel', 0.0) - baseline[i].get('dRel', 0.0):.3f}",
            f"conf_da={leads[i].get('aLeadK', 0.0) - baseline[i].get('aLeadK', 0.0):.3f}",
            f"lead_transition={self.lead_transition_recovery.reason} dv={self.lead_transition_recovery.correction:.2f}",
            f"future_motion={future.reason} future_v={future.speed:.2f} future_std={future.speed_std:.2f}",
            f"future_w={future.weight:.3f} future_dv={future.delta_speed:.3f} future_dr={future.delta_distance:.3f}",
            f"responsive={responsive.reason if responsive is not None else 'disabled'}",
            f"responsive_dv={responsive.delta if responsive is not None else 0.0:.3f}",
            f"responsive_fast={int(responsive.motion.fast_response) if responsive is not None else 0}",
            f"reacquire_shadow={shadow.status[i].reason if shadow is not None else 'disabled'}",
            f"shadow_frames={shadow.bridge_frames[i] if shadow is not None else 0}",
          )))
      if timestamp - self.last_vision_log >= 1.0:
        self.last_vision_log = timestamp
    else:
      self.vision_lead_tracker.reset()
      self.vision_confidence_filter.reset()
      self.lead_transition_recovery.reset()
      self.lead_future_motion.reset()
      if self.lead_responsive_motion is not None:
        self.lead_responsive_motion.reset()
      if self.lead_reacquisition_shadow is not None:
        self.lead_reacquisition_shadow.reset('missing_model')
      if self.lead_departure_shadow is not None:
        self.lead_departure_shadow.reset('missing_model')

  def publish(self, pm: messaging.PubMaster):
    assert self.radar_state is not None

    radar_msg = messaging.new_message("radarState")
    radar_msg.valid = self.radar_state_valid
    radar_msg.radarState = self.radar_state
    pm.send("radarState", radar_msg)


# fuses camera and radar data for best lead detection
def main() -> None:
  config_realtime_process(5, Priority.CTRL_LOW)

  # wait for stats about the car to come in from controls
  cloudlog.info("radard is waiting for CarParams")
  CP = messaging.log_from_bytes(Params().get("CarParams", block=True), car.CarParams)
  cloudlog.info("radard got CarParams")

  # *** setup messaging
  optional_motion = ['deviceMotion', 'extrinsicsCalibration']
  sm = messaging.SubMaster(['modelV2', 'carState', 'radarTracks', *optional_motion], poll='modelV2',
                           ignore_alive=optional_motion, ignore_avg_freq=optional_motion, ignore_valid=optional_motion)
  pm = messaging.PubMaster(['radarState'])

  RD = RadarD(CP.radarDelay)

  while 1:
    sm.update()

    RD.update(sm, sm['radarTracks'])
    RD.publish(pm)


if __name__ == "__main__":
  main()
