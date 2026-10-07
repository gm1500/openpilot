"""SLC enables E2E at qualified junctions or while usable map speed is unavailable."""
import math
from dataclasses import dataclass

from openpilot.selfdrive.modeld.constants import ModelConstants

# Aggressive, standard, relaxed. These size entry distance only, not actuator limits.
COMFORT_BRAKE = (2.2, 1.8, 1.4)  # m/s^2
COMFORT_JERK = (1.2, 1.0, .8)  # m/s^3
RELEASE_JERK = 1.0
PASS_DISTANCE = 20.0  # Matches mapd's signed junction look-behind.
MAP_GRACE = 2.25  # One missed GPS match plus confirmation of the next junction.
MAP_GRACE_DISTANCE = 40.0


def activation_distance(v_ego: float, a_ego: float, personality: int, actuator_delay: float) -> float:
  """Distance through response delay, a jerk ramp, and constant comfortable braking."""
  if not all(math.isfinite(x) for x in (v_ego, a_ego, actuator_delay)) or not -.1 <= v_ego <= 75:
    return 0.
  v_ego = max(0., v_ego)  # signed-speed filter noise around standstill; gear is checked by the caller
  profile = int(personality) if int(personality) in (0, 1, 2) else 1
  brake, jerk = COMFORT_BRAKE[profile], COMFORT_JERK[profile]
  accel = max(-brake, min(a_ego, 3.))
  # One second covers map/model qualification and planning response, in addition
  # to the car's actuator delay. Never assume existing hard braking will persist.
  response = 1. + max(0., min(actuator_delay, 2.))
  if accel < 0:
    response = min(response, v_ego / -accel)
  distance = v_ego * response + .5 * accel * response**2
  speed = max(0., v_ego + accel * response)
  ramp = min((accel + brake) / jerk, (accel + math.sqrt(accel**2 + 2 * jerk * speed)) / jerk)
  distance += speed * ramp + .5 * accel * ramp**2 - jerk * ramp**3 / 6
  speed = max(0., speed + accel * ramp - .5 * jerk * ramp**2)
  return max(20., distance + speed**2 / (2 * brake) + 12.)


def slc_fallback_request(sm, now: float, previous_fallback: bool) -> tuple[bool, bool]:
  """Read card's mode request and order recovery after its applied SET arrives."""
  service = 'mapCruiseState'
  if (service not in getattr(sm, 'services', ()) or not sm.valid[service] or
      not 0 <= now - sm.recv_time[service] <= .8 or
      not 0 <= now - sm.logMonoTime[service] * 1e-9 <= .8):
    # A missing status cannot establish a new request. Keep an established
    # fallback until card confirms recovery; selfdriveState still owns SLC off.
    return previous_fallback, previous_fallback
  msg = sm[service]
  automatic = msg.automaticE2e and str(msg.state) in ('active', 'armed', 'waiting', 'paused')
  fallback = automatic and msg.e2eFallback
  if automatic and previous_fallback and not fallback:
    # carState and mapCruiseState are separate sockets. Never remove the E2E
    # constraint while an older high SET is still visible to this planner.
    actual_set = sm['carState'].vCruise / 3.6
    fallback = not math.isfinite(msg.setSpeed) or abs(actual_set - msg.setSpeed) > .05
  return automatic, fallback


@dataclass(frozen=True)
class MapApproach:
  valid: bool = False
  node_id: int = 0
  distance: float = 0.
  observation_time: float = 0.
  can_hold: bool = False


def map_approach(sm, now: float) -> MapApproach:
  service = 'mapTrafficControl'
  if service not in getattr(sm, 'services', ()):
    return MapApproach()
  msg = sm[service]
  times = (sm.recv_time[service], sm.logMonoTime[service] * 1e-9)
  if not all(0 <= now - t <= 1. for t in times):
    return MapApproach()
  # mapd reports noRoadMatch with an invalid match and zero position fields.
  # Only a fresh, explicit matching interruption may retain an OLD approach;
  # the policy also bounds age of the last validated real GPS observation.
  can_hold = msg.reason in ('noRoadMatch', 'roadAlignment', 'ambiguousRoad', 'ambiguousFork')
  if not sm.valid[service]:
    return MapApproach(can_hold=can_hold)
  if (not 0 <= now - msg.gpsMonoTime * 1e-9 <= 3. or
      not 0 <= now - msg.positionMonoTime * 1e-9 <= 3.):
    return MapApproach()
  if str(msg.kind) == 'none':
    return MapApproach(valid=True, observation_time=msg.gpsMonoTime * 1e-9, can_hold=can_hold)
  if str(msg.kind) != 'junction' or msg.nodeId == 0 or not math.isfinite(msg.distance):
    return MapApproach()
  return MapApproach(True, msg.nodeId, msg.distance, msg.gpsMonoTime * 1e-9)


@dataclass(frozen=True)
class ModelIntent:
  valid: bool = False
  slowing: bool = False
  stop_distance: float = -1.


def model_intent(model, v_ego: float) -> ModelIntent:
  """Use speed reduction and sustained near-zero speed, not the drawn path length.

  Speed magnitude avoids interpreting a turn's falling forward component as a
  stop. Relative speed reduction avoids an instantaneous model/vehicle bias.
  """
  arrays = (model.velocity.x, model.velocity.y, model.position.x, model.position.y)
  times = ModelConstants.T_IDXS
  if any(len(a) != len(times) or not all(math.isfinite(x) for x in a) for a in arrays):
    return ModelIntent()
  vx, vy, px, py = arrays
  speeds = [math.hypot(x, y) for x, y in zip(vx, vy, strict=True)]
  if max(speeds) > 90:
    return ModelIntent()
  distance = [0.]
  for i in range(1, len(times)):
    step = math.hypot(px[i] - px[i - 1], py[i] - py[i - 1])
    if step > 100 * (times[i] - times[i - 1]) + 2:
      return ModelIntent()
    distance.append(distance[-1] + step)
  drop = max(1., .08 * max(v_ego, 0.))
  slowing = any(1. <= times[i] <= 6. and max(speeds[i:i + 2]) <= speeds[0] - drop for i in range(len(times) - 1))
  stop_distance = -1.
  for i in range(1, len(times) - 1):
    end = next((j for j in range(i + 1, len(times)) if times[j] - times[i] >= .75), None)
    if end is not None and max(speeds[i:end + 1]) <= .5:
      stop_distance = distance[i]
      break
  return ModelIntent(True, slowing, stop_distance)


class ConditionalExperimental:
  def __init__(self, dt: float, actuator_delay: float):
    self.dt = dt
    self.actuator_delay = actuator_delay
    self.reset()

  def reset(self):
    self.armed = False
    self.speed_fallback = False
    self._clear_assistance()
    self.target_id = self.candidate_id = self.candidate_samples = 0
    self.passed_id = 0
    self.last_observation = self.last_position_observation = 0.
    self.last_qualified_time = 0.
    self.last_update_time = None
    self.gap_time = self.gap_distance = 0.
    self.state, self.reason = 'off', 'disabled'
    self.activation_distance = self.target_distance = 0.
    self.activation_speed = 0.
    self.intent = ModelIntent()
    self.map_valid = False

  def _clear_assistance(self):
    self.active = self.contributing = self.stop_requested = False
    self.accel_limit = None

  def _update_approach(self, approach, now, v_ego):
    elapsed = 0. if self.last_update_time is None else now - self.last_update_time
    self.last_update_time = now
    self.map_valid = approach.valid
    has_target = approach.valid and approach.node_id != 0
    regressed = has_target and approach.observation_time < self.last_observation
    if approach.valid:
      self.last_position_observation = max(self.last_position_observation, approach.observation_time)
    qualified = False
    self.reason = 'noTarget' if approach.valid else 'mapLost'
    if has_target:
      if approach.node_id != self.candidate_id or regressed:
        self.candidate_id, self.candidate_samples = approach.node_id, 1
        self.last_observation = approach.observation_time
      elif approach.observation_time > self.last_observation:
        self.candidate_samples += 1
        self.last_observation = approach.observation_time
      if approach.node_id == self.passed_id and approach.distance > max(60., self.activation_distance):
        self.passed_id = 0
      if self.armed and approach.node_id == self.target_id and approach.distance < -PASS_DISTANCE:
        self.passed_id = approach.node_id
      in_range = 0 <= approach.distance <= self.activation_distance and approach.node_id != self.passed_id
      # An established approach stays qualified while slowing or changing SET.
      same_approach = self.armed and approach.node_id == self.target_id and approach.distance >= -PASS_DISTANCE
      qualified = not regressed and (same_approach or self.candidate_samples >= 2 and in_range)
      if qualified:
        self.armed, self.target_id = True, approach.node_id
        self.target_distance = approach.distance
        self.reason = 'junction'
      else:
        self.reason = ('targetPassed' if approach.node_id == self.passed_id else
                       'targetUnconfirmed' if self.candidate_samples < 2 else 'outsideRange')
    else:
      in_range = False

    if qualified:
      self.gap_time = self.gap_distance = 0.
      self.last_qualified_time = now
    else:
      self.gap_time = now - self.last_qualified_time
      self.gap_distance += max(v_ego, 0.) * elapsed
      # Keep the OLD qualified window while a nearby successor confirms, or
      # through an explicit matching interruption. Candidate changes never
      # refresh this budget. No grace for stale data or a definite clear road.
      hold = (self.armed and not regressed and (in_range or approach.can_hold) and
              self.gap_time <= MAP_GRACE and self.gap_distance <= MAP_GRACE_DISTANCE and
              0 <= now - self.last_position_observation <= 3.)
      if hold:
        self.reason = 'junctionHandoff' if has_target else 'mapHold'
      else:
        self.armed = False
        self.target_id = 0
        self.target_distance = approach.distance if has_target else 0.
        if not has_target:
          self.candidate_id = self.candidate_samples = 0

  def update(self, *, enabled, eligible, approach, now, fallback=False, model, v_ego, v_set, personality,
             regular_accel, regular_stop, e2e_accel, e2e_stop):
    if not enabled:
      self.reset()
      return regular_accel, False, False
    if v_set is None or not math.isfinite(v_set) or not 0 < v_set <= 75:
      self.reset()
      self.state, self.reason = 'ready', 'invalidSetSpeed'
      return regular_accel, False, False
    self.activation_speed = v_set
    self.activation_distance = activation_distance(v_set, 0., personality, self.actuator_delay)
    self.intent = model_intent(model, v_ego)
    if (self.activation_distance <= 0 or not math.isfinite(v_ego) or not -.1 <= v_ego <= 75 or not math.isfinite(now) or
        self.last_update_time is not None and now < self.last_update_time):
      self.reset()
      self.state, self.reason = 'ready', 'invalidKinematics'
      return regular_accel, False, False

    can_assist = eligible and self.intent.valid and all(math.isfinite(x) for x in (regular_accel, e2e_accel))
    # A stop already requested by E2E may remain held at rest across map loss.
    # Loss of the map cannot create a new stop request.
    holding = can_assist and self.stop_requested and v_ego < .3 and e2e_stop
    if (self.speed_fallback and not fallback and approach.valid and approach.node_id == self.target_id and
        approach.distance > self.activation_distance):
      self.armed = False  # recovered lower SET must not inherit the 105-kph entry window
    self.speed_fallback = fallback
    self._update_approach(approach, now, v_ego)
    if not can_assist:
      self._clear_assistance()
      self.state = 'inRange' if self.armed else 'ready'
      self.reason = 'inactive' if not eligible else 'invalidModel'
      return regular_accel, False, False

    # Either automatic condition directly enables the E2E candidate, including
    # a go or positive-acceleration proposal. No model-slowing confirmation gate.
    self.active = fallback or self.armed or holding
    automatic_stop = self.active and e2e_stop
    target = min(regular_accel, e2e_accel) if self.active else regular_accel
    # Preserve stronger regular braking and soften only the return to gas.
    if self.accel_limit is not None:
      target = min(target, self.accel_limit + RELEASE_JERK * self.dt)
    selected = min(regular_accel, target)
    self.contributing = selected < regular_accel - .01 or automatic_stop and not regular_stop
    self.accel_limit = selected if selected < regular_accel else None
    self.stop_requested = automatic_stop and (self.armed or holding)
    self.state = 'active' if self.active else 'inRange' if self.armed else 'ready'
    if fallback:
      self.reason = 'noSpeedLimit'
    elif holding and not self.armed:
      self.reason = 'holdingStop'
    elif self.contributing and not self.active:
      self.reason = 'releasing'
    return selected, automatic_stop, self.contributing

  def publish(self, msg, regular_accel, e2e_accel, e2e_stop):
    msg.state = self.state
    msg.reason = self.reason
    msg.targetId = self.target_id
    msg.targetDistance = self.target_distance
    msg.activationDistance = self.activation_distance
    msg.activationSpeed = self.activation_speed
    msg.modelStopDistance = self.intent.stop_distance
    msg.regularAcceleration = regular_accel
    msg.modelAcceleration = e2e_accel
    msg.modelShouldStop = e2e_stop
    msg.modelSlowing = self.intent.slowing
    msg.mapValid = self.map_valid
    msg.armed = self.armed
    msg.contributing = self.contributing
    msg.e2eEnabled = self.active
