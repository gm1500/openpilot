"""Map-gated E2E slowing. The map sizes an attention window, never a brake command."""
import math
from dataclasses import dataclass

from openpilot.selfdrive.modeld.constants import ModelConstants

# Aggressive, standard, relaxed. These size entry distance only, not actuator limits.
COMFORT_BRAKE = (2.2, 1.8, 1.4)  # m/s^2
COMFORT_JERK = (1.2, 1.0, .8)  # m/s^3
BRAKE_CONFIRM = .25
GO_CONFIRM = .3
RELEASE_JERK = 1.0
PASS_DISTANCE = 12.0


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


@dataclass(frozen=True)
class MapApproach:
  valid: bool = False
  node_id: int = 0
  distance: float = 0.
  observation_time: float = 0.


def map_approach(sm, now: float) -> MapApproach:
  service = 'mapTrafficControl'
  if service not in sm.services:
    return MapApproach()
  msg = sm[service]
  times = (sm.recv_time[service], sm.logMonoTime[service] * 1e-9)
  if (not sm.valid[service] or not all(0 <= now - t <= 1. for t in times) or
      not 0 <= now - msg.gpsMonoTime * 1e-9 <= 3. or
      not 0 <= now - msg.positionMonoTime * 1e-9 <= 3.):
    return MapApproach()
  if str(msg.kind) == 'none':
    return MapApproach(valid=True)
  if str(msg.kind) not in ('stopSign', 'trafficLight') or msg.nodeId == 0 or not math.isfinite(msg.distance):
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
    self._clear_assistance()
    self.target_id = self.candidate_id = self.candidate_samples = 0
    self.passed_id = 0
    self.last_observation = 0.
    self.state, self.reason = 'off', 'disabled'
    self.activation_distance = self.target_distance = 0.
    self.intent = ModelIntent()
    self.map_valid = False

  def _clear_assistance(self):
    self.braking = self.contributing = False
    self.brake_time = self.go_time = 0.
    self.accel_limit = None

  def update(self, *, enabled, eligible, approach, model, v_ego, a_ego, personality,
             regular_accel, regular_stop, e2e_accel, e2e_stop):
    if not enabled:
      self.reset()
      return regular_accel, False, False
    self.activation_distance = activation_distance(v_ego, a_ego, personality, self.actuator_delay)
    self.intent = model_intent(model, v_ego)
    if self.activation_distance <= 0:
      self.reset()
      self.state, self.reason = 'ready', 'invalidKinematics'
      return regular_accel, False, False

    self.map_valid = approach.valid
    can_assist = eligible and self.intent.valid and all(math.isfinite(x) for x in (regular_accel, e2e_accel))
    holding = can_assist and self.braking and v_ego < .3 and e2e_stop
    self.reason = 'armed' if self.armed else 'noTarget'
    has_target = approach.valid and approach.node_id != 0
    if has_target:
      self.target_distance = approach.distance
      if approach.node_id != self.candidate_id or approach.observation_time < self.last_observation:
        self.reason = 'targetUnconfirmed'
        self.armed = False
        self.candidate_id, self.candidate_samples = approach.node_id, 1
        self.last_observation = approach.observation_time
        self.brake_time = self.go_time = 0.
        if not holding:
          self.braking = False
      elif approach.observation_time > self.last_observation:
        self.candidate_samples += 1
        self.last_observation = approach.observation_time
      if self.armed and (approach.node_id != self.target_id or approach.distance < -PASS_DISTANCE):
        self.armed = False
        self.reason = 'targetPassed' if approach.node_id == self.target_id else 'targetChanged'
        if approach.node_id == self.target_id:
          self.passed_id = approach.node_id
      if approach.node_id == self.passed_id and approach.distance > max(60., self.activation_distance):
        self.passed_id = 0  # a genuinely new approach after leaving the junction
      if (not self.armed and approach.node_id != self.passed_id and self.candidate_samples >= 2 and
          0 <= approach.distance <= self.activation_distance):
        self.armed, self.target_id = True, approach.node_id
        self.reason = 'armed'
      elif not self.armed and self.reason == 'noTarget':
        self.reason = 'targetPassed' if approach.node_id == self.passed_id else 'targetUnconfirmed' if self.candidate_samples < 2 else 'outsideRange'
    else:
      self.armed = False
      self.candidate_id = self.candidate_samples = 0
      self.target_id = 0
      self.target_distance = 0.
      self.reason = 'mapLost' if not approach.valid else 'noTarget'

    # Cyan is map awareness. A pedal override, disengagement, or unusable model
    # clears assistance immediately without erasing a qualified mapped approach.
    if not can_assist:
      self._clear_assistance()
      if not self.armed:
        self.target_id = 0
      self.state = 'inRange' if self.armed else 'ready'
      self.reason = 'inactive' if not eligible else 'invalidModel'
      return regular_accel, False, False

    # Latch range eligibility through deceleration; recomputing a shorter
    # stopping distance must not chatter between ordinary and conditional mode.
    if not self.armed and not holding:
      self.braking = False
      self.brake_time = self.go_time = 0.
      self.target_id = 0
    elif self.armed:
      plausible_stop = self.intent.stop_distance < 0 or self.intent.stop_distance <= max(0., approach.distance) + max(20., .25 * approach.distance)
      slowing = self.intent.slowing and plausible_stop and e2e_accel <= -.2 and e2e_accel <= regular_accel - .15
      stop_at_rest = e2e_stop and v_ego < .3 and e2e_accel < .1
      self.brake_time = self.brake_time + self.dt if slowing or stop_at_rest else 0.
      if self.brake_time >= BRAKE_CONFIRM:
        self.braking = True
      elif self.brake_time > 0 and not self.braking:
        self.reason = 'confirmingModel'
      go = not e2e_stop and (e2e_accel > .1 or not self.intent.slowing and e2e_accel > -.05)
      self.go_time = self.go_time + self.dt if go else 0.
      if self.go_time >= GO_CONFIRM:
        self.braking = False
        self.reason = 'modelGo'

    junction_stop = self.braking and e2e_stop
    if self.braking:
      target = min(e2e_accel, 0.)
      self.reason = 'holdingStop' if junction_stop and v_ego < .3 else 'slowing'
    else:
      target = regular_accel
    # Preserve stronger regular braking, and soften only the return to gas.
    if self.accel_limit is not None:
      target = min(target, self.accel_limit + RELEASE_JERK * self.dt)
    selected = min(regular_accel, target)
    self.contributing = selected < regular_accel - .01 or junction_stop and not regular_stop
    self.accel_limit = selected if selected < regular_accel or self.braking and self.accel_limit is not None else None
    self.state = 'assisting' if self.contributing and (selected < 0 or junction_stop) else 'inRange' if self.armed else 'ready'
    if self.contributing and not self.braking:
      self.reason = 'releasing'
    return selected, junction_stop, self.contributing

  def publish(self, msg, regular_accel, e2e_accel, e2e_stop):
    msg.state = self.state
    msg.reason = self.reason
    msg.targetId = self.target_id
    msg.targetDistance = self.target_distance
    msg.activationDistance = self.activation_distance
    msg.modelStopDistance = self.intent.stop_distance
    msg.regularAcceleration = regular_accel
    msg.modelAcceleration = e2e_accel
    msg.modelShouldStop = e2e_stop
    msg.modelSlowing = self.intent.slowing
    msg.mapValid = self.map_valid
    msg.armed = self.armed
    msg.contributing = self.contributing
