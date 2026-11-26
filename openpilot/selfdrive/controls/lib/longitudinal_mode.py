"""Fresh manual E2E requests and automatic SLC fallback share the same planner."""
import math

MAP_MODE_MAX_AGE = .8


def _fresh(sm, service: str, now: float, max_age: float, replay: bool) -> bool:
  return (service in sm.services and math.isfinite(now) and sm.valid[service] and
          0 <= now - sm.logMonoTime[service] * 1e-9 <= max_age and
          (replay or 0 <= now - sm.recv_time[service] <= max_age))


def full_e2e_requested(sm, now: float, replay: bool = False) -> bool:
  """OFF retains automatic assists; a stale UI/controller request cannot enable E2E."""
  manual_or_automatic = (_fresh(sm, 'selfdriveState', now, .3, replay) and
                         sm['selfdriveState'].experimentalMode)
  return manual_or_automatic or slc_e2e_fallback(sm, now, replay)


def e2e_map_speed_cap(sm, now: float, replay: bool = False) -> float | None:
  """Apply the validated map target until the driver selects a different SET."""
  if not _fresh(sm, 'mapCruiseState', now, MAP_MODE_MAX_AGE, replay):
    return None
  state = sm['mapCruiseState']
  # Either publication may arrive first. The selected SET also bounds a stale,
  # higher carState packet when the driver lowers the ceiling.
  target = state.setSpeed if state.driverSpeedOverride else state.targetSpeed
  if str(state.state) == 'unsupported' or not math.isfinite(target) or not 8 <= round(target * 3.6, 3) <= 145:
    return None
  return target


def slc_e2e_fallback(sm, now: float, replay: bool = False) -> bool:
  """Use only a fresh card request, including the recovery-to-map handshake."""
  service = 'mapCruiseState'
  if service not in sm.services:
    return False
  if (not math.isfinite(now) or not sm.valid[service] or
      not 0 <= now - sm.logMonoTime[service] * 1e-9 <= MAP_MODE_MAX_AGE or
      not replay and not 0 <= now - sm.recv_time[service] <= MAP_MODE_MAX_AGE):
    return False
  state = sm[service]
  return state.automaticE2e and state.e2eFallback and str(state.state) not in ('off', 'unsupported')
