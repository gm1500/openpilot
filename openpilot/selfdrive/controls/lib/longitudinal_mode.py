"""Automatic SLC mode selection; there is no persistent manual E2E/CEM mode."""
import math

MAP_MODE_MAX_AGE = .8


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
