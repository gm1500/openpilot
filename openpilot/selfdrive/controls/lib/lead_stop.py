"""Shared lead stopping positions and virtual-stop overlap policy for control/UI."""
import math

COMFORT_BRAKE = 2.5
STOP_DISTANCE = 6.0
LEAD_STOP_SEPARATION = 6.0  # re-entry requires an extra metre outside the overlap band
LEAD_STOP_SEPARATION_RELEASE = 5.0  # car-length overlap band; real lead gets priority


def get_stopped_equivalence_factor(v_lead):
  return (v_lead**2) / (2 * COMFORT_BRAKE)


def get_lead_stop_distance(radarstate):
  """Ego stop implied by accepted leads, using ordinary MPC gap bookkeeping."""
  stops = [max(0., lead.dRel + get_stopped_equivalence_factor(max(0., lead.vLead)) - STOP_DISTANCE)
           for lead in (radarstate.leadOne, radarstate.leadTwo)
           if lead.present and math.isfinite(lead.dRel) and lead.dRel > 0. and math.isfinite(lead.vLead)]
  return min(stops) if stops else None


def lead_covers_stop(raw_stop, lead_stop_distance, active=False):
  """Give ordinary lead control priority for overlapping or later model stops."""
  if lead_stop_distance is None or not math.isfinite(lead_stop_distance):
    return False
  separation = LEAD_STOP_SEPARATION_RELEASE if active else LEAD_STOP_SEPARATION
  return raw_stop is None or not math.isfinite(raw_stop) or raw_stop >= max(0., lead_stop_distance) - separation


def get_lead_distance(radar_state):
  """Nearest accepted physical lead; no ID assignment or motion extrapolation."""
  distances = [lead.dRel for lead in (radar_state.leadOne, radar_state.leadTwo)
               if lead.present and math.isfinite(lead.dRel) and lead.dRel > 0.]
  return min(distances, default=None)


def lead_blocks_stop(raw_stop, lead_distance, active=False):
  """Keep the stop before a real lead with the original entry/release clearance."""
  if lead_distance is None or not math.isfinite(lead_distance):
    return False
  separation = LEAD_STOP_SEPARATION_RELEASE if active else LEAD_STOP_SEPARATION
  return raw_stop is None or not math.isfinite(raw_stop) or raw_stop >= lead_distance - separation
