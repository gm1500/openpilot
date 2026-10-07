"""The approved text-free stop-sign/traffic-light glyph, scaled in native UI units."""
import pyray as rl


def draw_conditional_icon(cx, cy, size, color):
  def point(x, y):
    return rl.Vector2(cx + (x - .5) * size, cy + (y - .5) * size)

  thickness = size * .045
  octagon = ((.17, .20), (.39, .20), (.54, .35), (.54, .57), (.39, .72), (.17, .72), (.02, .57), (.02, .35))
  for i, vertex in enumerate(octagon):
    rl.draw_line_ex(point(*vertex), point(*octagon[(i + 1) % 8]), thickness, color)
    rl.draw_circle_v(point(*vertex), thickness / 2, color)
  rl.draw_line_ex(point(.28, .72), point(.28, .93), thickness, color)
  # Rounded signal housing plus three neutral lamps (not the observed phase).
  housing = rl.Rectangle(cx + .17 * size, cy - .35 * size, .28 * size, .64 * size)
  rl.draw_rectangle_rounded_lines_ex(housing, .3, 8, thickness, color)
  for y in (.29, .47, .65):
    rl.draw_circle_v(point(.81, y), size * .065, color)
  rl.draw_line_ex(point(.81, .79), point(.81, .93), thickness, color)


def conditional_ring_state(sm, started_frame, now):
  """Fresh policy state, never the model proposal or lead-assist flag."""
  def fresh(service):
    return (sm.valid[service] and sm.recv_frame[service] >= started_frame and
            0 <= now - sm.recv_time[service] <= .3 and 0 <= now - sm.logMonoTime[service] * 1e-9 <= .3)

  if not all(fresh(service) for service in ('longitudinalPlan', 'selfdriveState', 'carState')):
    return 'ready'
  state, car = sm['selfdriveState'], sm['carState']
  if not state.conditionalExperimental or state.experimentalMode:
    return 'ready'
  policy = sm['longitudinalPlan'].conditionalExperimental
  active = state.enabled and fresh('carControl') and sm['carControl'].longActive and not (car.gasPressed or car.brakePressed)
  return 'assisting' if active and policy.contributing and str(policy.state) == 'assisting' else 'inRange' if policy.armed else 'ready'
