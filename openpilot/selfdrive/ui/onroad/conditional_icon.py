"""Approved intersection, stop sign and signal, drawn crisply at native HUD size."""
import pyray as rl


def draw_conditional_icon(cx, cy, size, color):
  # Design coordinates preserve the approved perspective and roadside placement.
  def point(x, y):
    return rl.Vector2(cx + (x - 425) / 480 * size, cy + (y - 465) / 480 * size)

  thickness = size / 48

  def stroke(vertices):
    points = [point(*v) for v in vertices]
    for a, b in zip(points, points[1:], strict=False):
      rl.draw_line_ex(a, b, thickness, color)
    for p in points:
      rl.draw_circle_v(p, thickness / 2, color)

  def corner(start, a, control, b, end):
    curve = [((1-t)**2*a[0] + 2*(1-t)*t*control[0] + t*t*b[0],
              (1-t)**2*a[1] + 2*(1-t)*t*control[1] + t*t*b[1]) for t in (i/12 for i in range(13))]
    stroke([start, *curve, end])

  # Four open curved curbs. The left lower edge is deliberately continuous.
  corner((195, 552), (329, 481), (355, 468), (330, 452), (297, 432))
  corner((330, 380), (408, 420), (430, 432), (452, 420), (520, 386))
  corner((540, 452), (530, 456), (509, 467), (533, 481), (655, 552))
  corner((257, 621), (407, 537), (430, 524), (454, 537), (593, 621))
  # Identical lane dashes; the crossing itself stays clear. The far-right
  # dash starts beyond the signal post, leaving a visible clearance gap.
  for a, b in (((319, 411), (338, 422)), ((359, 435), (378, 446)),
               ((483, 445), (521, 423)), ((583, 431), (595, 424)),
               ((243, 574), (280, 553)), ((309, 536), (348, 514)),
               ((508, 514), (546, 536)), ((574, 553), (611, 574))):
    stroke([a, b])

  # Upright signs on opposite roadside corners, never joined to the road.
  octagon = [(239, 345), (260, 323), (293, 323), (314, 345),
             (314, 377), (293, 398), (260, 398), (239, 377)]
  stroke([*octagon, octagon[0]])
  stroke([(276, 398), (276, 454)])
  p = point(535, 273)
  housing = rl.Rectangle(p.x, p.y, 55 / 480 * size, 110 / 480 * size)
  rl.draw_rectangle_rounded_lines_ex(housing, .35, 8, thickness, color)
  for y in (297, 327, 357):
    rl.draw_circle_v(point(562, y), size / 40, color)
  stroke([(562, 387), (562, 453)])


def conditional_ring_state(sm, started_frame, now):
  """Orange means conditional E2E enabled, even when no extra braking is needed."""
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
  return 'active' if active and policy.e2eEnabled else 'ready'
