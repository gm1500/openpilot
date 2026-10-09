"""Screen-space geometry for the lead indicator."""
import numpy as np


def lead_fill_alpha(d_rel, v_rel):
  """The regular lead's distance/closing-speed red fill, over opaque yellow."""
  if d_rel >= 40.:
    return 0
  return int(np.clip(255 * (1. - d_rel / 40. + max(0., -v_rel / 10.)), 0, 255))


def rounded_polygon(points, radius):
  """Convex triangle fan with a small, pixel-sized radius at every corner."""
  vertices = np.asarray(points, dtype=float)
  outline = []
  for i, corner in enumerate(vertices):
    before, after = vertices[i - 1] - corner, vertices[(i + 1) % len(vertices)] - corner
    start = corner + before * min(.45, radius / max(np.linalg.norm(before), 1e-6))
    end = corner + after * min(.45, radius / max(np.linalg.norm(after), 1e-6))
    for t in np.linspace(0., 1., 8):
      outline.append(tuple((1 - t)**2 * start + 2 * (1 - t) * t * corner + t**2 * end))
  return [tuple(vertices.mean(axis=0)), *outline, outline[0]]


def rounded_triangle(points):
  """Keep the top profile and use broader curves at the two sharp side tips."""
  vertices = np.asarray(points, dtype=float)
  top = int(np.argmin(vertices[:, 1]))
  outline = []
  for i, corner in enumerate(vertices):
    trim = .22 if i == top else .40
    start = corner + trim * (vertices[i - 1] - corner)
    end = corner + trim * (vertices[(i + 1) % 3] - corner)
    for t in np.linspace(0., 1., 16):
      point = (1 - t)**2 * start + 2 * (1 - t) * t * corner + t**2 * end
      outline.append(tuple(point))
  return [tuple(vertices.mean(axis=0)), *outline, outline[0]]
