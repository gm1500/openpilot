"""Screen-space geometry for the lead indicator."""
import numpy as np


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
