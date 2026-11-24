import math

import numpy as np
import pytest

from openpilot.system.sentry.motion import MotionDetector
from openpilot.system.sentry.stabilization import GyroHistory, alignment, sample_image


def scene():
  rng = np.random.default_rng(4)
  image = rng.uniform(25., 220., (80, 128))
  for _ in range(2):
    image = sum(np.roll(np.roll(image, dy, 0), dx, 1) for dx in (-1, 0, 1) for dy in (-1, 0, 1))/9.
  return image


def shifted(image, dx, dy, rotation=0.):
  y, x = np.indices(image.shape, dtype=float)
  cx, cy = (image.shape[1]-1)/2, (image.shape[0]-1)/2
  c, s = math.cos(rotation), math.sin(rotation)
  return sample_image(image, c*(x-cx-dx)+s*(y-cy-dy)+cx, -s*(x-cx-dx)+c*(y-cy-dy)+cy)[0]


def test_gyro_history_integrates_only_covered_camera_intervals():
  gyro = GyroHistory()
  for i in range(101):
    gyro.add(10.+i*.01, (0., .02, 0.))
  assert gyro.angle(10.5, 10.7) == pytest.approx(.004)
  assert gyro.angle(None, 10.7) is None
  assert gyro.angle(9.9, 10.7) is None
  assert gyro.angle(10.8, 11.1) is None
  assert gyro.angle(10.7, 10.5) is None
  gyro.add(10.9, (0., 10., 0.))  # replayed/duplicate readings cannot alter history
  gyro.add(11.01, (math.nan, 0., 0.))
  assert gyro.angle(10.5, 10.7) == pytest.approx(.004)
  gyro.add(11.2, (0., .02, 0.))
  assert gyro.angle(10.9, 11.2) is None
  for i in range(301):
    gyro.add(12.+i*.01, (0., .02, 0.))
  assert gyro.angle(10.5, 10.7) is None
  assert len(gyro.samples) <= 201


@pytest.mark.parametrize('dx,dy,rotation', [(.2, -.3, 0.), (1.4, -.8, 0.), (-1., .5, .015)])
def test_alignment_removes_camera_translation_and_roll(dx, dy, rotation):
  previous = scene()
  current = shifted(previous, dx, dy, rotation)+8.  # concurrent exposure change
  mapping = alignment(previous, current, .02)
  assert mapping is not None
  aligned, valid = sample_image(previous, *mapping)
  assert np.median(np.abs(current[valid]-aligned[valid]-8.)) < .15


@pytest.mark.parametrize('angle', [None, 0., math.nan, .5])
def test_missing_quiet_or_implausible_gyro_cannot_remove_scene_motion(angle):
  previous = scene()
  assert alignment(previous, shifted(previous, 1., 0.), angle) is None


def test_moving_subject_without_distributed_background_agreement_is_not_aligned():
  image = np.full((80, 128), 80.)
  image[25:45, 30:50] = scene()[25:45, 30:50]
  assert alignment(image, shifted(image, 1., 0.), .02) is None
  assert alignment(scene(), shifted(scene(), 4., 0.), .0005) is None  # beyond gyro bound


@pytest.mark.parametrize('subject', [False, True])
def test_camera_shake_is_removed_while_independent_subject_still_triggers(subject):
  background = scene()
  detector = MotionDetector(area_scale=.4)
  for i in range(20):
    detector.update(background, i*.2)
  results = []
  for i in range(14):
    current = shifted(background, math.sin(i*.8)*.8, math.sin(i*.6)*.5, math.sin(i*.5)*.006)
    if subject:
      x = 25+i*2
      current[30:48, x:x+15] = 220.
    results.append(detector.update(current, 4.+i*.2, camera_rotation=.012))
  if subject:
    assert all(r.active for r in results)
    assert any(r.triggered for r in results[3:])
    assert results[-1].x > results[0].x
  else:
    assert not any(r.active or r.triggered for r in results)
