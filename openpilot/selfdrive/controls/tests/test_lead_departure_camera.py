"""Optional offline OpenCV tests; not part of the on-device perception pipeline."""

from dataclasses import FrozenInstanceError
import importlib.util
import io
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

try:
  import cv2
except ImportError as e:
  raise unittest.SkipTest('offline camera tests require optional opencv-python-headless') from e

MODULE_PATH = Path(__file__).resolve().parents[4] / 'tools' / 'lead_departure_camera.py'
SPEC = importlib.util.spec_from_file_location('lead_departure_camera_test_module', MODULE_PATH)
camera = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = camera
SPEC.loader.exec_module(camera)


class TestLeadFeatureTrack(unittest.TestCase):
  def setUp(self):
    self.roi = (100, 80, 120, 80)
    self.frame = np.zeros((240, 320), dtype=np.uint8)
    rng = np.random.default_rng(421)
    patch = rng.integers(25, 235, size=(40, 60), dtype=np.uint8)
    self.frame[80:160, 100:220] = cv2.resize(patch, (120, 80), interpolation=cv2.INTER_NEAREST)
    self.track = camera.LeadFeatureTrack()

  def acquire(self):
    result = self.track.acquire(self.frame, 0., self.roi, 40.)
    self.assertTrue(result.valid)
    self.assertGreaterEqual(result.features, self.track.MIN_FEATURES)
    return result

  def transformed(self, dx=0., dy=0., scale=1., degrees=0.):
    matrix = cv2.getRotationMatrix2D((160., 120.), degrees, scale)
    matrix[:, 2] += (dx, dy)
    return cv2.warpAffine(self.frame, matrix, (320, 240))

  def test_translation_retains_original_features(self):
    acquired = self.acquire()
    for index in range(1, 11):
      result = self.track.update(self.transformed(dx=2. * index, dy=.5 * index), .05 * index, 40.)
      self.assertTrue(result.valid, result)
      self.assertFalse(result.can_ignore_lead)
      self.assertLessEqual(result.features, acquired.features)
      self.assertAlmostEqual(result.visual_scale, 1., delta=.02)

  def test_expansion_matches_range(self):
    self.acquire()
    for index in range(1, 6):
      scale = 1.01 ** index
      result = self.track.update(self.transformed(dx=index, scale=scale), .05 * index, 40. / scale)
      self.assertTrue(result.valid, result)
      self.assertAlmostEqual(result.visual_scale, scale, delta=.015)
      self.assertAlmostEqual(result.log_scale_residual, 0., delta=.02)

  def test_range_jump_without_growth_is_only_disagreement(self):
    self.acquire()
    result = self.track.update(self.transformed(dx=2.), .05, 32.)
    self.assertTrue(result.valid)
    self.assertAlmostEqual(result.range_scale, 1.25)
    self.assertLess(result.log_scale_residual, -.20)
    self.assertFalse(result.can_ignore_lead)
    self.assertEqual(result.reason, 'continuity_only')

  def test_no_range_is_not_invented(self):
    self.track.acquire(self.frame, 0., self.roi)
    result = self.track.update(self.frame, .05)
    self.assertTrue(result.valid)
    self.assertIsNone(result.range_scale)
    self.assertIsNone(result.log_scale_residual)

  def test_extreme_finite_range_ratio_is_rejected(self):
    self.track.acquire(self.frame, 0., self.roi, 1e308)
    result = self.track.update(self.frame, .05, 1e-308)
    self.assertEqual(result.reason, 'invalid_range_ratio')
    self.assertFalse(result.can_ignore_lead)

  def test_black_frame_revokes_without_reseeding(self):
    self.acquire()
    result = self.track.update(np.zeros_like(self.frame), .05)
    self.assertFalse(result.valid)
    self.assertFalse(self.track.update(self.frame, .1).valid)
    self.assertIsNone(self.track.points)

  def test_low_texture_rejected_at_acquisition(self):
    result = self.track.acquire(np.full_like(self.frame, 30), 0., self.roi)
    self.assertEqual(result.reason, 'insufficient_texture')

  def test_independent_distractor_cannot_authorize_ignoring(self):
    self.acquire()
    changed = self.transformed(dx=3.)
    changed[100:150, 160:220] = np.random.default_rng(4).integers(0, 255, (50, 60), dtype=np.uint8)
    result = self.track.update(changed, .05, 25.)
    self.assertFalse(result.can_ignore_lead)
    # Surviving features can still match under partial occlusion. This must not
    # become identity/clear-lane evidence or silently seed the distractor.
    self.assertLessEqual(result.features, self.track.MAX_FEATURES)

  def test_old_roi_replaced_by_another_texture_is_rejected(self):
    self.acquire()
    replaced = self.frame.copy()
    replaced[80:160, 100:220] = np.random.default_rng(99).integers(0, 255, (80, 120), dtype=np.uint8)
    result = self.track.update(replaced, .05)
    self.assertFalse(result.valid)

  def test_duplicate_backwards_and_dropped_frames_revoke(self):
    for timestamp in (0., -.01, .151, float('nan'), float('inf')):
      with self.subTest(timestamp=timestamp):
        self.acquire()
        self.assertFalse(self.track.update(self.frame, timestamp).valid)
        self.assertIsNone(self.track.points)

  def test_fixed_deadline_does_not_extend(self):
    self.acquire()
    for index in range(1, 26):
      self.assertTrue(self.track.update(self.frame, .1 * index).valid)
    self.assertEqual(self.track.update(self.frame, 2.6).reason, 'expired')

  def test_image_shape_dtype_and_range_rejected(self):
    for frame, distance in ((self.frame[:200], 40.), (self.frame.astype(float), 40.),
                            (self.frame, 0.), (self.frame, -1.), (self.frame, float('nan'))):
      with self.subTest(shape=frame.shape, distance=distance):
        self.acquire()
        self.assertFalse(self.track.update(frame, .05, distance).valid)

  def test_bad_roi_and_acquisition_input_rejected(self):
    for roi in ((-1, 80, 120, 80), (0, 0, 400, 80), (0, 0, 10, 10), (0, 0, float('nan'), 10)):
      with self.subTest(roi=roi):
        self.assertFalse(self.track.acquire(self.frame, 0., roi).valid)
    self.assertFalse(self.track.acquire(self.frame, float('nan'), self.roi).valid)
    self.assertFalse(self.track.acquire(self.frame, 0., self.roi, -5.).valid)

  def test_camera_rotation_compensation(self):
    self.acquire()
    angle = .01
    rotation = np.array([[np.cos(angle), 0., np.sin(angle)], [0., 1., 0.], [-np.sin(angle), 0., np.cos(angle)]])
    intrinsic = np.array([[250., 0., 160.], [0., 250., 120.], [0., 0., 1.]])
    homography = camera.rotation_homography(intrinsic, rotation)
    following = cv2.warpPerspective(self.frame, homography, (320, 240))
    result = self.track.update(following, .05, 40., homography)
    self.assertTrue(result.valid, result)
    self.assertTrue(result.rotation_compensated)
    self.assertAlmostEqual(result.visual_scale, 1., delta=.01)

  def test_invalid_camera_geometry_rejected(self):
    for rotation in (np.zeros((3, 3)), np.eye(2), np.full((3, 3), np.nan)):
      with self.subTest(rotation=rotation):
        with self.assertRaises(ValueError):
          camera.rotation_homography(np.eye(3), rotation)
        self.acquire()
        self.assertFalse(self.track.update(self.frame, .05, camera_rotation=rotation).valid)
    with self.assertRaises(ValueError):
      camera.rotation_homography(np.zeros((3, 3)), np.eye(3))

  def test_bounded_feature_count_and_no_input_mutation(self):
    original = self.frame.copy()
    result = self.acquire()
    self.assertLessEqual(result.features, self.track.MAX_FEATURES)
    self.track.update(self.frame, .05)
    np.testing.assert_array_equal(self.frame, original)
    with self.assertRaises(FrozenInstanceError):
      result.can_ignore_lead = True
    with self.assertRaises(TypeError):
      camera.VisualEvidence(True, 'test', can_ignore_lead=True)

  def test_explicit_reacquisition_after_loss(self):
    self.acquire()
    self.track.update(np.zeros_like(self.frame), .05)
    self.assertEqual(self.track.update(self.frame, .1).reason, 'untracked')
    self.assertTrue(self.track.acquire(self.frame, .15, self.roi).valid)

  def test_range_alignment_never_uses_future_or_stale_samples(self):
    samples = camera.RangeSamples([{'time': 1., 'distance': 40.}, {'time': 1.05, 'distance': 30.}])
    self.assertIsNone(samples.at(.99))
    self.assertEqual(samples.at(1.049), 40.)
    self.assertEqual(samples.at(1.05), 30.)
    self.assertEqual(samples.at(1.15), 30.)
    self.assertIsNone(samples.at(1.151))
    self.assertIsNone(samples.at(float('nan')))

  def test_range_alignment_rejects_invalid_or_reordered_samples(self):
    for rows in ([{'time': 1., 'distance': float('nan')}], [{'time': 1., 'distance': 0.}],
                 [{'time': 1., 'distance': 40.}, {'time': 1., 'distance': 30.}],
                 [{'time': 1., 'distance': 40.}, {'time': .95, 'distance': 30.}]):
      with self.subTest(rows=rows), self.assertRaises(ValueError):
        camera.RangeSamples(rows)

  def test_cli_rejects_zero_pts_instead_of_silently_scanning(self):
    class NoPts:
      def isOpened(self):
        return True

      def read(inner):
        return True, cv2.cvtColor(self.frame, cv2.COLOR_GRAY2BGR)

      def get(self, _):
        return 0.

      def set(self, *_):
        return True

      def release(self):
        pass

    argv = ['camera', 'raw.hevc', '--start', '5', '--roi', '100', '80', '120', '80']
    errors = io.StringIO()
    with patch.object(sys, 'argv', argv), patch.object(camera.cv2, 'VideoCapture', return_value=NoPts()), patch('sys.stderr', errors):
      with self.assertRaises(SystemExit) as error:
        camera.main()
    self.assertEqual(error.exception.code, 2)
    self.assertIn('no usable monotonic PTS', errors.getvalue())

  def test_cli_rejects_unopenable_or_empty_video(self):
    class EmptyVideo:
      def __init__(self, opened):
        self.opened = opened

      def isOpened(self):
        return self.opened

      def read(self):
        return False, None

      def set(self, *_):
        return True

      def release(self):
        pass

    argv = ['camera', 'missing.hevc', '--start', '0', '--roi', '100', '80', '120', '80']
    for opened, expected in ((False, 'could not be opened'), (True, 'no decoded frames')):
      errors = io.StringIO()
      with self.subTest(opened=opened), patch.object(sys, 'argv', argv), patch('sys.stderr', errors):
        with patch.object(camera.cv2, 'VideoCapture', return_value=EmptyVideo(opened)), self.assertRaises(SystemExit):
          camera.main()
      self.assertIn(expected, errors.getvalue())

  def test_cli_rejects_duration_beyond_fixed_deadline(self):
    argv = ['camera', 'video.ts', '--start', '0', '--duration', '3', '--roi', '100', '80', '120', '80']
    with patch.object(sys, 'argv', argv), patch('sys.stderr', io.StringIO()), self.assertRaises(SystemExit):
      camera.main()


if __name__ == '__main__':
  unittest.main()
