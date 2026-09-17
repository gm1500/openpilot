import unittest

from openpilot.selfdrive.ui.onroad.lane_policy_visuals import lane_policy_visual_active, point_in_polygon


class TestLanePolicyVisuals(unittest.TestCase):
  def test_lane_colors_require_selection_and_engagement(self):
    self.assertFalse(lane_policy_visual_active(False, False))
    self.assertFalse(lane_policy_visual_active(False, True))
    self.assertFalse(lane_policy_visual_active(True, False))
    self.assertTrue(lane_policy_visual_active(True, True))

  def test_path_hit_area_only_accepts_taps_inside_visible_path(self):
    path = [(100.0, 100.0), (200.0, 100.0), (180.0, 300.0), (120.0, 300.0)]
    self.assertTrue(point_in_polygon((150.0, 200.0), path))
    self.assertTrue(point_in_polygon((100.0, 100.0), path))
    self.assertFalse(point_in_polygon((50.0, 200.0), path))
    self.assertFalse(point_in_polygon((150.0, 350.0), path))


if __name__ == "__main__":
  unittest.main()
