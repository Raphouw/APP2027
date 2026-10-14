"""Offline regressions for finite, pose-compensated LiDAR history."""
import math
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'atelier_slam'))
from slam_components.lidar_filter import ScanHistory


class HistoryTests(unittest.TestCase):
    def apply(self, history, values, now, pose=(0., 0., 0.)):
        return history.filter(values, -.15, math.radians(1.5), .12, 3.5, pose, now)

    def test_recent_real_returns_bridge_bounded_six_degree_hole(self):
        h = ScanHistory(); self.apply(h, np.ones(20), 0.)
        values = np.ones(20); values[8:12] = np.nan
        distances, known, spatial, temporal = self.apply(h, values, .1)
        self.assertTrue(known.all())
        self.assertEqual(temporal.sum(), 4)
        self.assertFalse(spatial.any())
        np.testing.assert_allclose(distances[8:12], .93)

    def test_estimates_never_refresh_missing_sector(self):
        h = ScanHistory(); self.apply(h, np.ones(20), 0.)
        values = np.ones(20); values[8:12] = np.nan
        self.apply(h, values, .1)
        _, known, _, temporal = self.apply(h, values, .21)
        self.assertFalse(known[8:12].any())
        self.assertFalse(temporal.any())

    def test_large_holes_and_invalid_returns_remain_unknown(self):
        for missing in (np.nan, 0., -1.):
            h = ScanHistory(); self.apply(h, np.ones(20), 0.)
            values = np.ones(20); values[5:15] = missing
            _, known, _, temporal = self.apply(h, values, .1)
            self.assertFalse(known[5:15].any())
            self.assertFalse(temporal.any())

    def test_current_obstacle_wins_over_previous_clearance(self):
        h = ScanHistory(); self.apply(h, np.ones(20), 0.)
        values = np.ones(20); values[10] = .15
        distances, _, _, temporal = self.apply(h, values, .1)
        self.assertEqual(distances[10], .15)
        self.assertFalse(temporal[10])

    def test_pose_rotation_reprojects_raw_points(self):
        h = ScanHistory(); self.apply(h, np.ones(20), 0.)
        values = np.ones(20); values[8:12] = np.nan
        _, known, _, temporal = self.apply(h, values, .1, (0., 0., math.radians(1.5)))
        self.assertTrue(known.all())
        self.assertEqual(temporal.sum(), 4)

    def test_pose_jump_or_clock_reset_discards_history(self):
        for pose, now in (((.2, 0., 0.), .1), ((0., 0., 0.), -.1)):
            h = ScanHistory(); self.apply(h, np.ones(20), 0.)
            values = np.ones(20); values[8:12] = np.nan
            _, known, _, temporal = self.apply(h, values, now, pose)
            self.assertFalse(known[8:12].any())
            self.assertFalse(temporal.any())

    def test_invalid_geometry_does_not_fill(self):
        h = ScanHistory(); self.apply(h, np.ones(20), 0.)
        _, known, _, temporal = h.filter([np.nan]*20, -.15, 0., .12, 3.5, (0.,0.,0.), .1)
        self.assertFalse(known.any())
        self.assertFalse(temporal.any())


if __name__ == '__main__':
    unittest.main()
