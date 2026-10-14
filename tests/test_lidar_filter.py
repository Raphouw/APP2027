"""LiDAR/control regressions. No node, publisher or physical motion."""
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'mapping'))
from sensor_msgs.msg import LaserScan
from slam_components.lidar_filter import filter_scan, front_mask
from slam_components.tb3_perception import LidarPerceptionMixin
from slam_components.tb3_control import ReactiveControlMixin
from test_safety_and_mapping import Navigation


class Perception(LidarPerceptionMixin, ReactiveControlMixin):
    def __init__(self):
        self.robot_radius = .105
        self.FRONT_FOV = .25


def scan(values, amin=-math.pi, increment=math.pi/180):
    msg = LaserScan()
    msg.ranges = [float(v) for v in values]
    msg.range_min, msg.range_max = .12, 3.5
    msg.angle_min, msg.angle_increment = float(amin), float(increment)
    return msg


class LidarTests(unittest.TestCase):
    def test_isolated_nan_between_consistent_returns_is_conservative(self):
        distances, known, inferred = filter_scan([1., math.nan, 1.1], -.02, .02, .12, 3.5)
        np.testing.assert_array_equal(distances, [1, 1, 1.1])
        self.assertTrue(known.all())
        np.testing.assert_array_equal(inferred, [False, True, False])

    def test_wide_hole_stays_unknown_and_edge_uses_nearer_surface(self):
        d, known, filled = filter_scan([1]+[math.nan]*8+[1], -.1, .02, .12, 3.5)
        self.assertFalse(known[1:-1].any())
        self.assertFalse(filled.any())
        d, known, filled = filter_scan([.4, math.nan, 2], -.02, .02, .12, 3.5)
        self.assertTrue(known[1])
        self.assertEqual(d[1], .4)

    def test_partial_scan_does_not_bridge_edges_or_trust_zero(self):
        d, known, filled = filter_scan([math.nan, 1, 1, math.nan], -.03, .02, .12, 3.5)
        self.assertFalse(known[0] or known[-1])
        d, known, filled = filter_scan([1, 0, 1], -.02, .02, .12, 3.5)
        self.assertFalse(known[1])

    def test_full_scan_seam_is_bridged(self):
        values = np.ones(360)
        values[0] = values[-1] = np.nan
        d, known, filled = filter_scan(values, 0, math.pi/180, .12, 3.5)
        self.assertTrue(known.all())
        self.assertEqual(int(filled.sum()), 2)

    def test_side_unknown_is_not_inflated_into_front_wall(self):
        values = np.ones(360)
        values[270:280] = np.nan
        node = Perception()
        node._cb_scan(scan(values))
        self.assertAlmostEqual(node._front_min(), 1)
        self.assertTrue(node.lidar_front_known)
        self.assertIsNotNone(node.last_scan_received)
        self.assertTrue(np.isnan(node.raw_scan_ranges[270]))

    def test_small_front_hole_does_not_trigger_false_zero_obstacle(self):
        values = np.ones(360)
        values[180] = np.nan
        node = Perception()
        node._cb_scan(scan(values))
        self.assertEqual(node.lidar_inferred, 1)
        self.assertAlmostEqual(node._front_min(), 1)
        self.assertTrue(node.lidar_front_known)

    def test_unknown_front_stops_both_linear_and_angular_commands(self):
        node = Perception()
        values = np.ones(360); values[175:185] = np.nan
        node._cb_scan(scan(values))
        node.cmd_pub = Mock()
        node.prev_v, node.prev_w = .1, .4
        self.assertIsNone(node.last_scan_received)
        self.assertEqual(node._publish_cmd(.1, .5), (0., 0.))
        command = node.cmd_pub.publish.call_args.args[0]
        self.assertEqual(command.linear.x, 0.)
        self.assertEqual(command.angular.z, 0.)

    def test_clear_scan_with_small_hole_allows_mock_forward_command(self):
        values = np.ones(360); values[180] = np.nan
        node = Perception()
        node._cb_scan(scan(values))
        node.cmd_pub = Mock()
        node.prev_v = node.prev_w = 0.
        node.dt = .02
        node.max_v, node.max_w, node.max_dw = .16, 1., 1.
        node.max_dv_accel, node.max_dv_decel = .4, 1.2
        v, w = node._publish_cmd(.1, .2)
        self.assertGreater(v, 0.)
        self.assertGreater(node.cmd_pub.publish.call_args.args[0].linear.x, 0.)

    def test_real_close_obstacle_is_preserved(self):
        values = np.ones(360); values[180] = .18
        node = Perception()
        node._cb_scan(scan(values))
        self.assertTrue(node.lidar_front_known)
        self.assertAlmostEqual(node._front_min(), .18, places=5)

    def test_map_navigation_uses_same_small_hole_policy(self):
        nav = Navigation()
        values = np.ones(360); values[180] = np.nan
        nav.scan_cb(scan(values))
        self.assertIsNotNone(nav.last_scan_received)
        self.assertAlmostEqual(nav.front_distance, 1)
        self.assertEqual(nav.shared_data['lidar_inferred'], 1)
        values[175:185] = np.nan
        nav.scan_cb(scan(values))
        self.assertIsNone(nav.last_scan_received)
        self.assertEqual(nav.front_distance, 0)

    def test_all_nan_and_invalid_geometry_are_untrusted(self):
        for values, inc in [([math.nan]*20, .02), ([1]*20, 0), ([1]*20, math.nan)]:
            _, known, filled = filter_scan(values, -.1, inc, .12, 3.5)
            self.assertFalse(known.any())
            self.assertFalse(filled.any())


if __name__ == '__main__':
    unittest.main()
