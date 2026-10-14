import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'atelier_slam'))
from slam_components.sim_world import (BEAMS, RANGE_MAX, SimRobot, default_world, normalize, WHEEL_BASE, WHEEL_RADIUS)


class SimWorldTests(unittest.TestCase):
    def test_scan_sees_the_wall_at_the_right_distance(self):
        world = default_world()
        robot = SimRobot(world, pose=(1.5, 4.0, math.pi), noise=0, seed=1)
        ranges = robot.scan()
        self.assertEqual(len(ranges), BEAMS)
        # Face au mur ouest (x = 0,05 m) : ~1,45 m dans l'axe du robot.
        self.assertAlmostEqual(ranges[0], 1.5 - 0.05, delta=0.06)
        # Rayon vers l'est, loin de l'îlot (y = 4 est dans l'îlot) : obstacle à ~1,5 m.
        self.assertTrue(np.isfinite(ranges[180]))

    def test_nothing_in_range_gives_infinity(self):
        robot = SimRobot(default_world(), pose=(1.5, 4.0, 0.0), noise=0)
        self.assertTrue((robot.scan() <= RANGE_MAX).sum() < BEAMS or np.isinf(robot.scan()).any())

    def test_robot_moves_forward_and_wheels_match_distance(self):
        robot = SimRobot(default_world(), pose=(1.5, 2.0, 0.0), noise=0)
        for _ in range(600):
            robot.step(0.2, 0.0, 0.01)
        self.assertGreater(robot.x, 2.5)
        travelled = robot.x - 1.5
        left, right = robot.wheel_left, robot.wheel_right
        self.assertAlmostEqual(left * WHEEL_RADIUS, travelled, delta=0.01)
        self.assertAlmostEqual(left, right, delta=1e-6)

    def test_rotation_in_place_and_gyro(self):
        robot = SimRobot(default_world(), pose=(1.5, 2.0, 0.0), noise=0)
        for _ in range(100):
            robot.step(0.0, 1.0, 0.01)
        self.assertGreater(robot.theta, 0.7)
        self.assertAlmostEqual(robot.gyro(), 1.0, delta=0.05)
        diff = (robot.wheel_right - robot.wheel_left) * WHEEL_RADIUS / WHEEL_BASE
        self.assertAlmostEqual(diff, robot.theta, delta=0.01)

    def test_robot_stops_at_obstacles(self):
        robot = SimRobot(default_world(), pose=(1.5, 4.0, 0.0), noise=0)
        for _ in range(2000):
            robot.step(0.2, 0.0, 0.01)
        self.assertTrue(robot.collided)
        self.assertLess(robot.x, 3.0)

    def test_noise_makes_wheel_odometry_drift(self):
        a = SimRobot(default_world(), pose=(1.5, 2.0, 0.0), noise=3.0, seed=2)
        b = SimRobot(default_world(), pose=(1.5, 2.0, 0.0), noise=0)
        for _ in range(300):
            a.step(0.2, 0.0, 0.01)
            b.step(0.2, 0.0, 0.01)
        self.assertNotEqual(a.wheel_left, b.wheel_left)


if __name__ == '__main__':
    unittest.main()
