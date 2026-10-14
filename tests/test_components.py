"""Tests sans nœud ROS ni commande robot. Lancer après chargement de ROS 2."""
import importlib
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'atelier_slam'))
from slam_components.pid import PIDController
from slam_components.occupancy_grid import OccupancyGridMixin
from slam_components.map_localization import MapLocalizationMixin


class ComponentsTests(unittest.TestCase):
    def test_entry_points_import_without_starting_ros(self):
        import rclpy
        with patch.object(rclpy, 'init', side_effect=AssertionError('ROS ne doit pas démarrer')):
            for name in ('noeud_odometrie', 'navigation_carte', 'cartographe', 'atelier'):
                importlib.import_module(name)

    def test_both_controllers_share_the_pid(self):
        from noeud_odometrie import PIDController as reactive_pid
        from navigation_carte import PIDController as map_pid
        self.assertIs(reactive_pid, map_pid)

    def test_pid_limits_integral_and_resets(self):
        pid = PIDController(1, 1, 0, out_min=-0.7, out_max=0.7, integrator_max=0.2)
        for _ in range(100):
            self.assertEqual(pid.compute(10, 0.1), 0.7)
        self.assertAlmostEqual(pid._integral, 0.2)
        pid.reset()
        self.assertEqual(pid.compute(0, 0.1), 0)
        self.assertEqual(pid.compute(5, 0), 0)

    def test_world_grid_roundtrip_at_cell_centers(self):
        geometry = SimpleNamespace(origin=(-10, -10, 0), res=0.1, h=200, w=200)
        for row, col in [(0, 0), (100, 80), (199, 199)]:
            x, y = MapLocalizationMixin.grid_to_world(geometry, row, col)
            self.assertEqual(MapLocalizationMixin.world_to_grid(geometry, x, y), (row, col))

    def test_free_ray_excludes_occupied_endpoint(self):
        x, y = OccupancyGridMixin.get_free_indices(None, 5, 5, np.array([9]), np.array([5]))
        self.assertEqual(list(zip(x, y)), [(5, 5), (6, 5), (7, 5), (8, 5)])

    def test_rk4_integrates_a_known_arc(self):
        from slam_components.tb3_odometry import OdometryMixin
        from slam_components.tb3_control import ReactiveControlMixin
        state = SimpleNamespace(x=0.0, y=0.0, theta=0.0, dt=0.01,
                                norm_angle=ReactiveControlMixin.norm_angle)
        for _ in range(100):
            OdometryMixin._rk4_update(state, 1.0, 1.0)
        self.assertAlmostEqual(state.x, math.sin(1), places=8)
        self.assertAlmostEqual(state.y, 1-math.cos(1), places=8)
        self.assertAlmostEqual(state.theta, 1, places=8)


if __name__ == '__main__':
    unittest.main()
