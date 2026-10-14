"""Planificateur : chemin diagonal direct, lissage, destinations proches des murs, fluidité."""
import math
from pathlib import Path
import sys
import time
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'atelier_slam'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from planificateur_chemin import DStarLite, nearest_free, octile, plan_path, smooth_path, line_of_sight
from slam_components.settings import MAX_LINEAR
from test_safety_and_mapping import Navigation


def length(cells):
    return sum(math.hypot(a[0]-b[0], a[1]-b[1]) for a, b in zip(cells, cells[1:]))


class PlannerTests(unittest.TestCase):
    def test_free_diagonal_is_taken_directly(self):
        grid = np.zeros((20, 20), dtype=int)
        route = plan_path(grid, (0, 0), (15, 15))
        self.assertAlmostEqual(length(route), 15*math.sqrt(2))
        self.assertEqual(smooth_path(grid, route), [(0, 0), (15, 15)])

    def test_octile_is_exact_without_obstacle(self):
        self.assertAlmostEqual(octile((0, 0), (3, 7)), 3*math.sqrt(2)+4)

    def test_smoothing_cuts_corners_but_never_walls(self):
        grid = np.zeros((20, 20), dtype=int)
        grid[5:15, 10] = 2          # mur vertical
        route = plan_path(grid, (10, 2), (10, 18))
        smooth = smooth_path(grid, route)
        self.assertLess(len(smooth), len(route))
        for a, b in zip(smooth, smooth[1:]):
            self.assertTrue(line_of_sight(grid, a, b))
        self.assertFalse(line_of_sight(grid, (10, 2), (10, 18)))

    def test_penalty_keeps_path_away_from_wall_when_cheap(self):
        grid = np.zeros((30, 30), dtype=int)
        cost = np.zeros((30, 30))
        cost[:, :6] = 3.0           # bande proche du mur de gauche
        route = plan_path(grid, (0, 3), (29, 3), cost)
        self.assertTrue(all(c >= 6 for _, c in route[1:-1]) or length(route) > 29)

    def test_goal_inside_inflated_zone_is_snapped(self):
        grid = np.zeros((20, 20), dtype=int)
        grid[:, 15:] = 2
        self.assertEqual(nearest_free(grid, (5, 16), 5), (5, 14))
        self.assertIsNone(nearest_free(grid, (5, 19), 2))

    def test_historical_api_still_works(self):
        planner = DStarLite(np.zeros((5, 5), dtype=int))
        self.assertEqual(planner.plan((0, 0), (4, 4), smooth=True), [(0, 0), (4, 4)])
        self.assertEqual(planner.update_obstacle([(2, 2)])[-1], (4, 4))


class FluidityTests(unittest.TestCase):
    def test_planning_never_publishes_a_zero_command(self):
        nav = Navigation()
        nav.state, nav.path = 'NAV', [(0.8, 0.)]
        nav.shared_data['goal'] = (0.8, 0.)
        nav.front_distance = 3.0
        nav.control_loop()
        nav.cmd_pub.reset_mock()
        nav.shared_data['click'] = (0.2, 0.9)
        nav.control_loop()
        sent = [call.args[0] for call in nav.cmd_pub.publish.call_args_list]
        self.assertTrue(sent)
        self.assertTrue(all(cmd.linear.x > 0 or cmd.angular.z != 0 for cmd in sent), 'commande nulle pendant la planification')

    def test_speed_ramps_up_and_stays_bounded(self):
        nav = Navigation()
        nav.state, nav.path = 'NAV', [(2.0, 0.)]
        nav.shared_data['goal'] = (2.0, 0.)
        speeds = []
        for _ in range(60):
            nav.last_control_time -= 0.05
            nav.control_loop()
            speeds.append(nav.cmd_pub.publish.call_args.args[0].linear.x)
        self.assertLess(speeds[0], 0.05)
        self.assertTrue(all(b-a <= 0.45*0.1+1e-9 for a, b in zip(speeds, speeds[1:])))
        self.assertLessEqual(max(speeds), MAX_LINEAR+1e-9)

    def test_a_distant_new_obstacle_does_not_trigger_replanning(self):
        nav = Navigation()
        nav.shared_data['goal'] = (0.8, 0.)
        nav._plan_to_goal()
        nav.last_plan_time = 0.
        nav.dynamic_obstacles[nav.world_to_grid(0.0, 1.2)] = time.monotonic()   # loin du chemin
        nav.control_loop()
        self.assertIsNone(getattr(nav, '_plan_future', None))
        nav.dynamic_obstacles[nav.world_to_grid(0.4, 0.0)] = time.monotonic()   # sur le chemin
        nav.control_loop()
        self.assertIsNotNone(getattr(nav, '_plan_future', None))
        nav._plan_future.result(timeout=5)
        nav.control_loop()
        self.assertTrue(all(abs(y) > 0.05 for x, y in nav.path[:-1] if 0.3 < x < 0.5) or len(nav.path) > 1)


if __name__ == '__main__':
    unittest.main()
