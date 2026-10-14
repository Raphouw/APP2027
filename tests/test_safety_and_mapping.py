"""Régressions des corrections, sans nœud ROS, fenêtre ni commande physique."""
import csv
import json
import math
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'mapping'))
from dstar_lite import DStarLite
from slam_components.command_ownership import CommandLease
from slam_components.map_format import load_map
from slam_components.mapping_io import MappingIOMixin
from slam_components.occupancy_grid import OccupancyGridMixin
from slam_components.map_localization import MapLocalizationMixin
from slam_components.map_control import MapControlMixin
from slam_components.tb3_odometry import OdometryMixin
from slam_components.tb3_control import ReactiveControlMixin
from slam_components.tb3_logging import SessionWriter, save_pose
from slam_components.manager_processes import ManagerProcessesMixin
from slam_components.pid import PIDController
from slam_components.settings import ODOM_FRAME
from sensor_msgs.msg import JointState, LaserScan
from nav_msgs.msg import Odometry


class Grid(OccupancyGridMixin, MappingIOMixin):
    def __init__(self, directory):
        self.size, self.grow_size, self.scale = 20, 10, 10
        self.x0 = self.y0 = self.x = self.y = 10
        self.map_logodd = np.zeros((20, 20))
        self.map = np.full((20, 20), 0.5)
        self.cell_update_time = np.zeros((20, 20), dtype=int)
        self.cell_change_count = np.zeros((20, 20), dtype=int)
        self.cell_prev_state = np.full((20, 20), 0.5)
        self.path_x, self.path_y = [10], [10]
        self.orientation = 0.0
        self.dist_max_lidar = 3.5
        self.scan_counter = 0
        self.log_occ, self.log_free = math.log(0.7/0.3), math.log(0.3/0.7)
        self.state_diff_threshold, self.change_threshold = 0.2, 5
        self.output_dir, self.filename = directory, 'test'
        self.input_file, self.session_id = '', 'test-session'
        self.speed_multiplier = 1.0
        self.update_plot = lambda: None


class Navigation(MapLocalizationMixin, MapControlMixin):
    def __init__(self):
        self.meta = {'frame_id': ODOM_FRAME}
        self.res, self.origin, self.h, self.w = 0.1, [-1, -1, 0], 30, 30
        self.raw_odom, self.robot_pos, self.odom_offset = [0., 0., 0.], [0., 0., 0.], [0., 0., 0.]
        self.sensor_timeout, self.clearance = 0.5, 0.2
        self.last_odom_received = self.last_scan_received = time.monotonic()
        self.front_distance = 3.0
        self.dynamic_obstacles = {}
        self.shared_data = {'ui_mode': 'GOAL', 'state': 'READY', 'pose_received': True}
        self.static_grid = np.zeros((30, 30), dtype=int)
        self.planner = DStarLite(self.static_grid)
        self.path, self.state = [], 'READY'
        self.last_plan_time = 0.
        self.last_control_time = self.last_relocalize_time = time.monotonic()
        self.relocalize_interval = 10.
        self.enable_visual_relocalization = False
        self.global_localizer = self.last_camera_frame = None
        self.pid_heading = PIDController(0.9, 0.05, 0.3)
        self.cmd_pub = Mock()
        self.get_logger = lambda: Mock()


class ImprovementsTests(unittest.TestCase):
    def test_exclusive_command_lease_and_release(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'lock'
            first = CommandLease(path)
            with self.assertRaises(RuntimeError):
                CommandLease(path)
            first.close()
            second = CommandLease(path)
            second.close()
            second.close()

    def test_odometry_only_never_publishes_or_accepts_goals(self):
        controller = SimpleNamespace(cmd_pub=None, odometry_only=True, goal_x=None)
        ReactiveControlMixin._publish_cmd(controller, 0.1, 0.2)
        ReactiveControlMixin._cb_goal(controller, Mock())
        self.assertIsNone(controller.goal_x)

    def test_measured_wheels_use_the_message_interval(self):
        class Pose(OdometryMixin, ReactiveControlMixin):
            pass
        pose = Pose()
        pose.imu_ok = True
        pose.last_wl = pose.last_wr = pose.last_joint_time = None
        pose.wheel_radius, pose.wheel_base = 0.033, 0.160
        pose.sensor_timeout = 0.5
        pose.last_imu_received = time.monotonic()
        pose.alpha_comp, pose.gyro_w = 0.98, 0.0
        pose.x = pose.y = pose.theta = 0.0
        pose.dt, pose.prev_v = 0.02, 0.16
        def sample(sec, nanosec, wheel):
            msg = JointState()
            msg.header.stamp.sec, msg.header.stamp.nanosec = sec, nanosec
            msg.name, msg.position = ['wheel_left_joint', 'wheel_right_joint'], [wheel, wheel]
            pose._cb_joint(msg)
        sample(1, 0, 0.)
        sample(1, 100_000_000, 1.)
        self.assertAlmostEqual(pose.enc_v, 0.33)
        self.assertAlmostEqual(pose.x, 0.033)
        sample(1, 200_000_000, 1.)
        self.assertAlmostEqual(pose.x, 0.033)  # aucune roue tournée malgré prev_v non nul
        sample(1, 200_000_000, 2.)  # doublon temporel : ignoré
        self.assertAlmostEqual(pose.x, 0.033)

    def test_asymmetric_map_export_roundtrip(self):
        with tempfile.TemporaryDirectory() as directory:
            grid = Grid(directory)
            grid.map[:] = 0.1
            grid.map[15, 10] = 0.9  # obstacle à droite, pas devant
            grid.map[10, 17] = 0.5  # inconnu devant
            grid.scan_counter = 1
            path = grid.save_map()
            meta, pixels, occupancy = load_map(path)
            self.assertEqual(occupancy[10, 15], 2)
            self.assertEqual(occupancy[17, 10], 2)
            self.assertEqual(occupancy[15, 10], 0)
            self.assertTrue(np.allclose((255-pixels)/255, grid.map.T, atol=1/255))
            self.assertEqual(meta['frame_id'], ODOM_FRAME)

    def test_old_map_requires_regeneration(self):
        with self.assertRaises(ValueError):
            load_map(Path(__file__).resolve().parents[1]/'mapping/logs/map_out.yaml')

    def test_growth_on_each_side_preserves_world_coordinates(self):
        with tempfile.TemporaryDirectory() as directory:
            grid = Grid(directory)
            for x, y in [(2., 0.), (-2., 0.), (0., 2.), (0., -2.)]:
                grid.x, grid.y = grid.x0, grid.y0
                grid.write_map(np.array([x]), np.array([y]))
                for name in ['map_logodd', 'cell_update_time', 'cell_change_count', 'cell_prev_state']:
                    self.assertEqual(getattr(grid, name).shape, grid.map.shape)
                self.assertAlmostEqual((grid.x-grid.x0)/grid.scale, 0)
                self.assertAlmostEqual((grid.y-grid.y0)/grid.scale, 0)
            grid.save_map()

    def test_invalid_ranges_do_not_create_obstacles_at_robot(self):
        with tempfile.TemporaryDirectory() as directory:
            grid = Grid(directory)
            x, y = grid.process_data([float('nan'), float('-inf'), 0, -1], 0, math.pi)
            grid.write_map(x, y)
            self.assertEqual(grid.scan_counter, 0)
            self.assertTrue(np.all(grid.map == 0.5))

    def test_infinite_return_clears_space_without_inventing_a_wall(self):
        with tempfile.TemporaryDirectory() as directory:
            grid = Grid(directory)
            x, y = grid.process_data([float('inf')], 0, 0)
            grid.write_map(x, y)
            self.assertFalse(np.any(grid.map_logodd > 0))
            self.assertTrue(np.any(grid.map_logodd < 0))

    def test_progressive_session_preserves_variable_scans(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'session.csv'
            writer = SessionWriter(path)
            writer.append(1, [0, 0, 0], 'IDLE', 1, 0, 0.1, [1, 2])
            # Visible avant close, contrairement aux anciens buffers.
            with path.open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(json.loads(rows[0]['ranges_json']), [1, 2])
            writer.append(2, [0, 0, 0], 'IDLE', 1, 0, 0.1, [1])
            writer.close()
            writer.close()
            grid = Grid(directory)
            grid.input_file = str(path)
            with patch('slam_components.mapping_io.plt.pause'):
                self.assertEqual(grid.run_carto_offline(), 2)
            save_pose(Path(directory)/'last_pose.txt', (1, 2, 3))
            self.assertEqual((Path(directory)/'last_pose.txt').read_text(), '1,2,3')

    def test_live_and_offline_projection_agree(self):
        with tempfile.TemporaryDirectory() as directory:
            live, offline = Grid(directory), Grid(directory)
            path = Path(directory)/'legacy.csv'
            with path.open('w', newline='') as stream:
                writer = csv.writer(stream)
                writer.writerow(['timestamp','x','y','theta','state','front_min','angle_min','angle_max','scan_0'])
                for i in range(4):
                    writer.writerow([i, 0, 0, 0, 'IDLE', 0.5, 0, 0, 0.5])
                    x, y = live.process_data([0.5], 0, 0)
                    live.write_map(x, y)
            offline.input_file = str(path)
            with patch('slam_components.mapping_io.plt.pause'):
                offline.run_carto_offline()
            np.testing.assert_array_equal(live.map_logodd, offline.map_logodd)

    def test_empty_csv_cannot_generate_successful_map(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'empty.csv'
            path.write_text('timestamp,x,y,theta,state,front_min,angle_min,angle_max\n')
            grid = Grid(directory)
            grid.input_file = str(path)
            with self.assertRaises(ValueError):
                grid.run_carto_offline()
            with self.assertRaises(ValueError):
                grid.save_map()

    def test_pose_rotation_persists_in_odometry(self):
        nav = Navigation()
        nav.raw_odom = [1, 0, 0]
        nav.set_pose(2, 3, math.pi/2)
        msg = Odometry()
        msg.header.frame_id = ODOM_FRAME
        msg.pose.pose.position.x = 2.
        msg.pose.pose.orientation.w = 1.
        nav.odom_cb(msg)
        np.testing.assert_allclose(nav.robot_pos, [2, 4, math.pi/2], atol=1e-9)

    def test_out_of_map_and_wrong_frame_are_rejected(self):
        nav = Navigation()
        self.assertIsNone(nav.world_to_grid(-1.001, 0))
        self.assertIsNone(nav.world_to_grid(3, 0))
        nav.odom_cb(Odometry())
        self.assertIsNone(nav.last_odom_received)

    def test_sensor_timeout_and_stop_override_new_goal(self):
        nav = Navigation()
        nav.last_scan_received = time.monotonic()-1
        nav.shared_data['click'] = (0.5, 0.)
        nav.control_loop()
        self.assertEqual(nav.cmd_pub.publish.call_args.args[0].linear.x, 0)
        self.assertEqual(nav.shared_data['state'], 'WAITING_SENSORS')
        nav.last_scan_received = time.monotonic()
        nav.shared_data['state'] = 'STOPPED'
        nav.control_loop()
        self.assertIsNone(nav.shared_data['click'])
        self.assertIsNone(nav.shared_data['goal'])
        self.assertFalse(nav.path)

    def test_front_obstacle_stops_immediately(self):
        nav = Navigation()
        nav.state, nav.path = 'NAV', [(0.5, 0.)]
        nav.shared_data['goal'] = (0.5, 0.)
        nav.front_distance = 0.2
        nav.control_loop()
        cmd = nav.cmd_pub.publish.call_args.args[0]
        self.assertEqual((cmd.linear.x, cmd.angular.z), (0, 0))
        self.assertEqual(nav.shared_data['state'], 'OBSTACLE_STOP')

    def test_short_and_same_cell_goals_are_retained(self):
        nav = Navigation()
        nav.shared_data['goal'] = (0.08, 0.)
        nav._plan_to_goal()
        self.assertEqual(nav.state, 'NAV')
        self.assertEqual(nav.path[-1], (0.08, 0.))
        nav.shared_data['goal'] = (5., 0.)
        nav._plan_to_goal()
        self.assertEqual(nav.state, 'NO_PATH')
        self.assertFalse(nav.path)

    def test_dynamic_obstacle_causes_a_detour_and_expires(self):
        nav = Navigation()
        nav.shared_data['goal'] = (0.8, 0.)
        nav._plan_to_goal()
        first = nav.path[:]
        nav.dynamic_obstacles[nav.world_to_grid(0.4, 0)] = time.monotonic()
        self.assertTrue(nav._refresh_obstacles(time.monotonic()))
        nav._plan_to_goal()
        self.assertNotEqual(nav.path, first)
        self.assertTrue(nav._refresh_obstacles(time.monotonic()+2))
        self.assertFalse(nav.dynamic_obstacles)

    def test_nan_scan_is_not_treated_as_free(self):
        nav = Navigation()
        msg = LaserScan()
        msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max = -0.1, 0.1, 0.12, 3.5
        msg.ranges = [float('nan')]*3
        nav.scan_cb(msg)
        self.assertIsNone(nav.last_scan_received)
        msg.ranges = [float('inf')]*3
        nav.scan_cb(msg)
        self.assertIsNotNone(nav.last_scan_received)

    def test_manager_restarts_as_readonly_before_map_controller(self):
        class Supervisor(ManagerProcessesMixin):
            pass
        manager = Supervisor()
        manager.session_dir = Path('/tmp/session-test')
        events = []
        manager._stop_process = lambda name: events.append(('stop', name))
        manager._launch = lambda command: events.append(('launch', command)) or Mock()
        manager._manifest = lambda state: None
        manager._start_map_navigation('/tmp/map.yaml')
        self.assertEqual(events[:2], [('stop', 'odom_proc'), ('stop', 'nav_proc')])
        self.assertIn('odometry_only:=true', events[2][1])
        self.assertEqual(events[3][1][1], 'nav_odom.py')

    def test_manager_does_not_navigate_after_mapping_error(self):
        manager = ManagerProcessesMixin()
        manager.auto_mode_active, manager.auto_phase = True, 'mapping'
        manager._stop_process = lambda name: 1
        manager._validate_session_map = Mock()
        manager._start_map_navigation = Mock()
        manager._job = lambda work, completed: work()
        with self.assertRaises(RuntimeError):
            manager.finish_auto_mapping()
        manager._validate_session_map.assert_not_called()
        manager._start_map_navigation.assert_not_called()

    def test_manager_rejects_map_from_another_session(self):
        with tempfile.TemporaryDirectory() as directory:
            grid = Grid(directory)
            grid.filename = 'out'
            grid.scan_counter = 1
            grid.map[:] = 0.1
            grid.input_file = str(Path(directory)/'mapping.csv')
            grid.save_map()
            manager = ManagerProcessesMixin()
            manager.session_dir, manager.session_id = Path(directory), 'another-session'
            manager._manifest = Mock()
            with self.assertRaises(ValueError):
                manager._validate_session_map()
            manager._manifest.assert_not_called()

    def test_stale_wheels_do_not_publish_fresh_looking_odometry(self):
        now = time.monotonic()
        state = SimpleNamespace(imu_ok=True, last_control_monotonic=None,
                                last_joint_received=None, last_imu_received=now,
                                last_scan_received=now, sensor_timeout=.5,
                                enc_v=0., fused_w=0., odometry_only=True,
                                last_log_t=2., _last_log_data=2., _log_period_s=.1,
                                LOG_PERIOD=2., _publish_odom=Mock(),
                                get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=2_000_000_000)))
        ReactiveControlMixin._control_loop(state)
        state._publish_odom.assert_not_called()
        state.last_joint_received = time.monotonic()
        ReactiveControlMixin._control_loop(state)
        state._publish_odom.assert_called_once_with(0., 0.)

    def test_no_diagonal_corner_cutting_and_no_path(self):
        planner = DStarLite(np.array([[0, 2], [2, 0]]))
        self.assertIsNone(planner.plan((0, 0), (1, 1)))
        self.assertEqual(planner.plan((0, 0), (0, 0)), [(0, 0)])
        self.assertIsNone(planner.plan((-1, 0), (0, 0)))

    def test_planner_cost_matches_independent_graph_solver(self):
        from scipy.sparse.csgraph import dijkstra
        rng = np.random.default_rng(42)
        for _ in range(30):
            grid = np.where(rng.random((5, 5)) < .25, 2, 0)
            grid[0, 0] = grid[-1, -1] = 0
            graph = np.full((25, 25), np.inf)
            for a in range(25):
                for b in range(25):
                    ar, ac, br, bc = a//5, a%5, b//5, b%5
                    if a != b and max(abs(ar-br), abs(ac-bc)) == 1 and grid[ar, ac] == grid[br, bc] == 0:
                        if ar != br and ac != bc and (grid[ar, bc] or grid[br, ac]):
                            continue
                        graph[a, b] = math.hypot(ar-br, ac-bc)
            expected = dijkstra(graph, indices=0)[24]
            path = DStarLite(grid).plan((0, 0), (4, 4))
            if math.isinf(expected):
                self.assertIsNone(path)
            else:
                cost = sum(math.hypot(a[0]-b[0], a[1]-b[1]) for a, b in zip(path, path[1:]))
                self.assertAlmostEqual(cost, expected)


if __name__ == '__main__':
    unittest.main()
