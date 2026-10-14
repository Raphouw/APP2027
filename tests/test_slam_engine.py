"""Deterministic mathematical and integration regressions, no physical robot."""
from pathlib import Path
import json
import csv
import subprocess
import math
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock, patch
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'mapping'))
from slam_components.slam_engine import compose, inverse, relative, transform, match_scan, PoseBuffer, PoseGraph, SlamEngine
from slam_components.session_ipc import SessionChannel, atomic_json
from slam_components.slam_mapping import SlamMappingMixin
from slam_components.manager_processes import ManagerProcessesMixin
from test_safety_and_mapping import Grid, Navigation
from evaluate_trajectory import evaluate
from slam_components.tk_text import tk_safe_text
from slam_components.tb3_logging import SessionLoggingMixin, SessionWriter


def cloud():
    rng = np.random.default_rng(125)
    return rng.uniform([-1.6, -1.3], [1.5, 1.6], (180, 2))


def square_run():
    world = cloud()
    truth = []
    for a, b in [([0, 0], [1.8, 0]), ([1.8, 0], [1.8, 1.8]),
                 ([1.8, 1.8], [0, 1.8]), ([0, 1.8], [0, 0])]:
        for f in np.linspace(0, 1, 7)[:-1]:
            truth.append(np.array([*(np.asarray(a)*(1-f)+np.asarray(b)*f), 0.]))
    truth.append(np.zeros(3))
    engine, estimates, odometry = SlamEngine(), [], []
    for i, pose in enumerate(truth):
        odom = pose + np.array([.012*i, -.006*i, .003*i])
        points = transform(world, inverse(pose))
        keep = np.linalg.norm(points, axis=1) < 3.5
        points = points[keep]
        estimates.append(engine.update(odom, points, np.ones(len(points), bool)))
        odometry.append(odom)
    return engine, np.asarray(truth), np.asarray(odometry), np.asarray(estimates)


class SlamTests(unittest.TestCase):
    def test_reference_evaluation_interpolates_and_rejects_gaps(self):
        reference = np.array([[1, 0, 0, 0], [1.2, 2, 0, .2]])
        estimate = np.array([[1.1, 1, 0, .1], [1.5, 5, 0, 0]])
        result = evaluate(estimate, reference)
        self.assertEqual(result['samples'], 1)
        self.assertEqual(result['skipped'], 1)
        self.assertAlmostEqual(result['position_rmse_m'], 0)
        with self.assertRaises(ValueError):
            evaluate(estimate, reference, max_gap=.1)

    def test_new_csv_uses_interpolated_scan_pose_once(self):
        with tempfile.TemporaryDirectory() as directory:
            logger = SessionLoggingMixin()
            logger.pose_file_path = str(Path(directory)/'last_pose.txt')
            logger.session_writer = SessionWriter(Path(directory)/'mapping.csv')
            logger.log_pose_buffer = PoseBuffer()
            logger.log_pose_buffer.add(1, [0, 0, 0])
            logger.log_pose_buffer.add(1.2, [2, 0, .2])
            logger.raw_scan_stamp, logger.last_logged_scan = 1.1, None
            logger.x, logger.y, logger.theta = 99, 99, 0
            logger.state = 'IDLE'
            logger.laser_amin, logger.laser_ainc = 0, .1
            logger.raw_scan_ranges = [1, 2, 3]
            logger._log_data(9, 1)
            logger._log_data(10, 1)
            logger.session_writer.close()
            with (Path(directory)/'mapping.csv').open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 1)
            self.assertEqual(float(rows[0]['timestamp']), 1.1)
            self.assertAlmostEqual(float(rows[0]['x']), 1.)
            self.assertEqual((Path(directory)/'last_pose.txt').read_text(), '99,99,0')

    def test_standalone_runs_keep_unique_exception_logs(self):
        with tempfile.TemporaryDirectory() as directory:
            code = ("import sys; sys.path.insert(0, " + repr(str(Path(__file__).resolve().parents[1]/'mapping')) + "); "
                    "from slam_components.run_logging import install_run_logging; "
                    "install_run_logging('example', " + repr(directory) + "); print('before crash'); raise RuntimeError('debug marker')")
            for _ in range(2):
                process = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True, timeout=5)
                self.assertNotEqual(process.returncode, 0)
            logs = list(Path(directory).glob('*/example.log'))
            self.assertEqual(len(logs), 2)
            for path in logs:
                self.assertIn('before crash', path.read_text())
                self.assertIn('RuntimeError: debug marker', path.read_text())
                self.assertIn('Traceback', path.read_text())

    def test_tk_text_keeps_accents_and_removes_crash_trigger(self):
        self.assertEqual(tk_safe_text('Échap · prêt '+chr(0x1f916)), 'Échap · prêt [symbole]')

    def test_controller_restart_requires_pose_confirmation(self):
        manager = ManagerProcessesMixin()
        manager.odom_proc = object()
        manager.session_dir = None
        manager._require_pose = False
        manager._stop_process = Mock()
        manager._stop_all()
        self.assertTrue(manager._require_pose)

    def test_se2_group_and_rotation(self):
        pose = [.7, -.4, .6]
        np.testing.assert_allclose(compose(pose, inverse(pose)), 0, atol=1e-12)
        np.testing.assert_allclose(transform([[1, 0]], [0, 0, np.pi/2]), [[0, 1]], atol=1e-12)

    def test_matching_recovers_translation_and_rotation(self):
        source = cloud()
        truth = np.array([.2, -.12, .12])
        result = match_scan(source, transform(source, truth), [.12, -.06, .06])
        self.assertTrue(result.accepted, result)
        np.testing.assert_allclose(result.pose, truth, atol=1e-4)

    def test_matching_rejects_wall_and_nonoverlap(self):
        wall = np.column_stack([np.linspace(-1, 1, 80), np.zeros(80)])
        self.assertFalse(match_scan(wall, wall, [0, 0, 0]).accepted)
        self.assertFalse(match_scan(cloud(), cloud()+10, [0, 0, 0]).accepted)

    def test_rejected_match_keeps_initial_pose(self):
        initial = [.2, -.3, .1]
        match = match_scan(cloud(), cloud()+10, initial)
        np.testing.assert_array_equal(match.pose, initial)

    def test_pose_interpolates_across_pi_without_extrapolation(self):
        buffer = PoseBuffer()
        buffer.add(1, [0, 0, math.radians(179)])
        buffer.add(1.2, [2, 4, math.radians(-179)])
        pose = buffer.at(1.1)
        np.testing.assert_allclose(pose[:2], [1, 2])
        self.assertAlmostEqual(abs(pose[2]), np.pi)
        self.assertIsNone(buffer.at(.9))
        self.assertIsNone(buffer.at(1.3))
        buffer.add(1.1, [99, 99, 0])
        self.assertEqual(len(buffer.samples), 2)

    def test_pose_gap_is_not_interpolated(self):
        buffer = PoseBuffer()
        buffer.add(1, [0, 0, 0]); buffer.add(2, [0, 0, 0])
        self.assertIsNone(buffer.at(1.5))

    def test_pose_graph_loop_reduces_drift_with_fixed_anchor(self):
        graph = PoseGraph()
        for pose in [[0, 0, 0], [1.1, 0, 0], [1.1, 1.1, 0], [.1, .1, 0]]:
            graph.add(pose)
        for i in range(3):
            graph.constrain(i, i+1, relative(graph.poses[i], graph.poses[i+1]))
        graph.constrain(0, 3, [0, 0, 0], (.02, .02, .02), loop=True)
        before = np.linalg.norm(graph.poses[-1][:2])
        self.assertTrue(graph.optimize())
        self.assertLess(np.linalg.norm(graph.poses[-1][:2]), before*.2)
        np.testing.assert_array_equal(graph.poses[0], [0, 0, 0])

    def test_correction_persists_when_following_scan_is_rejected(self):
        engine = SlamEngine()
        points = cloud()
        engine.update([0, 0, 0], points, np.ones(len(points), bool))
        corrected = engine.update([.1, 0, .08], points, np.ones(len(points), bool))
        correction = engine.map_from_odom.copy()
        rejected = np.array([[1, 0], [2, 0]])
        pose = engine.update([.2, 0, .08], rejected, [True, True])
        np.testing.assert_allclose(pose, compose(correction, [.2, 0, .08]))
        self.assertLess(np.linalg.norm(corrected[:2]), .01)

    def test_square_revisit_closes_loop_and_improves_trajectory(self):
        engine, truth, odom, estimates = square_run()
        raw = np.sqrt(np.mean(np.sum((odom[:, :2]-truth[:, :2])**2, axis=1)))
        corrected = np.sqrt(np.mean(np.sum((estimates[:, :2]-truth[:, :2])**2, axis=1)))
        self.assertGreater(engine.loops, 0)
        self.assertLess(corrected, raw*.5)
        self.assertEqual(len(list(engine.scans())), len(truth))

    def test_rebuilt_raster_uses_optimized_past_poses(self):
        class Mapper(SlamMappingMixin, Grid):
            pass
        with tempfile.TemporaryDirectory() as directory:
            mapper = Mapper(directory)
            mapper.initialize_slam()
            mapper.integrate_scan([0, 0, 0], [1]*80, -1, 1)
            original = mapper.slam.update
            def optimized(*args):
                pose = original(*args)
                mapper.slam.graph.poses[0][0] += .5
                mapper.slam.revision += 1
                return pose
            mapper.slam.update = optimized
            mapper.integrate_scan([0, 0, 0], [1]*80, -1, 1)
            self.assertEqual(mapper.scan_counter, 2)
            self.assertAlmostEqual((mapper.path_x[0]-mapper.x0)/mapper.scale, .5)
            self.assertTrue((Path(directory)/'preview.png').exists())
            json.loads((Path(directory)/'slam_status.json').read_text())

    def test_map_offset_is_applied_to_raw_odom(self):
        nav = Navigation()
        nav.odom_offset = [1, 2, np.pi/2]
        nav.raw_odom = [1, 0, 0]
        nav._apply_odom()
        np.testing.assert_allclose(nav.robot_pos, [1, 3, np.pi/2])

    def test_ipc_token_expiry_invalid_and_duplicate(self):
        with tempfile.TemporaryDirectory() as directory:
            channel = SessionChannel(directory, 'new', 'navigation')
            SessionChannel(directory, 'old', 'navigation').send('goal', [1, 2])
            self.assertIsNone(channel.receive())
            channel.send('goal', [1, 2])
            self.assertEqual(channel.receive(), ('goal', [1., 2.]))
            self.assertIsNone(channel.receive())
            atomic_json(Path(directory)/'navigation_command.json', dict(token='new', id='expired', time=time.time()-4, kind='goal', values=[1, 2]))
            self.assertIsNone(channel.receive())
            channel.send('pose', [1, 2])
            self.assertIsNone(channel.receive())
            channel.send('stop')
            self.assertEqual(channel.receive(), ('stop', []))

    def test_run_log_contains_stdout_stderr_and_command(self):
        with tempfile.TemporaryDirectory() as directory:
            manager = ManagerProcessesMixin()
            manager._closing = manager._cancel_requested = False
            manager.session_dir = Path(directory)
            manager.base_dir = directory
            manager.debug_run_id = 'test-run'
            process = manager._launch([sys.executable, '-c', 'import sys; print("stdout marker"); print("stderr marker", file=sys.stderr)'])
            self.assertEqual(process.wait(timeout=5), 0)
            logs = list((Path(directory)/'debug').glob('*.log'))
            self.assertEqual(len(logs), 1)
            content = logs[0].read_text()
            self.assertIn('stdout marker', content)
            self.assertIn('stderr marker', content)
            self.assertEqual(json.loads(content.splitlines()[0])['manager_run_id'], 'test-run')


if __name__ == '__main__':
    unittest.main()
