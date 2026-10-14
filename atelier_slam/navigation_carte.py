#!/usr/bin/env python3
"""Navigation sur une carte validée, avec odométrie commune et surveillance LiDAR."""
import argparse
import math
import time
import signal
from multiprocessing import Process, Manager
import numpy as np
from scipy.ndimage import binary_dilation, distance_transform_edt
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from planificateur_chemin import DStarLite
from slam_components.session_ipc import SessionChannel
from slam_components.command_ownership import CommandLease
from slam_components.pid import PIDController
from slam_components.map_format import load_map
from slam_components.map_localization import MapLocalizationMixin
from slam_components.map_control import MapControlMixin
from slam_components.map_view import plotter_process
from slam_components.settings import ODOM_TOPIC, ODOM_FRAME, SENSOR_TIMEOUT, ROBOT_CLEARANCE, LIDAR_YAW_OFFSET, \
                                      SOFT_MARGIN, SOFT_WEIGHT, KNOWN_TOLERANCE

__all__ = ['TurtleController', 'PIDController', 'plotter_process', 'LIDAR_YAW_OFFSET', 'main']


class TurtleController(MapLocalizationMixin, MapControlMixin, Node):
    def _prepare_planning_layers(self):
        """Zone « déjà connue » (murs, inconnu) et coût de proximité des murs."""
        res = self.meta['resolution']
        pixels = self.grid/255.0 if self.meta.get('negate', 0) else (255.0-self.grid)/255.0
        not_free = pixels >= self.meta['free_thresh']
        tolerance = max(1, int(math.ceil(KNOWN_TOLERANCE/res)))
        yy, xx = np.ogrid[-tolerance:tolerance+1, -tolerance:tolerance+1]
        self.known_zone = binary_dilation(not_free, structure=xx*xx+yy*yy <= tolerance*tolerance, border_value=1)
        # Pénalité décroissante entre la marge dure et SOFT_MARGIN (ne bloque jamais).
        distance = distance_transform_edt(~not_free)*res
        soft = np.clip((self.clearance+SOFT_MARGIN-distance)/SOFT_MARGIN, 0.0, 1.0)
        self.soft_cost = SOFT_WEIGHT*soft*soft
        self.wall_distance = distance

    def __init__(self, yaml_path, shared_data, channel=None):
        super().__init__('turtle_controller')
        self.shared_data = shared_data
        self.clearance = ROBOT_CLEARANCE
        self.meta, self.grid, self.static_grid = load_map(yaml_path, clearance=self.clearance)
        self._prepare_planning_layers()
        if self.meta.get('frame_id') not in (ODOM_FRAME, 'map'):
            raise ValueError(f'Carte attendue dans le repère {ODOM_FRAME}.')
        self.res, self.origin = self.meta['resolution'], self.meta['origin']
        self.h, self.w = self.grid.shape
        self.planner = DStarLite(self.static_grid, cost=self.soft_cost)
        self.seen_obstacles = {}
        self.last_plan_time = time.monotonic()
        self.robot_pos = [0.0, 0.0, 0.0]
        self.raw_odom = [0.0, 0.0, 0.0]
        # L'origine de l'image n'est pas une transformation de pose.
        self.odom_offset = self.meta.get('map_from_odom', [0.0, 0.0, 0.0])
        self.dynamic_obstacles = {}
        self.sensor_timeout = SENSOR_TIMEOUT
        self.last_odom_received = self.last_scan_received = None
        self.front_distance = 0.0
        self.path = []
        self.state = 'READY'
        self.pid_heading = PIDController(0.9, 0.05, 0.3, out_min=-0.7, out_max=0.7,
                                         integrator_max=0.2, derivative_alpha=0.2)
        self.last_control_time = self.last_relocalize_time = time.monotonic()
        self.relocalize_interval = 10.0
        self.relocalisation_visuelle = self.last_camera_frame = None
        self.declare_parameter('enable_visual_relocalization', False)
        self.enable_visual_relocalization = self.get_parameter('enable_visual_relocalization').value
        if self.enable_visual_relocalization:
            from pathlib import Path
            from relocalisation_visuelle import GlobalLocalizer
            self.relocalisation_visuelle = GlobalLocalizer(str(Path(yaml_path).parent/self.meta['image']))
        self.command_lease = CommandLease()
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, ODOM_TOPIC, self.odom_cb, qos_profile_sensor_data)
        if self.enable_visual_relocalization:
            self.create_subscription(Image, '/camera/image_raw', self.camera_cb, qos_profile_sensor_data)
        def tick():
            if channel:
                command = channel.receive()
                if command:
                    kind, values = command
                    if kind == 'stop':
                        self.shared_data.update(state='STOPPED', click=None, set_pose_cmd=None)
                    elif kind == 'goal' and self.world_to_grid(*values) is not None:
                        # Never queue an instruction until sensors recover.
                        fresh = all(t is not None and time.monotonic()-t < self.sensor_timeout
                                    for t in (self.last_scan_received, self.last_odom_received))
                        if fresh:
                            self.shared_data.update(click=values, ui_mode='GOAL')
                    elif kind == 'pose' and self.world_to_grid(*values[:2]) is not None:
                        self.shared_data.update(set_pose_cmd=values, state='STOPPED', click=None)
                        self._pending_pose_id = channel.last_id
            self.control_loop()
            if getattr(self, '_pending_pose_id', None) and self.shared_data.get('set_pose_cmd') is None:
                self._pose_command_id = self._pending_pose_id
                self._pending_pose_id = None
            if channel and time.monotonic()-getattr(self, '_status_time', 0) > .2:
                channel.status(dict(state=self.shared_data.get('state'), pose=self.robot_pos,
                                    path=list(self.path), goal=self.shared_data.get('goal'),
                                    new_obstacles=list(self.shared_data.get('new_obstacles', [])),
                                    pose_received=self.shared_data.get('pose_received', False),
                                    pose_command_id=getattr(self, '_pose_command_id', None)))
                self._status_time = time.monotonic()
            if self.shared_data.get('quit') and rclpy.ok():
                rclpy.shutdown()
        self.create_timer(0.05, tick)


def main():
    from slam_components.run_logging import install_run_logging
    install_run_logging('navigation_carte')
    parser = argparse.ArgumentParser()
    parser.add_argument('yaml')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--ipc-dir')
    parser.add_argument('--token')
    args, ros_args = parser.parse_known_args()
    with Manager() as manager:
        shared = manager.dict(robot_pos=[0, 0, 0], state='READY', quit=False,
                              pose_received=False, ui_mode='GOAL', set_pose_cmd=None)
        node = process = None
        rclpy.init(args=ros_args)
        try:
            channel = SessionChannel(args.ipc_dir, args.token, 'navigation') if args.ipc_dir and args.token else None
            node = TurtleController(args.yaml, shared, channel)
            if not args.headless:
                process = Process(target=plotter_process, args=(shared, args.yaml))
                process.start()
            def stop(signum, frame):
                shared.update(quit=True, state='STOPPED')
            signal.signal(signal.SIGINT, stop)
            signal.signal(signal.SIGTERM, stop)
            rclpy.spin(node)
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            pass
        finally:
            shared['quit'] = True
            if node is not None:
                if rclpy.ok():
                    node.cmd_pub.publish(Twist())
                node.destroy_node()
                node.command_lease.close()
            if rclpy.ok():
                rclpy.shutdown()
            if process is not None:
                process.join(timeout=2)
                if process.is_alive():
                    process.terminate()
                    process.join(timeout=2)


if __name__ == '__main__':
    main()
