#!/usr/bin/env python3
"""Robot simulé : publie /joint_states, /imu et /scan, lit /cmd_vel, sans Gazebo.

Les nœuds de l'atelier tournent alors sans modification. La vérité terrain est
écrite dans un CSV (timestamp,x,y,theta) pour `evaluate_trajectory.py`.
"""
import argparse
import csv
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState, LaserScan
from tf2_ros import StaticTransformBroadcaster

from slam_components.sim_world import (BEAMS, RANGE_MAX, RANGE_MIN, SimRobot, World, default_world)

CMD_TIMEOUT = 0.5  # s : sans commande récente, le robot s'arrête
PHYSICS_HZ = 100.0
SCAN_EVERY = 10    # un scan tous les 10 pas de physique (10 Hz)


class SimulatorNode(Node):
    def __init__(self, world, pose, noise, truth_csv, seed=0):
        super().__init__('atelier_simulator')
        self.robot = SimRobot(world, pose=pose, noise=noise, seed=seed)
        self.cmd = (0.0, 0.0)
        self.last_cmd = 0.0
        self.tick = 0
        self.last_time = time.monotonic()
        self.joint_pub = self.create_publisher(JointState, '/joint_states', qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, '/imu', qos_profile_sensor_data)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.truth_pub = self.create_publisher(Odometry, '/sim/ground_truth', 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        static = TransformStamped()
        static.header.stamp = self.get_clock().now().to_msg()
        static.header.frame_id, static.child_frame_id = 'base_link', 'base_scan'
        static.transform.rotation.w = 1.0
        self._static = StaticTransformBroadcaster(self)
        self._static.sendTransform(static)
        self.truth_file = None
        if truth_csv:
            path = Path(truth_csv)
            path.parent.mkdir(parents=True, exist_ok=True)
            self.truth_file = path.open('w', newline='')
            self.truth_writer = csv.writer(self.truth_file)
            self.truth_writer.writerow(['timestamp', 'x', 'y', 'theta'])
        self.create_timer(1.0 / PHYSICS_HZ, self._step)
        self.get_logger().info(f'Simulateur prêt : pose {pose}, bruit {noise}.')

    def _on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)
        self.last_cmd = time.monotonic()

    def _step(self):
        now = time.monotonic()
        dt = min(now - self.last_time, 0.1)
        self.last_time = now
        v, w = self.cmd if now - self.last_cmd < CMD_TIMEOUT else (0.0, 0.0)
        robot = self.robot
        robot.step(v, w, dt)
        stamp = self.get_clock().now().to_msg()
        seconds = stamp.sec + stamp.nanosec * 1e-9

        joint = JointState()
        joint.header.stamp = stamp
        joint.name = ['wheel_left_joint', 'wheel_right_joint']
        joint.position = list(robot.wheel_positions())
        self.joint_pub.publish(joint)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'imu_link'
        imu.angular_velocity.z = robot.gyro()
        imu.orientation_covariance[0] = -1.0
        self.imu_pub.publish(imu)

        truth = Odometry()
        truth.header.stamp = stamp
        truth.header.frame_id = 'sim_world'
        truth.child_frame_id = 'base_link'
        truth.pose.pose.position.x, truth.pose.pose.position.y = robot.x, robot.y
        truth.pose.pose.orientation.z = math.sin(robot.theta / 2)
        truth.pose.pose.orientation.w = math.cos(robot.theta / 2)
        truth.twist.twist.linear.x, truth.twist.twist.angular.z = robot.v, robot.w
        self.truth_pub.publish(truth)
        if self.truth_file:
            self.truth_writer.writerow([f'{seconds:.6f}', f'{robot.x:.5f}', f'{robot.y:.5f}', f'{robot.theta:.5f}'])

        self.tick += 1
        if self.tick % SCAN_EVERY == 0:
            scan = LaserScan()
            scan.header.stamp = stamp
            scan.header.frame_id = 'base_scan'
            scan.angle_min = 0.0
            scan.angle_increment = 2 * math.pi / BEAMS
            scan.angle_max = 2 * math.pi - scan.angle_increment
            scan.time_increment = 0.1 / BEAMS
            scan.scan_time = 0.1
            scan.range_min, scan.range_max = RANGE_MIN, RANGE_MAX
            scan.ranges = [float(r) for r in robot.scan()]
            self.scan_pub.publish(scan)

    def close(self):
        if self.truth_file:
            self.truth_file.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', help='carte PGM (noir = obstacle) ; sinon salle intégrée')
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--origin', type=float, nargs=2, default=(0.0, 0.0))
    parser.add_argument('--pose', type=float, nargs=3, default=(1.5, 2.0, 0.0), metavar=('X', 'Y', 'THETA'))
    parser.add_argument('--noise', type=float, default=1.0, help='0 = capteurs parfaits')
    parser.add_argument('--seed', type=int, default=0, help='graine du bruit (reproductible)')
    parser.add_argument('--truth-csv', help='écrit la vérité terrain (timestamp,x,y,theta)')
    args, ros_args = parser.parse_known_args()
    world = World.from_pgm(args.map, args.resolution, args.origin) if args.map else default_world()
    rclpy.init(args=ros_args)
    node = SimulatorNode(world, tuple(args.pose), args.noise, args.truth_csv, args.seed)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
