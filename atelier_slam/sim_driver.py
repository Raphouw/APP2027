#!/usr/bin/env python3
"""Pilote automatique du robot simulé : fait N tours du couloir autour de l'îlot central.

Lit la vérité terrain `/sim/ground_truth` et publie `/cmd_vel`. Sert aux démonstrations
et aux mesures d'erreur sans télécommande.
"""
import argparse
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Circuit de la salle intégrée (mètres, repère du monde simulé).
LOOP = [(1.5, 2.0), (9.3, 2.0), (9.3, 7.0), (1.6, 7.0), (1.6, 2.0)]


class Driver(Node):
    def __init__(self, laps, speed):
        super().__init__('atelier_sim_driver')
        self.targets = LOOP * laps
        self.index = 0
        self.speed = speed
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/sim/ground_truth', self._on_pose, 10)
        self.done = False

    def _on_pose(self, msg):
        if self.index >= len(self.targets):
            self.pub.publish(Twist())
            self.done = True
            return
        pose = msg.pose.pose
        theta = 2 * math.atan2(pose.orientation.z, pose.orientation.w)
        tx, ty = self.targets[self.index]
        dx, dy = tx - pose.position.x, ty - pose.position.y
        if math.hypot(dx, dy) < 0.15:
            self.index += 1
            return
        error = (math.atan2(dy, dx) - theta + math.pi) % (2 * math.pi) - math.pi
        command = Twist()
        command.angular.z = max(-1.5, min(1.5, 2.0 * error))
        command.linear.x = self.speed * max(0.0, math.cos(error)) if abs(error) < 1.0 else 0.0
        self.pub.publish(command)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--laps', type=int, default=2)
    parser.add_argument('--speed', type=float, default=0.18)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = Driver(args.laps, args.speed)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
