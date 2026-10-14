#!/usr/bin/env python3
"""Sonde du graphe ROS : écrit la liste des topics détectés dans logs/ros_graph.json.

Lancée par l'interface (atelier.py), elle ne publie rien : elle observe seulement
quels topics existent et combien de publieurs alimentent les capteurs attendus.
"""
import argparse
import json
import os
import signal
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

# Topics dont dépend la cartographie : nom -> rôle affiché dans l'interface.
EXPECTED = {'/imu': 'IMU', '/joint_states': 'Roues', '/scan': 'LiDAR'}


def snapshot(node):
    topics = []
    for name, types in sorted(node.get_topic_names_and_types()):
        topics.append(dict(name=name, types=types,
                           publishers=node.count_publishers(name),
                           subscribers=node.count_subscribers(name)))
    return topics


def main():
    parser = argparse.ArgumentParser(description='Sonde du graphe ROS')
    parser.add_argument('--output', required=True)
    parser.add_argument('--period', type=float, default=1.0)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = Node('sonde_ros')
    stop = []
    signal.signal(signal.SIGINT, lambda *a: stop.append(1))
    signal.signal(signal.SIGTERM, lambda *a: stop.append(1))
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    try:
        while not stop and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            data = dict(updated_at=time.time(), domain_id=os.environ.get('ROS_DOMAIN_ID', '0 (défaut)'),
                        rmw=os.environ.get('RMW_IMPLEMENTATION', 'défaut'),
                        localhost_only=os.environ.get('ROS_LOCALHOST_ONLY', '0'),
                        topics=snapshot(node))
            tmp = output.with_suffix('.tmp')
            tmp.write_text(json.dumps(data))
            os.replace(tmp, output)
            time.sleep(args.period)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
