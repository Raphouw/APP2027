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
from collections import deque
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, LaserScan

# Topics dont dépend la cartographie : nom -> rôle affiché dans l'interface.
EXPECTED = {'/imu': 'IMU', '/joint_states': 'Roues', '/scan': 'LiDAR'}


ARRIVALS = {name: deque(maxlen=200) for name in EXPECTED}
WINDOW = 5.0  # secondes pour la fréquence


def rate_info(name, now):
    times = [t for t in ARRIVALS[name] if now-t <= WINDOW]
    age = now-ARRIVALS[name][-1] if ARRIVALS[name] else None
    hz = (len(times)-1)/(times[-1]-times[0]) if len(times) >= 2 and times[-1] > times[0] else 0.0
    return dict(hz=round(hz, 1), age=None if age is None else round(age, 2))


def snapshot(node):
    topics = []
    now = time.monotonic()
    for name, types in sorted(node.get_topic_names_and_types()):
        info = dict(name=name, types=types, publishers=node.count_publishers(name),
                    subscribers=node.count_subscribers(name))
        if name in EXPECTED:
            info.update(rate_info(name, now))
        topics.append(info)
    return topics


def main():
    parser = argparse.ArgumentParser(description='Sonde du graphe ROS')
    parser.add_argument('--output', required=True)
    parser.add_argument('--period', type=float, default=1.0)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = Node('sonde_ros')
    for topic, kind in (('/imu', Imu), ('/joint_states', JointState), ('/scan', LaserScan)):
        node.create_subscription(kind, topic, lambda msg, t=topic: ARRIVALS[t].append(time.monotonic()),
                                 qos_profile_sensor_data)
    stop = []
    signal.signal(signal.SIGINT, lambda *a: stop.append(1))
    signal.signal(signal.SIGTERM, lambda *a: stop.append(1))
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    try:
        while not stop and rclpy.ok():
            deadline = time.monotonic()+args.period
            while not stop and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            data = dict(updated_at=time.time(), domain_id=os.environ.get('ROS_DOMAIN_ID', '0 (défaut)'),
                        rmw=os.environ.get('RMW_IMPLEMENTATION', 'défaut'),
                        localhost_only=os.environ.get('ROS_LOCALHOST_ONLY', '0'),
                        topics=snapshot(node))
            tmp = output.with_suffix('.tmp')
            tmp.write_text(json.dumps(data))
            os.replace(tmp, output)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
