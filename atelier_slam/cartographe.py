#!/usr/bin/env python3
from slam_components.slam_mapping import SlamMappingMixin
from slam_components.settings import ODOM_TOPIC
from slam_components.occupancy_grid import OccupancyGridMixin
from slam_components.mapping_io import MappingIOMixin
from slam_components.mapping_runtime import MappingRuntimeMixin

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from slam_components.session_ipc import SessionChannel
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import numpy as np
import threading
import argparse
import signal

class Cartographer(SlamMappingMixin, OccupancyGridMixin, MappingIOMixin, MappingRuntimeMixin, Node):
    def __init__(self, filename, mode="live", input_file="", decay=0.999):
        if mode == 'live':
            super().__init__('lidar_visualizer')
        self.filename = "map" if filename == "" else filename
        self.mode = mode
        self.input_file = input_file
        self.stop_requested = False
        self.non_interactive = False
        self.pose_received = False
        self.output_dir = "./logs"
        self.session_id = None
        self.initialize_slam()

        self.dist_max_lidar = 3.5 # mètre
        self.scale = 10
        self.size = 200
        self.grow_size = 50

        # PHASE 3 : Représentation log-odds au lieu de probabilités simples
        # Log-odds = log(p / (1-p)), meilleure numériquement et plus robuste
        # 0 = log(0.5) = 0 (inconnu)
        # +inf = log(1) = occupé
        # -inf = log(0) = libre
        self.map_logodd = np.zeros([self.size, self.size])  # 0 = inconnu (log-odds)
        self.map = np.full([self.size, self.size], 0.5)      # Pour affichage (0.5 = inconnu)

        self.x0 = self.size // 2
        self.y0 = self.size // 2
        self.x = self.x0
        self.y = self.y0
        self.path_x = [self.x]
        self.path_y = [self.y]
        self.orientation = 0.0 # radian
        self.decay = decay  # 0.999 : oubli lent, appliqué par scan aux cellules instables
        self.grow_threshold = 0.1

        # Paramètres log-odds
        self.log_occ = np.log(0.7 / (1.0 - 0.7))      # Occupé : log(0.7 / 0.3) ≈ 0.847
        self.log_free = np.log(0.3 / (1.0 - 0.3))     # Libre : log(0.3 / 0.7) ≈ -0.847

        # PHASE 3+ : Tracker changements détectés pour decay robuste
        self.cell_update_time = np.zeros([self.size, self.size], dtype=np.int32)
        self.cell_change_count = np.zeros([self.size, self.size], dtype=np.int32)  # Compte les changements réels
        self.cell_prev_state = np.full([self.size, self.size], 0.5, dtype=np.float32)  # État précédent
        self.scan_counter = 0
        self.change_threshold = 5  # Nombre de changements détectés avant d'oublier
        self.state_diff_threshold = 0.2  # Différence minimale pour compter comme "changement"

        # PHASE 6 : Multiplicateur de vitesse pour mode live
        self.speed_multiplier = 1.0  # Sera assigné par le main
        
        self.latest_scan = None
        self.lock = threading.Lock()
        
        # Initialisation du graphique
        self.custom_cmap = ListedColormap(['green', 'lightgray', 'black'])
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.img = self.ax.imshow(self.map.T, cmap=self.custom_cmap, origin='lower', vmin=0, vmax=1)
        self.ax.set_title(f"Visualisation LiDAR - Mode: {self.mode.upper()}")
        self.path_line, = self.ax.plot([], [], color='red', linewidth=1.5, label="Chemin")

        if self.mode == "live":
            self.get_logger().info("Mode LIVE : En attente des topics /scan et /odom...")
            self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
            self.map_tf = TransformBroadcaster(self)
            self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
            self.create_subscription(Odometry, ODOM_TOPIC, self.pose_cb, qos_profile_sensor_data)
        else:
            print(f"Mode OFFLINE : Prêt à lire le fichier {self.input_file}")


def main():
    from slam_components.run_logging import install_run_logging
    install_run_logging('cartographe')
    parser = argparse.ArgumentParser(description='Cartographie LiDAR live/offline')
    parser.add_argument('-oname', '--output_file_name', default='out')
    parser.add_argument('-m', '--mode', choices=['live', 'offline'], default='live')
    parser.add_argument('-i', '--input', default='')
    parser.add_argument('--headless', action='store_true', help='Aucune fenêtre graphique')
    parser.add_argument('--auto', action='store_true', help='Sans menu ; sauvegarde à la fin')
    parser.add_argument('--speed', type=float, default=1.0)
    parser.add_argument('--output-dir', default='./logs')
    parser.add_argument('--session-id', default=None)
    parser.add_argument('--token')
    args, ros_args = parser.parse_known_args()
    if args.speed <= 0:
        parser.error('--speed doit être positif')
    if args.mode == "live":
        rclpy.init(args=ros_args)
    if args.headless:
        plt.switch_backend('Agg')
    viewer = Cartographer(args.output_file_name, args.mode, args.input)
    viewer.speed_multiplier = args.speed
    viewer.output_dir, viewer.session_id = args.output_dir, args.session_id
    viewer.non_interactive = args.auto
    viewer.headless = args.headless
    viewer.channel = SessionChannel(args.output_dir, args.token, 'mapping') if args.token else None
    def stop(signum, frame):
        viewer.stop_requested = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        if args.auto:
            if args.mode == 'offline':
                viewer.run_carto_offline()
            else:
                viewer.run_carto_live()
            viewer.save_map()
        else:
            viewer.run()
    finally:
        if args.mode == 'live':
            viewer.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        plt.close('all')


if __name__ == '__main__':
    main()
