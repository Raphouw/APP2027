"""Acquisition ROS, menu interactif et actualisation du cartographe.

Les attributs sont initialisés par le point d’entrée ; ce mixin ne lance aucun processus.
"""

from .settings import ODOM_FRAME
from .slam_engine import compose, inverse
from geometry_msgs.msg import PoseStamped, TransformStamped
import math
import numpy as np
import time
import select
import sys
import rclpy
import matplotlib.pyplot as plt

class MappingRuntimeMixin:

    def scan_callback(self, msg):
        with self.lock:
            self.latest_scan = msg

    def pose_cb(self, msg):
        if hasattr(msg, 'pose') and hasattr(msg.pose, 'pose'):
            if msg.header.frame_id != ODOM_FRAME:
                return
            self.pose_received = True
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            
            # Calcul de l'orientation (Yaw) en radians
            self.orientation = math.atan2(2.0*(o.w*o.z + o.x*o.y), 1.0 - 2.0*(o.y*o.y + o.z*o.z))
            self.x = p.x * self.scale + self.x0 
            self.y = p.y * self.scale + self.y0
            if hasattr(self, 'pose_buffer'):
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
                self.pose_buffer.add(stamp, [p.x, p.y, self.orientation])

    def update_plot(self):
        if getattr(self, 'headless', False):
            return
        self.img.set_data(self.map.T)
        self.img.set_extent([0, self.map.shape[1], 0, self.map.shape[0]])
        self.path_line.set_data(self.path_x, self.path_y)

        # On cherche tous les pixels qui ne sont plus "gris" (0.5)
        explored_indices = np.where(self.map.T != 0.5)
        
        # Si on a commencé à explorer la carte...
        if len(explored_indices[0]) > 0:
            # On trouve les limites de la zone dessinée
            y_min, y_max = np.min(explored_indices[0]), np.max(explored_indices[0])
            x_min, x_max = np.min(explored_indices[1]), np.max(explored_indices[1])
            
            # On ajoute une petite marge (ex: 5 pixels) pour que ça respire
            marge = 5
            self.ax.set_xlim(x_min - marge, x_max + marge)
            self.ax.set_ylim(y_min - marge, y_max + marge)

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def run(self):
        while True:
            print("\nMenu: ")
            print("1: lancer la cartographie (" + self.mode + ")")
            print("2: sauver la cartographie")
            print("0: quitter")
            try:
                q = int(input("> "))
            except ValueError:
                continue

            if q == 0:
                break
            elif q == 1:
                if self.mode == "live":
                    self.run_carto_live()
                else:
                    self.run_carto_offline()
            elif q == 2:
                self.save_map()
            else:
                print("> Commande inconnue !")

    def run_carto_live(self):
        last_save = time.monotonic()
        while rclpy.ok() and not self.stop_requested and plt.fignum_exists(self.fig.number):
            rclpy.spin_once(self, timeout_sec=0.01)
            if getattr(self, 'channel', None):
                command = self.channel.receive()
                if command and command[0] == 'goal' and self.pose_received:
                    raw_goal = compose(inverse(self.slam.map_from_odom), [*command[1], 0])
                    goal = PoseStamped()
                    goal.header.frame_id = ODOM_FRAME
                    goal.header.stamp = self.get_clock().now().to_msg()
                    goal.pose.position.x, goal.pose.position.y = float(raw_goal[0]), float(raw_goal[1])
                    goal.pose.orientation.w = 1.
                    self.goal_pub.publish(goal)
            if hasattr(self, 'map_tf'):
                pose = self.slam.map_from_odom
                tf = TransformStamped()
                tf.header.stamp = self.get_clock().now().to_msg()
                tf.header.frame_id, tf.child_frame_id = 'map', ODOM_FRAME
                tf.transform.translation.x, tf.transform.translation.y = float(pose[0]), float(pose[1])
                tf.transform.rotation.z, tf.transform.rotation.w = math.sin(pose[2]/2), math.cos(pose[2]/2)
                self.map_tf.sendTransform(tf)
            if not self.non_interactive:
                ready, _, _ = select.select([sys.stdin], [], [], 0)
                if ready and sys.stdin.readline().strip() == '0':
                    break
            with self.lock:
                msg, self.latest_scan = self.latest_scan, None
            if msg is not None and self.pose_received:
                if hasattr(self, 'slam'):
                    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
                    pose = self.pose_buffer.at(stamp)
                    if pose is None:
                        # Keep a just-arrived scan until its following odometry sample.
                        newest = self.pose_buffer.samples[-1][0] if self.pose_buffer.samples else 0
                        if stamp > newest and stamp-newest < .3:
                            with self.lock:
                                if self.latest_scan is None:
                                    self.latest_scan = msg
                        else:
                            self.unsynchronized_scans += 1
                            if self.unsynchronized_scans % 20 == 1:
                                self.get_logger().warning('Scan sans pose interpolable : vérifier les horodatages et les horloges ROS.')
                            self.write_preview()
                    else:
                        self.integrate_scan(pose, msg.ranges, msg.angle_min, msg.angle_max,
                                            msg.angle_increment, msg.range_min, stamp=stamp)
                else:
                    x, y = self.process_data(msg.ranges, msg.angle_min, msg.angle_max,
                                             msg.angle_increment, msg.range_min)
                    self.write_map(x, y)
                self.update_plot()
            if self.non_interactive and self.scan_counter and time.monotonic()-last_save >= 5:
                self.save_map()
                last_save = time.monotonic()
            if getattr(self, 'headless', False):
                time.sleep(.01/self.speed_multiplier)
            else:
                plt.pause(0.01/self.speed_multiplier)
