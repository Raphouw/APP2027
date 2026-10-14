"""Mesures roues/IMU, intégration de pose et publication odométrique.

Les attributs sont initialisés par le point d’entrée ; ce mixin ne lance aucun processus.
"""

import math
import time
from .settings import ODOM_FRAME, BASE_FRAME
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseStamped

class OdometryMixin:

    def _cb_joint(self, msg: JointState):
        """
        Calcule la vitesse linéaire encodeurs en gérant le wrapping [-π, π].
        On ne calcule que enc_v ici ; le cap est fourni par l'IMU (gyro).
        """
        if not self.imu_ok:
            return

        try:
            li = msg.name.index('wheel_left_joint')
            ri = msg.name.index('wheel_right_joint')
        except ValueError:
            return

        if max(li, ri) >= len(msg.position):
            return
        wl, wr = msg.position[li], msg.position[ri]
        if not math.isfinite(wl) or not math.isfinite(wr):
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = time.monotonic()
        uses_stamp = stamp > 0
        sample_time = stamp if uses_stamp else now
        basis_changed = getattr(self, '_joint_uses_stamp', uses_stamp) != uses_stamp
        self._joint_uses_stamp = uses_stamp
        if self.last_wl is None or basis_changed:
            self.last_wl, self.last_wr = wl, wr
            self.last_joint_time = sample_time
            self.last_joint_received = now
            if stamp > 0 and hasattr(self, 'log_pose_buffer'):
                self.log_pose_buffer.add(stamp, [self.x, self.y, self.theta])
            return
        dt = sample_time - self.last_joint_time
        if dt <= 0:
            return  # message dupliqué ou reçu dans le désordre
        dwl, dwr = self.norm_angle(wl - self.last_wl), self.norm_angle(wr - self.last_wr)
        self.last_wl, self.last_wr = wl, wr
        self.last_joint_time = sample_time
        self.last_joint_received = now
        # Après une interruption, établir une nouvelle référence sans saut de pose.
        if dt > self.sensor_timeout:
            self.enc_v = self.enc_dtheta = 0.0
            return
        dl, dr = dwl * self.wheel_radius, dwr * self.wheel_radius
        self.enc_v = (dl + dr) / (2 * dt)
        self.enc_dtheta = (dr - dl) / (self.wheel_base * dt)
        gyro_fresh = self.last_imu_received is not None and now-self.last_imu_received < self.sensor_timeout
        w = (self.alpha_comp * self.gyro_w + (1-self.alpha_comp) * self.enc_dtheta
             if gyro_fresh else self.enc_dtheta)
        self.fused_w = w
        # Intégrer exactement une fois par mesure de roues, avec son intervalle réel.
        self._rk4_update(self.enc_v, w, dt)
        if stamp > 0 and hasattr(self, 'log_pose_buffer'):
            self.log_pose_buffer.add(stamp, [self.x, self.y, self.theta])

    def _cb_imu(self, msg: Imu):
        wz = msg.angular_velocity.z
        if not math.isfinite(wz):
            return
        self.last_imu_received = time.monotonic()

        # Calibration : accumule des échantillons robot immobile
        if not self.imu_ok:
            self.calib_samples.append(wz)
            if len(self.calib_samples) >= self.CALIB_N:
                self.gyro_bias_z = sum(self.calib_samples) / float(self.CALIB_N)
                self.imu_ok = True
                self.get_logger().info(
                    f"[IMU] Calibration OK. Biais gyro Z = {self.gyro_bias_z:.6f} rad/s"
                )
            return

        # Correction biais + deadband
        corrected = wz - self.gyro_bias_z
        self.gyro_w = 0.0 if abs(corrected) < self.GYRO_DEADBAND else corrected

        # Filtre complémentaire : fusionne gyro (long terme) + encodeurs (court terme)
        # fused_w = alpha*gyro + (1-alpha)*encodeurs
        self.fused_w = (self.alpha_comp * self.gyro_w
                        + (1.0 - self.alpha_comp) * self.enc_dtheta)

    def _rk4_update(self, v: float, w: float, dt=None):
        """
        Intègre (x, y, θ) avec une méthode Runge-Kutta ordre 4.

        Modèle cinématique différentiel :
            dx/dt     = v · cos(θ)
            dy/dt     = v · sin(θ)
            dθ/dt     = w

        RK4 : y_{n+1} = y_n + (h/6)(k1 + 2·k2 + 2·k3 + k4)
        """
        h = self.dt if dt is None else dt
        if h <= 0:
            return

        def f(state, v, w):
            """Dérivées du modèle."""
            _, _, th = state
            return (v * math.cos(th), v * math.sin(th), w)

        x0, y0, th0 = self.x, self.y, self.theta

        # k1 : pente en début de pas
        dx1, dy1, dth1 = f((x0,             y0,             th0            ), v, w)
        # k2 : pente au milieu (estimée avec k1)
        dx2, dy2, dth2 = f((x0+h/2*dx1,     y0+h/2*dy1,     th0+h/2*dth1  ), v, w)
        # k3 : pente au milieu (estimée avec k2)
        dx3, dy3, dth3 = f((x0+h/2*dx2,     y0+h/2*dy2,     th0+h/2*dth2  ), v, w)
        # k4 : pente en fin de pas (estimée avec k3)
        dx4, dy4, dth4 = f((x0+h*dx3,       y0+h*dy3,       th0+h*dth3    ), v, w)

        self.x     += (h / 6.0) * (dx1 + 2*dx2 + 2*dx3 + dx4)
        self.y     += (h / 6.0) * (dy1 + 2*dy2 + 2*dy3 + dy4)
        self.theta  = self.norm_angle(
            self.theta + (h / 6.0) * (dth1 + 2*dth2 + 2*dth3 + dth4)
        )

    def _publish_odom(self, v: float, w: float):
        now = self.get_clock().now().to_msg()
        if getattr(self, '_joint_uses_stamp', False) and self.last_joint_time is not None:
            now.sec = int(self.last_joint_time)
            now.nanosec = min(999999999, int(round((self.last_joint_time-now.sec)*1e9)))
        qz  = math.sin(self.theta / 2.0)
        qw  = math.cos(self.theta / 2.0)

        # TF my_odom → base_footprint
        t = TransformStamped()
        t.header.stamp          = now
        t.header.frame_id       = ODOM_FRAME
        t.child_frame_id        = BASE_FRAME
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z  = qz
        t.transform.rotation.w  = qw
        self.tf_br.sendTransform(t)

        # Message Odometry
        o = Odometry()
        o.header.stamp          = now
        o.header.frame_id       = ODOM_FRAME
        o.child_frame_id        = BASE_FRAME
        o.pose.pose.position.x  = self.x
        o.pose.pose.position.y  = self.y
        o.pose.pose.orientation.z = qz
        o.pose.pose.orientation.w = qw
        o.twist.twist.linear.x  = v
        o.twist.twist.angular.z = w
        self.odom_pub.publish(o)
        
         # Publication du path RViz
        if (self._last_path_x is None
                or math.hypot(self.x - self._last_path_x,
                               self.y - self._last_path_y) >= self.PATH_MIN_DIST):
            pose = PoseStamped()
            pose.header.stamp    = now
            pose.header.frame_id = ODOM_FRAME  # <-- Modifié ici
            pose.pose.position.x = self.x
            pose.pose.position.y = self.y
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            self.path_msg.header.stamp = now
            self.path_msg.header.frame_id = ODOM_FRAME # <-- Sécurité supplémentaire
            self.path_msg.poses.append(pose)
            self.path_pub.publish(self.path_msg)
            self._last_path_x, self._last_path_y = self.x, self.y
