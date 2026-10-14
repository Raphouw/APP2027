"""Machine à états, régulation et commandes de navigation réactive.

Les attributs sont initialisés par le point d’entrée ; ce mixin ne lance aucun processus.
"""

import math
import time
from .settings import ODOM_FRAME, STOP_DISTANCE
import collections
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String, Float32

class ReactiveControlMixin:

    @staticmethod
    def norm_angle(a: float) -> float:
        """Ramène un angle dans ]-π, π]."""
        return math.atan2(math.sin(a), math.cos(a))

    @staticmethod
    def clamp(val, mn, mx):
        return max(mn, min(mx, val))

    def slew(self, target: float, current: float, max_rate: float) -> float:
        """Rampe : limite la variation de `current` vers `target` à max_rate·dt."""
        step = max_rate * self.dt
        return self.clamp(target, current - step, current + step)

    def slew_v(self, target: float, current: float) -> float:
        """
        Rampe asymétrique pour la vitesse linéaire :
        - Accélération limitée à max_dv_accel (douce)
        - Décélération limitée à max_dv_decel (rapide → pas de traîne en fin de course)
        """
        if target < current:  # décélération
            return self.slew(target, current, self.max_dv_decel)
        else:                  # accélération
            return self.slew(target, current, self.max_dv_accel)

    def get_target_vector(self):
        """
        Retourne (distance, angle_robot_frame) vers le goal courant.
        angle_robot_frame dans ]-π, π].
        """
        dx   = self.goal_x - self.x
        dy   = self.goal_y - self.y
        dist = math.hypot(dx, dy)
        angle = self.norm_angle(math.atan2(dy, dx) - self.theta)
        return dist, angle

    def _cb_goal(self, msg: PoseStamped):
        if self.odometry_only:
            return
        if msg.header.frame_id != ODOM_FRAME or not all(math.isfinite(v) for v in (msg.pose.position.x, msg.pose.position.y)):
            self.get_logger().warn(f'Objectif rejeté : coordonnées invalides ou repère différent de {ODOM_FRAME}.')
            return
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.state  = 'MOVE_TO_GOAL'

        # Reset path : nouveau goal = nouvelle trajectoire
        self.path_msg = Path()
        self.path_msg.header.frame_id = ODOM_FRAME
        self._last_path_x = None
        self._last_path_y = None

        # Reset PID et historique FSM
        self.pid_ang.reset()
        self.pid_ang_avoid.reset()
        self.pid_lin.reset()
        self.last_gap_sign = 0.0
        self._obs_votes = collections.deque(
            [False] * self._obs_window_size,
            maxlen=self._obs_window_size
        )
        # Reset verrou de côté
        self.locked_side        = 0.0
        self._lock_no_gap_count = 0
        # Reset anti-boucle
        self._loop_detected       = False
        self._loop_confidence     = 0.0
        self._loop_pos_history.clear()
        self._loop_last_trigger_t = 0.0

        self.get_logger().info(
            f"[GOAL] Nouvel objectif reçu : X={self.goal_x:.2f}, Y={self.goal_y:.2f}"
        )

    def _publish_cmd(self, v: float, w: float):
        if self.cmd_pub is None:
            return 0.0, 0.0
        if not getattr(self, 'lidar_front_known', True):
            self.prev_v = self.prev_w = 0.0
            self.cmd_pub.publish(Twist())
            return 0.0, 0.0
        if self.last_scan_received is None or self._front_min() <= STOP_DISTANCE:
            v = 0.0
            self.prev_v = 0.0
        # B4 – Slew rate asymétrique pour v, standard pour w
        v = self.slew_v(v, self.prev_v)
        w = self.slew(w, self.prev_w, self.max_dw)
        self.prev_v, self.prev_w = v, w

        msg = Twist()
        msg.linear.x  = float(self.clamp(v, -self.max_v, self.max_v))
        msg.angular.z = float(self.clamp(w, -self.max_w, self.max_w))
        self.cmd_pub.publish(msg)
        return msg.linear.x, msg.angular.z

    def _control_loop(self):
        if hasattr(self, 'write_health') and hasattr(self, 'pose_file_path'):
            self.write_health()
        if not self.imu_ok:
            return

        t_now = self.get_clock().now().nanoseconds * 1e-9

        monotonic_now = time.monotonic()
        if self.last_control_monotonic is not None:
            self.dt = min(0.1, max(0.001, monotonic_now-self.last_control_monotonic))
        self.last_control_monotonic = monotonic_now
        # La pose est intégrée par les encodeurs, jamais par les commandes.
        wheels_fresh = self.last_joint_received is not None and monotonic_now-self.last_joint_received < self.sensor_timeout
        imu_fresh = self.last_imu_received is not None and monotonic_now-self.last_imu_received < self.sensor_timeout
        v_curr = self.enc_v if wheels_fresh else 0.0
        w_curr = self.fused_w if wheels_fresh and imu_fresh else 0.0

        # A5 – Publication odométrie + TF
        if wheels_fresh and imu_fresh:
            self._publish_odom(v_curr, w_curr)

	# ---- Logs périodiques ----
        if t_now - self.last_log_t > self.LOG_PERIOD:
            self.get_logger().info(
                f"[ODOM] X={self.x:.2f} Y={self.y:.2f} "
                f"θ={math.degrees(self.theta):.1f}° | [FSM] {self.state} "
                f"| [LIDAR] front={self._front_min():.2f}m "
                f"unknown_front={getattr(self, 'lidar_front_unknown', 0)} "
                f"inferred={getattr(self, 'lidar_inferred', 0)} "
                f"recent={getattr(self, 'lidar_temporal', 0)}"
            )
            self.last_log_t = t_now

        # ---- Data logging (10 Hz) ----
        if t_now - self._last_log_data >= self._log_period_s:
            front_min_log = self._front_min() if self.laser_ranges else 0.0
            self._log_data(t_now, front_min_log)
            self._last_log_data = t_now


        scan_fresh = self.last_scan_received is not None and monotonic_now-self.last_scan_received < self.sensor_timeout
        if self.odometry_only:
            return
        if not (wheels_fresh and imu_fresh and scan_fresh):
            self.prev_v = self.prev_w = 0.0
            self.cmd_pub.publish(Twist())
            return
        if self._front_min() <= STOP_DISTANCE:
            # L'arrêt de translation est immédiat ; une rotation d'évitement reste possible.
            self.prev_v = 0.0

        # ---- État repos ----
        if self.state in ('IDLE', 'GOAL_REACHED') or self.goal_x is None:
            self._publish_cmd(0.0, 0.0)
            # Publication état FSM même au repos
            _ns = String(); _ns.data = self.state
            self.nav_state_pub.publish(_ns)
            return

        # ---- Vecteur vers goal (B2) ----
        dist_to_goal, angle_to_goal = self.get_target_vector()

        # D1 – Détection anti-boucle (échantillonnage position + publication)
        self._update_loop_detection(t_now)
        
        
        # Objectif atteint — vérifié dans TOUS les états
        if dist_to_goal < 0.12:
            self.state  = 'GOAL_REACHED'
            self.goal_x = None
            self.pid_ang.reset()
            self.pid_ang_avoid.reset()
            self.pid_lin.reset()
            self.last_gap_sign = 0.0
            self.get_logger().info("[FSM] ✓ Objectif atteint !")
            self._publish_cmd(0.0, 0.0)
            return
	

        # ---- LiDAR ----
        front_min    = self._front_min()
        goal_blocked = self._is_goal_line_blocked(angle_to_goal)

        # C4 – Vote hysteresis : fenêtre glissante pour éviter oscillations FSM
        obstacle_now = goal_blocked or (front_min < self.AVOID_START_DIST)
        self._obs_votes.append(obstacle_now)
        obs_ratio = sum(self._obs_votes) / self._obs_window_size

        # ---- Transitions FSM (C4) ----
        if self.state == 'MOVE_TO_GOAL':
            if obs_ratio >= self._OBS_TRIGGER_RATIO:
                self.state = 'FOLLOW_GAP'
                self.follow_enter_time = t_now
                self.last_gap_update   = 0.0
                # Initialise le gap heading
                self.gap_heading = self._find_best_gap_angle(angle_to_goal)
                self.pid_ang.reset()
                self.pid_ang_avoid.reset()
                self.desired_heading_filt = angle_to_goal
                self.get_logger().warn(
                    f"[FSM] Obstacle détecté ({obs_ratio:.0%}) → FOLLOW_GAP "
                    f"(front={front_min:.2f}m)"
                )

        elif self.state == 'FOLLOW_GAP':
            time_in = t_now - self.follow_enter_time
            stable_clear = (obs_ratio <= self._CLR_TRIGGER_RATIO
                            and time_in > self.MIN_FOLLOW_TIME)

            # Forcer retour MOVE_TO_GOAL si très proche du goal :
            # en dessous de 0.45m, l'évitement peut faire rater l'arrivée
            near_goal_override = (dist_to_goal < 0.45 and front_min > self.AVOID_STOP_DIST)

            if stable_clear or near_goal_override:
                self.state = 'MOVE_TO_GOAL'
                self.pid_ang.reset()
                self.pid_ang_avoid.reset()
                self.pid_lin.reset()
                reason = "proche goal" if near_goal_override else f"chemin libre ({obs_ratio:.0%})"
                self.get_logger().info(f"[FSM] {reason} → MOVE_TO_GOAL")


       

        # =========================================================
        # COMMANDES PAR ÉTAT
        # =========================================================
        tv, tw = 0.0, 0.0

        # ---------------------------------------------------------
        # État MOVE_TO_GOAL
        # ---------------------------------------------------------
        if self.state == 'MOVE_TO_GOAL':
            # PID angulaire (B3) : aligner vers le goal
            tw = self.pid_ang.compute(angle_to_goal, self.dt)
            tw = self.clamp(tw * self.max_w, -self.max_w, self.max_w)

            # Loi de vitesse directe : profil trapézoïdal
            #   - pleine vitesse tant que dist > RAMP_DIST
            #   - décroissance linéaire entre RAMP_DIST et 0.10m (seuil d'arrivée)
            #   - plancher V_MIN pour éviter de ramper à l'infini
            RAMP_DIST = 0.40   # m : début de la décélération
            V_MIN     = 0.04   # m/s : vitesse minimale en approche finale
            if dist_to_goal >= RAMP_DIST:
                v_target = self.max_v
            else:
                # Interpolation linéaire entre V_MIN (à 0.10m) et max_v (à RAMP_DIST)
                t_ramp = (dist_to_goal - 0.10) / max(1e-6, RAMP_DIST - 0.10)
                v_target = V_MIN + (self.max_v - V_MIN) * self.clamp(t_ramp, 0.0, 1.0)

            # Réduction douce si grand désalignement (>45°), plancher à 0.30
            ANG_THRESH = math.radians(45)
            if abs(angle_to_goal) <= ANG_THRESH:
                align_factor = 1.0
            else:
                excess = abs(angle_to_goal) - ANG_THRESH
                align_factor = max(0.30, 1.0 - excess / math.radians(90))

            tv = v_target * align_factor

        # ---------------------------------------------------------
        # État FOLLOW_GAP (C3, C4)
        # ---------------------------------------------------------
        elif self.state == 'FOLLOW_GAP':

            # C3 – Mise à jour du verrou de côté (doit précéder le scoring des gaps)
            self._update_side_lock(t_now, angle_to_goal)

            # Mise à jour du gap (forcée si bloqué, sinon périodique)
            force_update = (front_min <= self.AVOID_STOP_DIST)
            if force_update or (t_now - self.last_gap_update) > self.gap_update_period:
                new_gap = self._find_best_gap_angle(angle_to_goal)
                # Lissage EWMA
                self.gap_heading = (self.GAP_EWMA_ALPHA * new_gap
                                    + (1.0 - self.GAP_EWMA_ALPHA) * self.gap_heading)
                self.gap_heading = self.norm_angle(self.gap_heading)
                self.last_gap_update = t_now

            # Publication du gap sélectionné (pour live_mapper)
            _sg = Float32(); _sg.data = float(self.gap_heading)
            self.selected_gap_pub.publish(_sg)

            # Blend progressif goal → gap
            goal_dist  = self._min_in_sector(angle_to_goal, self.GOAL_FOV)
            danger_min = min(front_min, goal_dist)
            alpha = self.clamp(
                (self.AVOID_START_DIST - danger_min)
                / max(1e-6, self.AVOID_START_DIST - self.AVOID_STOP_DIST),
                0.0, 1.0
            )

            desired_heading = self.norm_angle(
                (1.0 - alpha) * angle_to_goal + alpha * self.gap_heading
            )

            # D1 – Ajout de la répulsion anti-boucle au heading
            if self._loop_detected and self._loop_confidence > 0.0:
                repulsion_robot = self.norm_angle(
                    self._loop_repulsion_angle - self.theta
                )
                rep_weight = self.LOOP_REPULSION_WEIGHT * self._loop_confidence
                desired_heading = self.norm_angle(
                    (1.0 - rep_weight) * desired_heading
                    + rep_weight * repulsion_robot
                )

            # Filtre passe-bas sur desired_heading
            self.desired_heading_filt = self.norm_angle(
                self.DH_FILTER_ALPHA * desired_heading
                + (1.0 - self.DH_FILTER_ALPHA) * self.desired_heading_filt
            )
            smooth_heading = self.desired_heading_filt

            # Fix 2 – PID dédié évitement (ki=0 → zéro accumulation → zéro dépassement)
            tw = self.pid_ang_avoid.compute(smooth_heading, self.dt)
            tw = self.clamp(tw * self.max_w, -0.80, 0.80)

            # Vitesse linéaire
            if front_min <= self.AVOID_STOP_DIST:
                tv = 0.0
                if abs(tw) < 0.25:
                    tw = math.copysign(
                        0.40,
                        smooth_heading if abs(smooth_heading) > 0.01
                        else self.gap_heading if abs(self.gap_heading) > 0.01
                        else self.last_gap_sign if self.last_gap_sign != 0.0 else 1.0
                    )
            else:
                beta = self.clamp(
                    (front_min - self.AVOID_STOP_DIST)
                    / max(1e-6, self.AVOID_START_DIST - self.AVOID_STOP_DIST),
                    0.0, 1.0
                )
                # ⚡ OPTIMISÉ: moins de ralentissement, meilleure agilité
                turn_pen = self.clamp(
                    1.0 - 0.35 * abs(smooth_heading) / math.pi,  # Moins agressif
                    0.50, 1.0  # Minimum plus élevé pour manœuvrabilité
                )
                v_min = 0.040  # Un peu plus rapide
                v_max = self.max_v * 0.85  # Plus de vitesse max
                tv = (v_min + (v_max - v_min) * beta) * turn_pen

        # ---------------------------------------------------------
        # Publish avec slew rate (B4)
        # ---------------------------------------------------------
        # Si le robot est à l'arrêt forcé (obstacle très proche), reset prev_v
        # pour que la reprise soit immédiate dès que l'espace se libère
        if self.state == 'FOLLOW_GAP' and front_min <= self.AVOID_STOP_DIST:
            self.prev_v = 0.0

        self._publish_cmd(tv, tw)

        # Publication état FSM (pour live_mapper)
        _ns = String(); _ns.data = self.state
        self.nav_state_pub.publish(_ns)
