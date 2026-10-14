"""Filtrage LiDAR, inflation et sélection de passages.

Les attributs sont initialisés par le point d’entrée ; ce mixin ne lance aucun processus.
"""

import math
import time
from .lidar_filter import front_mask, ScanHistory, tolerate_front_gaps
from sensor_msgs.msg import LaserScan

class LidarPerceptionMixin:

    def _cb_scan(self, msg: LaserScan):
        self.raw_scan_ranges = list(msg.ranges)
        self.raw_scan_range_min = msg.range_min if math.isfinite(msg.range_min) else None
        self.raw_scan_range_max = msg.range_max if math.isfinite(msg.range_max) else None
        self.lidar_front_known = False
        self.lidar_front_unknown = 0
        self.lidar_inferred = 0
        self.lidar_temporal = 0
        self.raw_scan_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        self.last_scan_received = time.monotonic() if any(not math.isnan(r) and r > 0 for r in msg.ranges) else None
        self.laser_amin = msg.angle_min
        self.laser_ainc = msg.angle_increment

        n = len(msg.ranges)
        if n == 0 or msg.angle_increment == 0:
            self.last_scan_received = None
            self.laser_ranges = self.inflated_ranges = []
            return
        if not hasattr(self, 'lidar_history'):
            self.lidar_history = ScanHistory()
        distances, known, inferred, temporal = self.lidar_history.filter(
            msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max,
            [getattr(self, 'x', 0.), getattr(self, 'y', 0.), getattr(self, 'theta', 0.)], time.monotonic())
        self.lidar_temporal = int(temporal.sum())
        front = front_mask(n, msg.angle_min, msg.angle_increment)
        distances, known, _ = tolerate_front_gaps(distances, known, front)
        self.lidar_front_known = bool(front.any() and known[front].all())
        self.lidar_front_unknown = int((~known[front]).sum())
        self.lidar_inferred = int(inferred.sum())
        self.last_scan_received = time.monotonic() if self.lidar_front_known else None
        cleaned = distances.tolist()

        # C2 – Inflation : pour chaque obstacle, on réduit les rayons voisins
        # dans un cône de ±arcsin(robot_radius / distance)
        # Cela simule le volume du robot sur la grille angulaire du LiDAR.
        inflated = list(cleaned)
        for i, r in enumerate(cleaned):
            if known[i] and 0 < r < msg.range_max * 0.95:   # point d'obstacle réel
                # Angle du cône d'inflation (rayon robot / distance)
                if r > self.robot_radius:
                    half_cone = math.asin(
                        min(1.0, self.robot_radius / r)
                    )
                else:
                    half_cone = math.pi / 2.0  # obstacle très proche → large cône

                n_spread = max(1, int(math.ceil(half_cone / abs(self.laser_ainc))))
                for j in range(-n_spread, n_spread + 1):
                    idx = (i + j) % n
                    # L'obstacle "vu" depuis le centre robot est à distance r
                    # Les rayons voisins doivent être au moins aussi proches
                    inflated[idx] = min(inflated[idx], r)

        self.laser_ranges    = cleaned
        self.inflated_ranges = inflated

    def _min_in_sector(self, center_angle: float, half_fov: float,
                       use_inflated=True) -> float:
        """Distance minimum dans un secteur angulaire (repère robot)."""
        ranges = self.inflated_ranges if use_inflated else self.laser_ranges
        if not ranges:
            return 3.5
        m = 3.5
        for i, r in enumerate(ranges):
            a = self.norm_angle(self.laser_amin + i * self.laser_ainc)
            if abs(self.norm_angle(a - center_angle)) < half_fov:
                m = min(m, r)
        return m

    def _front_min(self) -> float:
        return self._min_in_sector(0.0, self.FRONT_FOV)

    def _is_goal_line_blocked(self, angle_to_goal: float) -> bool:
        """Vrai si un obstacle est dans le cône goal à moins de AVOID_START_DIST."""
        m = self._min_in_sector(angle_to_goal, self.GOAL_FOV)
        return m < self.AVOID_START_DIST

    def _find_best_gap_angle(self, angle_to_goal: float) -> float:
        """
        Trouve le gap libre le plus large, le plus proche de la direction goal,
        et dont le milieu n'est pas à plus de 120° du cap courant
        (évite les demi-tours).

        Retourne l'angle (repère robot) du centre du meilleur gap.
        """
        ranges = self.inflated_ranges
        if not ranges:
            return math.copysign(0.8, angle_to_goal)

        n = len(ranges)
        FREE_THRESH = self.AVOID_START_DIST

        # Segmentation en gaps libres
        in_gap   = False
        gaps     = []
        cur_gap  = []

        for i in range(2 * n):   # double-pass pour gérer le wrap-around
            idx = i % n
            if ranges[idx] >= FREE_THRESH:
                cur_gap.append(idx)
                in_gap = True
            else:
                if in_gap and len(cur_gap) >= self.MIN_GAP_WIDTH:
                    gaps.append(list(cur_gap))
                cur_gap = []
                in_gap  = False
            if i == n - 1 and in_gap:
                # fin du premier tour, stoppe pour ne pas fusionner deux fois
                if len(cur_gap) >= self.MIN_GAP_WIDTH:
                    gaps.append(list(cur_gap))
                cur_gap = []

        # Déduplique (le wrap-around peut avoir créé des doublons)
        seen  = set()
        ugaps = []
        for g in gaps:
            key = tuple(sorted(set(g)))
            if key not in seen:
                seen.add(key)
                ugaps.append(g)
        gaps = ugaps

        if not gaps:
            # Pas de gap libre avec AVOID_START_DIST → cherche le rayon maximum
            # dans les ranges bruts (sans inflation) pour trouver la direction la plus ouverte
            raw = self.laser_ranges if self.laser_ranges else ranges
            max_r = -1.0
            best_escape = math.copysign(0.8, angle_to_goal)
            for i, r in enumerate(raw):
                a = self.norm_angle(self.laser_amin + i * self.laser_ainc)
                if r > max_r:
                    max_r = r
                    best_escape = a
            self.get_logger().warn(
                f"[GAP] Aucun gap libre trouvé — direction d'évasion : {math.degrees(best_escape):.1f}°"
            )
            return best_escape

        best_score = -1e9
        best_angle = self.gap_heading  # conserve l'angle précédent par défaut
        # Premier passage : sans pénalité demi-tour (trouve le meilleur gap absolu)
        # Second passage si rien trouvé : autorise les demi-tours
        for allow_uturn in (False, True):
            for g in gaps:
                mid_idx   = g[len(g) // 2]
                gap_angle = self.norm_angle(
                    self.laser_amin + mid_idx * self.laser_ainc
                )

                # Pénalise les demi-tours sauf si c'est le dernier recours
                if not allow_uturn and abs(gap_angle) > math.radians(130):
                    continue

                diff_goal  = abs(self.norm_angle(gap_angle - angle_to_goal))
                width      = len(g)
                depth      = sum(min(ranges[i], FREE_THRESH + 0.5) for i in g) / len(g)

                # Verrou de côté dur : pénalité massive si on est du mauvais côté
                side_pen = 0.0
                if self.locked_side != 0.0 and abs(gap_angle) > 0.05:
                    gap_side = math.copysign(1.0, gap_angle)
                    if gap_side != self.locked_side:
                        side_pen = 5000.0  # pratiquement interdit

                # Score prédictif : évalue la qualité du gap projeté 1m en avant
                pred_score = self._predictive_gap_score(gap_angle)

                score = (0.50 * width
                         - 2.0 * math.degrees(diff_goal)
                         - 1.0 * math.degrees(abs(gap_angle))
                         + 0.8 * depth
                         - side_pen
                         + 1.5 * math.degrees(1.0) * pred_score)

                if score > best_score:
                    best_score = score
                    best_angle = gap_angle

            if best_score > -1e9:
                break

        # Mémorise le signe du gap choisi pour le prochain appel
        if abs(best_angle) > 0.05:
            self.last_gap_sign = math.copysign(1.0, best_angle)

        return best_angle

    def _find_best_gap_angle_sided(self, angle_to_goal: float, side: float):
        """
        Comme _find_best_gap_angle, mais contraint au côté indiqué (side=+1 droite, -1 gauche).
        Retourne None si aucun gap acceptable n'est trouvé de ce côté.
        """
        ranges = self.inflated_ranges
        if not ranges:
            return None

        n = len(ranges)
        FREE_THRESH = self.AVOID_START_DIST

        # Segmentation identique
        in_gap, gaps, cur_gap = False, [], []
        for i in range(2 * n):
            idx = i % n
            if ranges[idx] >= FREE_THRESH:
                cur_gap.append(idx)
                in_gap = True
            else:
                if in_gap and len(cur_gap) >= self.MIN_GAP_WIDTH:
                    gaps.append(list(cur_gap))
                cur_gap = []
                in_gap = False
            if i == n - 1 and in_gap:
                if len(cur_gap) >= self.MIN_GAP_WIDTH:
                    gaps.append(list(cur_gap))
                cur_gap = []

        best_score, best_angle = -1e9, None
        for g in gaps:
            mid_idx   = g[len(g) // 2]
            gap_angle = self.norm_angle(self.laser_amin + mid_idx * self.laser_ainc)

            # Filtre de côté : le gap doit être du même côté que side
            if abs(gap_angle) > 0.05 and math.copysign(1.0, gap_angle) != side:
                continue
            if abs(gap_angle) > math.radians(130):
                continue

            diff_goal = abs(self.norm_angle(gap_angle - angle_to_goal))
            width     = len(g)
            depth     = sum(min(ranges[i], FREE_THRESH + 0.5) for i in g) / len(g)
            score     = 0.50 * width - 2.0 * math.degrees(diff_goal) - 1.0 * math.degrees(abs(gap_angle)) + 0.8 * depth

            if score > best_score:
                best_score = score
                best_angle = gap_angle

        return best_angle  # None si rien trouvé de ce côté

    def _predictive_gap_score(self, gap_angle_robot: float) -> float:
        """
        Projette une position virtuelle P_ahead = robot + D_AHEAD * direction(gap)
        en repère monde, puis évalue :
          - angular_improvement : est-ce que P_ahead fait face au goal mieux que maintenant ?
          - forward_clearance   : le chemin P_ahead → goal est-il dégagé ?
        Retourne un score normalisé dans [-1.0, +1.0].
        """
        if self.goal_x is None or not self.inflated_ranges:
            return 0.0

        D_AHEAD = 1.0  # m : distance de projection

        # Direction monde du gap
        world_gap_angle = self.norm_angle(self.theta + gap_angle_robot)

        # Position virtuelle projetée
        px = self.x + D_AHEAD * math.cos(world_gap_angle)
        py = self.y + D_AHEAD * math.sin(world_gap_angle)

        # Angle monde depuis P_ahead vers le goal
        dx_pg = self.goal_x - px
        dy_pg = self.goal_y - py
        angle_ahead_to_goal = math.atan2(dy_pg, dx_pg)

        # Angle monde depuis robot courant vers le goal
        dx_rg = self.goal_x - self.x
        dy_rg = self.goal_y - self.y
        angle_now_to_goal = math.atan2(dy_rg, dx_rg)

        # Amélioration angulaire : positif si la position projetée est mieux alignée
        # sur le goal que la position actuelle
        ang_diff_now   = abs(self.norm_angle(angle_now_to_goal   - world_gap_angle))
        ang_diff_ahead = abs(self.norm_angle(angle_ahead_to_goal - world_gap_angle))
        angular_improvement = self.clamp(
            (ang_diff_now - ang_diff_ahead) / math.pi, -1.0, 1.0
        )

        # Dégagement vers le goal depuis la position projetée (scan actuel, repère robot)
        ahead_to_goal_robot = self.norm_angle(angle_ahead_to_goal - self.theta)
        clearance = self._min_in_sector(ahead_to_goal_robot, 0.20)
        clearance_score = self.clamp(clearance / self.AVOID_START_DIST, 0.0, 1.0)

        return 0.6 * angular_improvement + 0.4 * clearance_score
