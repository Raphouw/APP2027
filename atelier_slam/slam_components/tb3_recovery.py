"""Détection de confinement et verrou du côté d’évitement.

Les attributs sont initialisés par le point d’entrée ; ce mixin ne lance aucun processus.
"""

import math
from std_msgs.msg import Bool, Float32

class RecoveryMixin:

    def _update_loop_detection(self, t_now: float) -> None:
        """
        Échantillonne la position toutes les LOOP_SAMPLE_PERIOD secondes.
        Si le robot reste confiné dans un cercle de rayon LOOP_SPATIAL_THRESH
        pendant LOOP_TRIGGER_RATIO de la fenêtre → loop détectée.
        Calcule un vecteur de répulsion pointant LOIN du centroïde.
        """
        if t_now - self._last_loop_sample_t < self.LOOP_SAMPLE_PERIOD:
            return
        self._last_loop_sample_t = t_now
        self._loop_pos_history.append((self.x, self.y))

        # Attendre que la fenêtre soit pleine
        if len(self._loop_pos_history) < self.LOOP_HISTORY_N:
            return

        # Centroïde des positions récentes
        cx = sum(p[0] for p in self._loop_pos_history) / self.LOOP_HISTORY_N
        cy = sum(p[1] for p in self._loop_pos_history) / self.LOOP_HISTORY_N

        # Fraction de positions dans le cercle "coinçé"
        inside = sum(
            1 for p in self._loop_pos_history
            if math.hypot(p[0] - cx, p[1] - cy) < self.LOOP_SPATIAL_THRESH
        )
        ratio = inside / self.LOOP_HISTORY_N
        self._loop_confidence = self.clamp(
            (ratio - self.LOOP_TRIGGER_RATIO) / max(1e-6, 1.0 - self.LOOP_TRIGGER_RATIO),
            0.0, 1.0
        )

        prev_detected = self._loop_detected
        cooldown_ok = (t_now - self._loop_last_trigger_t) > self.LOOP_COOLDOWN
        self._loop_detected = (ratio >= self.LOOP_TRIGGER_RATIO and cooldown_ok)

        if self._loop_detected and not prev_detected:
            self._loop_last_trigger_t = t_now
            self.get_logger().warn(
                f"[LOOP] Boucle détectée (ratio={ratio:.0%}). "
                f"Centroïde=({cx:.2f}, {cy:.2f})"
            )

        # Calcul du vecteur de répulsion (direction monde, loin du centroïde)
        if self._loop_detected:
            dx_away = self.x - cx
            dy_away = self.y - cy
            dist_from_centroid = math.hypot(dx_away, dy_away)
            if dist_from_centroid > 0.01:
                self._loop_repulsion_angle = math.atan2(dy_away, dx_away)
            else:
                # Robot exactement au centroïde → perpendiculaire à la direction goal
                self._loop_repulsion_angle = self.norm_angle(
                    math.atan2(self.goal_y - self.y, self.goal_x - self.x) + math.pi / 2.0
                )

        # Publication
        b_msg = Bool()
        b_msg.data = self._loop_detected
        self.loop_state_pub.publish(b_msg)

        f_msg = Float32()
        f_msg.data = float(self._loop_repulsion_angle)
        self.loop_repulsion_pub.publish(f_msg)

    def _update_side_lock(self, t_now: float, angle_to_goal: float) -> None:
        """
        Met à jour self.locked_side avant le scoring des gaps.
        Conditions de déverrouillage :
          (a) chemin vers le goal dégagé → verrou inutile
          (b) expiration naturelle après LOCK_DURATION secondes
          (c) LOCK_NO_GAP_CYCLES cycles consécutifs sans gap viable du côté verrouillé
        Acquisition du verrou : dès qu'un gap est choisi d'un côté.
        """
        # (a) Goal dégagé → libère
        if not self._is_goal_line_blocked(angle_to_goal):
            self.locked_side        = 0.0
            self._lock_no_gap_count = 0
            return

        # (b) Expiration naturelle
        if (self.locked_side != 0.0
                and (t_now - self.lock_start_time) > self.LOCK_DURATION):
            self.locked_side        = 0.0
            self._lock_no_gap_count = 0
            return

        # (c) Pas de gap disponible du côté verrouillé
        if self.locked_side != 0.0:
            sided = self._find_best_gap_angle_sided(angle_to_goal, self.locked_side)
            if sided is None:
                self._lock_no_gap_count += 1
            else:
                self._lock_no_gap_count = 0
            if self._lock_no_gap_count >= self.LOCK_NO_GAP_CYCLES:
                self.get_logger().warn(
                    f"[LOCK] Côté {self.locked_side:+.0f} bloqué {self.LOCK_NO_GAP_CYCLES} "
                    f"cycles → verrou libéré"
                )
                self.locked_side        = 0.0
                self._lock_no_gap_count = 0
                return

        # Acquisition d'un nouveau verrou si un côté vient d'être choisi
        if self.locked_side == 0.0 and abs(self.last_gap_sign) > 0.5:
            self.locked_side     = self.last_gap_sign
            self.lock_start_time = t_now
