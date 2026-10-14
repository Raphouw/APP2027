#!/usr/bin/env python3
"""
Loop Closure Detection — SIFT Bag of Words
===========================================

Détecte quand le robot revient à un endroit déjà exploré.
Utilise SIFT pour une reconnaissance robuste et invariante aux transformations.

Permet de :
1. Corriger la dérive odométrique cumulée
2. Fermer les boucles de cartographie
3. Améliorer la cohérence globale de la carte
"""

import cv2
import numpy as np
from collections import defaultdict
import time


class LoopClosureDetector:
    """
    Détecteur de boucles basé sur SIFT bag-of-words.
    Stocke des images clés avec leurs poses et détecte les revisites.
    """

    def __init__(self, distance_threshold=0.5, feature_threshold=0.7, min_matches=20):
        """
        Initialiser le détecteur de boucles.

        Args:
            distance_threshold: Distance spatiale (m) pour considérer comme même lieu
            feature_threshold: Ratio de Lowe pour filtrer les correspondances (0.75)
            min_matches: Nombre minimum de correspondances pour valider une boucle
        """
        # Détecteur SIFT
        self.sift = cv2.SIFT_create()

        # Paramètres
        self.distance_threshold = distance_threshold
        self.feature_threshold = feature_threshold
        self.min_matches = min_matches

        # Base de données de frames clés
        # key = timestamp, value = (keypoints, descriptors, pose)
        self.reference_frames = {}
        self.frame_counter = 0

        # Matcher BFMatcher (plus robuste que FLANN pour SIFT)
        self.matcher = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)

        # Historique des boucles détectées
        self.loop_closures = []

        # Filtre temporel : ignorer les frames récentes
        self.min_frame_gap = 30  # Minimum 30 frames entre reference et query

    def extract_features(self, image):
        """
        Extraire les features SIFT d'une image.

        Args:
            image: np.ndarray BGR ou RGB

        Returns:
            (keypoints, descriptors) ou (None, None) si pas de features
        """
        if image is None or image.size == 0:
            return None, None

        # Convertir en grayscale si nécessaire
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image

        # Extraire SIFT
        kp, desc = self.sift.detectAndCompute(gray, None)

        if desc is None or len(desc) == 0:
            return None, None

        return kp, desc

    def add_reference_frame(self, timestamp, image, pose_x, pose_y, pose_theta):
        """
        Ajouter une frame clé à la base de données.

        Args:
            timestamp: Timestamp unique (ex: time.time())
            image: Imagen BGR/RGB
            pose_x, pose_y: Position du robot (m)
            pose_theta: Orientation du robot (rad)
        """
        kp, desc = self.extract_features(image)

        if desc is None:
            return False

        self.reference_frames[timestamp] = {
            'keypoints': kp,
            'descriptors': desc,
            'pose': (pose_x, pose_y, pose_theta),
            'frame_id': self.frame_counter
        }
        self.frame_counter += 1

        return True

    def _lowe_ratio_test(self, matches):
        """
        Appliquer le test de ratio de Lowe pour filtrer les fausses correspondances.
        knnMatch retourne les 2 plus proches voisins (k=2).

        Args:
            matches: liste de listes [[m1, m2], ...] où m1 < m2 en distance

        Returns:
            Indices des correspondances valides
        """
        good_matches = []

        for match_pair in matches:
            if len(match_pair) == 2:
                m, n = match_pair
                if m.distance < self.feature_threshold * n.distance:
                    good_matches.append(m)

        return good_matches

    def detect_loop(self, current_image, current_pose_x, current_pose_y, current_pose_theta):
        """
        Vérifier si la frame courante correspond à une frame clé passée (loop closure).

        Args:
            current_image: Image BGR/RGB
            current_pose_x, current_pose_y: Position courante (m)
            current_pose_theta: Orientation courante (rad)

        Returns:
            Dict avec info de boucle ou None si pas de boucle détectée
            {
                'ref_timestamp': timestamp de la frame de référence,
                'n_matches': nombre de correspondances,
                'match_ratio': ratio de correspondances,
                'pose_error': (dx, dy, dtheta),
                'certainty': score de confiance (0-1)
            }
        """
        kp_curr, desc_curr = self.extract_features(current_image)

        if desc_curr is None:
            return None

        best_match_result = None
        best_ratio = 0

        current_frame_id = self.frame_counter

        # Parcourir toutes les frames de référence
        for ref_ts, ref_data in self.reference_frames.items():
            # Filtre temporel : ignorer les frames trop récentes
            frame_gap = current_frame_id - ref_data['frame_id']
            if frame_gap < self.min_frame_gap:
                continue

            kp_ref = ref_data['keypoints']
            desc_ref = ref_data['descriptors']

            # Filtre spatial : seulement si distance < threshold
            ref_pose = ref_data['pose']
            spatial_dist = np.sqrt(
                (current_pose_x - ref_pose[0])**2 +
                (current_pose_y - ref_pose[1])**2
            )

            # Élargir le threshold pour la détection initiale (3m au lieu de 0.5m)
            if spatial_dist > 3.0:
                continue

            # Descriptor matching avec KNN (k=2)
            try:
                matches = self.matcher.knnMatch(desc_ref, desc_curr, k=2)
            except:
                continue

            if not matches:
                continue

            # Appliquer Lowe ratio test
            good_matches = self._lowe_ratio_test(matches)

            if len(good_matches) < self.min_matches:
                continue

            # Calculer le ratio de correspondances
            match_ratio = len(good_matches) / len(kp_ref)

            # Score de confiance
            # Plus il y a de matches, plus c'est fiable
            # Match ratio > 10% est bon
            if match_ratio > 0.10:
                certainty = min(1.0, match_ratio * 2.0)  # Normaliser
            else:
                certainty = match_ratio

            # Garder le meilleur match
            if match_ratio > best_ratio:
                best_ratio = match_ratio

                # Calculer l'erreur de pose (simple, sans estimation d'homography)
                pose_error = (
                    current_pose_x - ref_pose[0],
                    current_pose_y - ref_pose[1],
                    current_pose_theta - ref_pose[2]
                )

                best_match_result = {
                    'ref_timestamp': ref_ts,
                    'ref_pose': ref_pose,
                    'n_matches': len(good_matches),
                    'match_ratio': match_ratio,
                    'pose_error': pose_error,
                    'certainty': certainty,
                    'spatial_dist': spatial_dist
                }

        if best_match_result and best_match_result['certainty'] > 0.15:
            # Sauvegarder la boucle détectée
            self.loop_closures.append({
                'timestamp': time.time(),
                'query_pose': (current_pose_x, current_pose_y, current_pose_theta),
                'result': best_match_result
            })

            return best_match_result

        return None

    def get_loop_closure_constraints(self):
        """
        Retourner tous les constraints de boucles détectées.
        Utile pour l'optimisation globale (pose graph optimization).

        Returns:
            Liste de tuples (timestamp1, timestamp2, pose_delta)
        """
        constraints = []
        for lc in self.loop_closures:
            ref_ts = lc['result']['ref_timestamp']
            pose_error = lc['result']['pose_error']
            constraints.append((ref_ts, lc['timestamp'], pose_error))

        return constraints

    def get_statistics(self):
        """Retourner des statistiques sur les boucles détectées."""
        return {
            'n_reference_frames': len(self.reference_frames),
            'n_loop_closures': len(self.loop_closures),
            'total_frames_processed': self.frame_counter
        }
