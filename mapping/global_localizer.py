#!/usr/bin/env python3
"""
Global Localizer — SIFT-based Relocalization
==============================================

Permet au robot de se relocaliser sur la carte existante
même après une dérive importante ou une réinitialisation.

Utilise SIFT pour matcher l'image caméra contre la carte.
"""

import cv2
import numpy as np
from PIL import Image as PILImage
import os


class GlobalLocalizer:
    """
    Relocalisateur global basé sur SIFT.
    Charge une carte et fournit une estimation de pose basée sur la caméra.
    """

    def __init__(self, map_image_path, feature_threshold=0.7):
        """
        Initialiser le relocalisateur avec une carte.

        Args:
            map_image_path: Chemin vers l'image PGM de la carte
            feature_threshold: Seuil Lowe ratio pour SIFT
        """
        self.feature_threshold = feature_threshold
        self.sift = cv2.SIFT_create()
        self.matcher = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)

        # Charger la carte
        if not os.path.exists(map_image_path):
            raise FileNotFoundError(f"Map not found: {map_image_path}")

        # Lire l'image de la carte
        map_image = PILImage.open(map_image_path).convert('L')
        self.map_array = np.array(map_image)
        self.map_h, self.map_w = self.map_array.shape

        # Extraire les features de la carte (une seule fois)
        self.map_kp, self.map_desc = self.sift.detectAndCompute(
            self.map_array.astype(np.uint8), None
        )

        if self.map_desc is None or len(self.map_desc) == 0:
            raise ValueError("No SIFT features found in map image")

        print(f"[GlobalLocalizer] Map loaded: {self.map_h}x{self.map_w}, "
              f"{len(self.map_kp)} features extracted")

    def localize(self, camera_image, min_matches=20):
        """
        Estimer la pose du robot à partir d'une image caméra.

        Args:
            camera_image: Image BGR/RGB de la caméra
            min_matches: Nombre minimum de correspondances

        Returns:
            Dict {
                'success': bool,
                'pose_estimate': (x, y, theta) en pixels sur la carte,
                'confidence': score 0-1,
                'n_matches': nombre de correspondances,
                'H': matrice d'homography si succès
            } ou None
        """
        if camera_image is None or camera_image.size == 0:
            return None

        # Convertir en grayscale
        if len(camera_image.shape) == 3:
            gray = cv2.cvtColor(camera_image, cv2.COLOR_BGR2GRAY)
        else:
            gray = camera_image

        # Redimensionner pour que ce soit comparable (optionnel)
        # Garder la résolution de la caméra pour plus de détails
        h, w = gray.shape

        # Extraire features de l'image caméra
        cam_kp, cam_desc = self.sift.detectAndCompute(gray, None)

        if cam_desc is None or len(cam_desc) < min_matches:
            return {
                'success': False,
                'reason': 'Not enough features in camera image',
                'confidence': 0.0
            }

        # Matcher SIFT
        try:
            matches = self.matcher.knnMatch(self.map_desc, cam_desc, k=2)
        except:
            return {
                'success': False,
                'reason': 'Matching failed',
                'confidence': 0.0
            }

        if not matches or len(matches) < min_matches:
            return {
                'success': False,
                'reason': f'Not enough matches: {len(matches)} < {min_matches}',
                'confidence': 0.0
            }

        # Appliquer Lowe ratio test
        good_matches = []
        for match_pair in matches:
            if len(match_pair) == 2:
                m, n = match_pair
                if m.distance < self.feature_threshold * n.distance:
                    good_matches.append(m)

        if len(good_matches) < min_matches:
            return {
                'success': False,
                'reason': f'Not enough good matches after ratio test: {len(good_matches)}',
                'confidence': 0.0
            }

        # Extraire les points correspondants
        src_pts = np.float32([self.map_kp[m.queryIdx].pt for m in good_matches])
        dst_pts = np.float32([cam_kp[m.trainIdx].pt for m in good_matches])

        # Estimer l'homography
        # Cela donne la transformation de la caméra vers la carte
        try:
            H, mask = cv2.findHomography(dst_pts, src_pts, cv2.RANSAC, 5.0)
        except:
            return {
                'success': False,
                'reason': 'Homography estimation failed',
                'confidence': 0.0
            }

        if H is None:
            return {
                'success': False,
                'reason': 'No valid homography',
                'confidence': 0.0
            }

        # Compter les inliers
        inliers = np.sum(mask)
        inlier_ratio = inliers / len(good_matches)

        # Confiance basée sur le ratio d'inliers
        confidence = inlier_ratio

        if confidence < 0.3:
            return {
                'success': False,
                'reason': f'Low inlier ratio: {inlier_ratio:.2f}',
                'confidence': confidence
            }

        # Extraire la pose de l'homography
        # Pour un problème planar, on peut utiliser l'homography directement
        # Décomposer H pour extraire rotation et translation

        # Version simple : utiliser le centre de l'image transformé
        # Transformer le centre de l'image caméra
        center_cam = np.array([[w / 2], [h / 2], [1]])
        center_map = H @ center_cam

        # Normaliser les coordonnées homogènes
        x_map = center_map[0, 0] / center_map[2, 0]
        y_map = center_map[1, 0] / center_map[2, 0]

        # Extraire l'angle de rotation approximatif
        # En utilisant les deux premiers vecteurs de colonne de H
        h1 = H[:2, 0]
        h2 = H[:2, 1]

        # Angle = atan2(||h2 - h1||, ||h2 + h1||)
        scale = (np.linalg.norm(h1) + np.linalg.norm(h2)) / 2.0
        theta = np.arctan2(h2[1], h2[0]) - np.pi / 4

        pose_estimate = (x_map, y_map, theta)

        return {
            'success': True,
            'pose_estimate': pose_estimate,
            'confidence': confidence,
            'n_matches': len(good_matches),
            'n_inliers': inliers,
            'H': H
        }

    def refine_pose(self, initial_pose, camera_image, iterations=3):
        """
        Affiner l'estimation de pose par itérations.

        Args:
            initial_pose: Pose initiale (x, y, theta)
            camera_image: Image caméra
            iterations: Nombre d'itérations de raffinement

        Returns:
            Pose affinée ou None
        """
        current_pose = initial_pose

        for i in range(iterations):
            # Pour chaque itération, on pourrait faire un matching
            # plus fin autour de la région attendue
            # Pour l'instant, on utilise juste le résultat du first localize
            pass

        return current_pose
