"""Acquisition caméra et reconnaissance de revisites.

Les attributs sont initialisés par le point d’entrée ; ce mixin ne lance aucun processus.
"""

import time
from sensor_msgs.msg import Image

class LoopVisionMixin:

    def _cb_camera(self, msg: Image):
        """
        PHASE 5 : Callback caméra pour détection de boucles.
        """
        try:
            from cv_bridge import CvBridge
            bridge = CvBridge()
            image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.last_camera_frame = image

            # Ajouter la frame actuelle à la base de données
            self.loop_closure.add_reference_frame(
                timestamp=time.time(),
                image=image,
                pose_x=self.x,
                pose_y=self.y,
                pose_theta=self.theta
            )

            # Détecter une boucle
            lc_result = self.loop_closure.detect_loop(
                image, self.x, self.y, self.theta
            )

            if lc_result:
                self.get_logger().info(
                    f"[PHASE5] LOOP CLOSURE DETECTED! "
                    f"certainty={lc_result['certainty']:.2f}, "
                    f"matches={lc_result['n_matches']}"
                )
                # Ici on pourrait déclencher une réoptimisation globale
                # ou corriger la pose
        except ImportError:
            pass  # cv_bridge pas disponible
        except Exception as e:
            self.get_logger().warn(f"[PHASE5] Camera callback error: {e}")
