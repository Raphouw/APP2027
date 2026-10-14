"""Réglages communs au navigateur sur carte (valeurs historiques)."""

LIDAR_YAW_OFFSET = 0.0

# L'odométrie maison est isolée de celle des pilotes du robot.
ODOM_TOPIC = '/my_odom'
ODOM_FRAME = 'my_odom'
BASE_FRAME = 'base_link'
SENSOR_TIMEOUT = 0.5  # secondes, horloge monotone de réception
STOP_DISTANCE = 0.30  # mètres, inclut la marge devant le robot

# Navigation sur carte : planification et poursuite de chemin.
ROBOT_CLEARANCE = 0.14        # m, marge dure autour des murs (rayon du Burger ≈ 0,105 m)
SOFT_MARGIN = 0.30            # m, au-delà de la marge dure, le coût décroît jusqu'à 0
SOFT_WEIGHT = 1.5             # surcoût maximal d'une cellule collée à la marge dure
MAX_LINEAR = 0.18             # m/s
MAX_ANGULAR = 1.0             # rad/s
LINEAR_ACCEL = 0.45           # m/s², montée de vitesse
LINEAR_DECEL = 1.2            # m/s², freinage
ANGULAR_ACCEL = 4.0           # rad/s²
LOOKAHEAD = 0.30              # m, point visé sur le chemin
HORIZON = 1.5                 # m, portion de chemin surveillée pour détecter un blocage
REPLAN_MIN_PERIOD = 0.4       # s entre deux calculs de chemin
OBSTACLE_TTL = 1.5            # s, un obstacle nouveau bloque la planification
OBSTACLE_DISPLAY_TTL = 8.0    # s, il reste affiché en couleur
KNOWN_TOLERANCE = 0.12        # m, un écho à moins de cela d'un mur de la carte (après recalage) n'est pas « nouveau »
GOAL_SNAP_RADIUS = 0.60       # m, une destination dans une zone interdite est ramenée au libre le plus proche
START_SNAP_RADIUS = 0.40      # m, idem pour le robot collé à un mur
