"""Réglages communs au navigateur sur carte (valeurs historiques)."""

LIDAR_YAW_OFFSET = 0.0

# L'odométrie maison est isolée de celle des pilotes du robot.
ODOM_TOPIC = '/my_odom'
ODOM_FRAME = 'my_odom'
BASE_FRAME = 'base_link'
SENSOR_TIMEOUT = 0.5  # secondes, horloge monotone de réception
STOP_DISTANCE = 0.30  # mètres, inclut la marge devant le robot
ROBOT_CLEARANCE = 0.20  # mètres pour l'inflation de la carte
