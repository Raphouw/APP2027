#!/usr/bin/env python3
from slam_components.slam_engine import PoseBuffer
"""TurtleBot3 : odométrie mesurée et navigation réactive.

Les encodeurs et le gyroscope alimentent l'intégration de pose ; les commandes
ne servent pas de mesure de mouvement. La pose est publiée sur /my_odom dans
le repère my_odom, avec TF vers base_link. Les capteurs périmés bloquent la
commande et la publication d'une pose qui semblerait fraîche.

Le paramètre odometry_only permet de fournir la pose sans posséder /cmd_vel.
Les sessions CSV sont écrites progressivement dans log_directory.
"""

from slam_components.tb3_odometry import OdometryMixin
from slam_components.tb3_perception import LidarPerceptionMixin
from slam_components.tb3_recovery import RecoveryMixin
from slam_components.tb3_control import ReactiveControlMixin
from slam_components.tb3_vision import LoopVisionMixin
from slam_components.tb3_logging import SessionLoggingMixin, SessionWriter, save_pose
from slam_components.settings import ODOM_TOPIC, ODOM_FRAME, SENSOR_TIMEOUT
from slam_components.command_ownership import CommandLease
from slam_components.pid import PIDController


import rclpy
from rclpy.node import Node

import collections
import os, signal, atexit
from datetime import datetime
from sensor_msgs.msg import JointState, Imu, LaserScan, Image
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, Float32, String
from tf2_ros import TransformBroadcaster
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from loop_closure import LoopClosureDetector  # PHASE 5 : Détection de boucles


# =============================================================================
# Classe PID (B3) — Proportionnel + Intégral + Dérivé avec anti-windup
# =============================================================================


# =============================================================================
# Node principal
# =============================================================================
class TurtleBot3NavNode(OdometryMixin, LidarPerceptionMixin, RecoveryMixin, ReactiveControlMixin, LoopVisionMixin, SessionLoggingMixin, Node):
    def __init__(self):
        super().__init__('tb3_nav_node')
        self.get_logger().info("=== Démarrage TB3 Nav Node ===")

        # NOUVEAU : Déclaration du paramètre ROS 2 pour choisir le mode
        self.declare_parameter('log_directory', './logs')
        self.log_directory = os.path.abspath(self.get_parameter('log_directory').value)
        os.makedirs(self.log_directory, exist_ok=True)
        self.declare_parameter('odometry_only', False)
        self.odometry_only = self.get_parameter('odometry_only').value
        self.command_lease = None
        self.declare_parameter('use_saved_pose', False)
        use_saved_pose = self.get_parameter('use_saved_pose').value

        # -----------------------------------------------------------------
        # Physique TurtleBot3 Burger
        # -----------------------------------------------------------------
        self.wheel_radius = 0.033      # m
        self.wheel_base   = 0.160      # m
        self.robot_radius = 0.105      # m

        # --- Initialisation de la pose selon le mode choisi ---
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.pose_file_path = os.path.join(self.log_directory, "last_pose.txt")

        if use_saved_pose:
            if os.path.exists(self.pose_file_path):
                try:
                    with open(self.pose_file_path, 'r') as f:
                        vals = f.read().strip().split(',')
                        self.x = float(vals[0])
                        self.y = float(vals[1])
                        self.theta = float(vals[2])
                    self.get_logger().info(f"[INIT] Mode Navigation : Pose restaurée (X={self.x:.2f}, Y={self.y:.2f})")
                except Exception as e:
                    self.get_logger().warn(f"[INIT] Erreur de lecture de la pose, départ à zéro : {e}")
            else:
                self.get_logger().warn("[INIT] Fichier de pose introuvable, départ à zéro.")
        else:
            self.get_logger().info("[INIT] Mode Mapping : Départ à zéro (X=0.0, Y=0.0, Theta=0.0)")

        # -----------------------------------------------------------------
        # A2 – Odométrie encodeurs
        # -----------------------------------------------------------------
        # IMPORTANT : Vérifie bien qu'il n'y a plus aucune initialisation 
        # "self.x = 0.0" dans cette section !
        self.sensor_timeout = SENSOR_TIMEOUT
        self.log_pose_buffer = PoseBuffer()
        self.last_logged_scan = None
        self.last_joint_time = None
        self.last_joint_received = None
        self.last_imu_received = None
        self.last_scan_received = None
        self.last_control_monotonic = None
        self.last_wl = None   
        self.last_wr = None
        
        self.enc_v    = 0.0   
        self.enc_dtheta = 0.0

        # -----------------------------------------------------------------
        # A3 – IMU + calibration biais
        # -----------------------------------------------------------------
        self.imu_ok        = False
        self.calib_samples = []
        self.gyro_bias_z   = 0.0
        self.CALIB_N       = 200       # échantillons pour la calibration
        self.GYRO_DEADBAND = 0.012     # rad/s : bruit résiduel ignoré

        # vitesse angulaire corrigée (filtre complémentaire)
        self.gyro_w = 0.0

        # Filtre complémentaire encodeurs/gyro
        # alpha_comp proche de 1 → confiance gyro (bonne pour cap long terme)
        self.alpha_comp = 0.98

        # -----------------------------------------------------------------
        # A4 – Intégration RK4
        # -----------------------------------------------------------------
        # RK4 nécessite l'état courant + les commandes (v, w)
        # On stocke la vitesse angulaire fusionnée
        self.fused_w = 0.0

        # -----------------------------------------------------------------
        # B1 – FSM
        # -----------------------------------------------------------------
        # États : IDLE | MOVE_TO_GOAL | FOLLOW_GAP | GOAL_REACHED
        self.state          = 'IDLE'
        self.goal_x         = None
        self.goal_y         = None

        # Hysteresis FSM par vote majoritaire
        # On maintient une fenêtre glissante de décisions "obstacle_present"
        self._obs_window_size = 8    # nombre de cycles évalués
        self._obs_votes = collections.deque(
            [False] * self._obs_window_size,
            maxlen=self._obs_window_size
        )
        self._OBS_TRIGGER_RATIO  = 0.65  # >65% de votes True → bascule FOLLOW
        self._CLR_TRIGGER_RATIO  = 0.25  # <25% de votes True → retour MOVE

        # Garde-fous temporels
        self.MIN_FOLLOW_TIME = 1.2     # s min en FOLLOW_GAP avant de sortir
        self.follow_enter_time = 0.0

        # -----------------------------------------------------------------
        # B3 – PIDs
        # -----------------------------------------------------------------
        # PID angulaire NAVIGATION (cap vers goal, en MOVE_TO_GOAL)
        # ki faible : utile pour corriger dérive longue distance
        self.pid_ang = PIDController(
            kp=1.4, ki=0.03, kd=0.12,
            out_min=-1.0, out_max=1.0,
            integrator_max=0.3,
            derivative_alpha=0.10
        )
        # PID angulaire ÉVITEMENT (cap vers gap, en FOLLOW_GAP)
        # ki=0 obligatoire : le terme I accumule l'erreur pendant les virages
        # et provoque les dépassements/oscillations observés dans les logs
        # ⚡ PID tuné pour meilleure réactivité d'évitement
        self.pid_ang_avoid = PIDController(
            kp=0.8, ki=0.1, kd=0.12,  # Meilleure stabilité + correction d'erreur
            out_min=-1.0, out_max=1.0,
            integrator_max=0.3,  # Permet accumulation légère
            derivative_alpha=0.15  # Moins de lissage = plus réactif
        )
        # PID linéaire (conservé mais non utilisé en vitesse directe)
        self.pid_lin = PIDController(
            kp=0.55, ki=0.0, kd=0.05,
            out_min=0.0, out_max=1.0,
            integrator_max=0.3,
            derivative_alpha=0.15
        )

        # -----------------------------------------------------------------
        # B4 – Slew rate (rampe d'accélération) + limites
        # -----------------------------------------------------------------
        self.max_v    = 0.16   # m/s max linéaire
        self.max_w    = 1.0     # rad/s max angulaire

        # Rampe asymétrique : décélération 3× plus rapide que l'accélération
        # → le robot ralentit vite en fin de course, mais accélère doucement
        self.max_dv_accel = 0.18   # m/s² montée
        self.max_dv_decel = 0.55   # m/s² descente (décélération rapide)
        self.max_dw       = 2.5    # rad/s²

        self.prev_v   = 0.0
        self.prev_w   = 0.0

        # -----------------------------------------------------------------
        # C1/C2 – LiDAR + inflation
        # -----------------------------------------------------------------
        self.raw_scan_ranges = []
        self.laser_ranges = []
        self.laser_amin   = 0.0
        self.laser_ainc   = 0.0
        self.inflated_ranges = []    # ranges après inflation (C2)

        # ⚡ PARAMÈTRES OPTIMISÉS POUR NAVIGATION ROBUSTE
        self.AVOID_START_DIST = 0.5   # m : détection obstacle plus proche (était 0.9)
        self.AVOID_STOP_DIST  = 0.2   # m : arrêt total (sécurité)
        self.GOAL_FOV         = 0.20  # rad : cône "goal line" (~22°)
        self.FRONT_FOV        = 0.25  # rad : détection précoce (était 0.15, now 14°)

        # -----------------------------------------------------------------
        # C3 – Gap stabilisé (EWMA)
        # -----------------------------------------------------------------
        # On lisse l'angle du gap via une moyenne exponentielle pondérée
        # pour éviter les saccades dues aux changements brusques d'indice
        self.gap_heading         = 0.0
        self.GAP_EWMA_ALPHA      = 0.20   # réactivité 2× plus rapide (était 0.10)
        self.gap_update_period   = 0.30   # s
        self.last_gap_update     = 0.0
        self.MIN_GAP_WIDTH       = 35     # indices min pour gap sûr (était 20, now 35°)

        # Filtre passe-bas sur desired_heading
        self.desired_heading_filt = 0.0
        self.DH_FILTER_ALPHA      = 0.30  # plus réactif (était 0.12)

        # -----------------------------------------------------------------
        # C3 – Verrou de côté (hard side lock)
        # Remplace le soft SIDE_SWITCH_PEN : une fois un côté choisi, on
        # l'interdit pendant LOCK_DURATION secondes sauf si le côté verrouillé
        # est complètement bouché (pas de gap viable).
        # -----------------------------------------------------------------
        self.last_gap_sign       = 0.0   # +1 droite, -1 gauche, 0 inconnu
        self.locked_side         = 0.0   # 0=aucun, +1=droite, -1=gauche
        self.lock_start_time     = 0.0
        self.LOCK_DURATION       = 2.5   # s : durée minimale du verrou
        self.LOCK_NO_GAP_CYCLES  = 6     # cycles sans gap du côté verrouillé → libère
        self._lock_no_gap_count  = 0

        # -----------------------------------------------------------------
        # D1 – Détection anti-boucle avec vecteur de répulsion
        # Si le robot tourne en rond, on calcule un vecteur pointant LOIN
        # du centroïde des positions récentes et on le mélange dans le heading.
        # -----------------------------------------------------------------
        self.LOOP_SAMPLE_PERIOD    = 0.10   # s entre deux échantillons de position
        self.LOOP_HISTORY_N        = 50     # 50 × 0.10s = 5s de fenêtre
        self.LOOP_SPATIAL_THRESH   = 0.30   # m : rayon du cercle "coinçé"
        self.LOOP_TRIGGER_RATIO    = 0.80   # fraction de la fenêtre dans le cercle
        self.LOOP_REPULSION_WEIGHT = 0.45   # poids max du vecteur de répulsion
        self.LOOP_COOLDOWN         = 4.0    # s : délai min entre deux déclenchements

        self._loop_pos_history     = collections.deque(maxlen=self.LOOP_HISTORY_N)
        self._last_loop_sample_t   = 0.0
        self._loop_detected        = False
        self._loop_confidence      = 0.0
        self._loop_repulsion_angle = 0.0   # angle monde pointant loin du centroïde
        self._loop_last_trigger_t  = 0.0

	# -----------------------------------------------------------------
        # Path RViz — historique de la trajectoire publiée sur /path
        # -----------------------------------------------------------------
        self.path_msg        = Path()
        self.path_msg.header.frame_id = ODOM_FRAME
        self.PATH_MIN_DIST   = 0.05    # m : distance min entre deux points du path
        self._last_path_x    = None
        self._last_path_y    = None

        # -----------------------------------------------------------------
        # PHASE 5 – Loop Closure Detection (optionnel si caméra disponible)
        # -----------------------------------------------------------------
        self.loop_closure = LoopClosureDetector(
            distance_threshold=0.5,
            feature_threshold=0.7,
            min_matches=15
        )
        self.last_camera_frame = None


        # -----------------------------------------------------------------
        # Boucle & logs
        # -----------------------------------------------------------------
        self.dt           = 0.02   # 50 Hz
        self.last_log_t   = 0.0
        self.LOG_PERIOD   = 2.0    # s
	
	
        # -----------------------------------------------------------------
        # Data logger (cartographie post-run en CSV)
        # -----------------------------------------------------------------
        run_id = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        
        log_dir = self.log_directory
        os.makedirs(log_dir, exist_ok=True)

        # On passe sur un fichier .csv
        self.declare_parameter("log_filename", f"run_{run_id}.csv")
        self._log_csv_path = os.path.join(log_dir, self.get_parameter("log_filename").value)
        self.session_writer = SessionWriter(self._log_csv_path)
        self._log_period_s  = 0.10
        self._last_log_data = 0.0

        self.get_logger().info(f"[LOG] Données seront sauvegardées dans : {self._log_csv_path}")

		
        #-----------------------------------------------------------------
        # Publishers, Subscribers, TF
        # -----------------------------------------------------------------
        self.odom_pub         = self.create_publisher(Odometry, ODOM_TOPIC, 10)
        self.cmd_pub = None
        if not self.odometry_only:
            self.command_lease = CommandLease()
            self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub         = self.create_publisher(Path, '/path', 10)
        # Nouveaux topics de diagnostic / visualisation
        self.loop_state_pub      = self.create_publisher(Bool,    '/loop_state',           10)
        self.loop_repulsion_pub  = self.create_publisher(Float32, '/loop_repulsion_angle',  10)
        self.nav_state_pub       = self.create_publisher(String,  '/nav_state',            10)
        self.selected_gap_pub    = self.create_publisher(Float32, '/selected_gap_angle',   10)
        self.tf_br               = TransformBroadcaster(self)

        qos_lidar = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self.create_subscription(
            JointState, '/joint_states', self._cb_joint, qos_profile_sensor_data
        )
        self.create_subscription(
            Imu, '/imu', self._cb_imu, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, '/scan', self._cb_scan, qos_lidar
        )
        if not self.odometry_only:
            self.create_subscription(PoseStamped, '/goal_pose', self._cb_goal, 10)

        # PHASE 5 – Optionnel : Subscribe à la caméra pour loop closure
        try:
            self.create_subscription(Image, '/camera/image_raw', self._cb_camera, 10)
            self.get_logger().info("[PHASE5] Camera subscription active (optionnel)")
        except Exception as e:
            self.get_logger().info(f"[PHASE5] Camera not available: {e}")

        self.create_timer(self.dt, self._control_loop)
        self.get_logger().info("[INIT] Calibration IMU en cours (ROBOT IMMOBILE)")


_node = None


def cleanup_handler():
    """Arrêt idempotent : arrêter, enregistrer, libérer le propriétaire de commande."""
    global _node
    node, _node = _node, None
    if node is None:
        return
    try:
        if node.cmd_pub is not None:
            node.cmd_pub.publish(Twist())
    finally:
        try:
            node.session_writer.close()
            save_pose(node.pose_file_path, (node.x, node.y, node.theta))
        finally:
            try:
                node.destroy_node()
            finally:
                if node.command_lease is not None:
                    node.command_lease.close()
                if rclpy.ok():
                    rclpy.shutdown()


def main(args=None):
    from slam_components.run_logging import install_run_logging
    install_run_logging('tb3_nav_node')
    global _node
    rclpy.init(args=args)
    try:
        _node = TurtleBot3NavNode()
        signal.signal(signal.SIGINT, lambda s, f: cleanup_handler())
        signal.signal(signal.SIGTERM, lambda s, f: cleanup_handler())
        atexit.register(cleanup_handler)
        rclpy.spin(_node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        cleanup_handler()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
