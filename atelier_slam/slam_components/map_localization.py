"""Pose dans le repère de carte, observations dynamiques et recalage SE(2)."""
import math
import time
import numpy as np
from .settings import LIDAR_YAW_OFFSET, ODOM_FRAME, OBSTACLE_TTL
from .lidar_filter import front_mask, ScanHistory, tolerate_front_gaps


class MapLocalizationMixin:
    def world_to_grid(self, x, y):
        if not math.isfinite(x) or not math.isfinite(y):
            return None
        c = math.floor((x-self.origin[0])/self.res)
        r = math.floor((y-self.origin[1])/self.res)
        return (r, c) if 0 <= r < self.h and 0 <= c < self.w else None

    def grid_to_world(self, r, c):
        return self.origin[0]+(c+0.5)*self.res, self.origin[1]+(r+0.5)*self.res

    def _apply_odom(self):
        x, y, theta = self.raw_odom
        ox, oy, angle = self.odom_offset
        self.robot_pos = [ox+x*math.cos(angle)-y*math.sin(angle),
                          oy+x*math.sin(angle)+y*math.cos(angle),
                          math.atan2(math.sin(theta+angle), math.cos(theta+angle))]
        self.shared_data['robot_pos'] = self.robot_pos

    def set_pose(self, x, y, theta):
        raw_x, raw_y, raw_theta = self.raw_odom
        angle = theta-raw_theta
        self.odom_offset = [x-raw_x*math.cos(angle)+raw_y*math.sin(angle),
                            y-raw_x*math.sin(angle)-raw_y*math.cos(angle), angle]
        self._apply_odom()
        self.lidar_history = ScanHistory()
        self.last_scan_received = None
        self.path = []
        self.dynamic_obstacles.clear()
        if getattr(self, 'seen_obstacles', None) is not None:
            self.seen_obstacles.clear()
        self.state = 'READY'
        self.shared_data.update(path=[], state=self.state, goal=None)

    def odom_cb(self, msg):
        if msg.header.frame_id != ODOM_FRAME:
            self.last_odom_received = None
            return
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2*(o.w*o.z+o.x*o.y), 1-2*(o.y*o.y+o.z*o.z))
        if not all(math.isfinite(v) for v in [p.x, p.y, yaw]):
            self.last_odom_received = None
            return
        self.raw_odom = [p.x, p.y, yaw]
        self._apply_odom()
        self.last_odom_received = time.monotonic()
        self.shared_data['pose_received'] = True

    def scan_cb(self, msg):
        now = time.monotonic()
        ranges = np.asarray(msg.ranges, dtype=float)
        if len(ranges) == 0 or msg.angle_increment == 0:
            self.last_scan_received = None
            return
        angles = msg.angle_min+np.arange(len(ranges))*msg.angle_increment+LIDAR_YAW_OFFSET
        angles = np.arctan2(np.sin(angles), np.cos(angles))
        finite = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges <= msg.range_max)
        if not hasattr(self, 'lidar_history'):
            self.lidar_history = ScanHistory()
        distances, known, inferred, temporal = self.lidar_history.filter(
            ranges, msg.angle_min+LIDAR_YAW_OFFSET, msg.angle_increment,
            msg.range_min, msg.range_max, self.robot_pos, now)
        front = front_mask(len(ranges), msg.angle_min+LIDAR_YAW_OFFSET, msg.angle_increment)
        distances, known, _ = tolerate_front_gaps(distances, known, front)
        front_valid = bool(front.any() and known[front].all())
        self.front_distance = float(distances[front].min()) if front_valid else 0.0
        self.last_scan_received = now if front_valid else None
        self.shared_data.update(lidar_front_known=front_valid,
                                lidar_front_unknown=int((~known[front]).sum()),
                                lidar_inferred=int(inferred.sum()), lidar_temporal=int(temporal.sum()))
        if self.last_odom_received is None or now-self.last_odom_received > self.sensor_timeout:
            return
        rx, ry, yaw = self.robot_pos
        self.dynamic_obstacles = {cell: stamp for cell, stamp in self.dynamic_obstacles.items()
                                  if now-stamp < OBSTACLE_TTL}
        hits_r, hits_c = ranges[finite], angles[finite]
        xs, ys = rx+hits_r*np.cos(yaw+hits_c), ry+hits_r*np.sin(yaw+hits_c)
        points = list(zip(xs.tolist(), ys.tolist()))
        sx, sy = self._align_to_map(xs, ys)
        cols = np.floor((sx-self.origin[0])/self.res).astype(int)
        rows = np.floor((sy-self.origin[1])/self.res).astype(int)
        inside = (rows >= 0) & (rows < self.h) & (cols >= 0) & (cols < self.w)
        rows, cols = rows[inside], cols[inside]
        known_zone = getattr(self, 'known_zone', None)
        if known_zone is not None:
            # Un écho sur (ou près d') un mur de la carte, ou en zone inconnue, n'est pas « nouveau ».
            novel = ~known_zone[rows, cols]
            rows, cols = rows[novel], cols[novel]
        cells = set(zip(rows.tolist(), cols.tolist()))
        # Confirmation : l'écho doit se répéter au scan suivant (à une cellule près) pour compter.
        previous = getattr(self, '_previous_hits', set())
        self._previous_hits = cells
        near_previous = {(r+dr, c+dc) for r, c in previous for dr in (-1, 0, 1) for dc in (-1, 0, 1)}
        confirmed = cells if known_zone is None else cells & near_previous
        seen = getattr(self, 'seen_obstacles', None)
        for cell in confirmed:
            self.dynamic_obstacles[cell] = now
            if seen is not None:
                seen[cell] = now
        self.shared_data['lidar_pts'] = points[::8]

    def _align_to_map(self, xs, ys):
        """Décalage (x, y) qui plaque au mieux les échos sur les murs de la carte.

        L'odométrie dérive : sans correction, des murs connus apparaissent « nouveaux ».
        Recherche exhaustive d'une translation (±0,4 m), score = distance moyenne tronquée
        aux murs, avec un léger rappel vers le décalage précédent. Les vrais obstacles
        nouveaux sont des valeurs aberrantes tronquées : ils ne tirent pas le résultat.
        """
        wall_distance = getattr(self, 'wall_distance', None)
        if wall_distance is None or len(xs) < 20:
            return xs, ys
        steps = max(1, int(math.ceil(0.4/self.res)))
        offsets = np.arange(-steps, steps+1)*self.res
        dx, dy = np.meshgrid(offsets, offsets)
        dx, dy = dx.ravel()[:, None], dy.ravel()[:, None]
        cols = np.floor((xs[None, :]+dx-self.origin[0])/self.res).astype(int)
        rows = np.floor((ys[None, :]+dy-self.origin[1])/self.res).astype(int)
        inside = (rows >= 0) & (rows < self.h) & (cols >= 0) & (cols < self.w)
        cap = 0.3
        values = np.full(rows.shape, cap)
        values[inside] = np.minimum(wall_distance[rows[inside], cols[inside]], cap)
        previous = getattr(self, 'map_shift', (0.0, 0.0))
        score = values.mean(axis=1)+0.05*np.hypot(dx[:, 0]-previous[0], dy[:, 0]-previous[1])
        best = int(np.argmin(score))
        zero = int(np.argmin(np.hypot(dx[:, 0], dy[:, 0])))
        if score[best] > score[zero]-0.005:
            best = zero
        self.map_shift = (float(dx[best, 0]), float(dy[best, 0]))
        return xs+dx[best, 0], ys+dy[best, 0]

    def camera_cb(self, msg):
        if not self.enable_visual_relocalization:
            return
        try:
            from cv_bridge import CvBridge
            self.last_camera_frame = CvBridge().imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'Caméra : {exc}')

    def relocalize(self):
        # Fonction expérimentale, désactivée par défaut : caméra et PGM ne partagent pas une géométrie validée.
        if not self.enable_visual_relocalization or self.relocalisation_visuelle is None or self.last_camera_frame is None:
            return False, None
        result = self.relocalisation_visuelle.localize(self.last_camera_frame, min_matches=15)
        if result and result['success'] and result['confidence'] > 0.4:
            x, y, theta = result['pose_estimate']
            pose = (self.origin[0]+(x+0.5)*self.res,
                    self.origin[1]+(self.h-y-0.5)*self.res, theta)
            if self.world_to_grid(*pose[:2]) is not None:
                return True, pose
        return False, None

    def process_pose_command(self):
        cmd = self.shared_data.get('set_pose_cmd')
        if cmd and self.last_odom_received is not None:
            x, y, yaw = cmd
            if self.world_to_grid(x, y) is not None:
                self.set_pose(x, y, self.robot_pos[2] if yaw is None else yaw)
            self.shared_data['set_pose_cmd'] = None
