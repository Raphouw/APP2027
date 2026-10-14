"""Suivi sur carte : planification hors boucle de contrôle, poursuite de chemin lissée.

Principes (voir aussi `planificateur_chemin.py`) :
- le calcul du chemin tourne dans un fil de travail ; la boucle de contrôle continue
  de suivre l'ancien chemin et ne publie jamais de commande nulle pendant le calcul ;
- un nouveau calcul n'est lancé que si le chemin restant est réellement bloqué
  (ou sur nouvelle destination), pas à chaque variation du LiDAR ;
- les consignes de vitesse passent par des rampes : plus de « avance / arrête ».
"""
import math
import time
from concurrent.futures import ThreadPoolExecutor
import numpy as np
from scipy.ndimage import binary_dilation
from geometry_msgs.msg import Twist
from planificateur_chemin import nearest_free, plan_path, smooth_path
from .settings import (STOP_DISTANCE, MAX_LINEAR, MAX_ANGULAR, LINEAR_ACCEL, LINEAR_DECEL, ANGULAR_ACCEL,
                       LOOKAHEAD, OBSTACLE_TTL, OBSTACLE_DISPLAY_TTL, GOAL_SNAP_RADIUS, START_SNAP_RADIUS,
                       REPLAN_MIN_PERIOD, HORIZON)


def _disc(radius):
    yy, xx = np.ogrid[-radius:radius+1, -radius:radius+1]
    return xx*xx+yy*yy <= radius*radius


def compute_route(grid, cost, start, goal_cell, goal_snap, start_snap):
    """Calcul pur (sans état partagé) : retourne (cellules lissées, but utilisé) ou (None, None)."""
    start = nearest_free(grid, start, start_snap) if start is not None else None
    goal = nearest_free(grid, goal_cell, goal_snap) if goal_cell is not None else None
    if start is None or goal is None:
        return None, None
    route = plan_path(grid, start, goal, cost)
    if route is None:
        return None, None
    return smooth_path(grid, route, cost), goal


class MapControlMixin:
    # -- planification -------------------------------------------------------------------------
    def _snapshot_request(self):
        start = self.world_to_grid(*self.robot_pos[:2])
        goal = self.shared_data.get('goal')
        goal_cell = self.world_to_grid(*goal) if goal else None
        cost = getattr(self, 'soft_cost', None)
        return start, goal, goal_cell, np.array(self.planner.grid, copy=True), cost

    def _route_to_path(self, cells, used_goal_cell, goal, goal_cell):
        """Cellules → points monde, tronqués devant le robot ; le but exact est conservé s'il est atteint."""
        points = [self.grid_to_world(*cell) for cell in cells]
        if used_goal_cell == goal_cell:
            points[-1] = tuple(goal)
        return self._trim_behind(points)

    def _trim_behind(self, poly):
        """Retire les sommets déjà dépassés : le robot continue d'avancer pendant le calcul."""
        if len(poly) < 2:
            return list(poly)
        rx, ry = self.robot_pos[:2]
        best, best_dist = 0, math.inf
        for i in range(len(poly)-1):
            d = _segment_distance(rx, ry, poly[i], poly[i+1])
            if d < best_dist-1e-9:
                best, best_dist = i, d
        return list(poly[best+1:])

    def _apply_route(self, cells, used_goal_cell, goal, goal_cell, new_goal):
        if cells is None:
            self.path = []
            self.state = 'NO_PATH'
            self._v_cmd = self._w_cmd = 0.0
        else:
            self.path = self._route_to_path(cells, used_goal_cell, goal, goal_cell)
            self.state = 'NAV' if self.path else 'READY'
            if not self.path:
                self.shared_data['goal'] = None
        if new_goal:
            self.pid_heading.reset()
        self.shared_data.update(path=list(self.path), state=self.state)

    def _plan_to_goal(self, new_goal=True):
        """Version synchrone (nouvelle destination, tests). N'arrête pas le robot."""
        start, goal, goal_cell, grid, cost = self._snapshot_request()
        if start is None or goal_cell is None:
            cells, used = None, None
        else:
            cells, used = compute_route(grid, cost, start, goal_cell, self._cells(GOAL_SNAP_RADIUS),
                                        self._cells(START_SNAP_RADIUS))
        self._apply_route(cells, used, goal, goal_cell, new_goal)
        self.last_plan_time = time.monotonic()

    def _cells(self, meters):
        return max(1, int(math.ceil(meters/self.res)))

    def _request_plan(self, new_goal=False):
        """Lance le calcul dans un fil ; un seul calcul à la fois, le plus récent l'emporte."""
        if getattr(self, '_plan_future', None) is not None:
            self._plan_wanted = True
            return
        start, goal, goal_cell, grid, cost = self._snapshot_request()
        if start is None or goal_cell is None:
            self._apply_route(None, None, goal, goal_cell, new_goal)
            return
        if getattr(self, '_plan_executor', None) is None:
            self._plan_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='planner')
        self._plan_meta = (goal, goal_cell, new_goal)
        self._plan_future = self._plan_executor.submit(
            compute_route, grid, cost, start, goal_cell, self._cells(GOAL_SNAP_RADIUS), self._cells(START_SNAP_RADIUS))
        self._plan_wanted = False
        self.last_plan_time = time.monotonic()

    def _poll_plan(self):
        future = getattr(self, '_plan_future', None)
        if future is None or not future.done():
            return
        self._plan_future = None
        goal, goal_cell, new_goal = self._plan_meta
        current = self.shared_data.get('goal')
        if current is None or tuple(current) != tuple(goal):
            return   # destination annulée ou changée pendant le calcul : résultat périmé
        try:
            cells, used = future.result()
        except Exception as exc:  # un échec de calcul ne doit pas arrêter le nœud
            self.get_logger().warn(f'Planification échouée : {exc}')
            cells, used = None, None
        self._apply_route(cells, used, goal, goal_cell, new_goal)

    def _cancel_plan(self):
        self._plan_future = None
        self._plan_wanted = False

    def _path_blocked(self):
        """Le chemin restant (sur HORIZON mètres) traverse-t-il une cellule occupée ?"""
        if not self.path:
            return False
        grid = self.planner.grid
        pts = [tuple(self.robot_pos[:2])]+list(self.path)
        left = HORIZON
        for a, b in zip(pts, pts[1:]):
            length = math.hypot(b[0]-a[0], b[1]-a[1])
            n = max(1, int(length/(self.res*0.5)))
            for k in range(n+1):
                t = k/n
                cell = self.world_to_grid(a[0]+(b[0]-a[0])*t, a[1]+(b[1]-a[1])*t)
                if cell is None or grid[cell] >= 2:
                    return True
            left -= length
            if left <= 0:
                break
        return False

    # -- obstacles nouveaux --------------------------------------------------------------------
    def _refresh_obstacles(self, now):
        """Met à jour la grille du planificateur avec les obstacles absents de la carte (TTL court)."""
        self.dynamic_obstacles = {cell: stamp for cell, stamp in self.dynamic_obstacles.items()
                                  if now-stamp < OBSTACLE_TTL}
        keys = frozenset(self.dynamic_obstacles)
        if keys == getattr(self, '_dyn_keys', frozenset()):
            return False
        self._dyn_keys = keys
        mask = np.zeros_like(self.static_grid, dtype=bool)
        for cell in keys:
            mask[cell] = True
        radius = max(1, int(math.ceil(self.clearance/self.res)))
        inflated = binary_dilation(mask, structure=_disc(radius))
        self.planner.grid = np.where(inflated, 2, self.static_grid)
        return True

    # -- boucle de contrôle ----------------------------------------------------------------------
    def _stop_motion(self):
        self._v_cmd = self._w_cmd = 0.0
        self.cmd_pub.publish(Twist())

    def _lookahead(self, distance):
        """Point du chemin à `distance` devant le robot et longueur restante jusqu'au but."""
        pts = [tuple(self.robot_pos[:2])]+list(self.path)
        total = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(pts, pts[1:]))
        left = distance
        for a, b in zip(pts, pts[1:]):
            length = math.hypot(b[0]-a[0], b[1]-a[1])
            if length >= left and length > 1e-9:
                t = left/length
                return (a[0]+(b[0]-a[0])*t, a[1]+(b[1]-a[1])*t), total
            left -= length
        return pts[-1], total

    def control_loop(self):
        now = time.monotonic()
        dt = max(0.001, min(0.1, now-self.last_control_time))
        self.last_control_time = now
        if self.shared_data.get('quit'):
            self._cancel_plan()
            self._stop_motion()
            return
        if self.shared_data.get('state') == 'STOPPED':
            self._cancel_plan()
            self.path = []
            self.state = 'READY'
            self.pid_heading.reset()
            self.shared_data.update(state=self.state, path=[], goal=None, click=None)
            self._stop_motion()
            return
        fresh = (self.last_odom_received is not None and self.last_scan_received is not None
                 and now-self.last_odom_received < self.sensor_timeout
                 and now-self.last_scan_received < self.sensor_timeout)
        if not fresh:
            self._cancel_plan()
            self.shared_data['state'] = 'WAITING_SENSORS'
            self._stop_motion()
            return
        self.process_pose_command()
        if now-self.last_relocalize_time > self.relocalize_interval:
            self.last_relocalize_time = now
            success, pose = self.relocalize()
            if success:
                self.set_pose(*pose)
        self._refresh_obstacles(now)
        self._publish_new_obstacles(now)
        self._poll_plan()
        click = self.shared_data.get('click')
        if click is not None and self.shared_data.get('ui_mode') == 'GOAL':
            self.shared_data.update(click=None, goal=tuple(click))
            self._cancel_plan()
            self._plan_to_goal(new_goal=True)   # court : la destination est vérifiée tout de suite
        elif self.shared_data.get('goal'):
            due = now-self.last_plan_time >= (1.0 if self.state == 'NO_PATH' else REPLAN_MIN_PERIOD)
            blocked = self.state == 'NAV' and self._path_blocked()
            if (blocked or self.state == 'NO_PATH' or getattr(self, '_plan_wanted', False)) and due:
                self._request_plan()
        twist = Twist()
        front_blocked = self.front_distance <= STOP_DISTANCE
        v_target = w_target = 0.0
        if self.state == 'NAV' and self.path:
            # Un sommet atteint est retiré ; le dernier exige la tolérance d'arrivée.
            while len(self.path) > 1 and math.hypot(self.path[0][0]-self.robot_pos[0],
                                                    self.path[0][1]-self.robot_pos[1]) < 0.10:
                self.path.pop(0)
            end = self.path[-1]
            if math.hypot(end[0]-self.robot_pos[0], end[1]-self.robot_pos[1]) < 0.06:
                self.path = []
                self.state = 'READY'
                self.shared_data['goal'] = None
                self.pid_heading.reset()
            else:
                target, remaining = self._lookahead(LOOKAHEAD)
                angle = math.atan2(target[1]-self.robot_pos[1], target[0]-self.robot_pos[0])-self.robot_pos[2]
                error = math.atan2(math.sin(angle), math.cos(angle))
                w_target = max(-MAX_ANGULAR, min(MAX_ANGULAR, self.pid_heading.compute(error, dt)*MAX_ANGULAR/0.7))
                align = max(0.0, 1.0-abs(error)/1.2)     # ralentit progressivement dans les virages
                arrival = max(0.04, 0.8*remaining)       # et à l'approche du but
                near = max(0.25, min(1.0, (self.front_distance-STOP_DISTANCE)/0.35))
                v_target = min(MAX_LINEAR, arrival)*align*near
        if front_blocked:
            v_target = 0.0
            if abs(w_target) < 0.15:
                w_target = 0.0
        # Rampes : accélération douce, freinage plus ferme, jamais de saut de consigne.
        v_prev, w_prev = getattr(self, '_v_cmd', 0.0), getattr(self, '_w_cmd', 0.0)
        v = min(v_prev+LINEAR_ACCEL*dt, v_target) if v_target >= v_prev else max(v_prev-LINEAR_DECEL*dt, v_target)
        w = max(w_prev-ANGULAR_ACCEL*dt, min(w_prev+ANGULAR_ACCEL*dt, w_target))
        if front_blocked:
            v = 0.0
        self._v_cmd, self._w_cmd = v, w
        twist.linear.x, twist.angular.z = float(v), float(w)
        out_state = 'OBSTACLE_STOP' if front_blocked and self.state == 'NAV' else self.state
        self.shared_data.update(path=list(self.path), state=out_state)
        self.cmd_pub.publish(twist)

    def _publish_new_obstacles(self, now):
        seen = getattr(self, 'seen_obstacles', None)
        if seen is None or now-getattr(self, '_obstacles_pub_time', 0) < 0.2:
            return
        self._obstacles_pub_time = now
        for cell in [c for c, stamp in seen.items() if now-stamp > OBSTACLE_DISPLAY_TTL]:
            del seen[cell]
        self.shared_data['new_obstacles'] = [self.grid_to_world(*cell) for cell in seen]


def _segment_distance(px, py, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    sq = dx*dx+dy*dy
    t = 0.0 if sq < 1e-12 else max(0.0, min(1.0, ((px-a[0])*dx+(py-a[1])*dy)/sq))
    return math.hypot(px-(a[0]+t*dx), py-(a[1]+t*dy))
