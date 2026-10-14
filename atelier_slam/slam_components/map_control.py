"""Suivi sur carte avec arrêt capteurs/obstacles et replanification dynamique."""
import math
import time
import numpy as np
from scipy.ndimage import binary_dilation
from geometry_msgs.msg import Twist
from .settings import STOP_DISTANCE


class MapControlMixin:
    def _plan_to_goal(self):
        # Immobiliser le robot pendant le calcul ; aucune ancienne commande ne doit persister.
        self.cmd_pub.publish(Twist())
        start = self.world_to_grid(*self.robot_pos[:2])
        goal = self.shared_data.get('goal')
        goal_cell = self.world_to_grid(*goal) if goal else None
        if start is None or goal_cell is None:
            route = None
        else:
            route = self.planner.plan(start, goal_cell)
        self.path = [self.grid_to_world(*cell) for cell in route[1:]] if route else []
        if route:
            # Conserver le point exact demandé, y compris pour un but dans la même cellule.
            self.path.append(tuple(goal))
        self.state = 'NAV' if route else 'NO_PATH'
        self.pid_heading.reset()
        self.shared_data.update(path=self.path, state=self.state)

    def _refresh_obstacles(self, now):
        self.dynamic_obstacles = {cell: stamp for cell, stamp in self.dynamic_obstacles.items() if now-stamp < 1.0}
        mask = np.zeros_like(self.static_grid, dtype=bool)
        for cell in self.dynamic_obstacles:
            mask[cell] = True
        radius = int(math.ceil(self.clearance/self.res))
        yy, xx = np.ogrid[-radius:radius+1, -radius:radius+1]
        inflated = binary_dilation(mask, structure=xx*xx+yy*yy <= radius*radius)
        grid = np.where(inflated, 2, self.static_grid)
        changed = not np.array_equal(grid, self.planner.grid)
        self.planner.grid = grid
        return changed

    def control_loop(self):
        now = time.monotonic()
        dt = max(0.001, min(0.1, now-self.last_control_time))
        self.last_control_time = now
        if self.shared_data.get('quit'):
            self.cmd_pub.publish(Twist())
            return
        if self.shared_data.get('state') == 'STOPPED':
            self.path = []
            self.state = 'READY'
            self.pid_heading.reset()
            self.shared_data.update(state=self.state, path=[], goal=None, click=None)
            self.cmd_pub.publish(Twist())
            return
        fresh = (self.last_odom_received is not None and self.last_scan_received is not None
                 and now-self.last_odom_received < self.sensor_timeout
                 and now-self.last_scan_received < self.sensor_timeout)
        if not fresh:
            self.shared_data['state'] = 'WAITING_SENSORS'
            self.cmd_pub.publish(Twist())
            return
        self.process_pose_command()
        if now-self.last_relocalize_time > self.relocalize_interval:
            self.last_relocalize_time = now
            success, pose = self.relocalize()
            if success:
                self.set_pose(*pose)
        if self.front_distance <= STOP_DISTANCE:
            self.shared_data['state'] = 'OBSTACLE_STOP'
            self.cmd_pub.publish(Twist())
            return
        changed = self._refresh_obstacles(now)
        click = self.shared_data.get('click')
        if click is not None and self.shared_data.get('ui_mode') == 'GOAL':
            self.shared_data.update(click=None, goal=tuple(click))
            self._plan_to_goal()
            self.last_plan_time = now
        elif self.shared_data.get('goal') and (changed or self.state == 'NO_PATH') and now-self.last_plan_time >= 0.5:
            self._plan_to_goal()
            self.last_plan_time = now
        twist = Twist()
        if self.state == 'NAV' and self.path:
            target = self.path[0]
            distance = math.hypot(target[0]-self.robot_pos[0], target[1]-self.robot_pos[1])
            if distance < 0.06:
                self.path.pop(0)
                if not self.path:
                    self.state = 'READY'
                    self.shared_data['goal'] = None
                    self.pid_heading.reset()
            else:
                angle = math.atan2(target[1]-self.robot_pos[1], target[0]-self.robot_pos[0])-self.robot_pos[2]
                error = math.atan2(math.sin(angle), math.cos(angle))
                twist.angular.z = self.pid_heading.compute(error, dt)
                if abs(error) < 0.3:
                    twist.linear.x = min(0.15, max(0.03, distance))
        self.shared_data.update(path=self.path, state=self.state)
        self.cmd_pub.publish(twist)
