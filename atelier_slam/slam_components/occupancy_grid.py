"""Grille interne [x, y] ; export transposé vers le format commun [y, x]."""
import numpy as np


class OccupancyGridMixin:
    decay = 0.999  # Default also used when the grid is instantiated independently.
    def process_data(self, ranges, angle_min, angle_max, angle_increment=None, range_min=0.12):
        ranges = np.asarray(ranges, dtype=float)
        angles = (angle_min+np.arange(len(ranges))*angle_increment if angle_increment is not None
                  else np.linspace(angle_min, angle_max, len(ranges)))
        # +inf et distances au-delà de la portée cartographiée : rayon libre sans obstacle terminal.
        valid = (np.isfinite(ranges) & (ranges > range_min)) | np.isposinf(ranges)
        self.scan_hits = np.isfinite(ranges[valid]) & (ranges[valid] <= self.dist_max_lidar)
        distance = np.minimum(ranges[valid], self.dist_max_lidar)
        return distance*np.cos(angles[valid]), distance*np.sin(angles[valid])

    def scan_match(self, x_scan, y_scan):
        # Correction locale de translation, sans prétendre optimiser une carte globale.
        cx = np.rint(self.x+self.scale*(x_scan*np.cos(self.orientation)-y_scan*np.sin(self.orientation))).astype(int)
        cy = np.rint(self.y+self.scale*(x_scan*np.sin(self.orientation)+y_scan*np.cos(self.orientation))).astype(int)
        corrections = []
        for x, y in zip(cx, cy):
            best = None
            for dx in range(-2, 3):
                for dy in range(-2, 3):
                    nx, ny = x+dx, y+dy
                    if 0 <= nx < self.size and 0 <= ny < self.size and self.map_logodd[nx, ny] > 1.5:
                        if best is None or dx*dx+dy*dy < best[0]:
                            best = (dx*dx+dy*dy, dx, dy)
            if best:
                corrections.append(best[1:])
        return tuple(np.median(corrections, axis=0)) if len(corrections) >= 5 else (0.0, 0.0)

    def grow_map(self):
        pad = self.grow_size
        for name, fill in [('map', 0.5), ('map_logodd', 0.0), ('cell_update_time', 0),
                           ('cell_change_count', 0), ('cell_prev_state', 0.5)]:
            setattr(self, name, np.pad(getattr(self, name), pad, constant_values=fill))
        self.size = self.map.shape[0]
        self.x0 += pad
        self.y0 += pad
        self.x += pad
        self.y += pad
        self.path_x = [v+pad for v in self.path_x]
        self.path_y = [v+pad for v in self.path_y]

    def get_free_indices(self, xr, yr, x_obs, y_obs):
        # Tracé entier par rayon, sans inclure son extrémité occupée.
        result = set()
        for xe, ye in zip(x_obs, y_obs):
            steps = int(max(abs(xe-xr), abs(ye-yr)))
            if steps:
                t = np.arange(steps)/steps
                result.update(zip(np.rint(xr+t*(xe-xr)).astype(int), np.rint(yr+t*(ye-yr)).astype(int)))
        if not result:
            return np.array([], dtype=int), np.array([], dtype=int)
        points = np.array(sorted(result))
        return points[:, 0], points[:, 1]

    def write_map(self, x, y):
        valid = np.isfinite(x) & np.isfinite(y) & (x*x+y*y > 0) & (x*x+y*y <= self.dist_max_lidar**2+1e-9)
        scan_hits = getattr(self, "scan_hits", np.ones(len(x), dtype=bool))
        hits = scan_hits[valid] if len(scan_hits) == len(x) else np.ones(np.count_nonzero(valid), dtype=bool)
        x, y = np.asarray(x)[valid], np.asarray(y)[valid]
        if not len(x):
            return
        corr_x, corr_y = (0., 0.) if hasattr(self, 'slam') else self.scan_match(x[hits], y[hits])
        self.x += corr_x
        self.y += corr_y
        px = np.rint(self.x+self.scale*(x*np.cos(self.orientation)-y*np.sin(self.orientation))).astype(int)
        py = np.rint(self.y+self.scale*(x*np.sin(self.orientation)+y*np.cos(self.orientation))).astype(int)
        while min(px.min(), py.min(), self.x, self.y) < 0 or max(px.max(), py.max(), self.x, self.y) >= self.size:
            self.grow_map()
            px += self.grow_size
            py += self.grow_size
        self.path_x.append(self.x)
        self.path_y.append(self.y)
        fx, fy = self.get_free_indices(int(round(self.x)), int(round(self.y)), px, py)
        occupied = set(zip(px[hits], py[hits]))
        free = (set(zip(fx, fy)) | set(zip(px[~hits], py[~hits])))-occupied
        self.scan_counter += 1
        for cells, delta in [(free, self.log_free), (occupied, self.log_occ)]:
            if cells:
                indices = np.array(sorted(cells))
                xx, yy = indices[:, 0], indices[:, 1]
                self.map_logodd[xx, yy] += delta
                self.cell_update_time[xx, yy] = self.scan_counter
        current = 1/(1+np.exp(-np.clip(self.map_logodd, -3, 3)))
        changed = np.abs(current-self.cell_prev_state) > self.state_diff_threshold
        self.cell_change_count[changed] += 1
        # Slow forgetting of unstable cells; configured by Cartographer(decay=...).
        self.map_logodd[self.cell_change_count >= self.change_threshold] *= self.decay
        self.map_logodd = np.clip(self.map_logodd, -3, 3)
        self.map = 1/(1+np.exp(-self.map_logodd))
        self.cell_prev_state = current.astype(np.float32)
