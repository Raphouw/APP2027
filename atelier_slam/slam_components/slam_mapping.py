"""Adapter between the metric SLAM backend and the occupancy raster."""
import json
import os
import time
import uuid
from pathlib import Path
import numpy as np
from PIL import Image
from .slam_engine import SlamEngine, PoseBuffer
from .map_format import encode_probability


# Après une fermeture de boucle, la grille doit être reconstruite depuis toutes les observations.
# Pour tenir le temps réel, on attend au moins ce nombre de scans entre deux reconstructions
# (la première est immédiate) ; `flush_raster` termine le travail avant toute sauvegarde.
REBUILD_EVERY_SCANS = 20


class SlamMappingMixin:
    def initialize_slam(self):
        self.slam = SlamEngine()
        self._raster_revision = 0
        self._scans_since_rebuild = REBUILD_EVERY_SCANS
        self.pose_buffer = PoseBuffer()
        self.unsynchronized_scans = 0
        self._preview_time = 0
        self.scan_stamps = []

    def integrate_scan(self, odom, ranges, amin, amax, increment=None, range_min=.12, stamp=None):
        x, y = self.process_data(ranges, amin, amax, increment, range_min)
        if not len(x):
            return
        points = np.column_stack([x, y])
        self.scan_stamps.append(stamp)
        revision = self.slam.revision
        pose = self.slam.update(odom, points, self.scan_hits)
        self._scans_since_rebuild += 1
        rebuilt = False
        if (self.slam.revision != self._raster_revision
                and self._scans_since_rebuild >= REBUILD_EVERY_SCANS):
            self._rebuild_raster()
            rebuilt = True
        else:
            self._raster_scan(pose, points, self.scan_hits)
        debug_dir = Path(self.output_dir)/'debug'
        debug_dir.mkdir(parents=True, exist_ok=True)
        if not hasattr(self, '_events_path'):
            self._events_path = debug_dir/('slam_events_'+uuid.uuid4().hex[:12]+'.jsonl')
        event = self.slam.diagnostics()
        event.update(updated_at=time.time(), odom=np.asarray(odom).tolist(), scans=self.scan_counter,
                     rebuilt=rebuilt, unsynchronized_scans=self.unsynchronized_scans)
        with self._events_path.open('a') as stream:
            stream.write(json.dumps(event, allow_nan=False)+'\n')
        self.write_preview()

    def _rebuild_raster(self):
        # Rebuild ALL retained observations, including free-space rays.
        for name, fill in [('map', .5), ('map_logodd', 0.), ('cell_update_time', 0),
                           ('cell_change_count', 0), ('cell_prev_state', .5)]:
            getattr(self, name).fill(fill)
        self.path_x, self.path_y, self.scan_counter = [], [], 0
        for past_pose, past_points, hits in self.slam.scans():
            self._raster_scan(past_pose, past_points, hits)
        self._raster_revision = self.slam.revision
        self._scans_since_rebuild = 0

    def flush_raster(self):
        """Termine une reconstruction différée (à appeler avant d'enregistrer la carte)."""
        if getattr(self, 'slam', None) is not None and self.slam.revision != self._raster_revision:
            self._rebuild_raster()

    def _raster_scan(self, pose, points, hits):
        self.x, self.y = pose[:2]*self.scale + [self.x0, self.y0]
        self.orientation = pose[2]
        self.scan_hits = hits
        self.write_map(points[:, 0], points[:, 1])

    def write_preview(self, force=False):
        if not force and time.monotonic()-self._preview_time < 1:
            return
        out = Path(self.output_dir)
        out.mkdir(parents=True, exist_ok=True)
        image = out/'preview.png'
        tmp = out/'preview.tmp.png'
        Image.fromarray(encode_probability(self.map.T)).save(tmp)
        os.replace(tmp, image)
        data = self.slam.diagnostics()
        data.update(session_id=self.session_id, updated_at=time.time(), scans=self.scan_counter,
                    unsynchronized_scans=self.unsynchronized_scans,
                    origin=[-self.x0/self.scale, -self.y0/self.scale], resolution=1/self.scale,
                    path=[[(x-self.x0)/self.scale, (y-self.y0)/self.scale]
                          for x, y in zip(self.path_x[::5], self.path_y[::5])])
        path = out/'slam_status.json'
        tmp = path.with_suffix('.tmp')
        tmp.write_text(json.dumps(data, allow_nan=False))
        os.replace(tmp, path)
        self._preview_time = time.monotonic()
