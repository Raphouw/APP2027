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


class SlamMappingMixin:
    def initialize_slam(self):
        self.slam = SlamEngine()
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
        if revision != self.slam.revision:
            # Rebuild ALL retained observations, including free-space rays.
            for name, fill in [('map', .5), ('map_logodd', 0.), ('cell_update_time', 0),
                               ('cell_change_count', 0), ('cell_prev_state', .5)]:
                getattr(self, name).fill(fill)
            self.path_x, self.path_y, self.scan_counter = [], [], 0
            for past_pose, past_points, hits in self.slam.scans():
                self._raster_scan(past_pose, past_points, hits)
        else:
            self._raster_scan(pose, points, self.scan_hits)
        debug_dir = Path(self.output_dir)/'debug'
        debug_dir.mkdir(parents=True, exist_ok=True)
        if not hasattr(self, '_events_path'):
            self._events_path = debug_dir/('slam_events_'+uuid.uuid4().hex[:12]+'.jsonl')
        event = self.slam.diagnostics()
        event.update(updated_at=time.time(), odom=np.asarray(odom).tolist(), scans=self.scan_counter,
                     rebuilt=revision != self.slam.revision, unsynchronized_scans=self.unsynchronized_scans)
        with self._events_path.open('a') as stream:
            stream.write(json.dumps(event, allow_nan=False)+'\n')
        self.write_preview()

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
