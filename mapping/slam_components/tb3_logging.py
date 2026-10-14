"""Écriture progressive des CSV et sauvegarde atomique de la dernière pose."""
import csv
import json
import math
import os
from pathlib import Path
import time


class SessionWriter:
    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.path.open('x', newline='')
        self.writer = csv.writer(self.stream)
        # Le champ JSON conserve les angles et le nombre de rayons propres à chaque scan.
        self.writer.writerow(['timestamp', 'x', 'y', 'theta', 'state', 'front_min',
                              'angle_min', 'angle_max', 'ranges_json', 'range_min', 'range_max'])
        self.last_sync = time.monotonic()
        self.stream.flush()

    def append(self, timestamp, pose, state, front_min, angle_min, increment, ranges, range_min=None, range_max=None):
        maximum = angle_min+(len(ranges)-1)*increment if len(ranges) else angle_min
        self.writer.writerow([timestamp, *pose, state, front_min, angle_min, maximum,
                              json.dumps([float(v) if math.isfinite(v) else ("inf" if v > 0 else "-inf" if v < 0 else None) for v in ranges], separators=(',', ':'), allow_nan=False), range_min, range_max])
        # Chaque ligne est visible immédiatement ; fsync au plus une fois par seconde.
        self.stream.flush()
        now = time.monotonic()
        if now-self.last_sync >= 1:
            os.fsync(self.stream.fileno())
            self.last_sync = now

    def close(self):
        if not self.stream.closed:
            self.stream.flush()
            os.fsync(self.stream.fileno())
            self.stream.close()


def save_pose(path, pose):
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_suffix(target.suffix+'.tmp')
    temporary.write_text(','.join(str(value) for value in pose))
    os.replace(temporary, target)


class SessionLoggingMixin:
    def write_health(self):
        now = time.monotonic()
        if now-getattr(self, '_health_time', 0) < .2:
            return
        self._health_time = now
        fresh = lambda name: getattr(self, name, None) is not None and now-getattr(self, name) < self.sensor_timeout
        data = dict(updated_at=time.time(), imu=fresh('last_imu_received'),
                    wheels=fresh('last_joint_received'), lidar=fresh('last_scan_received'),
                    lidar_front_known=getattr(self, 'lidar_front_known', False),
                    lidar_front_unknown=getattr(self, 'lidar_front_unknown', 0),
                    lidar_inferred=getattr(self, 'lidar_inferred', 0),
                    lidar_temporal=getattr(self, 'lidar_temporal', 0),
                    lidar_range_min=getattr(self, 'raw_scan_range_min', None),
                    lidar_range_max=getattr(self, 'raw_scan_range_max', None),
                    front_distance=self._front_min() if self.laser_ranges else None,
                    calibrated=self.imu_ok, calibration_samples=len(self.calib_samples),
                    calibration_total=self.CALIB_N, state=self.state,
                    pose=[self.x, self.y, self.theta])
        path = Path(self.pose_file_path).parent/'robot_status.json'
        tmp = path.with_suffix('.tmp')
        tmp.write_text(json.dumps(data, allow_nan=False))
        os.replace(tmp, path)

    def _log_data(self, t, front_min):
        stamp = getattr(self, 'raw_scan_stamp', None)
        buffer = getattr(self, 'log_pose_buffer', None)
        pose = buffer.at(stamp) if buffer is not None and stamp is not None else None
        if pose is not None and stamp != self.last_logged_scan:
            self.session_writer.append(stamp, pose, self.state, front_min,
                                       self.laser_amin, self.laser_ainc, self.raw_scan_ranges,
                                       range_min=getattr(self, 'raw_scan_range_min', None),
                                       range_max=getattr(self, 'raw_scan_range_max', None))
            self.last_logged_scan = stamp
        save_pose(self.pose_file_path, (self.x, self.y, self.theta))
