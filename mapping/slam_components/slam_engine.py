"""SLAM orchestration: persistent correction, keyframes and revisits."""
import math
import numpy as np
from .se2 import angle, compose, inverse, relative, transform
from .scan_matching import Match, match_scan
from .pose_buffer import PoseBuffer
from .pose_graph import PoseGraph

class SlamEngine:
    """Persistent map<-odom correction, keyframes, verified loops, pose graph.

    Each observation stores its pose relative to an anchor keyframe. Rebuilding
    from observations after optimization moves past rays as well as trajectory.
    """
    def __init__(self):
        self.map_from_odom = np.zeros(3)
        self.graph = PoseGraph()
        self.key_scans, self.key_odom, self.observations = [], [], []
        self.last_match = Match(np.zeros(3))
        self.loops = 0
        self.revision = 0
        self.rejected = 0
        self.accepted = 0
        self.last_pose = np.zeros(3)

    def update(self, odom, points, hits):
        odom, points, hits = np.asarray(odom, float), np.asarray(points, float), np.asarray(hits, bool)
        source = points[hits][::2]
        predicted = compose(self.map_from_odom, odom)
        corrected = predicted.copy()
        if self.graph.poses:
            target = np.concatenate([transform(scan, pose) for scan, pose in
                                     list(zip(self.key_scans, self.graph.poses))[-6:]])
            self.last_match = match_scan(source, target, predicted)
            if self.last_match.accepted:
                corrected = self.last_match.pose
                self.accepted += 1
            else:
                self.rejected += 1
        self.map_from_odom = compose(corrected, inverse(odom))
        create = not self.graph.poses
        if not create:
            d = relative(self.graph.poses[-1], corrected)
            create = np.linalg.norm(d[:2]) >= .22 or abs(d[2]) >= .15
        if create and len(source) >= 20:
            previous = len(self.graph.poses)-1
            current = self.graph.add(corrected)
            self.key_scans.append(source.copy())
            self.key_odom.append(odom.copy())
            if previous >= 0:
                # Independent odometry prior plus scan-derived relative constraint.
                self.graph.constrain(previous, current, relative(self.key_odom[previous], odom), (.15, .15, .12))
                if self.last_match.accepted:
                    self.graph.constrain(previous, current, relative(self.graph.poses[previous], corrected))
            # Revisit search is deliberately bounded; no all-to-all matching.
            candidates = [i for i in range(max(0, current-400), current-15)
                          if np.linalg.norm(self.graph.poses[i][:2]-corrected[:2]) < .75]
            candidates.sort(key=lambda i: np.linalg.norm(self.graph.poses[i][:2]-corrected[:2]))
            for i in candidates[:3]:
                initial = relative(self.graph.poses[i], corrected)
                loop = match_scan(source, self.key_scans[i], initial,
                                  max_distance=.4, max_translation=.6, max_rotation=.4, min_overlap=.85)
                if not loop.accepted or loop.rmse > .045:
                    continue
                # Symmetric check prevents a small partial scan from fitting a larger one.
                reverse = match_scan(self.key_scans[i], source, inverse(loop.pose), min_overlap=.85)
                if not reverse.accepted or reverse.rmse > .045:
                    continue
                self.graph.constrain(i, current, loop.pose, (.035, .035, .025), loop=True)
                if self.graph.optimize():
                    corrected = self.graph.poses[current].copy()
                    self.map_from_odom = compose(corrected, inverse(odom))
                    self.loops += 1
                    self.revision += 1
                else:
                    self.graph.edges.pop()
                break
        anchor = len(self.graph.poses)-1
        offset = relative(self.graph.poses[anchor], corrected) if anchor >= 0 else corrected.copy()
        self.observations.append((anchor, offset, points.copy(), hits.copy()))
        self.last_pose = corrected.copy()
        return corrected

    def scans(self):
        for anchor, offset, points, hits in self.observations:
            pose = compose(self.graph.poses[anchor], offset) if anchor >= 0 else offset
            yield pose, points, hits

    def diagnostics(self):
        return dict(keyframes=len(self.graph.poses), loops=self.loops, revision=self.revision,
                    accepted=self.accepted, rejected=self.rejected,
                    rmse=self.last_match.rmse if math.isfinite(self.last_match.rmse) else None,
                    overlap=self.last_match.overlap, match_reason=self.last_match.reason,
                    pose=self.last_pose.tolist(), map_from_odom=self.map_from_odom.tolist())
