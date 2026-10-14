"""Timestamped pose interpolation without extrapolation."""
from collections import deque
import math
import numpy as np
from .se2 import angle

class PoseBuffer:
    """Ordered stamped poses; interpolation only, never stale extrapolation."""
    def __init__(self, seconds=5):
        self.seconds = seconds
        self.samples = deque()

    def add(self, stamp, pose):
        if not math.isfinite(stamp) or not np.all(np.isfinite(pose)):
            return
        if self.samples and stamp <= self.samples[-1][0]:
            return
        self.samples.append((float(stamp), np.asarray(pose, float)))
        while len(self.samples) > 2 and self.samples[0][0] < stamp-self.seconds:
            self.samples.popleft()

    def at(self, stamp, max_gap=.3):
        for t, pose in self.samples:
            if abs(t-stamp) < 1e-8:
                return pose.copy()
        for (ta, a), (tb, b) in zip(self.samples, list(self.samples)[1:]):
            if ta <= stamp <= tb and tb-ta <= max_gap:
                f = (stamp-ta)/(tb-ta)
                return np.array([*(a[:2]+f*(b[:2]-a[:2])), angle(a[2]+f*angle(b[2]-a[2]))])
        return None


