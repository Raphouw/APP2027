"""Conservative angular hole filling shared by both motion controllers."""
import math
import numpy as np


def filter_scan(ranges, angle_min, increment, range_min, range_max, max_hole_degrees=5.):
    """Return distances, trusted mask and inferred mask.

    Only small NaN runs between two FINITE returns are bridged,
    using the nearer endpoint. Large holes, unbounded holes and invalid numeric
    returns stay unknown (zero for control). +inf is a valid no-return ray.
    Inferred values are for control only: mapping/logging retain original rays.
    """
    raw = np.asarray(ranges, dtype=float)
    distance = np.zeros_like(raw)
    known = np.zeros(len(raw), dtype=bool)
    inferred = np.zeros(len(raw), dtype=bool)
    if (not len(raw) or not all(math.isfinite(v) for v in (angle_min, increment, range_min, range_max))
            or increment == 0 or range_min < 0 or range_max <= range_min):
        return distance, known, inferred
    finite = np.isfinite(raw) & (raw > range_min) & (raw <= range_max)
    no_return = np.isposinf(raw)
    known = finite | no_return
    distance[finite] = raw[finite]
    distance[no_return] = range_max
    missing = np.isnan(raw)
    circular = abs(increment)*len(raw) >= 2*math.pi-3*abs(increment)
    anchors = np.flatnonzero(known)
    if not len(anchors):
        return distance, known, inferred
    # Rotate a full scan to begin at a real ray, so seam holes are one run.
    start = int(anchors[0]) if circular else 0
    order = (np.arange(len(raw))+start) % len(raw)
    ordered_missing = missing[order]
    i = 0
    while i < len(raw):
        if not ordered_missing[i]:
            i += 1
            continue
        end = i
        while end < len(raw) and ordered_missing[end]:
            end += 1
        count = end-i
        bounded = (i > 0 and end < len(raw)) or (circular and i > 0 and end == len(raw))
        if bounded and count*abs(increment) <= math.radians(max_hole_degrees)+1e-9:
            left, right = order[i-1], order[end % len(raw)]
            if finite[left] and finite[right]:
                # At an edge use the nearer surface, never the average/far wall.
                near = min(raw[left], raw[right])
                indices = order[i:end]
                distance[indices] = near
                known[indices] = inferred[indices] = True
        i = end
    return distance, known, inferred


def front_mask(count, angle_min, increment, half_fov=math.radians(35)):
    angles = angle_min+np.arange(count)*increment
    return np.abs(np.arctan2(np.sin(angles), np.cos(angles))) < half_fov


class ScanHistory:
    """Bridge small intermittent holes using real returns up to 0.20 s old.

    Stored points are transformed through measured poses. Estimated rays are
    NEVER stored, so a blind sector cannot keep refreshing its own history.
    """
    def __init__(self, max_age=.20):
        self.max_age = max_age
        self.frames = []
        self.last_now = None

    def filter(self, ranges, angle_min, increment, range_min, range_max, pose, now):
        from .se2 import transform, inverse, angle
        distance, known, spatial = filter_scan(ranges, angle_min, increment, range_min, range_max)
        temporal = np.zeros(len(distance), dtype=bool)
        pose = np.asarray(pose, float)
        if (len(pose) != 3 or not np.all(np.isfinite(pose)) or not math.isfinite(now)
                or not math.isfinite(increment) or increment == 0):
            self.frames = []
            return distance, known, spatial, temporal
        if self.last_now is not None and now < self.last_now:
            self.frames = []
        self.last_now = now
        self.frames = [f for f in self.frames if 0 <= now-f[0] <= self.max_age
                       and np.linalg.norm(pose[:2]-f[2][:2]) <= .08
                       and abs(angle(pose[2]-f[2][2])) <= .25]
        raw = np.asarray(ranges, float)
        finite = np.isfinite(raw) & (raw > range_min) & (raw <= range_max)
        # Eligibility only: bounded NaN holes <=8 degrees. Do not extrapolate
        # partial-FOV edges, large blind sectors, numeric zeros or infinities.
        candidate, eligible, _ = filter_scan(raw, angle_min, increment, range_min, range_max,
                                             max_hole_degrees=8.)
        eligible &= ~known
        angles = angle_min+np.arange(len(raw))*increment
        if eligible.any() and self.frames:
            points, ages = [], []
            for stamp, world, _ in self.frames:
                local = transform(world, inverse(pose))
                points.append(local)
                ages.extend([now-stamp]*len(local))
            points = np.concatenate(points)
            ages = np.asarray(ages)
            bearings = np.arctan2(points[:, 1], points[:, 0])
            lengths = np.linalg.norm(points, axis=1)
            tolerance = min(abs(increment)*.65, math.radians(1.5))
            for index in np.flatnonzero(eligible):
                delta = np.abs(np.arctan2(np.sin(bearings-angles[index]), np.cos(bearings-angles[index])))
                matches = delta <= tolerance
                if matches.any():
                    # Extra 0.5 m/s closing margin +2 cm, beyond ego compensation.
                    conservative = np.min(lengths[matches]-.5*ages[matches]-.02)
                    distance[index] = max(0., min(candidate[index], conservative))
                    known[index] = temporal[index] = True
        if finite.any():
            local = np.column_stack([raw[finite]*np.cos(angles[finite]), raw[finite]*np.sin(angles[finite])])
            self.frames.append((now, transform(local, pose), pose.copy()))
            self.frames = self.frames[-10:]
        return distance, known, spatial, temporal
