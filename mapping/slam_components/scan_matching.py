"""Robust local scan alignment and acceptance diagnostics."""
from dataclasses import dataclass
import math
import numpy as np
from scipy.spatial import cKDTree
from .se2 import compose, relative, transform

@dataclass
class Match:
    pose: np.ndarray
    accepted: bool = False
    rmse: float = math.inf
    overlap: float = 0.0
    reason: str = 'points insuffisants'


def match_scan(source, target, initial, max_distance=.35, max_translation=.55,
               max_rotation=.35, min_overlap=.65):
    """Trimmed point-to-point ICP with bounded correction and geometry gates.

    Initial is the source->target SE2 pose. Rejected results retain initial pose.
    This local solver requires overlapping scans and a reasonable odometry guess.
    """
    source, target = np.asarray(source, float), np.asarray(target, float)
    initial = np.asarray(initial, float)
    result = Match(initial.copy())
    if len(source) < 20 or len(target) < 20 or not np.all(np.isfinite(initial)):
        return result
    if not np.all(np.isfinite(source)) or not np.all(np.isfinite(target)):
        return result
    # A single wall cannot reliably constrain motion along itself.
    eig = np.linalg.eigvalsh(np.cov(source.T))
    if eig[0] < .0025 or eig[0]/max(eig[1], 1e-9) < .015:
        result.reason = 'géométrie dégénérée'
        return result
    tree = cKDTree(target)
    pose = initial.copy()
    for _ in range(25):
        moved = transform(source, pose)
        distances, indices = tree.query(moved)
        mask = distances < max_distance
        if np.count_nonzero(mask) < 20:
            return result
        threshold = np.quantile(distances[mask], .85)
        keep = mask & (distances <= threshold)
        a, b = moved[keep], target[indices[keep]]
        ac, bc = a.mean(axis=0), b.mean(axis=0)
        u, _, vt = np.linalg.svd((a-ac).T @ (b-bc))
        rotation = vt.T @ u.T
        if np.linalg.det(rotation) < 0:
            vt[-1] *= -1
            rotation = vt.T @ u.T
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        translation = bc-rotation @ ac
        delta = np.array([*translation, yaw])
        pose = compose(delta, pose)
        if np.linalg.norm(delta[:2]) < 1e-5 and abs(delta[2]) < 1e-5:
            break
    distances, _ = tree.query(transform(source, pose))
    mask = distances < min(max_distance, .15)
    result.overlap = float(mask.mean())
    result.rmse = float(np.sqrt(np.mean(distances[mask]**2))) if mask.any() else math.inf
    correction = relative(initial, pose)
    if result.overlap < min_overlap or result.rmse > .09:
        result.reason = 'recouvrement ou résidu insuffisant'
    elif np.linalg.norm(correction[:2]) > max_translation or abs(correction[2]) > max_rotation:
        result.reason = 'correction trop grande'
    else:
        result.pose, result.accepted, result.reason = pose, True, 'accepté'
    return result


