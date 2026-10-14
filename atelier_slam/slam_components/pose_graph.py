"""Sparse robust optimization of SE(2) pose constraints."""
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix

def _wrap(value):
    return (value + np.pi) % (2 * np.pi) - np.pi


def _relative_many(a, b):
    """`relative(a[k], b[k])` pour toutes les lignes : poses (N, 3), sans boucle Python."""
    c, s = np.cos(a[:, 2]), np.sin(a[:, 2])
    dx, dy = b[:, 0]-a[:, 0], b[:, 1]-a[:, 1]
    return np.column_stack([c*dx+s*dy, -s*dx+c*dy, _wrap(b[:, 2]-a[:, 2])])


class PoseGraph:
    def __init__(self):
        self.poses, self.edges = [], []

    def add(self, pose):
        self.poses.append(np.asarray(pose, float).copy())
        return len(self.poses)-1

    def constrain(self, i, j, measurement, sigma=(.08, .08, .06), loop=False):
        self.edges.append((i, j, np.asarray(measurement, float), np.asarray(sigma), loop))

    def optimize(self):
        if len(self.poses) < 3:
            return False
        original = np.asarray(self.poses)
        first = np.array([edge[0] for edge in self.edges])
        second = np.array([edge[1] for edge in self.edges])
        measurements = np.array([edge[2] for edge in self.edges])
        sigmas = np.array([edge[3] for edge in self.edges])
        def residual(values):
            poses = np.vstack([original[0], values.reshape(-1, 3)])
            predicted = _relative_many(poses[first], poses[second])
            return (_relative_many(measurements, predicted)/sigmas).ravel()
        before = original[1:].ravel()
        sparsity = lil_matrix((3*len(self.edges), len(before)), dtype=int)
        for row, (i, j, _, _, _) in enumerate(self.edges):
            for node in (i, j):
                if node:
                    sparsity[3*row:3*row+3, 3*(node-1):3*node] = 1
        solution = least_squares(residual, before, loss='soft_l1', max_nfev=30, ftol=1e-6, xtol=1e-6,
                                 jac_sparsity=sparsity.tocsr())
        robust_cost = lambda x: np.sum(2*(np.sqrt(1+residual(x)**2)-1))
        if not solution.success or not np.all(np.isfinite(solution.x)) or robust_cost(solution.x) > robust_cost(before)+1e-6:
            return False
        self.poses = [p.copy() for p in np.vstack([original[0], solution.x.reshape(-1, 3)])]
        return True


