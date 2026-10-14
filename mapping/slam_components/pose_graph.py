"""Sparse robust optimization of SE(2) pose constraints."""
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from .se2 import relative

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
        def residual(values):
            poses = np.vstack([original[0], values.reshape(-1, 3)])
            errors = []
            for i, j, z, sigma, _ in self.edges:
                error = relative(z, relative(poses[i], poses[j]))
                errors.extend(error/sigma)
            return np.asarray(errors)
        before = original[1:].ravel()
        sparsity = lil_matrix((3*len(self.edges), len(before)), dtype=int)
        for row, (i, j, _, _, _) in enumerate(self.edges):
            for node in (i, j):
                if node:
                    sparsity[3*row:3*row+3, 3*(node-1):3*node] = 1
        solution = least_squares(residual, before, loss='soft_l1', max_nfev=80,
                                 jac_sparsity=sparsity.tocsr())
        robust_cost = lambda x: np.sum(2*(np.sqrt(1+residual(x)**2)-1))
        if not solution.success or not np.all(np.isfinite(solution.x)) or robust_cost(solution.x) > robust_cost(before)+1e-6:
            return False
        self.poses = [p.copy() for p in np.vstack([original[0], solution.x.reshape(-1, 3)])]
        return True


