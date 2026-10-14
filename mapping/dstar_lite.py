#!/usr/bin/env python3
"""API historique DStarLite, recherche A* bornée et déterministe.

La version incrémentale précédente n'était pas validée. Cette base recalcule
le chemin lorsque la grille change ; elle ne revendique pas la complexité D*.
"""
import heapq
import math
import numpy as np


class DStarLite:
    def __init__(self, grid, heuristic=None):
        self.grid = np.array(grid, copy=True)
        self.h, self.w = self.grid.shape
        self.heuristic = heuristic or (lambda a, b: math.hypot(a[0]-b[0], a[1]-b[1]))
        self.last_start = self.last_goal = None
        self.path = []

    def _valid(self, node):
        r, c = node
        return 0 <= r < self.h and 0 <= c < self.w and self.grid[r, c] < 2

    def _get_neighbors(self, node):
        r, c = node
        for dr, dc in [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
            other = (r+dr, c+dc)
            if not self._valid(other):
                continue
            if dr and dc and (not self._valid((r+dr, c)) or not self._valid((r, c+dc))):
                continue
            yield other

    def plan(self, start, goal):
        start, goal = tuple(start), tuple(goal)
        self.last_start, self.last_goal = start, goal
        self.path = []
        if not self._valid(start) or not self._valid(goal):
            return None
        queue = [(self.heuristic(start, goal), 0.0, start)]
        costs, parents = {start: 0.0}, {}
        while queue:
            _, cost, node = heapq.heappop(queue)
            if cost != costs[node]:
                continue
            if node == goal:
                path = [node]
                while node != start:
                    node = parents[node]
                    path.append(node)
                self.path = list(reversed(path))
                return self.path
            for other in self._get_neighbors(node):
                candidate = cost+math.hypot(other[0]-node[0], other[1]-node[1])
                if candidate < costs.get(other, math.inf):
                    costs[other], parents[other] = candidate, node
                    heapq.heappush(queue, (candidate+self.heuristic(other, goal), candidate, other))
        return None

    def update_obstacle(self, obstacle_nodes, goal=None):
        for r, c in obstacle_nodes:
            if 0 <= r < self.h and 0 <= c < self.w:
                self.grid[r, c] = 2
        if self.last_start is None:
            return None
        return self.plan(self.last_start, self.last_goal if goal is None else goal)
