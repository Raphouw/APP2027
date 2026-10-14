#!/usr/bin/env python3
"""Planification de chemin sur grille : A* 8-connexe, heuristique octile, lissage.

Valeurs de grille : 0 libre, 2 occupé (inflé). Un tableau `cost` optionnel
(même forme, ≥ 0) ajoute une pénalité de proximité des murs : le chemin reste
au milieu des couloirs sans refuser les passages étroits.

L'API historique `DStarLite` est conservée. Elle ne revendique pas la complexité
de D* : on relance un A* borné, que l'appelant exécute hors de la boucle de
contrôle (voir `map_control.py`), donc sans jamais arrêter le robot.
"""
import heapq
import math
import numpy as np

SQRT2 = math.sqrt(2.0)
_MOVES = [(0, 1, 1.0), (0, -1, 1.0), (1, 0, 1.0), (-1, 0, 1.0),
          (1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2), (-1, -1, SQRT2)]


def octile(a, b):
    """Distance exacte sur grille 8-connexe sans obstacle (coût diagonal √2)."""
    dr, dc = abs(a[0]-b[0]), abs(a[1]-b[1])
    return (dr+dc)+(SQRT2-2.0)*min(dr, dc)


def nearest_free(grid, cell, max_radius):
    """Cellule libre la plus proche de `cell` (rayon en cellules), ou None."""
    grid = np.asarray(grid)
    h, w = grid.shape
    r0, c0 = cell
    if 0 <= r0 < h and 0 <= c0 < w and grid[r0, c0] < 2:
        return (r0, c0)
    r_lo, r_hi = max(0, r0-max_radius), min(h, r0+max_radius+1)
    c_lo, c_hi = max(0, c0-max_radius), min(w, c0+max_radius+1)
    if r_lo >= r_hi or c_lo >= c_hi:
        return None
    window = grid[r_lo:r_hi, c_lo:c_hi] < 2
    rows, cols = np.nonzero(window)
    if not len(rows):
        return None
    rows, cols = rows+r_lo, cols+c_lo
    dist = (rows-r0)**2+(cols-c0)**2
    keep = dist <= max_radius**2
    if not keep.any():
        return None
    best = int(np.argmin(np.where(keep, dist, np.iinfo(dist.dtype).max)))
    return (int(rows[best]), int(cols[best]))


def plan_path(grid, start, goal, cost=None, heuristic=None, max_expansions=None):
    """A* 8-connexe. Retourne la liste de cellules (start..goal) ou None.

    Pas de coupe de coin : une diagonale exige les deux cellules adjacentes libres.
    """
    grid = np.asarray(grid)
    h, w = grid.shape
    start, goal = tuple(start), tuple(goal)
    blocked = grid >= 2
    for r, c in (start, goal):
        if not (0 <= r < h and 0 <= c < w) or blocked[r, c]:
            return None
    heuristic = heuristic or octile
    # Listes Python : plus rapide que l'indexation numpy dans la boucle chaude.
    blocked_rows = blocked.tolist()
    penalty = cost.tolist() if cost is not None else None
    queue = [(heuristic(start, goal), 0.0, start)]
    costs, parents = {start: 0.0}, {}
    expansions = 0
    while queue:
        _, g, node = heapq.heappop(queue)
        if g != costs[node]:
            continue
        if node == goal:
            path = [node]
            while node != start:
                node = parents[node]
                path.append(node)
            return path[::-1]
        expansions += 1
        if max_expansions is not None and expansions > max_expansions:
            return None
        r, c = node
        for dr, dc, step in _MOVES:
            nr, nc = r+dr, c+dc
            if not (0 <= nr < h and 0 <= nc < w) or blocked_rows[nr][nc]:
                continue
            if dr and dc and (blocked_rows[r+dr][c] or blocked_rows[r][c+dc]):
                continue
            if penalty is not None:
                step = step*(1.0+penalty[nr][nc])
            candidate = g+step
            other = (nr, nc)
            if candidate < costs.get(other, math.inf):
                costs[other], parents[other] = candidate, node
                heapq.heappush(queue, (candidate+heuristic(other, goal), candidate, other))
    return None


def _line_cells(a, b):
    """Cellules traversées par le segment a→b (échantillonnage au quart de cellule)."""
    n = int(max(abs(b[0]-a[0]), abs(b[1]-a[1]))*4)+1
    rows = np.rint(np.linspace(a[0], b[0], n+1)).astype(int)
    cols = np.rint(np.linspace(a[1], b[1], n+1)).astype(int)
    return rows, cols


def line_of_sight(grid, a, b, cost=None):
    """Vrai si le segment a→b ne traverse aucune cellule occupée (diagonales strictes)."""
    grid = np.asarray(grid)
    rows, cols = _line_cells(a, b)
    if (grid[rows, cols] >= 2).any():
        return False
    # Les cellules successives doivent se toucher par un côté : pas de passage entre deux coins.
    dr, dc = np.diff(rows), np.diff(cols)
    diag = (dr != 0) & (dc != 0)
    for i in np.nonzero(diag)[0]:
        if grid[rows[i]+dr[i], cols[i]] >= 2 or grid[rows[i], cols[i]+dc[i]] >= 2:
            return False
    return True


def _segment_cost(a, b, cost):
    length = math.hypot(b[0]-a[0], b[1]-a[1])
    if cost is None:
        return length
    rows, cols = _line_cells(a, b)
    return length*(1.0+float(cost[rows, cols].mean()))


def smooth_path(grid, path, cost=None):
    """Raccourcit le chemin par lignes de vue ; ne garde que les sommets utiles.

    Un raccourci n'est accepté que s'il n'est pas plus coûteux (longueur ×
    pénalité de proximité) que le tronçon qu'il remplace : on ne rase pas un mur
    pour gagner quelques centimètres.
    """
    if not path or len(path) < 3:
        return list(path or [])
    cum = [0.0]
    for a, b in zip(path, path[1:]):
        cum.append(cum[-1]+_segment_cost(a, b, cost))
    result, i = [path[0]], 0
    while i < len(path)-1:
        j = len(path)-1
        while j > i+1:
            if line_of_sight(grid, path[i], path[j]) and _segment_cost(path[i], path[j], cost) <= cum[j]-cum[i]+1e-9:
                break
            j -= 1
        result.append(path[j])
        i = j
    return result


class DStarLite:
    """API historique : planificateur à grille modifiable."""

    def __init__(self, grid, heuristic=None, cost=None):
        self.grid = np.array(grid, copy=True)
        self.h, self.w = self.grid.shape
        self.heuristic = heuristic or octile
        self.cost = cost
        self.last_start = self.last_goal = None
        self.path = []

    def _valid(self, node):
        r, c = node
        return 0 <= r < self.h and 0 <= c < self.w and self.grid[r, c] < 2

    def plan(self, start, goal, smooth=False):
        start, goal = tuple(start), tuple(goal)
        self.last_start, self.last_goal = start, goal
        self.path = []
        route = plan_path(self.grid, start, goal, self.cost, self.heuristic)
        if route is None:
            return None
        self.path = smooth_path(self.grid, route, self.cost) if smooth else route
        return self.path

    def update_obstacle(self, obstacle_nodes, goal=None):
        for r, c in obstacle_nodes:
            if 0 <= r < self.h and 0 <= c < self.w:
                self.grid[r, c] = 2
        if self.last_start is None:
            return None
        return self.plan(self.last_start, self.last_goal if goal is None else goal)
