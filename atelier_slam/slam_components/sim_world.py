"""Monde et robot simulés, sans ROS : grille d'occupation, lancer de rayons, cinématique.

Sert au nœud `simulateur.py` et aux tests. Les mesures imitent un TurtleBot3 Burger
(LiDAR 360 rayons, roues de 33 mm, voie de 160 mm) avec un bruit réglable.
"""
import math
import numpy as np

WHEEL_RADIUS = 0.033
WHEEL_BASE = 0.160
ROBOT_RADIUS = 0.105
MAX_V = 0.22
MAX_W = 2.84
RANGE_MIN, RANGE_MAX = 0.12, 3.5
BEAMS = 360


def default_world(resolution=0.05):
    """Salle 10 m x 8 m avec un îlot central (couloir bouclant) et quelques piliers."""
    width, height = int(10 / resolution), int(8 / resolution)
    grid = np.zeros((height, width), dtype=bool)
    grid[0, :] = grid[-1, :] = grid[:, 0] = grid[:, -1] = True

    def box(x0, y0, x1, y1):
        grid[int(y0 / resolution):int(y1 / resolution), int(x0 / resolution):int(x1 / resolution)] = True

    box(3.0, 2.5, 7.0, 5.5)          # îlot central : on peut en faire le tour
    box(1.0, 5.5, 1.3, 7.0)          # cloison
    box(8.0, 1.0, 9.0, 1.4)          # muret
    box(2.0, 1.2, 2.3, 1.5)          # piliers
    box(8.4, 6.2, 8.7, 6.5)
    return World(grid, resolution, (0.0, 0.0))


class World:
    def __init__(self, occupied, resolution, origin):
        self.occupied = np.asarray(occupied, dtype=bool)
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))

    @classmethod
    def from_pgm(cls, pgm_path, resolution, origin, occupied_below=100):
        """Charge une carte PGM (noir occupé, gris inconnu traité comme libre)."""
        from PIL import Image
        image = np.array(Image.open(pgm_path).convert('L'))
        return cls(np.flipud(image) < occupied_below, resolution, origin)

    def is_occupied(self, x, y):
        col = np.floor((np.asarray(x) - self.origin[0]) / self.resolution).astype(int)
        row = np.floor((np.asarray(y) - self.origin[1]) / self.resolution).astype(int)
        inside = (row >= 0) & (row < self.occupied.shape[0]) & (col >= 0) & (col < self.occupied.shape[1])
        result = np.ones(np.shape(col), dtype=bool)  # hors carte = obstacle
        result[inside] = self.occupied[row[inside], col[inside]]
        return result

    def raycast(self, x, y, theta, relative_angles, max_range=RANGE_MAX):
        """Distance au premier obstacle pour chaque angle ; `inf` si rien avant `max_range`."""
        step = self.resolution / 2
        distances = np.arange(step, max_range + step, step)
        angles = theta + np.asarray(relative_angles)
        px = x + np.cos(angles)[:, None] * distances[None, :]
        py = y + np.sin(angles)[:, None] * distances[None, :]
        hit = self.is_occupied(px, py)
        first = hit.argmax(axis=1)
        found = hit[np.arange(hit.shape[0]), first]
        result = np.where(found, distances[first], np.inf)
        return np.where(result <= max_range, result, np.inf)

    def free_position(self, x, y, radius=ROBOT_RADIUS):
        angles = np.linspace(0, 2 * math.pi, 12, endpoint=False)
        px = np.append(x + radius * np.cos(angles), x)
        py = np.append(y + radius * np.sin(angles), y)
        return not self.is_occupied(px, py).any()


def normalize(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class SimRobot:
    """Robot différentiel : vérité terrain + mesures bruitées de roues, gyro et LiDAR."""

    def __init__(self, world, pose=(1.5, 2.0, 0.0), noise=1.0, seed=None, dropout=0.0):
        self.world = world
        self.x, self.y, self.theta = pose
        self.noise = float(noise)
        self.dropout = float(dropout)   # part des rayons rendus invalides (0.0 m), comme un vrai LDS
        self.rng = np.random.default_rng(seed)
        self.v = self.w = 0.0
        self.wheel_left = self.wheel_right = 0.0
        self.gyro_bias = float(self.rng.normal(0, 0.003)) * self.noise
        # Erreurs systématiques (rayon de roue réel != 33 mm, etc.) : elles font dériver l'odométrie.
        self.scale_left = 1 + float(self.rng.normal(0, 0.015)) * self.noise
        self.scale_right = 1 + float(self.rng.normal(0, 0.015)) * self.noise
        self.collided = False
        self.angles = np.arange(BEAMS) * 2 * math.pi / BEAMS

    def step(self, cmd_v, cmd_w, dt):
        """Avance de `dt` secondes vers la consigne ; bloque sur les obstacles."""
        cmd_v = max(-MAX_V, min(MAX_V, cmd_v))
        cmd_w = max(-MAX_W, min(MAX_W, cmd_w))
        # Réponse du moteur : rampe d'accélération simple.
        self.v += max(-0.5 * dt * 2, min(0.5 * dt * 2, cmd_v - self.v))
        self.w += max(-3.0 * dt * 2, min(3.0 * dt * 2, cmd_w - self.w))
        theta_mid = self.theta + self.w * dt / 2
        nx = self.x + self.v * math.cos(theta_mid) * dt
        ny = self.y + self.v * math.sin(theta_mid) * dt
        self.collided = not self.world.free_position(nx, ny)
        if self.collided:
            self.v = 0.0
        else:
            self.x, self.y = nx, ny
        self.theta = normalize(self.theta + self.w * dt)
        # Roues : glissement multiplicatif indépendant (erreur d'odométrie réaliste).
        slip_l = self.scale_left * (1 + self.rng.normal(0, 0.01) * self.noise)
        slip_r = self.scale_right * (1 + self.rng.normal(0, 0.01) * self.noise)
        v_left = (self.v - self.w * WHEEL_BASE / 2) * slip_l
        v_right = (self.v + self.w * WHEEL_BASE / 2) * slip_r
        self.wheel_left += v_left * dt / WHEEL_RADIUS
        self.wheel_right += v_right * dt / WHEEL_RADIUS

    def wheel_positions(self):
        return normalize(self.wheel_left), normalize(self.wheel_right)

    def gyro(self):
        return self.w + self.gyro_bias + float(self.rng.normal(0, 0.004)) * self.noise

    def scan(self):
        ranges = self.world.raycast(self.x, self.y, self.theta, self.angles)
        ranges = ranges + self.rng.normal(0, 0.01 * self.noise, ranges.shape)
        ranges[ranges < RANGE_MIN] = np.inf
        ranges[ranges > RANGE_MAX] = np.inf   # le bruit ne doit pas créer de mesures hors portée
        if self.dropout > 0:
            ranges[self.rng.random(ranges.shape) < self.dropout] = 0.0
        return ranges
