"""Map viewport regressions without a display or robot process."""
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock
import numpy as np
from matplotlib.figure import Figure
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'atelier_slam'))
from slam_components.manager_ui import ManagerUIMixin


class MapGrowthTests(unittest.TestCase):
    def setUp(self):
        self.app = ManagerUIMixin()
        self.app.ax = Figure().subplots()
        self.app.canvas = Mock()
        self.app.empty_hint = Mock()
        self.app.map_artist = None
        self.app.session_id = 'test'
        self.pixels = np.full((100, 100), 128, dtype=np.uint8)
        self.pixels[45:55, 45:55] = 255
        self.app.show_map(self.pixels, [-5, -5], .1)

    def test_new_explored_cells_expand_view_in_every_direction(self):
        self.pixels[2:98, 2:98] = 255
        self.app.show_map(self.pixels, [-5, -5], .1)
        np.testing.assert_allclose(self.app.ax.get_xlim(), [-5, 5])
        np.testing.assert_allclose(self.app.ax.get_ylim(), [-5, 5])
        # Resized grid with a shifted origin also remains fully visible.
        self.app.show_map(np.full((160, 140), 255, dtype=np.uint8), [-7, -8], .1)
        np.testing.assert_allclose(self.app.ax.get_xlim(), [-7, 7])
        np.testing.assert_allclose(self.app.ax.get_ylim(), [-8, 8])

    def test_unchanged_map_preserves_manual_zoom(self):
        self.app.ax.set_xlim(-.5, .5)
        self.app.ax.set_ylim(-.4, .4)
        self.app.show_map(self.pixels.copy(), [-5, -5], .1)
        np.testing.assert_allclose(self.app.ax.get_xlim(), [-.5, .5])
        np.testing.assert_allclose(self.app.ax.get_ylim(), [-.4, .4])


if __name__ == '__main__':
    unittest.main()
