"""Optional Tk smoke tests on a PRIVATE virtual display; no robot process."""
import os
from pathlib import Path
import sys
import tempfile
import tkinter as tk
import unittest
from unittest.mock import Mock
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'atelier_slam'))
from slam_components.manager_ui import ManagerUIMixin
from slam_components.manager_processes import ManagerProcessesMixin


class Harness(ManagerUIMixin, ManagerProcessesMixin):
    def __init__(self, root, directory):
        self.root = root
        root.geometry('1280x840')
        self.logs_dir = self.base_dir = directory
        self.nav_proc = self.odom_proc = self.carto_proc = self.rviz_proc = None
        self.video_reader = None
        self.video_enabled = False
        self.initialize_supervision()
        self.build_workspace()
        root.bind('<Escape>', lambda event: self.stop_auto_mode())
    def start_video(self):
        pass


@unittest.skipUnless(os.environ.get('TB3_TEST_DISPLAY'), 'Set TB3_TEST_DISPLAY to an isolated X display')
class UITests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.root = tk.Tk(screenName=os.environ['TB3_TEST_DISPLAY'])
        self.app = Harness(self.root, self.directory.name)
        self.root.update()
    def tearDown(self):
        self.app.executor.shutdown(wait=True)
        for handler in self.app.logger.handlers[:]:
            handler.close(); self.app.logger.removeHandler(handler)
        for timer in self.root.tk.call('after', 'info'):
            self.root.tk.call('after', 'cancel', timer)
        self.root.destroy()
        self.directory.cleanup()
    def test_initial_buttons_and_no_process_started(self):
        self.app.refresh_workspace(); self.root.update()
        self.assertEqual(str(self.app.btn_start['state']), 'normal')
        self.assertEqual(str(self.app.btn_stop['state']), 'disabled')
        self.assertEqual(str(self.app.btn_nav['state']), 'disabled')
        self.assertIsNone(self.app.nav_proc)
        self.assertEqual(len(self.app.book.tabs()), 3)
    def test_mapping_gates_actions_and_busy_keeps_stop_shortcut(self):
        self.app.carto_proc = Mock(poll=lambda: None)
        self.app.refresh_workspace(); self.root.update()
        self.assertEqual(str(self.app.btn_start['state']), 'disabled')
        self.assertEqual(str(self.app.btn_stop['state']), 'normal')
        self.app._busy = True
        self.app.refresh_workspace()
        self.assertEqual(str(self.app.btn_stop['state']), 'disabled')
        self.assertTrue(self.root.bind('<Escape>'))
    def test_text_rendering_avoids_native_crash_pictograms(self):
        original = 'Robot '+chr(0x1f916)+' prêt'
        self.app._status(original)
        self.root.update()
        self.assertIn('[symbole]', self.app.info_label['text'])
        self.assertNotIn(chr(0x1f916), self.app.journal.get('1.0', 'end'))
        self.assertIn(original, (self.app.debug_dir/'manager.log').read_text())
        queue = [self.root]
        while queue:
            widget = queue.pop()
            queue.extend(widget.winfo_children())
            try:
                text = str(widget.cget('text'))
            except tk.TclError:
                continue
            self.assertTrue(all(ord(char) <= 0xffff for char in text))

    def test_embedded_map_and_journal(self):
        self.app.show_map(np.array([[0, 255], [128, 255]]), [-1, -2], .1)
        self.app._status('Test du journal')
        self.app.canvas.draw(); self.root.update()
        np.testing.assert_allclose(self.app._preview_extent, [-1, -.8, -2, -1.8])
        self.assertIn('Test du journal', self.app.journal.get('1.0', 'end'))
        self.assertFalse(self.app.empty_hint.get_visible())


if __name__ == '__main__':
    unittest.main()
