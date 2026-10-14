"""Guided desktop application; process work stays off the Tk event thread."""
import os
import sys
from pathlib import Path
# Keep the historical `python3 atelier.py` command, with the same clean launch
# path as lancer.sh. Module imports in tests never launch a process.
if __name__ == '__main__' and os.environ.get('TB3_LAUNCH_READY') != '1':
    launcher = Path(__file__).resolve().parents[1]/'lancer.sh'
    os.execv('/bin/bash', ['/bin/bash', str(launcher), *sys.argv[1:]])
os.environ['MPLBACKEND'] = 'TkAgg'
import signal
import tkinter as tk
from slam_components.manager_ui import ManagerUIMixin
from slam_components.manager_processes import ManagerProcessesMixin
from slam_components.manager_video import ManagerVideoMixin
from slam_components.manager_ros import ManagerRosMixin
from slam_components.manager_robot import ManagerRobotMixin
from slam_components.system_tools import check_ffmpeg

__all__ = ['TB3Manager', 'check_ffmpeg']


class TB3Manager(ManagerUIMixin, ManagerProcessesMixin, ManagerVideoMixin, ManagerRosMixin, ManagerRobotMixin):
    def __init__(self, root):
        self.root = root
        root.title('Atelier SLAM · TurtleBot3')
        root.geometry('1280x840')
        root.minsize(1000, 700)
        self.base_dir = os.path.abspath(os.path.dirname(__file__))
        self.logs_dir = os.path.join(self.base_dir, 'logs')
        os.makedirs(self.logs_dir, exist_ok=True)
        self.nav_proc = self.rviz_proc = self.odom_proc = self.carto_proc = self.sim_proc = None
        self.video_reader = self.video_thread = None
        self.video_enabled = False
        self.rviz_config_path = os.path.join(self.base_dir, 'config', 'atelier.rviz')
        self.create_rviz_config()
        self.initialize_supervision()
        self.build_workspace()
        self.start_ros_probe()
        root.bind('<Escape>', lambda event: self.stop_auto_mode())
        root.protocol('WM_DELETE_WINDOW', self.on_closing)
        self.refresh_workspace()


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Atelier SLAM')
    parser.add_argument('--smoke-test', action='store_true', help='Fermer après vérification du démarrage, sans robot')
    args = parser.parse_args()
    from slam_components.run_logging import install_run_logging
    console_log = install_run_logging('console', os.path.join(os.path.dirname(__file__), 'logs', 'manager_runs'))
    os.environ['TB3_MANAGER_RUN_DIR'] = str(console_log.parent)
    from slam_components import instance
    lock = None
    if not args.smoke_test:
        lock, holder = instance.acquire()
        if lock is None:
            # Une autre fenêtre tourne déjà (ou a planté en laissant des processus) : on propose de la remplacer.
            from tkinter import messagebox
            asker = tk.Tk()
            asker.withdraw()
            replace = messagebox.askyesno(
                'Atelier SLAM déjà ouvert',
                'Une autre instance d’Atelier SLAM tourne déjà'+(f' (processus {holder})' if holder else '')+'.\n\n'
                'La remplacer ? Elle sera fermée avec tout ce qu’elle a lancé (nœuds, simulateur, caméra locale).')
            asker.destroy()
            if not replace:
                sys.exit(0)
            instance.stop_all()
            lock, holder = instance.acquire()
            if lock is None:
                print('[ATELIER] Impossible de prendre la place de l’instance existante.')
                sys.exit(3)
    root = tk.Tk()
    app = TB3Manager(root)
    signal.signal(signal.SIGINT, lambda *args: app.on_closing())
    signal.signal(signal.SIGTERM, lambda *args: app.on_closing())
    if args.smoke_test:
        root.after(800, app.on_closing)
    root.mainloop()
