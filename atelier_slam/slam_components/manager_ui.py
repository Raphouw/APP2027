"""Guided Tk/ttk workspace with one map, explicit states and local IPC."""
from pathlib import Path
import json
import math
import time
import uuid
import subprocess
import tkinter as tk
from tkinter import ttk, filedialog
import numpy as np
from PIL import Image
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from .map_format import load_map
from .session_ipc import SessionChannel
from .tk_text import tk_safe_text


class ManagerUIMixin:
    def build_workspace(self):
        style = ttk.Style(self.root)
        style.theme_use('clam')
        style.configure('.', font=('DejaVu Sans', 10), background='#f2f5f8', foreground='#18293d')
        style.configure('TButton', padding=(12, 9))
        style.configure('Title.TLabel', font=('DejaVu Sans', 22, 'bold'))
        style.configure('Subtitle.TLabel', foreground='#52667d')
        style.configure('Step.TLabel', font=('DejaVu Sans', 11, 'bold'), padding=(0, 9))
        style.configure('Primary.TButton', background='#146b8a', foreground='white')
        style.configure('Stop.TButton', background='#b52d3a', foreground='white', font=('DejaVu Sans', 11, 'bold'))
        self.root.configure(bg='#f2f5f8')
        header = ttk.Frame(self.root, padding=(24, 18))
        header.pack(fill='x')
        ttk.Label(header, text='Atelier SLAM', style='Title.TLabel').pack(side='left')
        ttk.Label(header, text='TurtleBot3  /  cartographie & navigation', style='Subtitle.TLabel').pack(side='left', padx=20)
        ttk.Button(header, text='ARRÊTER LE ROBOT  ·  Échap', style='Stop.TButton', command=self.stop_auto_mode).pack(side='right')
        body = ttk.Panedwindow(self.root, orient='horizontal')
        body.pack(fill='both', expand=True, padx=20)
        sidebar = ttk.Frame(body, padding=(8, 6), width=285)
        body.add(sidebar, weight=0)
        ttk.Label(sidebar, text='VOTRE PARCOURS', style='Subtitle.TLabel').pack(anchor='w', pady=(0, 10))
        self.step_labels = []
        for text in ('1  Préparer le robot', '2  Explorer et cartographier', '3  Vérifier la carte', '4  Choisir une destination'):
            label = ttk.Label(sidebar, text=text, style='Step.TLabel')
            label.pack(anchor='w')
            self.step_labels.append(label)
        self.guidance = ttk.Label(sidebar, wraplength=265, justify='left', text='')
        self.guidance.pack(fill='x', pady=(12, 16))
        self.btn_start = ttk.Button(sidebar, text='Nouvelle cartographie', style='Primary.TButton', command=self.start_live_mapping)
        self.btn_start.pack(fill='x', pady=4)
        self.btn_stop = ttk.Button(sidebar, text='Terminer et sauvegarder', command=self.stop_live_mapping)
        self.btn_stop.pack(fill='x', pady=4)
        self.btn_nav = ttk.Button(sidebar, text='Passer à la navigation', command=self.start_navigation)
        self.btn_nav.pack(fill='x', pady=4)
        ttk.Separator(sidebar).pack(fill='x', pady=16)
        self.sim_var = tk.BooleanVar(value=False)
        self.sim_check = ttk.Checkbutton(sidebar, text='Mode simulation (sans robot)', variable=self.sim_var,
                                         command=self.toggle_simulation)
        self.sim_check.pack(anchor='w', pady=(0, 8))
        self.open_button = ttk.Button(sidebar, text='Ouvrir une session…', command=self.open_session)
        self.open_button.pack(fill='x', pady=4)
        self.replay_button = ttk.Button(sidebar, text='Reconstruire un CSV…', command=self.replay_csv)
        self.replay_button.pack(fill='x', pady=4)
        self.progress = ttk.Progressbar(sidebar, mode='indeterminate')
        self.progress.pack(fill='x', pady=16)
        self.session_label = ttk.Label(sidebar, text='Aucune session active', style='Subtitle.TLabel', wraplength=265)
        self.session_label.pack(fill='x')
        self.book = ttk.Notebook(body)
        body.add(self.book, weight=1)
        map_tab = ttk.Frame(self.book, padding=12)
        self.book.add(map_tab, text='  Carte & destination  ')
        self.health_labels = {}
        health = ttk.Frame(map_tab)
        health.pack(fill='x', pady=(0, 8))
        for key, title in [('imu', 'IMU'), ('wheels', 'Roues'), ('lidar', 'LiDAR')]:
            label = ttk.Label(health, text=title+' · en attente', padding=8)
            label.pack(side='left', padx=(0, 8))
            self.health_labels[key] = label
        self.phase_label = ttk.Label(health, text='Prêt', font=('DejaVu Sans', 11, 'bold'))
        self.phase_label.pack(side='right', padx=8)
        self.figure = Figure(figsize=(8, 5), dpi=100, facecolor='#ffffff')
        self.ax = self.figure.add_subplot(111)
        self.figure.subplots_adjust(left=.08, right=.98, top=.95, bottom=.12)
        self.ax.set_facecolor('#edf2f6')
        self.ax.set_aspect('equal')
        self.ax.set_xlabel('x (m)')
        self.ax.set_ylabel('y (m)')
        self.empty_hint = self.ax.text(.5, .5, 'Votre carte apparaîtra ici\nDémarrez une session ou ouvrez une carte.',
                                       transform=self.ax.transAxes, ha='center', va='center', color='#52667d')
        self.map_artist = None
        self.path_artist, = self.ax.plot([], [], color='#1486a7', lw=1.8)
        self.robot_artist, = self.ax.plot([], [], 'o', color='#e46532', ms=8)
        self.goal_artist, = self.ax.plot([], [], '+', color='#b52d3a', ms=14, mew=2)
        self.canvas = FigureCanvasTkAgg(self.figure, map_tab)
        self.canvas.get_tk_widget().pack(fill='both', expand=True)
        self.toolbar = NavigationToolbar2Tk(self.canvas, map_tab, pack_toolbar=False)
        self.toolbar.pack(fill='x')
        self.canvas.mpl_connect('button_press_event', self.on_map_press)
        self.canvas.mpl_connect('button_release_event', self.on_map_release)
        actions = ttk.Frame(map_tab)
        actions.pack(fill='x', pady=(8, 0))
        self.click_mode = tk.StringVar(value='goal')
        ttk.Radiobutton(actions, text='Destination : cliquer', variable=self.click_mode, value='goal').pack(side='left')
        ttk.Radiobutton(actions, text='Replacer le robot : cliquer-glisser', variable=self.click_mode, value='pose').pack(side='left', padx=12)
        ttk.Button(actions, text='Centrer la carte', command=self.center_map).pack(side='right')
        self.metrics_label = ttk.Label(map_tab, text='Scans —   ·   Boucles —   ·   Recalage —', style='Subtitle.TLabel')
        self.metrics_label.pack(anchor='w', pady=(8, 0))
        journal_tab = ttk.Frame(self.book, padding=12)
        self.book.add(journal_tab, text='  Journal & aide  ')
        ttk.Label(journal_tab, justify='left', wraplength=750, text=(
            '1. Lancez le pilote ROS sur le robot et vérifiez ROS_DOMAIN_ID sur les deux machines.\n'
            '2. Démarrez une cartographie. Laissez le robot immobile pendant la calibration IMU.\n'
            '3. Quand les capteurs sont disponibles, cliquez dans la carte pour explorer.\n'
            '4. Terminez la carte, vérifiez les murs, puis passez à la navigation.\n'
            '5. Cliquez sur une zone libre pour choisir la destination. Échap arrête les processus.\n\n'
            'Replacer le robot : en navigation, sélectionnez cet outil puis tracez sa direction.\n'
            'Une carte ancienne nécessite une reconstruction CSV. Une session rouverte exige\n'
            'de replacer le robot : sa position physique a pu changer.\n'
            'Blanc : libre · gris : inconnu · noir : obstacle · orange : robot · bleu : trajet.\n'
            'La barre sous la carte permet de zoomer et de déplacer la vue.')).pack(anchor='w', pady=(0, 12))
        self.journal = tk.Text(journal_tab, height=12, wrap='word', bg='white', fg='#18293d', relief='flat', padx=12, pady=12)
        self.journal.pack(fill='both', expand=True)
        self.journal.configure(state='disabled')
        ttk.Button(journal_tab, text='Ouvrir les logs de ce run', command=self.open_debug_logs).pack(anchor='w', pady=8)
        ttk.Button(journal_tab, text='Ouvrir RViz (outils avancés)', command=self.open_rviz).pack(anchor='w', pady=8)
        if hasattr(self, 'build_robot_tab'):
            self.build_robot_tab()
        self.build_ros_tab()
        video_tab = ttk.Frame(self.book, padding=12)
        self.book.add(video_tab, text='  Caméra optionnelle  ')
        video_controls = ttk.Frame(video_tab)
        video_controls.pack(fill='x')
        ttk.Label(video_controls, text='Adresse du flux').pack(side='left')
        self.rtsp_entry = ttk.Entry(video_controls, width=36)
        self.rtsp_entry.insert(0, 'tcp://10.141.177.146:9999')
        self.rtsp_entry.pack(side='left', padx=8)
        self.btn_video = ttk.Button(video_controls, text='Connecter', command=self.start_video)
        self.btn_video.pack(side='left')
        ttk.Button(video_controls, text='Déconnecter', command=self.disconnect_video).pack(side='left', padx=8)
        self.video_label = tk.Label(video_tab, bg='#18293d', fg='white', text='La caméra est facultative pour le SLAM LiDAR.')
        self.video_label.pack(fill='both', expand=True, pady=12)
        self.info_label = tk.Label(self.root, text='Prêt. Démarrez une session ou ouvrez une carte.', bg='#e5edf4', fg='#18293d',
                                   anchor='w', padx=24, pady=12, wraplength=1150)
        self.info_label.pack(side='bottom', fill='x', pady=(12, 0))
        body.pack_configure(after=self.info_label)
        self.auto_info = None
        self._preview_key = None
        self._preview_extent = None
        self._press = None
        self._busy_animation = False
        self._require_pose = False
        self._pending_pose_id = None
        self._last_health = {}
        self._last_navigation = {}
        self._last_slam = {}

    def reset_ui(self):
        # State is derived from supervised processes in refresh_workspace.
        pass

    def disconnect_video(self):
        self.video_enabled = False
        if self.video_reader:
            self.video_reader.stop()
        self.video_reader = None
        self.btn_video.config(state='normal', text='Connecter')
        self.video_label.config(image='', text='Caméra déconnectée')
        self.video_label.image = None

    def _read_status(self, name, token=False):
        if not self.session_dir:
            return {}
        try:
            data = json.loads((self.session_dir/name).read_text())
            if not 0 <= time.time()-data['updated_at'] < 2:
                return {}
            if name == 'slam_status.json' and data.get('session_id') != self.session_id:
                return {}
            if token and data.get('token') != getattr(self, 'ipc_token', None):
                return {}
            return data
        except (OSError, ValueError, KeyError, TypeError):
            return {}

    def refresh_workspace(self):
        if self._closing:
            return
        alive = lambda process: process is not None and process.poll() is None
        mapping = alive(self.carto_proc)
        navigating = alive(self.odom_proc)
        active = any(alive(p) for p in (self.nav_proc, self.odom_proc, self.carto_proc))
        health = self._read_status('robot_status.json') if active else {}
        nav = self._read_status('navigation_status.json', token=True) if navigating else {}
        slam = self._read_status('slam_status.json')
        self._last_health, self._last_navigation = health, nav
        if self._pending_pose_id and nav.get('pose_command_id') == self._pending_pose_id:
            self._pending_pose_id = None
            self._require_pose = False
            self.click_mode.set('goal')
            self._status('Position du robot mise à jour. Choisissez une destination.')
        if slam:
            self._last_slam = slam
        ready = bool(health.get('calibrated')) and all(health.get(k) for k in ('imu', 'wheels', 'lidar'))
        for key, label in self.health_labels.items():
            value = health.get(key, False)
            suffix = ' · disponible' if value else ' · en attente'
            if key == 'lidar' and health.get('lidar_front_unknown', 0):
                suffix = ' · zone avant inconnue'
            label.config(text={'imu':'IMU','wheels':'Roues','lidar':'LiDAR'}[key]+suffix,
                         foreground='#14704b' if value else '#846332')
        self.refresh_ros_panel()
        phase = 'Prêt'
        guidance = 'Démarrez une nouvelle carte ou ouvrez une session existante. Aucun mouvement au démarrage.'
        step = 0
        if self._busy:
            phase, guidance = 'Traitement en cours', 'Patientez pendant la sauvegarde ou le lancement. Échap reste disponible.'
        elif navigating:
            phase, step = 'Navigation', 3
            guidance = 'Cliquez sur une zone libre pour choisir la destination.'
            if self._require_pose:
                guidance = 'Session rouverte : replacez le robot dans la carte avec l’outil cliquer-glisser.'
            elif not ready or nav.get('state') == 'WAITING_SENSORS':
                guidance = 'Le robot attend des mesures fraîches. Les destinations sont désactivées.'
            elif nav.get('state') == 'NO_PATH':
                guidance = 'Aucun chemin libre. Choisissez une autre destination.'
            elif nav.get('state') == 'OBSTACLE_STOP':
                guidance = 'Obstacle proche : robot arrêté. Dégagez son passage.'
        elif mapping:
            phase, step = 'Cartographie', 1
            guidance = 'Cliquez dans la carte pour explorer. Repassez par un lieu connu pour rechercher une boucle.'
            if not ready:
                if self._ros_level == 'warn' and not health.get('calibrated') and not health.get('imu'):
                    phase = 'Robot non détecté'
                    guidance = 'Aucun capteur ne publie sur ROS. Consultez l’onglet « Connexion ROS » : pilote du robot lancé ? même ROS_DOMAIN_ID ?'
                elif not health.get('calibrated'):
                    phase = 'Calibration IMU'
                    guidance = 'Gardez le robot immobile pendant la calibration IMU.'
                elif health.get('lidar_front_unknown', 0):
                    phase = 'Mesures LiDAR insuffisantes'
                    guidance = 'Calibration terminée. Le robot attend des mesures fiables devant lui. Consultez le diagnostic LiDAR si cet état persiste.'
                else:
                    phase = 'Capteur interrompu'
                    guidance = 'Calibration terminée. Vérifiez le capteur signalé en attente.'
            elif health.get('state') == 'GOAL_REACHED':
                phase = 'Destination atteinte'
                guidance = 'Destination atteinte : arrêt normal. Cliquez pour choisir une nouvelle destination.' 
        elif self.map_yaml:
            phase, step = 'Carte prête', 2
            guidance = 'Vérifiez la continuité des murs et les zones libres, puis passez à la navigation.'
        self.phase_label.config(text=phase)
        self.guidance.config(text=guidance)
        for i, label in enumerate(self.step_labels):
            label.config(foreground='#146b8a' if i == step else '#52667d')
        self.btn_start.config(state='disabled' if self._busy or active else 'normal')
        self.btn_stop.config(state='normal' if mapping and not self._busy else 'disabled')
        self.btn_nav.config(state='normal' if self.map_yaml and not active and not self._busy else 'disabled')
        for button in (self.open_button, self.replay_button):
            button.config(state='disabled' if active or self._busy else 'normal')
        if self._busy != self._busy_animation:
            self.progress.start(12) if self._busy else self.progress.stop()
            self._busy_animation = self._busy
        if getattr(self, '_display_session', None) != self.session_id:
            self._display_session = self.session_id
            self._preview_key = None
            self._preview_extent = None
            self.empty_hint.set_visible(True)
            if self.map_artist is not None:
                self.map_artist.set_visible(False)
            for artist in (self.path_artist, self.robot_artist, self.goal_artist):
                artist.set_data([], [])
        if self.session_dir:
            self.session_label.config(text='Session\n'+tk_safe_text(self.session_dir.name))
        try:
            image_path = self.session_dir/'preview.png' if mapping and self.session_dir else None
            if image_path and image_path.exists() and slam:
                key = (str(image_path), image_path.stat().st_mtime_ns)
                if key != self._preview_key:
                    pixels = np.asarray(Image.open(image_path).convert('L'))
                    origin, res = slam['origin'], slam['resolution']
                    self.show_map(pixels, origin, res)
                    self._preview_key = key
            elif self.map_yaml and self._preview_key != self.map_yaml:
                meta, pixels, _ = load_map(self.map_yaml)
                self.show_map(np.flipud(pixels), meta['origin'], meta['resolution'])
                self._preview_key = self.map_yaml
            data = nav or (slam if mapping else {})
            if data.get('pose'):
                self.robot_artist.set_data([data['pose'][0]], [data['pose'][1]])
            if data.get('path') is not None:
                points = np.asarray(data['path'])
                self.path_artist.set_data(points[:, 0] if len(points) else [], points[:, 1] if len(points) else [])
            if 'goal' in data:
                goal = data['goal']
                self.goal_artist.set_data([goal[0]] if goal else [], [goal[1]] if goal else [])
            quality = self._last_slam
            rmse = quality.get('rmse')
            self.metrics_label.config(text=f"Scans {quality.get('scans', '—')}   ·   Boucles {quality.get('loops', '—')}   ·   "
                                      f"Recalage {quality.get('match_reason', '—')}" + (f' ({rmse:.3f} m)' if rmse is not None else ''))
            self.canvas.draw_idle()
        except (OSError, ValueError, KeyError):
            pass  # Atomic writer may have replaced the file between stat and read.
        self.root.after(350, self.refresh_workspace)

    def show_map(self, pixels, origin, res):
        previous_view = getattr(self, '_map_view_extent', None)
        h, w = pixels.shape
        extent = [origin[0], origin[0]+w*res, origin[1], origin[1]+h*res]
        self.empty_hint.set_visible(False)
        if self.map_artist is not None:
            self.map_artist.set_visible(True)
        if self.map_artist is None:
            self.map_artist = self.ax.imshow(pixels, cmap='gray', vmin=0, vmax=255, origin='upper', extent=extent, zorder=0)
            self.ax.set_xlim(extent[:2]); self.ax.set_ylim(extent[2:])
        else:
            self.map_artist.set_data(pixels)
            self.map_artist.set_extent(extent)
        self._preview_extent = extent
        rows, columns = np.where(np.abs(pixels.astype(float)-128) > 2)
        if len(rows):
            self._map_view_extent = [origin[0]+max(0, columns.min()-8)*res,
                                     origin[0]+min(w, columns.max()+9)*res,
                                     origin[1]+max(0, h-rows.max()-9)*res,
                                     origin[1]+min(h, h-rows.min()+8)*res]
        else:
            self._map_view_extent = extent
        if getattr(self, '_map_session', None) != self.session_id:
            self._map_session = self.session_id
            self.center_map()
        elif previous_view is not None:
            view = self._map_view_extent
            grew = (view[0] < previous_view[0] or view[1] > previous_view[1]
                    or view[2] < previous_view[2] or view[3] > previous_view[3])
            if grew:
                # Preserve the existing view and include newly explored ground.
                xmin, xmax = sorted(self.ax.get_xlim())
                ymin, ymax = sorted(self.ax.get_ylim())
                self.ax.set_xlim(min(xmin, view[0]), max(xmax, view[1]))
                self.ax.set_ylim(min(ymin, view[2]), max(ymax, view[3]))
                self.canvas.draw_idle()

    def center_map(self):
        if self._preview_extent:
            self.ax.set_xlim(self._map_view_extent[:2]); self.ax.set_ylim(self._map_view_extent[2:])
            self.canvas.draw_idle()

    def on_map_press(self, event):
        if event.inaxes != self.ax or event.button != 1 or self.toolbar.mode or self._busy:
            return
        if not all(math.isfinite(v) for v in (event.xdata, event.ydata)):
            return
        self._press = (event.xdata, event.ydata)

    def on_map_release(self, event):
        start, self._press = self._press, None
        if start is None or event.inaxes != self.ax or self.toolbar.mode or self._busy:
            return
        navigating = self.odom_proc is not None and self.odom_proc.poll() is None
        mapping = self.carto_proc is not None and self.carto_proc.poll() is None
        if not (mapping or navigating):
            self._status('Démarrez la cartographie ou la navigation avant de choisir une destination.')
            return
        if navigating and self.click_mode.get() == 'pose':
            yaw = math.atan2(event.ydata-start[1], event.xdata-start[0])
            self._pending_pose_id = self._channel('navigation').send('pose', [*start, yaw])
            self._require_pose = True
            self._status('Repositionnement envoyé. Attendez sa prise en compte avant une destination.')
            return
        if mapping and not self._read_status('slam_status.json').get('scans', 0):
            self._status('Attendez le premier scan synchronisé avant de choisir une destination.')
            return
        if self._require_pose or self._pending_pose_id or not self._last_health.get('calibrated') or not all(self._last_health.get(k) for k in ('imu', 'wheels', 'lidar')):
            self._status('Destination indisponible : vérifier la pose et attendre les capteurs.')
            return
        if self._preview_extent and not (self._preview_extent[0] <= start[0] < self._preview_extent[1] and self._preview_extent[2] <= start[1] < self._preview_extent[3]):
            self._status('Choisissez une destination dans la carte.')
            return
        self._channel('navigation' if navigating else 'mapping').send('goal', start)
        self.goal_artist.set_data([start[0]], [start[1]])
        self._status(f'Destination envoyée : x={start[0]:.2f} m, y={start[1]:.2f} m.')

    def _channel(self, role):
        return SessionChannel(self.session_dir, self.ipc_token, role)

    def open_session(self):
        selected = filedialog.askopenfilename(title='Ouvrir une carte de session', initialdir=self.logs_dir, filetypes=[('Carte YAML', '*.yaml')])
        if not selected:
            return
        try:
            meta, _, _ = load_map(selected)
            self.session_dir = Path(selected).parent
            self.session_id = meta.get('session_id')
            if not self.session_id or not (self.session_dir/'session.json').exists():
                raise ValueError('Ouvrez une carte appartenant à une session, ou reconstruisez son CSV.')
            self.map_yaml = str(Path(selected).resolve())
            self._require_pose = True
            self._last_slam = dict(meta.get('slam', {}), scans=meta.get('scans', '—'))
            self._preview_key = None
            self._status('Session ouverte. Replacez le robot après le lancement de la navigation.')
        except (OSError, ValueError, KeyError) as exc:
            self.map_yaml = None
            self._status(f'Ouverture impossible : {exc}')

    def replay_csv(self):
        selected = filedialog.askopenfilename(title='Reconstruire une acquisition', initialdir=self.logs_dir, filetypes=[('Scans CSV', '*.csv')])
        if not selected:
            return
        self._job(lambda: self._replay_session(selected), lambda result: self._status('Reconstruction terminée. Carte prête à vérifier.'))

    def open_debug_logs(self):
        directory = self.session_dir/'debug' if self.session_dir else self.debug_dir
        directory.mkdir(parents=True, exist_ok=True)
        try:
            subprocess.Popen(['xdg-open', str(directory)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self._status('Logs : '+str(directory)+' · Manager : '+str(self.debug_dir/'manager.log'))
        except OSError as exc:
            self._status(f'Logs disponibles dans {directory} ({exc})')

    def open_rviz(self):
        if not self._busy and (self.rviz_proc is None or self.rviz_proc.poll() is not None):
            self._job(lambda: setattr(self, 'rviz_proc', self._launch(['rviz2', '-d', self.rviz_config_path])),
                      lambda result: self._status('RViz ouvert. La carte principale reste dans cet atelier.'))
