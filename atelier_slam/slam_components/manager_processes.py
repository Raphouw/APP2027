"""Sessions explicites et orchestration asynchrone des processus supervisés."""
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from pathlib import Path
import json
import logging
import shutil
import sys
import os
import signal
import subprocess
import uuid
import tkinter as tk
from .map_format import load_map
from .tk_text import tk_safe_text


class ManagerProcessesMixin:
    def initialize_supervision(self):
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='tb3-supervision')
        self.debug_run_id = datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8]
        self.debug_dir = Path(os.environ.get('TB3_MANAGER_RUN_DIR', Path(self.logs_dir)/'manager_runs'/self.debug_run_id))
        self.debug_run_id = self.debug_dir.name
        self.debug_dir.mkdir(parents=True, exist_ok=True)
        self.logger = logging.getLogger('tb3.manager.'+self.debug_run_id)
        self.logger.setLevel(logging.DEBUG)
        handler = logging.FileHandler(self.debug_dir/'manager.log', encoding='utf-8')
        handler.setFormatter(logging.Formatter('%(asctime)s %(levelname)s [%(threadName)s] %(message)s'))
        self.logger.addHandler(handler)
        self.logger.propagate = False
        self.root.report_callback_exception = self._callback_error
        self.logger.info('Manager started: %s', self.debug_run_id)
        self._busy = False
        self._closing = False
        self._cancel_requested = False
        self._operation_id = 0
        self.session_dir = self.session_id = self.map_yaml = None
        self.auto_mode_active = False
        self.auto_phase = 'idle'
        sim = getattr(self, 'sim_proc', None)
        if sim is not None and sim.poll() is not None and not self._busy:
            self.sim_proc = None
            self.sim_var.set(False)
            self._status('Le simulateur s’est arrêté. Voir les logs du run.')
        self.root.after(500, self.check_auto_completion)

    def _callback_error(self, kind, value, tb):
        self.logger.error('Tk callback failed', exc_info=(kind, value, tb))
        self._status(f'Erreur interface : {value}. Voir les logs du run.')

    def _status(self, text):
        if hasattr(self, 'logger'):
            self.logger.info(text)
        journal = getattr(self, 'journal', None)
        if journal is not None:
            journal.configure(state='normal')
            journal.insert('end', datetime.now().strftime('%H:%M:%S')+'  '+tk_safe_text(text)+'\n')
            journal.see('end')
            journal.configure(state='disabled')
        for name in ('info_label', 'auto_info'):
            label = getattr(self, name, None)
            if label is not None:
                label.config(text=tk_safe_text(text))

    def _job(self, work, completed=None):
        if self._busy or self._closing:
            return
        self._busy = True
        self._operation_id += 1
        operation_id = self._operation_id
        self._status('Opération en cours…')
        def guarded():
            try:
                return work()
            except Exception:
                try:
                    self._stop_all()
                finally:
                    if self.session_dir is not None:
                        self._manifest('failed')
                raise
        future = self.executor.submit(guarded)
        def poll():
            if not future.done():
                self.root.after(100, poll)
                return
            self._busy = False
            if self._closing or operation_id != self._operation_id:
                return
            try:
                result = future.result()
            except Exception as exc:
                if hasattr(self, 'logger'):
                    self.logger.error('Operation failed', exc_info=(type(exc), exc, exc.__traceback__))
                self._status(f'Erreur : {exc}. Ouvrez les logs du run pour le détail.')
                self.auto_mode_active = False
                self.reset_ui()

            else:
                if completed is not None and not self._closing:
                    completed(result)
        self.root.after(100, poll)

    def _stop_process(self, name):
        process = getattr(self, name, None)
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
                setattr(self, name, None)
                raise RuntimeError(f'{name} a dû être arrêté de force ; résultat non validé.')
        if process is not None and hasattr(self, 'logger'):
            self.logger.info('Stopped %s pid=%s exit=%s', name, process.pid, process.returncode)
        setattr(self, name, None)
        return process.returncode if process is not None else None

    def _stop_all(self):
        # A restarted map controller must not silently discard manual alignment.
        if getattr(self, 'odom_proc', None) is not None:
            self._require_pose = True
        failure = None
        carto_code = None
        mapping_was_active = getattr(self, 'carto_proc', None) is not None
        for name in ('odom_proc', 'carto_proc', 'nav_proc', 'rviz_proc'):
            try:
                code = self._stop_process(name)
                if name == 'carto_proc':
                    carto_code = code
            except Exception as exc:
                failure = exc
        saved = False
        if mapping_was_active and carto_code == 0 and failure is None and self.session_dir is not None:
            # Arrêt d'urgence (Échap / bouton rouge) pendant la cartographie : le cartographe
            # a écrit sa carte en quittant, on la valide pour pouvoir naviguer ensuite.
            try:
                self._validate_session_map()
                saved = True
            except Exception as exc:
                if hasattr(self, 'logger'):
                    self.logger.warning('Carte non récupérée après arrêt : %s', exc)
        if self.session_dir is not None and not saved:
            self._manifest('failed' if failure else 'stopped')
        if failure:
            raise failure

    def _manifest(self, state):
        path = self.session_dir/'session.json'
        tmp = path.with_suffix('.tmp')
        tmp.write_text(json.dumps({'session_id': self.session_id, 'state': state,
                                  'csv': str(self.session_dir/'mapping.csv'),
                                  'map': str(self.session_dir/'map_out.yaml'),
                                  'manager_run_id': getattr(self, 'debug_run_id', None),
                                  'manager_log': str(self.debug_dir/'manager.log') if hasattr(self, 'debug_dir') else None,
                                  'launcher_logs': os.environ.get('TB3_LAUNCH_LOG_DIR')}, indent=2))
        os.replace(tmp, path)

    def _new_session(self, live):
        self._stop_all()
        self.session_id = datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8]
        self.session_dir = Path(self.logs_dir)/'sessions'/self.session_id
        self.session_dir.mkdir(parents=True)
        self.map_yaml = None
        self.ipc_token = uuid.uuid4().hex
        self._require_pose = False
        self._last_slam = {}
        self._preview_key = None
        self._manifest('mapping')
        self.nav_proc = self._launch(['python3', 'noeud_odometrie.py', '--ros-args',
                                    '-p', f'log_directory:={self.session_dir}', '-p', 'log_filename:=mapping.csv'])

        if live:
            self.carto_proc = self._launch(self._carto_command('live'))

    def _launch(self, command):
        if self._closing or self._cancel_requested:
            raise RuntimeError('Opération annulée.')
        log_dir = (self.session_dir/'debug') if self.session_dir is not None else self.debug_dir
        log_dir.mkdir(parents=True, exist_ok=True)
        name = Path(command[1] if len(command) > 1 and 'python' in Path(command[0]).name else command[0]).stem
        launch_id = datetime.now().strftime('%H%M%S_%f')+'_'+uuid.uuid4().hex[:6]
        log_path = log_dir/(name+'_'+launch_id+'.log')
        env = dict(os.environ, PYTHONUNBUFFERED='1', PYTHONFAULTHANDLER='1',
                   TB3_MANAGED_LOG=str(log_path), ROS_LOG_DIR=str(log_dir/'ros'/launch_id))
        actual_command = list(command)
        if actual_command[0] == 'python3':
            actual_command[0] = sys.executable
        with log_path.open('x', encoding='utf-8') as stream:
            stream.write(json.dumps(dict(time=datetime.now().isoformat(), command=actual_command,
                                        cwd=self.base_dir, manager_run_id=getattr(self, 'debug_run_id', None)))+'\n')
            stream.flush()
            process = subprocess.Popen(actual_command, cwd=self.base_dir, env=env,
                                       stdout=stream, stderr=subprocess.STDOUT)
        if hasattr(self, 'logger'):
            self.logger.info('Started pid=%s command=%r log=%s', process.pid, actual_command, log_path)
        return process

    def _carto_command(self, mode):
        return ['python3', 'cartographe.py', '-m', mode, '-i', str(self.session_dir/'mapping.csv'),
                '--auto', '--headless', '--token', getattr(self, 'ipc_token', ''), '--output-dir', str(self.session_dir), '--session-id', self.session_id,
                '-oname', 'out']

    def _validate_session_map(self):
        path = Path(self.map_yaml) if getattr(self, 'map_yaml', None) else self.session_dir/'map_out.yaml'
        meta, _, _ = load_map(path)
        if meta.get('session_id') != self.session_id or meta.get('source_csv') != str((self.session_dir/'mapping.csv').resolve()):
            raise ValueError('La carte ne correspond pas à la session active.')
        self.map_yaml = str(path)
        self._last_slam = dict(meta.get('slam', {}), scans=meta.get('scans', '—'))
        self._manifest('map_ready')
        return self.map_yaml

    def _replay_session(self, csv_path):
        self._stop_all()
        self.session_id = datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8]
        self.session_dir = Path(self.logs_dir)/'sessions'/self.session_id
        self.session_dir.mkdir(parents=True)
        self.ipc_token = uuid.uuid4().hex
        self.map_yaml = None
        self._require_pose = True
        self._last_slam = {}
        self._preview_key = None
        shutil.copyfile(csv_path, self.session_dir/'mapping.csv')
        self._manifest('reconstructing')
        self.carto_proc = self._launch(self._carto_command('offline'))
        code = self.carto_proc.wait(timeout=300)
        self.carto_proc = None
        if code:
            raise RuntimeError('Reconstruction échouée. Consultez les logs cartographe du run.')
        return self._validate_session_map()

    def start_mapping(self):
        def done(result):
            self.btn_start.config(state=tk.DISABLED)
            self.btn_stop.config(state=tk.NORMAL)
            self.btn_nav.config(state=tk.DISABLED)
            self._status('Acquisition active — attendre la calibration IMU, robot immobile.')
        self._job(lambda: self._new_session(False), done)

    def stop_and_generate(self):
        def work():
            if self.session_dir is None:
                raise ValueError('Aucune session active.')
            self._stop_process('nav_proc')
            self._stop_process('rviz_proc')
            self.carto_proc = self._launch(self._carto_command('offline'))
            code = self.carto_proc.wait(timeout=300)
            if code:
                raise RuntimeError('La reconstruction a échoué ; consulter le terminal.')
            return self._validate_session_map()
        def done(path):
            self.btn_stop.config(state=tk.DISABLED)
            self.btn_nav.config(state=tk.NORMAL)
            self._status('Carte de la session prête.')
        self._job(work, done)

    def _start_map_navigation(self, yaml_path):
        self._stop_process('odom_proc')
        self._stop_process('nav_proc')
        self.nav_proc = self._launch(['python3', 'noeud_odometrie.py', '--ros-args',
                                     '-p', 'odometry_only:=true', '-p', 'use_saved_pose:=true',
                                     '-p', f'log_directory:={self.session_dir}',
                                     '-p', 'log_filename:=navigation_'+uuid.uuid4().hex[:8]+'.csv'])
        self.ipc_token = uuid.uuid4().hex
        self.odom_proc = self._launch(['python3', 'navigation_carte.py', yaml_path, '--headless',
                                       '--ipc-dir', str(self.session_dir), '--token', self.ipc_token])
        self._manifest('navigating')

    def start_navigation(self):
        def work():
            if not self.map_yaml:
                raise ValueError('Générer la carte de la session avant la navigation.')
            self._validate_session_map()
            self._start_map_navigation(self.map_yaml)
        self._job(work, lambda result: self._status('Navigation prête : choisir une destination sur la carte.'))

    def toggle_simulation(self):
        """Démarre ou arrête le robot simulé (sans Gazebo), seulement hors session."""
        def alive(process):
            return process is not None and process.poll() is None
        running = alive(getattr(self, 'sim_proc', None))
        wanted = self.sim_var.get()
        if wanted == running:
            return
        if self._busy or any(alive(getattr(self, name, None)) for name in ('nav_proc', 'odom_proc', 'carto_proc')):
            self.sim_var.set(running)
            self._status('Terminez la session en cours avant de changer le mode simulation.')
            return
        def work():
            if wanted:
                truth = Path(self.logs_dir)/'simulation'/('verite_terrain_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'.csv')
                self.sim_proc = self._launch(['python3', 'simulateur.py', '--truth-csv', str(truth)])
            else:
                self._stop_process('sim_proc')
        self._job(work, lambda result: self._status(
            'Simulation active : un robot virtuel remplace le robot réel.' if wanted else 'Simulation arrêtée.'))

    def start_live_mapping(self):
        self.auto_mode_active = False
        self._job(lambda: self._new_session(True), lambda result: self._status('Cartographie live active.'))

    def stop_live_mapping(self):
        self.auto_mode_active = False
        def work():
            self._stop_process('nav_proc')
            code = self._stop_process('carto_proc')
            self._stop_process('rviz_proc')
            if code != 0:
                raise RuntimeError('Le cartographe ne s’est pas arrêté correctement.')
            return self._validate_session_map()
        self._job(work, lambda result: self._status('Carte sauvegardée. Vérifiez-la avant de passer à la navigation.'))

    def start_auto_mode(self):
        def done(result):
            self.auto_mode_active = True
            self.auto_phase = 'mapping'
            self._status('Cartographie active. Terminer la carte pour passer à la navigation.')
        self._job(lambda: self._new_session(True), done)

    def finish_auto_mapping(self):
        if not self.auto_mode_active or self.auto_phase != 'mapping':
            return
        def work():
            self._stop_process('nav_proc')
            code = self._stop_process('carto_proc')
            if code != 0:
                raise RuntimeError('Cartographie interrompue en erreur.')
            path = self._validate_session_map()
            self._start_map_navigation(path)
        def done(result):
            self.auto_phase = 'navigating'
            self._status('Navigation prête : choisir une destination sur la carte.')
        self._job(work, done)

    def trigger_auto_navigation(self):
        self.finish_auto_mapping()

    def check_auto_completion(self):
        if self._closing:
            return
        if not self._busy:
            if self.auto_mode_active and self.auto_phase == 'mapping' and self.carto_proc is not None and self.carto_proc.poll() is not None:
                self.finish_auto_mapping()
            elif self.carto_proc is not None and self.carto_proc.poll() is not None:
                self._job(self._stop_all, lambda result: self._status('Le cartographe s’est arrêté. Session interrompue ; consulter les logs.'))
            elif any(process is not None and process.poll() is not None for process in (self.nav_proc, self.odom_proc)):
                self.auto_mode_active = False
                self._job(self._stop_all, lambda result: self._status('Un contrôleur s’est arrêté. Les autres processus ont été arrêtés.'))
        self.root.after(500, self.check_auto_completion)

    def stop_auto_mode(self):
        self.auto_mode_active = False
        self.auto_phase = 'idle'
        if self._closing:
            return
        # Ne pas ignorer STOP lorsqu'une opération est en cours : l'arrêt est mis
        # en file dans le worker et les contrôleurs actifs reçoivent SIGINT tout de suite.
        if self._busy:
            self._operation_id += 1
            self._cancel_requested = True
            for name in ('odom_proc', 'nav_proc', 'carto_proc'):
                process = getattr(self, name, None)
                if process is not None and process.poll() is None:
                    process.send_signal(signal.SIGINT)
            def stop_pending():
                try:
                    self._stop_all()
                finally:
                    self._cancel_requested = False
            future = self.executor.submit(stop_pending)
            def poll_stop():
                if not future.done():
                    self.root.after(100, poll_stop)
                    return
                if not self._closing:
                    self._status('Arrêté.' if future.exception() is None else f'Erreur d’arrêt : {future.exception()}')
            self.root.after(100, poll_stop)
            self._status('Arrêt demandé.')
        else:
            self._job(self._stop_all, lambda result: self._status('Arrêté.'))

    def on_closing(self):
        if self._closing:
            return
        self._closing = True
        self.auto_mode_active = False
        self.stop_ros_probe()
        for name in ('odom_proc', 'carto_proc', 'nav_proc'):
            process = getattr(self, name, None)
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGINT)
        # Le worker unique attend l'opération en cours puis ferme tous les processus.
        sim = getattr(self, 'sim_proc', None)
        if sim is not None and sim.poll() is None:
            sim.send_signal(signal.SIGINT)
        def work():
            try:
                self._stop_process('sim_proc')
                self._stop_all()
            finally:
                if self.video_reader is not None:
                    self.video_reader.stop()
        future = self.executor.submit(work)
        def poll():
            if not future.done():
                self.root.after(100, poll)
                return
            try:
                future.result()
            except Exception as exc:
                print(f'[STOP] {exc}')
            self.executor.shutdown(wait=False)
            self.root.destroy()
        self.root.after(100, poll)

    def create_rviz_config(self):
        """Génère un fichier de configuration RViz2 sur mesure si inexistant."""
        if not os.path.exists(self.rviz_config_path):
            print("[MANAGER] Création du fichier de configuration RViz par défaut...")
            rviz_yaml = """Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Value: true
    - Class: rviz_default_plugins/TF
      Name: TF
      Value: true
    - Class: rviz_default_plugins/LaserScan
      Name: LaserScan
      Topic:
        Depth: 5
        Reliability Policy: Best Effort
        Value: /scan
      Color: 255; 0; 0
      Size (m): 0.05
      Value: true
      Style: Points
    - Class: rviz_default_plugins/Path
      Name: Path
      Topic:
        Value: /path
      Color: 0; 255; 0
      Value: true
    - Class: rviz_default_plugins/Odometry
      Name: Odometry
      Topic:
        Value: /my_odom
      Value: true
  Global Options:
    Fixed Frame: my_odom
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Value: /goal_pose
Window Geometry:
  Height: 800
  Width: 1000
"""
            os.makedirs(os.path.dirname(self.rviz_config_path), exist_ok=True)
            with open(self.rviz_config_path, 'w') as f:
                f.write(rviz_yaml)
