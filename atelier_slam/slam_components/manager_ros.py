"""Panneau « Connexion ROS » : topics détectés et état du lien avec le robot.

Le graphe ROS est observé par `sonde_ros.py` (processus séparé) qui écrit
logs/ros_graph.json ; l'interface ne fait que le lire.
"""
from pathlib import Path
import json
import time
from tkinter import ttk

# Capteurs requis par la cartographie : topic -> libellé.
SENSOR_TOPICS = {'/imu': 'IMU', '/joint_states': 'Roues', '/scan': 'LiDAR'}


def read_ros_graph(path):
    """Contenu récent de la sonde, ou None si elle est absente ou figée."""
    try:
        data = json.loads(Path(path).read_text())
        if 0 <= time.time()-data['updated_at'] < 4:
            return data
    except (OSError, ValueError, KeyError, TypeError):
        pass
    return None


def connection_summary(graph):
    """(niveau, texte) : 'ok', 'warn' ou 'wait' selon les publieurs des capteurs."""
    if graph is None:
        return 'wait', 'Sonde ROS inactive : graphe ROS non observé (ROS 2 sourcé ?).'
    publishers = {t['name']: t['publishers'] for t in graph['topics']}
    missing = [label for topic, label in SENSOR_TOPICS.items() if not publishers.get(topic)]
    if not missing:
        return 'ok', 'Robot connecté : IMU, roues et LiDAR sont publiés.'
    if len(missing) == len(SENSOR_TOPICS):
        return 'warn', ('Aucun robot détecté : personne ne publie /imu, /joint_states ni /scan. '
                        'Lancez le pilote du robot (ou le mode simulation) et vérifiez que '
                        f'ROS_DOMAIN_ID ({graph["domain_id"]}) est identique sur les deux machines.')
    return 'warn', 'Publication incomplète : '+', '.join(missing)+' sans publieur.'


class ManagerRosMixin:
    def build_ros_tab(self):
        tab = ttk.Frame(self.book, padding=12)
        self.book.add(tab, text='  Connexion ROS  ')
        self.ros_summary = ttk.Label(tab, text='Sonde ROS : démarrage…', font=('DejaVu Sans', 11, 'bold'),
                                     wraplength=800, justify='left')
        self.ros_summary.pack(anchor='w', pady=(0, 4))
        self.ros_env = ttk.Label(tab, text='', style='Subtitle.TLabel')
        self.ros_env.pack(anchor='w', pady=(0, 8))
        columns = ('topic', 'type', 'pub', 'sub')
        self.ros_tree = ttk.Treeview(tab, columns=columns, show='headings', height=14)
        for column, title, width in (('topic', 'Topic', 260), ('type', 'Type', 280),
                                     ('pub', 'Publieurs', 80), ('sub', 'Abonnés', 80)):
            self.ros_tree.heading(column, text=title)
            self.ros_tree.column(column, width=width, anchor='w' if column in ('topic', 'type') else 'center')
        self.ros_tree.tag_configure('sensor_ok', foreground='#14704b')
        self.ros_tree.tag_configure('sensor_missing', foreground='#b52d3a')
        self.ros_tree.pack(fill='both', expand=True)
        self._ros_level = 'wait'

    def start_ros_probe(self):
        if getattr(self, 'ros_proc', None) is None or self.ros_proc.poll() is not None:
            try:
                self.ros_graph_path = Path(self.logs_dir)/'ros_graph.json'
                self.ros_proc = self._launch(['python3', 'sonde_ros.py', '--output', str(self.ros_graph_path)])
            except Exception as exc:
                self.ros_proc = None
                self.logger.error('Sonde ROS non démarrée : %s', exc)

    def stop_ros_probe(self):
        process = getattr(self, 'ros_proc', None)
        if process is not None and process.poll() is None:
            process.terminate()
        self.ros_proc = None

    def refresh_ros_panel(self):
        graph = read_ros_graph(self.ros_graph_path) if getattr(self, 'ros_proc', None) is not None else None
        level, text = connection_summary(graph)
        self._ros_level = level
        self.ros_summary.config(text=text, foreground={'ok': '#14704b', 'warn': '#b52d3a'}.get(level, '#846332'))
        self.ros_env.config(text='' if graph is None else
                            f'ROS_DOMAIN_ID : {graph["domain_id"]}   ·   RMW : {graph["rmw"]}   ·   '
                            f'ROS_LOCALHOST_ONLY : {graph["localhost_only"]}   ·   {len(graph["topics"])} topics')
        rows = [] if graph is None else [
            (t['name'], ', '.join(t['types']), t['publishers'], t['subscribers'],
             ('sensor_ok' if t['publishers'] else 'sensor_missing') if t['name'] in SENSOR_TOPICS else '')
            for t in graph['topics']]
        if rows != getattr(self, '_ros_rows', None):
            self._ros_rows = rows
            self.ros_tree.delete(*self.ros_tree.get_children())
            for *values, tag in rows:
                self.ros_tree.insert('', 'end', values=values, tags=(tag,) if tag else ())
