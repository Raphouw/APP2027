"""Connexion SSH au robot et lancement du bringup TurtleBot3, sans ligne de commande.

Aucun mot de passe n'est écrit sur disque : il reste en mémoire le temps de la connexion.
Les réglages sans secret (hôte, utilisateur, clé, domaine ROS…) sont gardés dans
~/.config/atelier_slam/robot.json, hors du dépôt.
"""
import json
import os
import queue
import re
import shlex
import socket
import threading
import time
from pathlib import Path

CONFIG_PATH = Path(os.environ.get('ATELIER_CONFIG_DIR', Path.home()/'.config'/'atelier_slam'))/'robot.json'
KNOWN_HOSTS = Path.home()/'.ssh'/'known_hosts'
DEFAULTS = dict(host='', user='ubuntu', port=22, key_file='', ros_domain_id=30,
                model='burger', ros_distro='humble', bringup_command='')
MODELS = ('burger', 'waffle', 'waffle_pi')

DISCONNECTED, CONNECTING, CONNECTED, BRINGUP, ERROR = 'deconnecte', 'connexion', 'connecte', 'bringup', 'erreur'


def load_settings(path=CONFIG_PATH):
    settings = dict(DEFAULTS)
    try:
        stored = json.loads(Path(path).read_text())
        settings.update({k: v for k, v in stored.items() if k in DEFAULTS})
    except (OSError, ValueError, TypeError):
        pass
    return settings


def save_settings(settings, path=CONFIG_PATH):
    """Enregistre les réglages publics ; tout autre champ (dont un mot de passe) est ignoré."""
    data = {k: settings[k] for k in DEFAULTS if k in settings}
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(data, indent=2, ensure_ascii=False))
    os.chmod(tmp, 0o600)
    os.replace(tmp, path)


def validate(settings):
    """Message d'erreur lisible, ou None si les réglages sont utilisables."""
    host = str(settings.get('host', '')).strip()
    if not host:
        return 'Indiquez l’adresse du robot (nom ou IP).'
    if not re.fullmatch(r'[A-Za-z0-9._:-]+', host):
        return 'Adresse du robot invalide.'
    if not str(settings.get('user', '')).strip():
        return 'Indiquez le nom d’utilisateur du robot.'
    try:
        port = int(settings.get('port', 22))
        domain = int(settings.get('ros_domain_id', 30))
    except (TypeError, ValueError):
        return 'Le port et le ROS_DOMAIN_ID doivent être des nombres.'
    if not 1 <= port <= 65535:
        return 'Port SSH invalide.'
    if not 0 <= domain <= 232:
        return 'ROS_DOMAIN_ID doit être entre 0 et 232.'
    if settings.get('model', 'burger') not in MODELS:
        return 'Modèle de robot inconnu.'
    key = str(settings.get('key_file', '')).strip()
    if key and not Path(key).expanduser().is_file():
        return 'Fichier de clé introuvable : '+key
    return None


def bringup_command(settings):
    """Commande exécutée sur le robot (réglable par `bringup_command`)."""
    custom = str(settings.get('bringup_command', '')).strip()
    if custom:
        return custom
    distro = re.sub(r'[^a-z]', '', str(settings.get('ros_distro', 'humble'))) or 'humble'
    return (f'source /opt/ros/{distro}/setup.bash && export TURTLEBOT3_MODEL={shlex.quote(str(settings["model"]))} '
            f'ROS_DOMAIN_ID={int(settings["ros_domain_id"])} && ros2 launch turtlebot3_bringup robot.launch.py')


def explain_error(exc):
    """Traduit une erreur SSH ou réseau en phrase compréhensible."""
    import paramiko
    if isinstance(exc, paramiko.AuthenticationException):
        return 'Identifiants refusés : vérifiez l’utilisateur, le mot de passe ou la clé.'
    if isinstance(exc, (socket.timeout, TimeoutError)):
        return 'Le robot ne répond pas (délai dépassé) : est-il allumé et sur le même réseau ?'
    if isinstance(exc, socket.gaierror):
        return 'Adresse du robot introuvable : vérifiez le nom ou l’adresse IP.'
    if isinstance(exc, paramiko.ssh_exception.NoValidConnectionsError) or isinstance(exc, ConnectionRefusedError):
        return 'Connexion refusée : SSH est-il activé sur le robot (port 22) ?'
    if isinstance(exc, paramiko.SSHException):
        return 'Erreur SSH : '+str(exc)
    return f'{type(exc).__name__} : {exc}'


class RobotLink:
    """Une connexion SSH, un bringup. Les méthodes bloquantes sont à appeler dans un thread."""

    def __init__(self, client_factory=None):
        self._client_factory = client_factory
        self.state = DISCONNECTED
        self.message = 'Non connecté.'
        self.output = queue.Queue()      # lignes de sortie du bringup (et messages)
        self._client = None
        self._channel = None
        self._reader = None

    # -- connexion -------------------------------------------------------------------------
    def connect(self, settings, password='', confirm_host_key=lambda host, fingerprint: False):
        problem = validate(settings)
        if problem:
            self._set(ERROR, problem)
            return False
        self.disconnect(quiet=True)
        self._set(CONNECTING, 'Connexion à '+str(settings['host'])+'…')
        try:
            client = self._make_client(confirm_host_key)
            key = str(settings.get('key_file', '')).strip()
            client.connect(settings['host'].strip(), port=int(settings['port']), username=settings['user'].strip(),
                           password=password or None, key_filename=str(Path(key).expanduser()) if key else None,
                           look_for_keys=not password, allow_agent=not password,
                           timeout=10, banner_timeout=10, auth_timeout=10)
        except Exception as exc:
            self._set(ERROR, explain_error(exc))
            return False
        self._client = client
        self._set(CONNECTED, 'Connecté à '+settings['user']+'@'+settings['host']+'.')
        return True

    def _make_client(self, confirm_host_key):
        if self._client_factory is not None:
            return self._client_factory()
        import paramiko

        class AskPolicy(paramiko.MissingHostKeyPolicy):
            def missing_host_key(self, client, hostname, key):
                import base64
                import hashlib
                fingerprint = 'SHA256:'+base64.b64encode(hashlib.sha256(key.asbytes()).digest()).decode().rstrip('=')
                if not confirm_host_key(hostname, fingerprint):
                    raise paramiko.SSHException('Empreinte du robot refusée : connexion annulée.')
                KNOWN_HOSTS.parent.mkdir(parents=True, exist_ok=True)
                with KNOWN_HOSTS.open('a') as stream:
                    stream.write(f'{hostname} {key.get_name()} {key.get_base64()}\n')

        client = paramiko.SSHClient()
        client.load_system_host_keys()
        client.set_missing_host_key_policy(AskPolicy())
        return client

    def disconnect(self, quiet=False):
        self.stop_bringup(quiet=True)
        if self._client is not None:
            try:
                self._client.close()
            except Exception:
                pass
        self._client = None
        if not quiet:
            self._set(DISCONNECTED, 'Déconnecté.')

    # -- bringup ---------------------------------------------------------------------------
    def start_bringup(self, settings):
        if self.state not in (CONNECTED,):
            self._set(self.state, 'Connectez-vous au robot avant de lancer le bringup.')
            return False
        command = bringup_command(settings)
        try:
            channel = self._client.get_transport().open_session()
            channel.get_pty()
            channel.exec_command('bash -lc '+shlex.quote(command))
        except Exception as exc:
            self._set(ERROR, 'Bringup impossible : '+explain_error(exc))
            return False
        self._channel = channel
        self.output.put('$ '+command)
        self._reader = threading.Thread(target=self._read, args=(channel,), daemon=True)
        self._reader.start()
        self._set(BRINGUP, 'Bringup lancé sur le robot.')
        return True

    def _read(self, channel):
        buffer = ''
        try:
            while True:
                chunk = channel.recv(4096)
                if not chunk:
                    break
                buffer += chunk.decode('utf-8', 'replace').replace('\r', '')
                *lines, buffer = buffer.split('\n')
                for line in lines:
                    self.output.put(line)
        except Exception as exc:
            self.output.put('[lecture interrompue] '+str(exc))
        if buffer:
            self.output.put(buffer)
        if self._channel is channel:
            code = channel.recv_exit_status() if channel.exit_status_ready() else None
            self._channel = None
            if self.state == BRINGUP:
                self._set(CONNECTED, 'Le bringup s’est arrêté'+('' if code in (None, 0) else f' (code {code})')+'.')

    def stop_bringup(self, quiet=False, timeout=8.0):
        channel = self._channel
        if channel is None:
            return
        try:
            channel.send('\x03')       # Ctrl-C dans le terminal virtuel du robot
            deadline = time.monotonic()+timeout
            while not channel.exit_status_ready() and time.monotonic() < deadline:
                time.sleep(0.1)
        except Exception:
            pass
        try:
            channel.close()
        except Exception:
            pass
        self._channel = None
        if self.state == BRINGUP:
            self._set(CONNECTED, 'Bringup arrêté.' if not quiet else self.message)

    def _set(self, state, message):
        self.state, self.message = state, message
