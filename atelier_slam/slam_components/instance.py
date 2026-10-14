"""Instance unique de l'atelier et arrêt de tout ce qu'il a lancé (sans ROS, bibliothèque standard seule)."""
import argparse
import fcntl
import os
import signal
import sys
import time
from pathlib import Path

LOCK_PATH = Path(os.environ.get('ATELIER_LOCK_DIR', Path.home()/'.cache'/'atelier_slam'))/'instance.lock'
APP_DIR = Path(__file__).resolve().parents[1]
# Scripts lancés par l'atelier (et le manager lui-même) : on ne tue que ceux de CE dossier.
SCRIPTS = ('atelier.py', 'navigation_carte.py', 'noeud_odometrie.py', 'cartographe.py', 'simulateur.py',
           'sonde_ros.py', 'serveur_camera.py', 'pilote_simulation.py', 'diagnostic_environnement.py')


def _cmdline(pid):
    try:
        return Path(f'/proc/{pid}/cmdline').read_bytes().decode('utf-8', 'replace').split('\0')[:-1]
    except OSError:
        return []


def _cwd(pid):
    try:
        return os.readlink(f'/proc/{pid}/cwd')
    except OSError:
        return ''


def atelier_processes(exclude=()):
    """PID des processus de l'atelier de ce dossier (manager, nœuds, sondes, simulateur, fils orphelins)."""
    found = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit() or int(entry) in exclude:
            continue
        args = _cmdline(entry)
        for arg in args[1:6]:
            name = os.path.basename(arg)
            if name in SCRIPTS:
                path = Path(arg) if os.path.isabs(arg) else Path(_cwd(entry))/arg
                if path.resolve().parent == APP_DIR:
                    found.append(int(entry))
                    break
    return found


def descendants(pid):
    children = {}
    for entry in os.listdir('/proc'):
        if entry.isdigit():
            try:
                fields = Path(f'/proc/{entry}/stat').read_text().rsplit(')', 1)[1].split()
                children.setdefault(int(fields[1]), []).append(int(entry))
            except (OSError, IndexError, ValueError):
                pass
    result, stack = [], [pid]
    while stack:
        for child in children.get(stack.pop(), []):
            result.append(child)
            stack.append(child)
    return result


def stop_processes(pids, grace=4.0):
    """SIGTERM puis, après `grace` secondes, SIGKILL. Retourne le nombre de processus visés."""
    pids = [p for p in dict.fromkeys(pids) if p != os.getpid()]
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            pass
    deadline = time.monotonic()+grace
    while time.monotonic() < deadline and any(Path(f'/proc/{p}').exists() for p in pids):
        time.sleep(0.1)
    for pid in pids:
        if Path(f'/proc/{pid}').exists():
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                pass
    return len(pids)


def stop_all(grace=4.0):
    """Arrête tout l'atelier : le manager d'abord (il ferme proprement ses fils), puis les restes."""
    mine = os.getpid()
    managers = [p for p in atelier_processes((mine,)) if os.path.basename(next((a for a in _cmdline(p)[1:6] if a.endswith('atelier.py')), '')) == 'atelier.py']
    stop_processes(managers, grace)
    return stop_processes([p for p in atelier_processes((mine,))], 2.0)+len(managers)


def acquire():
    """Prend le verrou d'instance. Retourne (fichier, None) si obtenu, sinon (None, pid du détenteur)."""
    LOCK_PATH.parent.mkdir(parents=True, exist_ok=True)
    handle = LOCK_PATH.open('a+')
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        handle.seek(0)
        text = handle.read().strip()
        handle.close()
        return None, int(text) if text.isdigit() else None
    handle.seek(0)
    handle.truncate()
    handle.write(str(os.getpid()))
    handle.flush()
    return handle, None


def child_dies_with_parent():
    """À appeler dans un processus fils (preexec_fn) : SIGTERM si le manager disparaît, même tué net."""
    try:
        import ctypes
        ctypes.CDLL('libc.so.6', use_errno=True).prctl(1, signal.SIGTERM)   # PR_SET_PDEATHSIG
    except Exception:
        pass


def main(argv=None):
    parser = argparse.ArgumentParser(description='Arrête Atelier SLAM et tout ce qu’il a lancé.')
    parser.add_argument('--list', action='store_true', help='liste seulement les processus')
    args = parser.parse_args(argv)
    pids = atelier_processes((os.getpid(),))
    if args.list:
        for pid in pids:
            print(pid, ' '.join(_cmdline(pid))[:120])
        return 0
    if not pids:
        print('Atelier SLAM : rien à arrêter.')
        return 0
    stopped = stop_all()
    print(f'Atelier SLAM arrêté ({stopped} processus).')
    return 0


if __name__ == '__main__':
    sys.exit(main())
