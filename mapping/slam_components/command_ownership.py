"""Exclusion des contrôleurs locaux : un seul propriétaire de /cmd_vel."""
import fcntl
import os
from pathlib import Path


class CommandLease:
    """Verrou interprocessus libéré automatiquement par le noyau à la sortie.

    Les autres machines et les logiciels extérieurs au projet doivent utiliser
    leur propre arbitre ROS. Le chemin est surchargeable pour les tests/robots.
    """
    def __init__(self, path=None):
        path = path or os.environ.get('TB3_COMMAND_LOCK', f'/tmp/tb3-cmd-vel-{os.getuid()}.lock')
        self._file = Path(path).open('a+')
        try:
            fcntl.flock(self._file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self._file.close()
            raise RuntimeError('Un autre contrôleur local possède déjà /cmd_vel.') from None

    def close(self):
        if not self._file.closed:
            fcntl.flock(self._file, fcntl.LOCK_UN)
            self._file.close()
