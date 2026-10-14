"""Atomic local UI messages with per-launch tokens and short command lifetime."""
import json
import math
import os
import time
import uuid
from pathlib import Path


def atomic_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
    tmp.write_text(json.dumps(data, allow_nan=False))
    os.replace(tmp, path)


class SessionChannel:
    def __init__(self, directory, token, role):
        self.directory, self.token, self.role = Path(directory), token, role
        self.last_id = None

    def send(self, kind, values=()):
        command_id = uuid.uuid4().hex
        atomic_json(self.directory/(self.role+'_command.json'),
                    dict(token=self.token, id=command_id, time=time.time(), kind=kind, values=list(values)))
        return command_id

    def receive(self):
        try:
            data = json.loads((self.directory/(self.role+'_command.json')).read_text())
            if data['token'] != self.token or data['id'] == self.last_id:
                return None
            self.last_id = data['id']
            if not 0 <= time.time()-float(data['time']) < 3:
                return None
            kind, values = data['kind'], data['values']
            size = {'goal': 2, 'pose': 3, 'stop': 0}.get(kind)
            if size is None or len(values) != size or not all(math.isfinite(float(v)) for v in values):
                return None
            return kind, [float(v) for v in values]
        except (OSError, ValueError, KeyError, TypeError):
            return None

    def status(self, data):
        atomic_json(self.directory/(self.role+'_status.json'), dict(data, token=self.token, updated_at=time.time()))
