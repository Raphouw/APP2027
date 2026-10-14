"""Debug files for standalone CLI runs; managed children are already redirected."""
import atexit
from datetime import datetime
import faulthandler
import json
import os
from pathlib import Path
import sys
import uuid


class Tee:
    def __init__(self, original, stream):
        self.original, self.stream = original, stream
    def write(self, text):
        self.original.write(text)
        self.stream.write(text)
        self.stream.flush()
        return len(text)
    def flush(self):
        self.original.flush()
        self.stream.flush()
    def __getattr__(self, name):
        return getattr(self.original, name)


def install_run_logging(role, directory=None):
    if os.environ.get('TB3_MANAGED_LOG'):
        return Path(os.environ['TB3_MANAGED_LOG'])
    directory = Path(directory or os.environ.get('TB3_DEBUG_DIR', Path(__file__).resolve().parents[1]/'logs'/'standalone_runs'))
    run_id = datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'_'+uuid.uuid4().hex[:8]
    directory = directory/run_id
    directory.mkdir(parents=True)
    path = directory/(role+'.log')
    stream = path.open('x', encoding='utf-8', buffering=1)
    stdout, stderr = sys.stdout, sys.stderr
    stream.write(json.dumps(dict(run_id=run_id, role=role, argv=sys.argv, cwd=str(Path.cwd())))+'\n')
    sys.stdout, sys.stderr = Tee(stdout, stream), Tee(stderr, stream)
    faulthandler.enable(file=stream)
    def close():
        # This runs after uncaught exceptions have been printed by Python.
        sys.stdout.flush(); sys.stderr.flush()
        sys.stdout, sys.stderr = stdout, stderr
        faulthandler.disable()
        stream.close()
    atexit.register(close)
    print('[DEBUG] '+str(path))
    return path
