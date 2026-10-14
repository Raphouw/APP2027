#!/usr/bin/env python3
"""Read-only native dependency probes in separate processes; never ROS init."""
import argparse
import importlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import time
import traceback

MODULES = ('tkinter', 'numpy', 'scipy', 'PIL.Image', 'PIL.ImageTk', 'yaml',
           'matplotlib', 'matplotlib.backends.backend_tkagg', 'rclpy',
           'sensor_msgs.msg', 'nav_msgs.msg', 'geometry_msgs.msg', 'std_msgs.msg', 'tf2_ros', 'cv2')


def probe(name):
    try:
        module = importlib.import_module(name)
        print(json.dumps(dict(module=name, file=getattr(module, '__file__', None),
                              version=getattr(module, '__version__', None))))
        return 0
    except ModuleNotFoundError:
        traceback.print_exc()
        return 2
    except Exception:
        traceback.print_exc()
        return 3


def check(output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    report = dict(time=time.time(), python=sys.executable, version=sys.version,
                  platform=platform.platform(), modules=[],
                  display=os.environ.get('DISPLAY'), ros_distro=os.environ.get('ROS_DISTRO'),
                  ros_domain_id=os.environ.get('ROS_DOMAIN_ID', '0'),
                  matplotlib_backend=os.environ.get('MPLBACKEND'),
                  optional_tools={name: shutil.which(name) for name in ('rviz2', 'ffmpeg')})
    failure = 0
    for name in MODULES:
        path = output/('probe_'+name.replace('.', '_')+'.log')
        command = [sys.executable, '-s', '-X', 'faulthandler', str(Path(__file__).resolve()), '--probe', name]
        with path.open('w') as stream:
            try:
                result = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT, timeout=20)
                code = result.returncode
            except subprocess.TimeoutExpired:
                code = 124
        entry = dict(module=name, returncode=code, log=str(path))
        if code == 0:
            entry.update(json.loads(path.read_text().splitlines()[-1]))
            print('[OK] '+name)
        else:
            failure = max(failure, 2 if code == 2 else 3)
            print(f'[ÉCHEC] {name} : code {code}, détails : {path}')
        report['modules'].append(entry)
    report['success'] = failure == 0
    (output/'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
    if not report['display']:
        print('Affichage graphique absent : lancer le manager dans une session de bureau (ou X11 transféré).')
    return failure


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='./logs/diagnostic')
    parser.add_argument('--probe', choices=MODULES)
    args = parser.parse_args()
    return probe(args.probe) if args.probe else check(args.output)


if __name__ == '__main__':
    raise SystemExit(main())
