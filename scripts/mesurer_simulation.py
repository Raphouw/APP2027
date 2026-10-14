#!/usr/bin/env python3
"""Mesure l'erreur de trajectoire en simulation : odométrie seule contre SLAM, vérité terrain connue.

Lance simulateur + odométrie + cartographe + pilote automatique, attend la fin des tours,
puis écrit `resultats.json` dans le dossier de sortie. ROS doit être chargé (source setup.bash).
Usage : python3 scripts/mesurer_simulation.py [--laps 2] [--noise 1] [--output DOSSIER]
"""
import argparse
import csv
import json
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

APP = Path(__file__).resolve().parents[1]/'atelier_slam'
sys.path.insert(0, str(APP))
from evaluate_trajectory import evaluate, read_trajectory  # noqa: E402


def start(command, out, log):
    return subprocess.Popen(command, cwd=APP, stdout=open(log, 'w'), stderr=subprocess.STDOUT)


def stop(process, wait=15):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=wait)
        except subprocess.TimeoutExpired:
            process.kill()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--laps', type=int, default=2)
    parser.add_argument('--noise', type=float, default=1.0)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--output', default=str(APP/'logs'/'simulation'/('mesure_'+datetime.now().strftime('%Y%m%d_%H%M%S'))))
    args = parser.parse_args()
    out = Path(args.output)
    out.mkdir(parents=True)
    start_pose = (1.5, 2.0, 0.0)
    sim = start(['python3', 'simulator.py', '--truth-csv', str(out/'verite_terrain.csv'), '--noise', str(args.noise),
                 '--seed', str(args.seed)], out, out/'simulateur.log')
    time.sleep(2)
    nav = start(['python3', 'tb3_nav_node.py', '--ros-args', '-p', 'odometry_only:=true', '-p', f'log_directory:={out}',
                 '-p', 'log_filename:=mapping.csv'], out, out/'odometrie.log')
    carto = start(['python3', 'carto_mat.py', '-m', 'live', '--auto', '--headless', '--output-dir', str(out),
                   '-oname', 'map'], out, out/'cartographe.log')
    time.sleep(8)  # calibration IMU, robot immobile
    driver = start(['python3', 'sim_driver.py', '--laps', str(args.laps)], out, out/'pilote.log')
    try:
        driver.wait()
    except KeyboardInterrupt:
        stop(driver)
    time.sleep(3)
    for process in (carto, nav, sim):
        stop(process)
    rows = list(csv.DictReader((out/'verite_terrain.csv').open()))
    with (out/'verite_terrain_repere_odometrie.csv').open('w', newline='') as target:
        writer = csv.writer(target)
        writer.writerow(['timestamp', 'x', 'y', 'theta'])
        for row in rows:
            writer.writerow([row['timestamp'], float(row['x'])-start_pose[0], float(row['y'])-start_pose[1], row['theta']])
    truth = read_trajectory(out/'verite_terrain_repere_odometrie.csv')
    with (out/'odometrie_seule.csv').open('w', newline='') as target:
        writer = csv.writer(target)
        writer.writerow(['timestamp', 'x', 'y', 'theta'])
        for row in csv.DictReader((out/'mapping.csv').open()):
            writer.writerow([row['timestamp'], row['x'], row['y'], row['theta']])
    results = dict(laps=args.laps, noise=args.noise, seed=args.seed,
                   odometrie_seule=evaluate(read_trajectory(out/'odometrie_seule.csv'), truth),
                   slam=evaluate(read_trajectory(out/'trajectory.csv'), truth))
    slam_status = out/'slam_status.json'
    if slam_status.exists():
        status = json.loads(slam_status.read_text())
        results['fermetures_de_boucle'] = status.get('loops')
        results['images_cles'] = status.get('keyframes')
    (out/'resultats.json').write_text(json.dumps(results, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps(results, indent=2, ensure_ascii=False))
    print('Dossier :', out)


if __name__ == '__main__':
    main()
