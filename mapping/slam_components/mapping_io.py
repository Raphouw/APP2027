"""Relecture CSV et export atomique d'une carte associée à sa session."""
import csv
import json
import os
from pathlib import Path
import uuid
import math

from PIL import Image
import yaml
import matplotlib.pyplot as plt
from .map_format import encode_probability, FORMAT_VERSION
from .settings import ODOM_FRAME


class MappingIOMixin:
    def run_carto_offline(self):
        if not self.input_file:
            raise ValueError('Aucun fichier CSV fourni.')
        count = 0
        json_ranges = False
        with open(self.input_file, newline='') as stream:
            for line, row in enumerate(csv.reader(stream), 1):
                if getattr(self, 'stop_requested', False):
                    raise RuntimeError('Reconstruction interrompue : carte non validée.')
                if not row:
                    continue
                if row[0] == 'timestamp':
                    json_ranges = 'ranges_json' in row
                    continue
                try:
                    if len(row) < 8:
                        raise ValueError('moins de huit champs')
                    world_x, world_y, theta = map(float, row[1:4])
                    amin, amax = map(float, row[6:8])
                    if not all(math.isfinite(v) for v in [world_x, world_y, theta, amin, amax]):
                        raise ValueError('pose ou angles non finis')
                    # Garder l'emplacement des valeurs absentes pour ne pas décaler les angles.
                    ranges = ([float(v) if v is not None else float('nan') for v in json.loads(row[8])]
                              if json_ranges else [float(v) if v.strip() else float('nan') for v in row[8:]])
                    if not any((math.isfinite(v) or v == math.inf) and v > 0.12 for v in ranges):
                        continue
                    self.x, self.y = world_x*self.scale+self.x0, world_y*self.scale+self.y0
                    self.orientation = theta
                    x, y = self.process_data(ranges, amin, amax)
                    if hasattr(self, 'slam'):
                        self.integrate_scan([world_x, world_y, theta], ranges, amin, amax, stamp=float(row[0]))
                    else:
                        self.write_map(x, y)
                    count += 1
                    if count % 20 == 0:
                        self.update_plot()
                        if not getattr(self, 'headless', False):
                            plt.pause(0.001/self.speed_multiplier)
                except (ValueError, IndexError) as exc:
                    raise ValueError(f'CSV ligne {line}: {exc}') from exc
        if not count:
            raise ValueError('Session sans scan exploitable : aucune carte générée.')
        self.update_plot()
        self.processed_scans = count
        return count

    def save_map(self):
        if self.scan_counter == 0:
            raise ValueError('Aucun scan traité : sauvegarde refusée.')
        out = Path(getattr(self, 'output_dir', './logs'))
        out.mkdir(parents=True, exist_ok=True)
        # L'image reçoit un identifiant unique ; le YAML est le point de commit.
        image_name = f'map_{self.filename}_{uuid.uuid4().hex[:8]}.pgm'
        Image.fromarray(encode_probability(self.map.T)).save(out/image_name)
        meta = {'image': image_name, 'resolution': 1/self.scale,
                'origin': [-self.x0/self.scale, -self.y0/self.scale, 0.0],
                'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.25,
                'format_version': FORMAT_VERSION, 'frame_id': ODOM_FRAME,
                'source_csv': str(Path(self.input_file).resolve()) if self.input_file else None,
                'session_id': getattr(self, 'session_id', None),
                'scans': self.scan_counter}
        if hasattr(self, 'slam'):
            meta['map_from_odom'] = self.slam.map_from_odom.tolist()
            meta['slam'] = self.slam.diagnostics()
            meta['frame_id'] = 'map'
            self.write_preview(force=True)
            archive = dict(poses=[p.tolist() for p in self.slam.graph.poses],
                           edges=[dict(i=i, j=j, measurement=z.tolist(), sigma=sigma.tolist(), loop=loop)
                                  for i, j, z, sigma, loop in self.slam.graph.edges])
            trajectory = out/'trajectory.csv'
            trajectory_tmp = trajectory.with_suffix('.tmp')
            with trajectory_tmp.open('w', newline='') as stream:
                writer = csv.writer(stream)
                writer.writerow(['timestamp', 'x', 'y', 'theta'])
                for stamp, (pose, _, _) in zip(self.scan_stamps, self.slam.scans()):
                    writer.writerow([stamp, *pose])
            os.replace(trajectory_tmp, trajectory)
            graph_path = out/'pose_graph.json'
            graph_tmp = graph_path.with_suffix('.tmp')
            graph_tmp.write_text(json.dumps(archive, allow_nan=False))
            os.replace(graph_tmp, graph_path)
        yaml_path = out/f'map_{self.filename}.yaml'
        tmp = yaml_path.with_suffix('.yaml.tmp')
        tmp.write_text(yaml.safe_dump(meta, sort_keys=False))
        os.replace(tmp, yaml_path)
        return str(yaml_path)
