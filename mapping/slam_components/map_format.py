"""Format commun : grille [ligne y, colonne x], origine en bas à gauche."""
from pathlib import Path
import math
import numpy as np
from PIL import Image
import yaml
from scipy.ndimage import binary_dilation

FORMAT_VERSION = 2


def encode_probability(probability_yx):
    """Libre blanc, occupé noir, inconnu gris ; PGM stocké du haut vers le bas."""
    return np.flipud(np.rint((1-np.clip(probability_yx, 0, 1))*255).astype(np.uint8))


def load_map(yaml_path, clearance=0.0):
    path = Path(yaml_path)
    meta = yaml.safe_load(path.read_text())
    if meta.get('format_version') != FORMAT_VERSION:
        raise ValueError('Carte ancienne sans convention validée : la régénérer depuis un CSV avec carto_mat.py.')
    correction = meta.get('map_from_odom', [0., 0., 0.])
    if len(correction) != 3 or not all(math.isfinite(float(x)) for x in correction):
        raise ValueError('Transformation carte/odom invalide.')
    meta['map_from_odom'] = [float(x) for x in correction]
    if meta.get('frame_id') == 'map' and 'map_from_odom' not in yaml.safe_load(path.read_text()):
        raise ValueError('Transformation carte/odom absente.')
    res = float(meta['resolution'])
    origin = meta['origin']
    if not math.isfinite(res) or res <= 0 or len(origin) != 3 or not all(math.isfinite(float(x)) for x in origin):
        raise ValueError('Résolution ou origine invalide.')
    meta['resolution'] = res
    meta['origin'] = [float(v) for v in origin]
    free, occupied = float(meta.get('free_thresh', .25)), float(meta.get('occupied_thresh', .65))
    if not 0 <= free < occupied <= 1 or meta.get('negate', 0) not in (0, 1):
        raise ValueError('Seuils d’occupation ou negate invalides.')
    meta['free_thresh'], meta['occupied_thresh'] = free, occupied
    if abs(float(origin[2])) > 1e-9:
        raise ValueError('Les cartes tournées ne sont pas prises en charge.')
    pixels = np.flipud(np.array(Image.open(path.parent/meta['image']).convert('L')))
    probability = pixels/255.0 if meta.get('negate', 0) else (255-pixels)/255.0
    free = probability < float(meta.get('free_thresh', 0.25))
    # Les cellules inconnues sont bloquées au même titre que les obstacles.
    blocked = ~free
    radius = int(math.ceil(clearance/res))
    if radius:
        yy, xx = np.ogrid[-radius:radius+1, -radius:radius+1]
        blocked = binary_dilation(blocked, structure=xx*xx+yy*yy <= radius*radius, border_value=1)
    return meta, pixels, np.where(blocked, 2, 0)
