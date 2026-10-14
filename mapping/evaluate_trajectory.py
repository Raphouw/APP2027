#!/usr/bin/env python3
"""Compare a stamped estimated trajectory to a reference in the SAME frame."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from slam_components.slam_engine import angle


def read_trajectory(path):
    with Path(path).open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    data = np.array([[float(row[key]) for key in ('timestamp', 'x', 'y', 'theta')] for row in rows])
    if data.ndim != 2 or data.shape[0] < 2 or data.shape[1] != 4 or not np.all(np.isfinite(data)):
        raise ValueError('Au moins deux poses finies horodatées sont nécessaires.')
    if not np.all(np.diff(data[:, 0]) > 0):
        raise ValueError('Les horodatages doivent être strictement croissants.')
    return data


def evaluate(estimate, reference, max_gap=.3):
    errors = []
    for row in estimate:
        index = np.searchsorted(reference[:, 0], row[0])
        if index < len(reference) and abs(reference[index, 0]-row[0]) < 1e-8:
            expected = reference[index, 1:]
        elif index == 0 or index == len(reference):
            continue
        else:
            left, right = reference[index-1], reference[index]
            if right[0]-left[0] > max_gap:
                continue
            f = (row[0]-left[0])/(right[0]-left[0])
            expected = np.array([*(left[1:3]+f*(right[1:3]-left[1:3])), angle(left[3]+f*angle(right[3]-left[3]))])
        errors.append([np.linalg.norm(row[1:3]-expected[:2]), abs(angle(row[3]-expected[2]))])
    if not errors:
        raise ValueError('Aucun horodatage commun exploitable.')
    errors = np.asarray(errors)
    return dict(samples=len(errors), skipped=len(estimate)-len(errors),
                position_rmse_m=float(np.sqrt(np.mean(errors[:, 0]**2))),
                position_p95_m=float(np.quantile(errors[:, 0], .95)),
                heading_rmse_rad=float(np.sqrt(np.mean(errors[:, 1]**2))))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('estimate')
    parser.add_argument('reference')
    parser.add_argument('--output', default='evaluation.json')
    parser.add_argument('--max-gap', type=float, default=.3)
    args = parser.parse_args()
    if args.max_gap <= 0:
        parser.error('--max-gap doit être positif')
    metrics = evaluate(read_trajectory(args.estimate), read_trajectory(args.reference), args.max_gap)
    Path(args.output).write_text(json.dumps(metrics, indent=2, allow_nan=False)+'\n')
    print(json.dumps(metrics, indent=2))


if __name__ == '__main__':
    main()
