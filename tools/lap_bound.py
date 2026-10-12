#!/usr/bin/env python3
"""Ideal lap time along a fixed path (how much is left on the table?).

Classic forward/backward speed profile: speed is limited by grip in corners
(v <= sqrt(a_lat / curvature)), by acceleration on exits and by braking on
entries. The limits are the ones measured in the simulator. This is a bound
for the given *path*: a perfect driver on that line could not be faster.

    python3 tools/lap_bound.py                               # replica centreline
    python3 tools/lap_bound.py --trace runs/trace_Z03_final_0.csv   # line actually driven
"""

import argparse
import csv
import json
import os

import numpy as np

TRACK = os.path.join(os.path.dirname(__file__), '..', 'tests', 'data', 'practice_track.json')


def resample_closed(xy, step):
    seg = np.linalg.norm(np.diff(xy, axis=0, append=xy[:1]), axis=1)
    s = np.concatenate([[0], np.cumsum(seg)])
    grid = np.arange(0, s[-1], step)
    loop = np.vstack([xy, xy[:1]])
    return np.stack([np.interp(grid, s, loop[:, 0]), np.interp(grid, s, loop[:, 1])], axis=1)


def smooth_closed(a, k):
    return sum(np.roll(a, i, axis=0) for i in range(-k, k + 1)) / (2 * k + 1)


def curvature(xy):
    p0, p1, p2 = np.roll(xy, 1, axis=0), xy, np.roll(xy, -1, axis=0)
    a = np.linalg.norm(p1 - p0, axis=1)
    b = np.linalg.norm(p2 - p1, axis=1)
    c = np.linalg.norm(p2 - p0, axis=1)
    d1, d2 = p1 - p0, p2 - p0
    cross = d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]
    return 2 * np.abs(cross) / np.maximum(a * b * c, 1e-9)


def lap_time(xy, a_lat, a_acc, a_brake, v_max, step=0.05):
    xy = smooth_closed(resample_closed(xy, step), 6)
    k = curvature(xy)
    v = np.minimum(np.sqrt(a_lat / np.maximum(k, 1e-6)), v_max)
    n = len(v)
    for _ in range(2):  # two passes so the closed loop wraps around consistently
        for i in range(1, 2 * n):
            j, prev = i % n, (i - 1) % n
            v[j] = min(v[j], np.sqrt(v[prev] ** 2 + 2 * a_acc * step))
        for i in range(2 * n - 2, -1, -1):
            j, nxt = i % n, (i + 1) % n
            v[j] = min(v[j], np.sqrt(v[nxt] ** 2 + 2 * a_brake * step))
    return float(np.sum(step / v)), n * step, float(1 / np.max(k))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--trace', help='experiment trace CSV; uses its second lap')
    ap.add_argument('--a-lat', type=float, default=6.5)
    ap.add_argument('--a-acc', type=float, default=6.0)
    ap.add_argument('--a-brake', type=float, default=6.0)
    ap.add_argument('--v-max', type=float, default=10.0)
    args = ap.parse_args()
    if args.trace:
        with open(args.trace) as f:
            rows = list(csv.DictReader(f))
        lap0 = int(rows[0]['lap_count'])
        xy = np.array([[float(r['x']), float(r['y'])] for r in rows
                       if int(r['lap_count']) == lap0 + 1])
        xy = xy[np.r_[True, np.linalg.norm(np.diff(xy, axis=0), axis=1) > 1e-4]]
        name = os.path.basename(args.trace)
    else:
        with open(TRACK) as f:
            xy = np.array(json.load(f)['centerline'])
        name = 'track centreline (replica)'
    t, length, rmin = lap_time(xy, args.a_lat, args.a_acc, args.a_brake, args.v_max)
    print(f'{name}: length {length:.1f} m, tightest radius {rmin:.2f} m -> '
          f'ideal lap {t:.2f} s (a_lat {args.a_lat}, accel {args.a_acc}, brake {args.a_brake})')


if __name__ == '__main__':
    main()
