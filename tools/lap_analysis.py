#!/usr/bin/env python3
"""Where does the lap time go? Compare a driven lap to the ideal profile on its own path.

For one flying lap of an experiment trace, the point-mass speed profile (grip,
acceleration and braking limits as measured in the simulator) is computed on
the path the car actually drove. The lap is then split into segments and the
time lost to that ideal is reported per segment, with the limiting factor.

    python3 tools/lap_analysis.py runs/trace_W01_final_0.csv
    python3 tools/lap_analysis.py runs/trace_W01_final_0.csv --a-lat 8.0 --segment 2.0
"""

import argparse
import csv
import math

import numpy as np

from lap_bound import curvature, resample_closed, smooth_closed


def load_lap(path, lap_index=1):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    lap0 = int(rows[0]['lap_count'])
    lap = [r for r in rows if int(r['lap_count']) == lap0 + lap_index]
    t = np.array([float(r['t']) for r in lap])
    xy = np.array([[float(r['x']), float(r['y'])] for r in lap])
    v = np.array([float(r['speed']) for r in lap])
    yaw = np.unwrap(np.array([float(r['yaw']) for r in lap]))
    keep = np.r_[True, np.linalg.norm(np.diff(xy, axis=0), axis=1) > 1e-4]
    return t[keep], xy[keep], v[keep], yaw[keep], lap


def ideal_profile(xy, a_lat, a_acc, a_brake, v_max, step):
    path = smooth_closed(resample_closed(xy, step), 6)
    k = curvature(path)
    v_grip = np.minimum(np.sqrt(a_lat / np.maximum(k, 1e-6)), v_max)
    v = v_grip.copy()
    n = len(v)
    for _ in range(2):
        for i in range(1, 2 * n):
            j, prev = i % n, (i - 1) % n
            v[j] = min(v[j], math.sqrt(v[prev] ** 2 + 2 * a_acc * step))
        for i in range(2 * n - 2, -1, -1):
            j, nxt = i % n, (i + 1) % n
            v[j] = min(v[j], math.sqrt(v[nxt] ** 2 + 2 * a_brake * step))
    limit = np.where(v >= v_grip - 1e-6, 'grip', 'accel/brake')
    limit = np.where(v >= v_max - 1e-6, 'v_max', limit)
    return path, k, v, limit


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('trace')
    ap.add_argument('--lap', type=int, default=1, help='1 = first flying lap')
    ap.add_argument('--a-lat', type=float, default=8.0)
    ap.add_argument('--a-acc', type=float, default=6.0)
    ap.add_argument('--a-brake', type=float, default=6.0)
    ap.add_argument('--v-max', type=float, default=12.0)
    ap.add_argument('--segment', type=float, default=2.0, help='m of path per report row')
    args = ap.parse_args()

    t, xy, v, yaw, rows = load_lap(args.trace, args.lap)
    step = 0.05
    path, k, v_ideal, limit = ideal_profile(xy, args.a_lat, args.a_acc, args.a_brake,
                                            args.v_max, step)
    # actual speed sampled onto the resampled path (nearest driven point)
    idx = np.array([np.argmin(np.sum((xy - p) ** 2, axis=1)) for p in path])
    v_act = np.maximum(v[idx], 0.3)
    dt_act, dt_ideal = step / v_act, step / v_ideal
    yaw_rate = np.gradient(yaw, t)
    a_lat_act = np.abs(yaw_rate * v)
    print(f'{args.trace}: lap {t[-1] - t[0]:.2f} s measured, path {len(path) * step:.1f} m, '
          f'ideal on this path {dt_ideal.sum():.2f} s '
          f'(a_lat {args.a_lat}, accel {args.a_acc}, brake {args.a_brake})')
    print(f'actual lateral accel: p50 {np.percentile(a_lat_act, 50):.1f} '
          f'p90 {np.percentile(a_lat_act, 90):.1f} p99 {np.percentile(a_lat_act, 99):.1f} '
          f'max {a_lat_act.max():.1f} m/s^2; speed min {v.min():.2f} max {v.max():.2f}')
    print()
    print(f'{"segment (m)":>12} {"x,y at start":>14} {"v act":>6} {"v ideal":>8} '
          f'{"lost (s)":>9} {"limit":>12}')
    n_seg = int(args.segment / step)
    total = 0.0
    for s in range(0, len(path), n_seg):
        sl = slice(s, s + n_seg)
        lost = dt_act[sl].sum() - dt_ideal[sl].sum()
        total += lost
        lim = max(set(limit[sl]), key=list(limit[sl]).count)
        print(f'{s * step:6.1f}-{min(s + n_seg, len(path)) * step:5.1f} '
              f'{path[s][0]:6.2f},{path[s][1]:6.2f} {v_act[sl].mean():6.2f} '
              f'{v_ideal[sl].mean():8.2f} {lost:9.3f} {lim:>12}')
    print(f'\ntotal lost vs ideal on this path: {total:.2f} s')


if __name__ == '__main__':
    main()
