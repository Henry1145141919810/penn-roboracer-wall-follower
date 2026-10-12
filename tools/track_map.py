#!/usr/bin/env python3
"""Draw the track walls from saved scans, plus driven paths, as a PNG.

    python3 tools/track_map.py runs/scans_map_0.npz runs/trace_a_0.csv [...] -o runs/map.png

Walls are black. Each trace is drawn coloured by speed (blue slow, red fast);
the last point of a trace that ended in a collision is marked with a cross.
Only numpy is needed (the PNG writer is built in).
"""

import argparse
import csv
import struct
import zlib

import numpy as np

LIDAR_X = 0.2733  # m, LiDAR ahead of the rear axle (vehicle frame origin)
ANGLE_MIN = -2.35619
ANGLE_INC = 0.004363323


def wall_points(npz_path):
    data = np.load(npz_path)
    pose, ranges = data['pose'], data['ranges']
    angles = ANGLE_MIN + ANGLE_INC * np.arange(ranges.shape[1])
    pts = []
    for (x, y, yaw), r in zip(pose, ranges):
        ok = np.isfinite(r) & (r > 0.06) & (r < 9.5)
        lx = LIDAR_X + r[ok] * np.cos(angles[ok])
        ly = r[ok] * np.sin(angles[ok])
        c, s = np.cos(yaw), np.sin(yaw)
        pts.append(np.stack([x + c * lx - s * ly, y + s * lx + c * ly], axis=1))
    return np.concatenate(pts)


def read_trace(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    xy = np.array([[float(r['x']), float(r['y'])] for r in rows])
    v = np.array([float(r['speed']) for r in rows])
    coll = [int(r['collisions']) if r['collisions'] not in ('None', '') else 0 for r in rows]
    crashed = len(coll) > 1 and coll[-1] > coll[0]
    return xy, v, crashed


def write_png(path, img):
    h, w, _ = img.shape
    raw = b''.join(b'\x00' + img[row].tobytes() for row in range(h))

    def chunk(tag, data):
        return (struct.pack('>I', len(data)) + tag + data
                + struct.pack('>I', zlib.crc32(tag + data) & 0xffffffff))
    png = (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
           + chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b''))
    with open(path, 'wb') as f:
        f.write(png)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('scans')
    ap.add_argument('traces', nargs='*')
    ap.add_argument('-o', '--out', default='runs/track_map.png')
    ap.add_argument('--res', type=float, default=0.02, help='m per pixel')
    ap.add_argument('--vmax', type=float, default=0.0, help='speed for full red')
    args = ap.parse_args()

    walls = wall_points(args.scans)
    traces = [read_trace(t) for t in args.traces]
    allpts = np.concatenate([walls] + [t[0] for t in traces])
    lo = allpts.min(axis=0) - 0.3
    hi = allpts.max(axis=0) + 0.3
    w, h = (np.ceil((hi - lo) / args.res).astype(int) + 1)
    img = np.full((h, w, 3), 255, dtype=np.uint8)

    def to_px(p):
        c = ((p - lo) / args.res).astype(int)
        return np.clip(c[:, 0], 0, w - 1), np.clip(h - 1 - c[:, 1], 0, h - 1)

    # 1 m grid
    for gx in np.arange(np.ceil(lo[0]), hi[0], 1.0):
        img[:, int((gx - lo[0]) / args.res)] = (225, 225, 225)
    for gy in np.arange(np.ceil(lo[1]), hi[1], 1.0):
        img[h - 1 - int((gy - lo[1]) / args.res), :] = (225, 225, 225)
    cx, cy = to_px(walls)
    img[cy, cx] = (0, 0, 0)

    vmax = args.vmax or max([t[1].max() for t in traces] + [1.0])
    for xy, v, crashed in traces:
        px, py = to_px(xy)
        f = np.clip(v / vmax, 0, 1)
        color = np.stack([255 * f, 40 + 0 * f, 255 * (1 - f)], axis=1).astype(np.uint8)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                img[np.clip(py + dy, 0, h - 1), np.clip(px + dx, 0, w - 1)] = color
        if crashed:
            ex, ey = px[-1], py[-1]
            for d in range(-8, 9):
                for t in (-1, 0, 1):
                    img[np.clip(ey + d, 0, h - 1), np.clip(ex + d + t, 0, w - 1)] = (0, 160, 0)
                    img[np.clip(ey - d, 0, h - 1), np.clip(ex + d + t, 0, w - 1)] = (0, 160, 0)
    sx, sy = to_px(traces[0][0][:1]) if traces else (None, None)
    if sx is not None:
        img[max(sy[0] - 6, 0):sy[0] + 7, max(sx[0] - 6, 0):sx[0] + 7] = (0, 160, 0)
    write_png(args.out, img)
    print(f'wrote {args.out} ({w}x{h}px, {args.res} m/px, x {lo[0]:.1f}..{hi[0]:.1f}, '
          f'y {lo[1]:.1f}..{hi[1]:.1f})')


if __name__ == '__main__':
    main()
