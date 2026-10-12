#!/usr/bin/env python3
"""Small offline simulator for regression tests (no Unity, no ROS needed).

Kinematic bicycle car + ray-cast 270 deg LiDAR on a track made of two wall
polylines. Three things are matched to measurements from the AutoDRIVE
simulator: speed = 25 m/s x throttle with a fast lag (acceleration up to
6 m/s^2, braking up to 7.5 m/s^2), a grip limit of about 7 m/s^2 lateral
acceleration (asking for more makes the car slide wide), and a 40 Hz LiDAR
read by a ~160 Hz control loop. It is still NOT a replacement
for the real simulator, and lap times here are only indicative. It answers one
question cheaply: does the controller still get round without touching a wall?

    python3 tools/sim2d.py --laps 3                     # replica of the practice track
    python3 tools/sim2d.py --mirror -p v_max=4          # counter-clockwise version
    python3 tools/sim2d.py --build-track runs/trace_map_0.csv   # rebuild the replica

The replica (tests/data/practice_track.json) was built from a slow lap in the
real simulator: the driven path plus the measured distance to each wall.
"""

import argparse
import csv
from dataclasses import fields
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'autodrive_devkit'))

TRACK_FILE = os.path.join(os.path.dirname(__file__), '..', 'tests', 'data', 'practice_track.json')
WHEELBASE = 0.33
LIDAR_X = 0.2733
# Car footprint (rear-axle frame): bumpers at -0.09 / +0.43 m, half width 0.15 m.
FOOTPRINT = np.array([[x, y] for x in (-0.09, 0.17, 0.43) for y in (-0.15, 0.0, 0.15)
                      if (x, y) != (0.17, 0.0)])
A_GRIP = 7.0  # m/s^2, measured: 6-7 m/s^2 max in the real simulator
ANGLE_MIN, ANGLE_INC, N_BEAMS, RANGE_MAX = -2.35619, 0.004363323, 1080, 10.0
V_FULL_THROTTLE = 25.0
SPEED_TAU = 0.1  # s, speed response (measured: ~0.2 s to 90 %)
ACCEL_MAX = 6.0  # m/s^2, measured max acceleration
DECEL_MAX = 7.5  # m/s^2, measured max braking (even with negative throttle)
STEER_TAU = 0.05  # s, steering servo


def smooth_closed(a, k):
    """Circular moving average with window 2k+1 along axis 0."""
    out = np.zeros_like(a)
    for s in range(-k, k + 1):
        out += np.roll(a, s, axis=0)
    return out / (2 * k + 1)


def build_track(trace_csv, out_path, step=0.1):
    """Make wall polylines from a recorded lap (path + measured side distances)."""
    with open(trace_csv) as f:
        rows = list(csv.DictReader(f))
    xy = np.array([[float(r['x']), float(r['y'])] for r in rows])
    left = np.array([float(r['left_min']) for r in rows])
    right = np.array([float(r['right_min']) for r in rows])
    # resample the closed path every `step` metres
    seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    s = np.concatenate([[0], np.cumsum(seg)])
    n = int(s[-1] / step)
    grid = np.linspace(0, s[-1], n, endpoint=False)
    c = np.stack([np.interp(grid, s, xy[:, 0]), np.interp(grid, s, xy[:, 1])], axis=1)
    lw = np.interp(grid, s, left)
    rw = np.interp(grid, s, right)
    c = smooth_closed(c, 3)
    lw, rw = smooth_closed(lw, 5), smooth_closed(rw, 5)
    tangent = np.roll(c, -1, axis=0) - np.roll(c, 1, axis=0)
    tangent /= np.linalg.norm(tangent, axis=1, keepdims=True)
    normal = np.stack([-tangent[:, 1], tangent[:, 0]], axis=1)  # points left
    track = {
        'centerline': np.round(c, 3).tolist(),
        'left': np.round(c + normal * lw[:, None], 3).tolist(),
        'right': np.round(c - normal * rw[:, None], 3).tolist(),
        'start': [float(xy[0, 0]), float(xy[0, 1]),
                  float(math.atan2(xy[5, 1] - xy[0, 1], xy[5, 0] - xy[0, 0]))],
    }
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, 'w') as f:
        json.dump(track, f)
    print(f'track: {n} points, length {s[-1]:.1f} m, width {np.median(lw + rw):.2f} m '
          f'(min {np.min(lw + rw):.2f}) -> {out_path}')


class Track:
    def __init__(self, path=TRACK_FILE, mirror=False):
        with open(path) as f:
            t = json.load(f)
        flip = np.array([-1.0, 1.0]) if mirror else np.array([1.0, 1.0])
        self.center = np.array(t['centerline']) * flip
        walls = [np.array(t['left']) * flip, np.array(t['right']) * flip]
        a = np.concatenate([w for w in walls])
        b = np.concatenate([np.roll(w, -1, axis=0) for w in walls])
        self.seg_a, self.seg_d = a, b - a
        x, y, yaw = t['start']
        self.start = (x * flip[0], y, math.pi - yaw if mirror else yaw)
        seg = np.linalg.norm(np.diff(self.center, axis=0, append=self.center[:1]), axis=1)
        self.s = np.concatenate([[0], np.cumsum(seg)[:-1]])
        self.length = float(np.sum(seg))

    def progress(self, x, y):
        i = int(np.argmin(np.sum((self.center - [x, y]) ** 2, axis=1)))
        return self.s[i]

    def scan(self, x, y, yaw, noise=0.0, rng=None):
        ang = yaw + ANGLE_MIN + ANGLE_INC * np.arange(N_BEAMS)
        ox, oy = x + LIDAR_X * math.cos(yaw), y + LIDAR_X * math.sin(yaw)
        dx, dy = np.cos(ang)[:, None], np.sin(ang)[:, None]
        ax, ay = self.seg_a[:, 0] - ox, self.seg_a[:, 1] - oy
        ex, ey = self.seg_d[:, 0], self.seg_d[:, 1]
        den = dx * ey - dy * ex
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (ax * ey - ay * ex) / den  # distance along the ray
            u = (ax * dy - ay * dx) / den  # position along the segment
        hit = (t > 0) & (u >= 0) & (u <= 1) & np.isfinite(t)
        r = np.where(hit, t, np.inf).min(axis=1)
        if noise > 0.0:
            r = r + rng.normal(0.0, noise, r.shape)
        return np.minimum(r, RANGE_MAX)

    def clearance(self, points):
        """Smallest distance from any of the points (N x 2) to a wall segment."""
        px = points[:, 0:1] - self.seg_a[:, 0]
        py = points[:, 1:2] - self.seg_a[:, 1]
        dd = np.sum(self.seg_d ** 2, axis=1)
        u = np.clip((px * self.seg_d[:, 0] + py * self.seg_d[:, 1]) / dd, 0, 1)
        return float(np.min(np.hypot(px - u * self.seg_d[:, 0], py - u * self.seg_d[:, 1])))


def simulate(params, laps=3, mirror=False, scan_rate=40.0, control_rate=160.0, noise=0.0,
             latency=0, seed=0, max_time=120.0, track_file=TRACK_FILE):
    """Drive ``laps`` laps. Returns dict with status, lap times and min clearance.

    A new scan is taken at ``scan_rate``; the controller is called at
    ``control_rate`` and sees the latest scan again in between, like the real
    bridge. ``latency`` delays commands by that many control steps.
    """
    from autodrive_roboracer.wall_follow_node import WallFollowController, MAX_STEER_RAD
    track = Track(track_file, mirror)
    ctrl = WallFollowController(params)
    rng = np.random.default_rng(seed)
    x, y, yaw = track.start
    v, delta = 0.0, 0.0
    dt = 1.0 / control_rate
    next_scan, r = 0.0, None
    queue = [(0.0, 0.0)] * latency  # command delay in control steps
    s_prev = track.progress(x, y)
    travelled, lap_start, laps_done, min_clear, t = 0.0, 0.0, [], float('inf'), 0.0
    while t < max_time:
        if t >= next_scan:
            r = track.scan(x, y, yaw, noise, rng)
            next_scan += 1.0 / scan_rate
        throttle, steer, _ = ctrl.step(r, ANGLE_MIN, ANGLE_INC, RANGE_MAX, t, v)
        queue.append((throttle, steer))
        throttle, steer = queue.pop(0)
        dv = (V_FULL_THROTTLE * throttle - v) / SPEED_TAU
        v += float(np.clip(dv, -DECEL_MAX, ACCEL_MAX)) * dt
        delta += (steer * MAX_STEER_RAD - delta) * dt / STEER_TAU
        curvature = math.tan(delta) / WHEELBASE
        if v * v * abs(curvature) > A_GRIP:  # beyond grip: the car slides wide
            curvature = math.copysign(A_GRIP / (v * v), curvature)
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw += v * curvature * dt
        t += dt
        c, s_ = math.cos(yaw), math.sin(yaw)
        body = np.stack([x + c * FOOTPRINT[:, 0] - s_ * FOOTPRINT[:, 1],
                         y + s_ * FOOTPRINT[:, 0] + c * FOOTPRINT[:, 1]], axis=1)
        clear = track.clearance(body)
        min_clear = min(min_clear, clear)
        if clear < 0.01:
            return {'status': 'collision', 'laps': laps_done, 'time': round(t, 2),
                    'min_clearance': round(min_clear, 3)}
        s = track.progress(x, y)
        ds = s - s_prev
        if ds < -track.length / 2:
            ds += track.length
        elif ds > track.length / 2:
            ds -= track.length
        travelled += ds
        s_prev = s
        if travelled >= track.length * (len(laps_done) + 1):
            laps_done.append(round(t - lap_start, 2))
            lap_start = t
            if len(laps_done) >= laps:
                return {'status': 'ok', 'laps': laps_done, 'time': round(t, 2),
                        'min_clearance': round(min_clear, 3)}
    return {'status': 'timeout', 'laps': laps_done, 'time': round(t, 2),
            'min_clearance': round(min_clear, 3)}


def main():
    from autodrive_roboracer.wall_follow_node import Params, cast_param
    ap = argparse.ArgumentParser()
    ap.add_argument('--laps', type=int, default=3)
    ap.add_argument('--mirror', action='store_true', help='counter-clockwise track')
    ap.add_argument('--scan-rate', type=float, default=40.0, help='new LiDAR scans per s')
    ap.add_argument('--control-rate', type=float, default=160.0, help='controller calls per s')
    ap.add_argument('--noise', type=float, default=0.0, help='LiDAR noise std in m')
    ap.add_argument('--latency', type=int, default=0, help='command delay in steps')
    ap.add_argument('--build-track', metavar='TRACE_CSV')
    ap.add_argument('-p', '--param', action='append', default=[], help='name=value')
    args = ap.parse_args()
    if args.build_track:
        build_track(args.build_track, TRACK_FILE)
        return
    params = Params()
    by_name = {f.name: f for f in fields(Params)}
    for item in args.param:
        key, value = item.split('=', 1)
        setattr(params, key, cast_param(by_name[key], value))
    print(simulate(params, args.laps, args.mirror, args.scan_rate, args.control_rate,
                   args.noise, args.latency))


if __name__ == '__main__':
    main()
