#!/usr/bin/env python3
"""Measure acceleration and braking on the start straight (system identification).

The controller steers (so the car stays in the middle of the straight); the
throttle follows a fixed program. Each trial starts from a reset.

    python3 tools/sysid.py accel     # full throttle vs traction-limited throttle
    python3 tools/sysid.py brake     # braking from ~5 m/s with different throttle values
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))
import rclpy  # noqa: E402
from experiment import Runner  # noqa: E402
from autodrive_roboracer.wall_follow_node import Params, WallFollowController  # noqa: E402

V_FULL = 25.0


def trial(node, phases):
    """Run (throttle or f(speed), seconds) phases. Returns (t, v, x, y) samples."""
    node.reset_car()
    node.controller = WallFollowController(Params())
    node.raw_throttle = 0.0
    node.active = True
    t0, log = time.time(), []
    for thr, dur in phases:
        end = time.time() + dur
        while time.time() < end:
            node.raw_throttle = thr(node.speed) if callable(thr) else thr
            node.spin_for(0.005)
            log.append((time.time() - t0, node.speed, node.pose[0], node.pose[1]))
    node.active = False
    node.send(0.0, 0.0)
    return log


def speed_at(log, t):
    return min(log, key=lambda s: abs(s[0] - t))[1]


def main():
    what = sys.argv[1] if len(sys.argv) > 1 else 'accel'
    rclpy.init()
    node = Runner()
    try:
        if what == 'accel':
            programs = {'full throttle': 1.0}
            for slip in (0.1, 0.2, 0.4):
                programs[f'traction slip {slip}'] = (
                    lambda v, s=slip: min(1.0, (v * (1 + s) + 0.5) / V_FULL))
            for name, thr in programs.items():
                log = trial(node, [(thr, 1.4)])
                speeds = ' '.join(f'v({t}s)={speed_at(log, t):.2f}' for t in (0.4, 0.8, 1.2))
                print(f'{name:20s} {speeds} m/s', flush=True)
        else:
            for brake in (0.0, -0.3, -1.0, 0.04):
                log = trial(node, [(0.2, 1.5), (brake, 0.6)])
                v0, v1 = speed_at(log, 1.5), speed_at(log, 1.8)
                print(f'brake throttle {brake:+.2f}: {v0:.2f} -> {v1:.2f} m/s in 0.3 s = '
                      f'{(v0 - v1) / 0.3:.1f} m/s^2', flush=True)
        node.reset_car()
    finally:
        node.send(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
