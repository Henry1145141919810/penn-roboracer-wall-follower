#!/usr/bin/env python3
"""Measure acceleration and braking on the start straight (system identification).

The controller steers (so the car stays in the middle of the straight); the
throttle follows a fixed program. Each trial starts from a reset.

    python3 tools/sysid.py accel     # full throttle vs traction-limited throttle
    python3 tools/sysid.py brake     # braking from ~5 m/s with different throttle values
    python3 tools/sysid.py steer     # steering latency and real curvature per command
"""

import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))
import rclpy  # noqa: E402
from experiment import Runner  # noqa: E402
from autodrive_roboracer.wall_follow_node import Params, WallFollowController  # noqa: E402

V_FULL = 25.0


def trial(node, phases, steer=0.0):
    """Run (throttle or f(speed), seconds) phases. Returns (t, v, x, y, yaw) samples.

    A phase whose throttle is the string 'steer' keeps the previous throttle and
    applies a fixed steering command ``steer`` instead of the controller's.
    """
    node.reset_car()
    node.controller = WallFollowController(Params())
    node.raw_throttle = 0.0
    node.fixed_steer = None
    node.active = True
    t0, log = time.time(), []
    for thr, dur in phases:
        end = time.time() + dur
        while time.time() < end:
            if thr == 'steer':
                node.fixed_steer = steer
            else:
                node.raw_throttle = thr(node.speed) if callable(thr) else thr
            node.spin_for(0.005)
            log.append((time.time() - t0, node.speed, node.pose[0], node.pose[1], node.pose[2]))
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
        elif what == 'steer':
            # Steering response at 3 m/s on the straight: latency and real curvature.
            for cmd in (0.15, 0.3, 0.5, 0.8):
                log = trial(node, [(0.12, 1.2), ('steer', 0.5)], steer=cmd)
                t = [s[0] for s in log]
                yaw = [s[4] for s in log]
                v = [s[1] for s in log]
                rate = [(yaw[i + 2] - yaw[i - 2]) / (t[i + 2] - t[i - 2])
                        for i in range(2, len(log) - 2)]
                t0 = 1.2
                onset = next((t[i + 2] - t0 for i, r in enumerate(rate)
                              if t[i + 2] > t0 and abs(r) > 0.15), None)
                late = [abs(r) for i, r in enumerate(rate) if t[i + 2] > t0 + 0.3]
                vv = [v[i + 2] for i, r in enumerate(rate) if t[i + 2] > t0 + 0.3]
                k_act = (sum(late) / len(late)) / max(sum(vv) / len(vv), 0.1)
                k_cmd = math.tan(cmd * 0.5236) / 0.33
                print(f'steer {cmd:.2f}: onset {onset if onset is None else round(onset, 3)} s, '
                      f'curvature actual {k_act:.2f} vs kinematic {k_cmd:.2f} 1/m '
                      f'(ratio {k_act / k_cmd:.2f}) at {sum(vv) / len(vv):.1f} m/s', flush=True)
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
