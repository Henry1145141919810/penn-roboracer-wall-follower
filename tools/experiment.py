#!/usr/bin/env python3
"""Run timed laps in the simulator and record the results.

Runs inside the container (ROS sourced) while the bridge and the simulator are
up. Each run: reset the car, drive with ``WallFollowController`` (same code as
the node), stop at N laps, a wall contact, a stall, or the timeout.

    python3 tools/experiment.py --label baseline --laps 3 -p mode=right -p v_const=1.5
    python3 tools/experiment.py --label raw --raw-throttle 0.1 --seconds 8   # calibration

One JSON line per run goes to ``runs/results.jsonl``; a per-scan trace goes to
``runs/trace_<label>_<n>.csv`` (position, speed, commands, controller state).
"""

import argparse
from dataclasses import asdict, fields
import json
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, Int32

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'autodrive_devkit'))
from autodrive_roboracer.wall_follow_node import (  # noqa: E402
    Params, WallFollowController, THROTTLE_TOPIC, STEERING_TOPIC, LIDAR_TOPIC, ODOM_TOPIC)

NS = '/autodrive/roboracer_1/'


def parse_value(text):
    for cast in (int, float):
        try:
            return cast(text)
        except ValueError:
            pass
    if text.lower() in ('true', 'false'):
        return text.lower() == 'true'
    return text


class Runner(Node):
    def __init__(self):
        super().__init__('experiment_runner')
        qos = QoSProfile(depth=1)
        self.pub_t = self.create_publisher(Float32, THROTTLE_TOPIC, qos)
        self.pub_s = self.create_publisher(Float32, STEERING_TOPIC, qos)
        self.pub_reset = self.create_publisher(Bool, '/autodrive/reset_command', qos)
        self.create_subscription(LaserScan, LIDAR_TOPIC, self.on_scan, qos)
        self.create_subscription(Odometry, ODOM_TOPIC, self.on_odom, qos)
        self.create_subscription(Int32, NS + 'lap_count', self.on_lap, qos)
        self.create_subscription(Int32, NS + 'collision_count', self.on_coll, qos)
        self.create_subscription(Float32, NS + 'last_lap_time', self.on_last, qos)
        self.create_subscription(Float32, NS + 'lap_time', self.on_laptime, qos)
        self.controller = None
        self.raw_throttle = None
        self.active = False
        self.speed = 0.0
        self.pose = (0.0, 0.0, 0.0)
        self.lap_count = None
        self.collisions = None
        self.last_lap_time = None
        self.lap_time = 0.0
        self.trace = None
        self.scans = 0
        self.save_every = 0
        self.saved = []

    # --- sensor callbacks -------------------------------------------------
    def on_odom(self, msg):
        self.speed = msg.twist.twist.linear.x
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def on_lap(self, msg):
        self.lap_count = msg.data

    def on_coll(self, msg):
        self.collisions = msg.data

    def on_last(self, msg):
        self.last_lap_time = msg.data

    def on_laptime(self, msg):
        self.lap_time = msg.data

    def on_scan(self, msg):
        if not self.active:
            return
        self.scans += 1
        stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        if self.raw_throttle is not None:
            throttle, steer, info = self.raw_throttle, 0.0, {}
            if self.controller is not None:  # steer with the controller, fixed throttle
                _, steer, info = self.controller.step(
                    msg.ranges, msg.angle_min, msg.angle_increment, msg.range_max,
                    stamp, self.speed)
        else:
            throttle, steer, info = self.controller.step(
                msg.ranges, msg.angle_min, msg.angle_increment, msg.range_max,
                stamp, self.speed)
        self.send(throttle, steer)
        if self.save_every and self.scans % self.save_every == 0:
            self.saved.append((self.pose, list(msg.ranges)))
        if self.trace is not None:
            x, y, yaw = self.pose
            self.trace.write(
                f'{time.time():.4f},{self.lap_time:.3f},{self.lap_count},{self.collisions},'
                f'{x:.3f},{y:.3f},{yaw:.4f},{self.speed:.3f},{throttle:.4f},{steer:.4f},'
                f"{info.get('error', 0):.4f},{info.get('clearance', 0):.3f},"
                f"{info.get('v_target', 0):.3f},{info.get('left_min', 0):.3f},"
                f"{info.get('right_min', 0):.3f}\n")

    # --- helpers ----------------------------------------------------------
    def send(self, throttle, steer):
        self.pub_t.publish(Float32(data=float(throttle)))
        self.pub_s.publish(Float32(data=float(steer)))

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def reset_car(self):
        self.active = False
        for _ in range(5):
            self.send(0.0, 0.0)
            self.pub_reset.publish(Bool(data=True))
            self.spin_for(0.05)
        for _ in range(5):
            self.pub_reset.publish(Bool(data=False))
            self.spin_for(0.05)
        self.spin_for(1.0)


def run_once(node, args, params, index):
    node.reset_car()
    while node.lap_count is None or node.collisions is None:
        node.spin_for(0.1)
    node.controller = WallFollowController(params) if not args.no_steer else None
    node.raw_throttle = args.raw_throttle
    lap0, coll0 = node.lap_count, node.collisions
    os.makedirs(args.out_dir, exist_ok=True)
    trace_path = os.path.join(args.out_dir, f'trace_{args.label}_{index}.csv')
    node.trace = open(trace_path, 'w')
    node.trace.write('t,lap_time,lap_count,collisions,x,y,yaw,speed,throttle,steering,'
                     'error,clearance,v_target,left_min,right_min\n')

    laps, status = [], 'timeout'
    seen_lap, start, slow_since, max_speed = lap0, time.time(), None, 0.0
    speed_sum, speed_n = 0.0, 0
    node.scans = 0
    node.save_every = args.save_scans
    node.saved = []
    node.active = True
    while time.time() - start < args.timeout:
        node.spin_for(0.02)
        max_speed = max(max_speed, node.speed)
        speed_sum += node.speed
        speed_n += 1
        if node.collisions > coll0:
            status = 'collision'
            break
        if node.lap_count != seen_lap:
            node.spin_for(0.1)  # let last_lap_time update
            laps.append(round(node.last_lap_time, 3))
            seen_lap = node.lap_count
            print(f'  lap {len(laps)}: {laps[-1]:.3f} s', flush=True)
            if len(laps) >= args.laps:
                status = 'ok'
                break
        if args.seconds and time.time() - start > args.seconds:
            status = 'ok'
            break
        if node.speed < 0.05 and time.time() - start > 3.0:
            slow_since = slow_since or time.time()
            if time.time() - slow_since > 3.0:
                status = 'stalled'
                break
        else:
            slow_since = None
    wall = time.time() - start
    node.active = False
    node.send(0.0, 0.0)
    node.trace.close()
    node.trace = None
    if node.saved:
        import numpy as np
        np.savez_compressed(
            os.path.join(args.out_dir, f'scans_{args.label}_{index}.npz'),
            pose=np.array([p for p, _ in node.saved]),
            ranges=np.array([r for _, r in node.saved], dtype=np.float32))
    result = {
        'label': args.label, 'run': index, 'status': status, 'laps': laps,
        'best': min(laps) if laps else None,
        'mean': round(sum(laps) / len(laps), 3) if laps else None,
        'collisions': node.collisions - coll0, 'wall_s': round(wall, 2),
        'scan_hz': round(node.scans / wall, 1), 'max_speed': round(max_speed, 2),
        'mean_speed': round(speed_sum / max(speed_n, 1), 2),
        'raw_throttle': args.raw_throttle, 'params': asdict(params),
        'trace': os.path.basename(trace_path), 'time': time.strftime('%Y-%m-%d %H:%M:%S'),
    }
    with open(os.path.join(args.out_dir, 'results.jsonl'), 'a') as f:
        f.write(json.dumps(result) + '\n')
    keys = ('label', 'run', 'status', 'laps', 'best', 'collisions', 'max_speed', 'scan_hz')
    print(json.dumps({k: result[k] for k in keys}), flush=True)
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--label', required=True)
    ap.add_argument('--laps', type=int, default=3)
    ap.add_argument('--repeat', type=int, default=1)
    ap.add_argument('--timeout', type=float, default=180.0)
    ap.add_argument('--seconds', type=float, default=0.0, help='stop after this long')
    ap.add_argument('--raw-throttle', type=float, default=None,
                    help='fixed throttle; steering still from the controller')
    ap.add_argument('--no-steer', action='store_true', help='with --raw-throttle: steer 0')
    ap.add_argument('--save-scans', type=int, default=0, help='save every Nth scan + pose')
    ap.add_argument('-p', '--param', action='append', default=[], help='name=value')
    ap.add_argument('--out-dir', default=os.path.join(os.path.dirname(__file__), '..', 'runs'))
    args = ap.parse_args()

    params = Params()
    names = {f.name for f in fields(Params)}
    for item in args.param:
        key, value = item.split('=', 1)
        if key not in names:
            sys.exit(f'unknown parameter {key}')
        setattr(params, key, parse_value(value))

    rclpy.init()
    node = Runner()
    try:
        for i in range(args.repeat):
            run_once(node, args, params, i)
        node.reset_car()
    finally:
        node.send(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
