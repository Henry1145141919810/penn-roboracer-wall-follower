#!/usr/bin/env python3
"""LiDAR wall follower for the AutoDRIVE RoboRacer simulator.

One LaserScan in, one throttle and one steering command out:

1. Clean the scan: NaN, inf and out-of-range beams become ``range_max``.
2. Fit each side wall from two beams. This gives the distance to the wall and
   the car's heading relative to it, projected ``lookahead`` metres forward.
3. Steer with PD control on the lateral error.
   ``mode='center'`` drives the midline between both walls (default).
   ``mode='left'`` or ``'right'`` is the classic single-wall follower.
4. Pick a target speed. ``speed_mode='adaptive'`` (default) uses the free
   distance straight ahead (braking model) and the steering angle (lateral
   acceleration limit). ``'steps'`` is the textbook F1TENTH schedule (slower
   as the steering angle grows); ``'const'`` is one fixed speed.
5. A side guard pushes away from any wall closer than ``side_clearance``.

``WallFollowController`` is plain numpy so it can be unit tested and run in
the offline simulator without ROS. ``WallFollowNode`` is the ROS 2 wrapper.
"""

from dataclasses import dataclass, fields
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

LIDAR_TOPIC = '/autodrive/roboracer_1/lidar'
ODOM_TOPIC = '/autodrive/roboracer_1/odom'
THROTTLE_TOPIC = '/autodrive/roboracer_1/throttle_command'
STEERING_TOPIC = '/autodrive/roboracer_1/steering_command'

MAX_STEER_RAD = 0.5236  # steering command 1.0 is full left lock
WHEELBASE = 0.33  # m, rear axle to front axle (from the bridge TF tree)


@dataclass
class Params:
    """Tunable parameters. Each one is also a ROS parameter of the node."""

    # Steering
    mode: str = 'center'  # 'center', 'left' or 'right'
    wall_distance: float = 0.8  # m, target distance in 'left'/'right' mode
    beam_angle: float = 50.0  # deg between the two beams of each wall fit
    lookahead: float = 0.5  # m, project the lateral error this far ahead ...
    lookahead_per_speed: float = 0.15  # ... plus this many metres per m/s of speed
    kp: float = 0.9  # rad of steering per m of error
    ki: float = 0.0
    kd: float = 0.06
    d_filter_tau: float = 0.05  # s, low-pass on the error derivative
    steer_tau: float = 0.0  # s, low-pass on the steering command (0 = off)
    use_side_guard: bool = True
    side_clearance: float = 0.35  # m, push away from walls closer than this
    side_gain: float = 2.0  # rad per m of intrusion
    # Speed (m/s targets, turned into throttle below)
    speed_mode: str = 'adaptive'  # 'adaptive', 'steps' or 'const'
    v_max: float = 5.0  # m/s on open straights
    v_min: float = 1.0  # m/s floor so the car never stalls
    v_const: float = 1.5  # m/s in 'const' mode
    step_speeds: str = '1.5,1.0,0.5'  # m/s for |steer| < 10 deg, < 20 deg, above
    brake_decel: float = 4.0  # m/s^2 assumed braking for the clearance limit
    lat_accel: float = 6.0  # m/s^2 allowed lateral acceleration in turns
    stop_margin: float = 0.5  # m kept in front when braking to a stop
    car_half_width: float = 0.25  # m, corridor half width for the clearance check
    # Throttle mapping: throttle = v_target / v_full_throttle + speed_kp * error.
    # Measured in the simulator: steady speed = 25 m/s * throttle, reached in < 0.5 s.
    v_full_throttle: float = 25.0  # m/s the car reaches at throttle 1.0
    speed_kp: float = 0.0  # throttle per m/s of speed error (needs odometry)
    max_throttle: float = 1.0
    min_throttle: float = -1.0


def cast_param(field, value):
    """Convert a ROS parameter value to the type of a ``Params`` field."""
    if field.type in (bool, 'bool') and isinstance(value, str):
        return value.lower() == 'true'
    for typ in (bool, float, str):
        if field.type in (typ, typ.__name__):
            return typ(value)
    return value


def clean_ranges(ranges, range_max):
    """Return ranges as floats with invalid beams set to ``range_max``."""
    r = np.asarray(ranges, dtype=float)
    r = np.where(np.isfinite(r) & (r > 0.0), r, range_max)
    return np.minimum(r, range_max)


def beam_index(angle, angle_min, angle_increment, n):
    """Index of the beam closest to ``angle`` (rad), clamped to the scan."""
    i = int(round((angle - angle_min) / angle_increment))
    return min(max(i, 0), n - 1)


def wall_estimate(ranges, angle_min, angle_increment, side, beam_angle, range_max):
    """Fit one side wall from two beams.

    ``side`` is +1 for the left wall and -1 for the right wall. Beam ``b`` is
    perpendicular to the car (90 deg to that side) and beam ``a`` is
    ``beam_angle`` closer to straight ahead. For a straight wall at distance D
    whose direction is rotated by alpha away from the car:

        b = D / cos(alpha),   a = D / cos(theta + alpha)
        tan(alpha) = (a cos(theta) - b) / (a sin(theta))

    Returns ``(distance, alpha)`` or ``None`` when either beam sees no wall.
    """
    n = len(ranges)
    theta = math.radians(beam_angle)
    b = ranges[beam_index(side * math.pi / 2, angle_min, angle_increment, n)]
    a = ranges[beam_index(side * (math.pi / 2 - theta), angle_min, angle_increment, n)]
    if a >= 0.98 * range_max or b >= 0.98 * range_max:
        return None
    alpha = math.atan2(a * math.cos(theta) - b, a * math.sin(theta))
    return b * math.cos(alpha), alpha


def forward_clearance(ranges, angles, half_width):
    """Distance to the nearest return inside a straight corridor ahead."""
    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)
    in_path = (x > 0.0) & (np.abs(y) < half_width)
    if not np.any(in_path):
        return float(np.max(ranges))
    return float(np.min(x[in_path]))


def side_minimum(ranges, angles, side, lo_deg=20.0, hi_deg=160.0):
    """Closest return in the sector [lo, hi] deg on one side (+1 left, -1 right)."""
    a = side * angles
    sector = (a > math.radians(lo_deg)) & (a < math.radians(hi_deg))
    if not np.any(sector):
        return float('inf')
    return float(np.min(ranges[sector]))


class WallFollowController:
    """Turns one LiDAR scan into (throttle, steering) commands in [-1, 1]."""

    def __init__(self, params=None):
        self.p = params if params is not None else Params()
        self.reset()

    def reset(self):
        """Clear controller memory (call after the car is reset)."""
        self.prev_error = None
        self.prev_time = None
        self.integral = 0.0
        self.d_filtered = 0.0
        self.track_width = None
        self.prev_steer = 0.0
        self.v_estimate = 0.0

    def lateral_error(self, ranges, angle_min, angle_increment, range_max, lookahead):
        """Signed error in metres. Positive means steer left."""
        p = self.p
        left = wall_estimate(ranges, angle_min, angle_increment, +1,
                             p.beam_angle, range_max)
        right = wall_estimate(ranges, angle_min, angle_increment, -1,
                              p.beam_angle, range_max)
        d_left = d_right = None
        if left is not None:
            d_left = left[0] + lookahead * math.sin(left[1])
        if right is not None:
            d_right = right[0] + lookahead * math.sin(right[1])

        if left is not None and right is not None:
            width = left[0] + right[0]
            self.track_width = (width if self.track_width is None
                                else 0.9 * self.track_width + 0.1 * width)

        if p.mode == 'left' and d_left is not None:
            return d_left - p.wall_distance
        if p.mode == 'right' and d_right is not None:
            return p.wall_distance - d_right

        # Centre mode, or fallback when the followed wall is missing.
        half = (self.track_width if self.track_width is not None
                else 2.0 * p.wall_distance) / 2.0
        if d_left is not None and d_right is not None:
            return (d_left - d_right) / 2.0
        if d_left is not None:
            return d_left - half
        if d_right is not None:
            return half - d_right
        return 0.0

    def step(self, ranges, angle_min, angle_increment, range_max, stamp=None,
             speed=None):
        """Compute commands for one scan.

        ``stamp`` is the scan time in seconds and ``speed`` the measured speed
        in m/s (``None`` when odometry is not available).
        Returns ``(throttle, steering, info)``.
        """
        p = self.p
        r = clean_ranges(ranges, range_max)
        angles = angle_min + angle_increment * np.arange(len(r))

        # Steering: PD on the lateral error projected ahead of the car.
        v_now = speed if speed is not None else self.v_estimate
        lookahead = p.lookahead + p.lookahead_per_speed * max(v_now, 0.0)
        error = self.lateral_error(r, angle_min, angle_increment, range_max, lookahead)
        dt = 0.025
        if stamp is not None and self.prev_time is not None:
            if 1e-4 < stamp - self.prev_time < 0.5:
                dt = stamp - self.prev_time
        if self.prev_error is not None:
            raw_d = (error - self.prev_error) / dt
            k = dt / (p.d_filter_tau + dt)  # time-based, so the scan rate does not matter
            self.d_filtered += k * (raw_d - self.d_filtered)
        self.integral = float(np.clip(self.integral + error * dt, -1.0, 1.0))
        self.prev_error = error
        self.prev_time = stamp
        delta = p.kp * error + p.kd * self.d_filtered + p.ki * self.integral

        # Side guard: push away from a wall that is too close.
        left_min = side_minimum(r, angles, +1)
        right_min = side_minimum(r, angles, -1)
        if p.use_side_guard:
            if left_min < p.side_clearance:
                delta -= p.side_gain * (p.side_clearance - left_min)
            if right_min < p.side_clearance:
                delta += p.side_gain * (p.side_clearance - right_min)

        delta = float(np.clip(delta, -MAX_STEER_RAD, MAX_STEER_RAD))
        steer = delta / MAX_STEER_RAD
        if p.steer_tau > 0.0:
            steer = self.prev_steer + dt / (p.steer_tau + dt) * (steer - self.prev_steer)
        self.prev_steer = steer

        # Speed: braking-distance limit ahead and lateral-acceleration limit.
        clearance = forward_clearance(r, angles, p.car_half_width)
        if p.speed_mode == 'adaptive':
            v_brake = math.sqrt(2.0 * p.brake_decel * max(clearance - p.stop_margin, 0.0))
            curvature = abs(math.tan(delta)) / WHEELBASE
            v_turn = math.sqrt(p.lat_accel / curvature) if curvature > 1e-6 else p.v_max
            v_target = max(min(p.v_max, v_brake, v_turn), p.v_min)
        elif p.speed_mode == 'steps':
            fast, mid, slow = (float(v) for v in p.step_speeds.split(','))
            angle = abs(math.degrees(delta))
            v_target = fast if angle < 10.0 else mid if angle < 20.0 else slow
        else:
            v_target = p.v_const
        self.v_estimate = v_target

        throttle = v_target / p.v_full_throttle
        if speed is not None and p.speed_kp > 0.0:
            throttle += p.speed_kp * (v_target - speed)
        throttle = float(np.clip(throttle, p.min_throttle, p.max_throttle))
        steer = float(np.clip(steer, -1.0, 1.0))

        info = {'error': error, 'clearance': clearance, 'v_target': v_target,
                'left_min': left_min, 'right_min': right_min,
                'track_width': self.track_width}
        return throttle, steer, info


class WallFollowNode(Node):
    """ROS 2 node: LiDAR scan in, throttle and steering commands out."""

    def __init__(self):
        super().__init__('wall_follow_node')
        defaults = Params()
        any_type = ParameterDescriptor(dynamic_typing=True)  # accept 6 as well as 6.0
        for f in fields(Params):
            self.declare_parameter(f.name, getattr(defaults, f.name), any_type)
        self.declare_parameter('log_path', '')  # CSV telemetry, empty = off
        self.controller = WallFollowController(self._read_params())
        self.add_on_set_parameters_callback(self._on_params)

        qos = QoSProfile(depth=1)
        self.pub_throttle = self.create_publisher(Float32, THROTTLE_TOPIC, qos)
        self.pub_steering = self.create_publisher(Float32, STEERING_TOPIC, qos)
        self.create_subscription(LaserScan, LIDAR_TOPIC, self._on_scan, qos)
        self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, qos)
        self.speed = None

        self.log_file = None
        log_path = self.get_parameter('log_path').value
        if log_path:
            self.log_file = open(log_path, 'w')
            self.log_file.write('t,throttle,steering,speed,error,clearance,v_target,'
                                'left_min,right_min\n')
        self.get_logger().info(f'wall_follow_node started: {self.controller.p}')

    def _read_params(self):
        return Params(**{f.name: cast_param(f, self.get_parameter(f.name).value)
                         for f in fields(Params)})

    def _on_params(self, changes):
        p = self.controller.p
        by_name = {f.name: f for f in fields(Params)}
        try:
            for change in changes:
                if change.name in by_name:
                    setattr(p, change.name, cast_param(by_name[change.name], change.value))
        except (TypeError, ValueError) as e:
            return SetParametersResult(successful=False, reason=str(e))
        return SetParametersResult(successful=True)

    def _on_odom(self, msg):
        v = msg.twist.twist.linear
        self.speed = math.hypot(v.x, v.y)

    def _on_scan(self, msg):
        stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        throttle, steering, info = self.controller.step(
            msg.ranges, msg.angle_min, msg.angle_increment, msg.range_max,
            stamp, self.speed)
        self.publish(throttle, steering)
        if self.log_file is not None:
            self.log_file.write(
                f'{time.time():.3f},{throttle:.3f},{steering:.3f},{self.speed or 0.0:.3f},'
                f"{info['error']:.3f},{info['clearance']:.3f},{info['v_target']:.3f},"
                f"{info['left_min']:.3f},{info['right_min']:.3f}\n")
        self.get_logger().info(
            f"v={self.speed or 0.0:.2f} target={info['v_target']:.2f} "
            f"thr={throttle:.2f} steer={steering:+.2f} err={info['error']:+.2f} "
            f"ahead={info['clearance']:.2f}", throttle_duration_sec=1.0)

    def publish(self, throttle, steering):
        """Send one throttle and one steering command."""
        self.pub_throttle.publish(Float32(data=float(throttle)))
        self.pub_steering.publish(Float32(data=float(steering)))

    def close(self):
        """Stop the car and close the log. The bridge keeps the last command."""
        try:
            self.publish(0.0, 0.0)
        except Exception:  # context may already be shut down
            pass
        if self.log_file is not None:
            self.log_file.close()


def main(args=None):
    # Handle Ctrl+C ourselves so close() can still publish the stop command.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = WallFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
