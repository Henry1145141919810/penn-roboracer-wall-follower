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
   distance along the arc the car is steering on (braking model) and the
   steering angle (lateral acceleration limit). ``'steps'`` is the textbook
   F1TENTH schedule (slower as the steering angle grows); ``'const'`` is one
   fixed speed.
5. A side guard pushes away from any wall closer than ``side_clearance``.
6. Corner preview: the mean direction of the farthest returns shows where the
   track turns; ``preview_gain`` adds it to the error so the car turns in a
   little earlier and takes a wider, faster arc.

The LiDAR produces ~40 new scans/s but the bridge republishes each one 3-5
times (~160 messages/s). Steering is only recomputed on a new scan, so the
derivative sees real changes; throttle is refreshed on every message.

``WallFollowController`` is plain numpy so it can be unit tested and run in
the offline simulator without ROS. ``WallFollowNode`` is the ROS 2 wrapper.
"""

from dataclasses import dataclass, fields, replace
import math
import signal
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
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
    steer_mode: str = 'pursuit'  # 'pd' (PD on the lateral error) or 'pursuit' (see below)
    # Steering law for the wall-centring error. 'pd': delta = kp * error (fixed gain).
    # 'pursuit_wall': pure-pursuit geometry on the same error, delta = steer_gain *
    # atan(WB * 2 * error / L_d^2), i.e. the arc that reaches the corridor midline L_d ahead.
    # The gain falls with speed (L_d grows), so hairpins get the steering they need without
    # the straights becoming twitchy. L_d is floored at lookahead_min.
    steer_law: str = 'pd'
    steer_gain: float = 0.9
    lookahead_min: float = 0.45  # m
    # Inside line: a bounded offset towards the inside of the coming turn (sign of the deepest
    # free direction), saturating with tanh and never closer than inner_clearance_min to the
    # inner wall. Replaces the linear preview when inside_offset > 0.
    inside_offset: float = 0.0  # m, 0 = off
    inside_alpha: float = 0.6  # rad, turn angle at which the offset is 76 % saturated
    inner_clearance_min: float = 0.45  # m
    wall_distance: float = 0.8  # m, target distance in 'left'/'right' mode
    beam_angle: float = 50.0  # deg between the two beams of each wall fit
    lookahead: float = 0.26  # m, project the lateral error this far ahead ...
    lookahead_per_speed: float = 0.15  # ... plus this many metres per m/s of speed
    kp: float = 0.084  # rad of steering per m of error
    ki: float = 0.0
    kd: float = 0.03
    d_filter_tau: float = 0.05  # s, low-pass on the error derivative
    max_wall_angle: float = 45.0  # deg, clamp on the fitted wall angle (rejects bad fits)
    max_error: float = 0.8  # m, clamp on the lateral error (track is ~1.5 m wide)
    steer_tau: float = 0.0  # s, low-pass on the steering command (0 = off)
    preview_gain: float = 0.0  # m of error per rad towards the deepest free direction
    preview_fov: float = 60.0  # deg, half-width of the sector searched for that direction
    # Pursuit steering: aim at the centre of the free sector at a lookahead range with
    # pure-pursuit geometry (curvature = 2 sin(theta) / L). The scan is first disparity-
    # extended by the car's half width, so a "free" direction is one the car fits through.
    # The PD centring term (kp) and the side guard still apply on top.
    pursuit_lookahead: float = 1.93  # m at standstill ...
    pursuit_lookahead_per_speed: float = 0.20  # ... plus this many metres per m/s
    pursuit_min_lookahead: float = 1.17  # m, shrink to this when nothing is free further out
    disparity_threshold: float = 0.3  # m, range jump between neighbours that marks an edge
    disparity_margin: float = 0.035  # m added to the half width when extending edges
    apex_shift: float = 0.24  # m, move the aim point towards the inside of the turn
    use_side_guard: bool = True
    side_clearance: float = 0.37  # m, push away from walls closer than this
    side_gain: float = 2.0  # rad per m of intrusion
    # Speed (m/s targets, turned into throttle below)
    speed_mode: str = 'adaptive'  # 'adaptive', 'steps' or 'const'
    v_max: float = 7.61  # m/s on open straights
    v_min: float = 1.0  # m/s floor so the car never stalls
    v_const: float = 1.5  # m/s in 'const' mode
    step_speeds: str = '1.5,1.0,0.5'  # m/s for |steer| < 10 deg, < 20 deg, above
    brake_decel: float = 5.35  # m/s^2 assumed braking (measured ~6 m/s^2)
    v_corner: float = 1.18  # m/s braking aims for this speed at the obstacle, not a full stop
    lat_accel: float = 8.34  # m/s^2 allowed lateral acceleration (CMA-ES tuned; grip ~8)
    stop_margin: float = 0.31  # m kept in front when braking to a stop
    car_half_width: float = 0.25  # m, corridor half width for the clearance check
    clearance_mode: str = 'arc'  # 'arc' (along the steering arc) or 'straight'
    # Throttle mapping: throttle = v_target / v_full_throttle + speed_kp * error.
    # Measured in the simulator: steady speed = 25 m/s * throttle, reached in < 0.5 s.
    v_full_throttle: float = 25.0  # m/s the car reaches at throttle 1.0
    speed_kp: float = 0.2  # throttle per m/s of speed error (needs odometry)
    max_throttle: float = 1.0
    min_throttle: float = -1.0
    # Traction control: throttle sets wheel speed, and full throttle from low speed spins the
    # wheels (measured: 5.4 m/s after 1.2 s, vs 7.4 m/s with the wheel speed capped 10 % above
    # the car's speed). Cap = speed * (1 + traction_slip) + traction_offset. 0 = off.
    traction_slip: float = 0.16
    traction_offset: float = 0.5  # m/s, lets the car start from standstill


def cast_param(field, value):
    """Convert a ROS parameter value to the type of a ``Params`` field."""
    if field.type in (bool, 'bool') and isinstance(value, str):
        return value.lower() == 'true'
    for typ in (bool, float, str):
        if field.type in (typ, typ.__name__):
            return typ(value)
    return value


def validate(params):
    """Raise ValueError if a parameter set cannot be used."""
    choices = {'mode': ('center', 'left', 'right'),
               'steer_mode': ('pd', 'pursuit'),
               'steer_law': ('pd', 'pursuit_wall'),
               'speed_mode': ('adaptive', 'steps', 'const'),
               'clearance_mode': ('arc', 'straight')}
    for name, allowed in choices.items():
        if getattr(params, name) not in allowed:
            raise ValueError(f'{name} must be one of {allowed}')
    speeds = params.step_speeds.split(',')
    if len(speeds) != 3:
        raise ValueError('step_speeds needs three comma-separated speeds')
    [float(v) for v in speeds]
    if params.v_full_throttle <= 0.0 or params.d_filter_tau < 0.0 or params.steer_tau < 0.0:
        raise ValueError('v_full_throttle must be > 0 and filter time constants >= 0')
    if params.traction_slip > 0.0 and params.traction_offset <= 0.0:
        raise ValueError('traction_offset must be > 0, or the car can never start moving')


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


def arc_clearance(ranges, angles, half_width, curvature, max_angle=math.pi / 2):
    """Free distance along the circular arc the car is steering on.

    The arc starts at the sensor, tangent to the heading, with signed
    ``curvature`` (1/m, + = left). A return blocks the path when it lies within
    ``half_width`` of the arc. Only the first ``max_angle`` of the turn is
    checked. Falls back to the straight corridor for tiny curvature.
    """
    if abs(curvature) < 1e-3:
        return forward_clearance(ranges, angles, half_width)
    radius = 1.0 / curvature  # centre of the turn at (0, radius)
    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles) - radius
    in_band = np.abs(np.hypot(x, y) - abs(radius)) < half_width
    # angle travelled around the centre, measured in the driving direction
    swept = np.arctan2(x, -y) if radius > 0 else np.arctan2(x, y)
    blocked = in_band & (swept > 0.0) & (swept < max_angle)
    if not np.any(blocked):
        return abs(radius) * max_angle
    return float(abs(radius) * np.min(swept[blocked]))


def deepest_direction(ranges, angles, fov_deg):
    """Mean angle of the farthest returns within +-fov: where the track opens up.

    Beams within 90 % of the maximum range (after a 5-beam moving average) are
    averaged, so a single long beam through a gap does not dominate.
    """
    sector = np.abs(angles) < math.radians(fov_deg)
    r = np.convolve(ranges, np.ones(5) / 5.0, mode='same')[sector]
    far = r >= 0.9 * np.max(r)
    return float(np.mean(angles[sector][far]))


def extend_disparities(ranges, angle_increment, half_width, threshold):
    """Otterness-style disparity extender.

    Wherever two neighbouring beams differ by more than ``threshold``, the
    nearer range is copied over the beams on the far side that the car's half
    width would sweep at that distance. A direction left "free" afterwards is
    one the car fits through without clipping the edge.
    """
    r = ranges.copy()
    n = len(r)
    jumps = np.flatnonzero(np.abs(np.diff(ranges)) > threshold)
    for i in jumps:
        if ranges[i] < ranges[i + 1]:
            near, start, step = ranges[i], i + 1, +1
        else:
            near, start, step = ranges[i + 1], i, -1
        if near < 1e-3:
            continue
        width = int(math.asin(min(1.0, half_width / near)) / angle_increment) + 1
        if step > 0:
            lo, hi = start, min(n, start + width)
        else:
            lo, hi = max(0, start - width + 1), start + 1
        r[lo:hi] = np.minimum(r[lo:hi], near)
    return r


def pursuit_target(ranges, angles, lookahead, min_lookahead, fov_deg, hint):
    """Aim point for pure pursuit: centre of the free sector at range ``lookahead``.

    Free beams are those longer than the lookahead within +-fov. Of the
    contiguous free sectors, the one containing ``hint`` (the deepest
    direction) is used, or else the one closest to it. If nothing is free at
    that range the range shrinks, down to ``min_lookahead``.
    Returns ``(theta, range_used)``; ``theta`` falls back to ``hint``.
    """
    fov = math.radians(fov_deg)
    in_fov = np.abs(angles) < fov
    L, last = lookahead, False
    while not last:
        if L <= min_lookahead:
            L, last = min_lookahead, True
        free = in_fov & (ranges > L)
        if np.any(free):
            edges = np.flatnonzero(np.diff(np.r_[False, free, False]))
            starts, stops = edges[::2], edges[1::2]
            best = None
            for a, b in zip(starts, stops):
                lo, hi = angles[a], angles[b - 1]
                dist = 0.0 if lo <= hint <= hi else min(abs(lo - hint), abs(hi - hint))
                if best is None or dist < best[0]:
                    best = (dist, 0.5 * (lo + hi))
            return float(best[1]), L
        L *= 0.8
    # Nothing is free even at min_lookahead: aim at the farthest (extended) beam in the FOV.
    far = np.where(in_fov, ranges, -1.0)
    return float(angles[int(np.argmax(far))]), min_lookahead


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
        validate(self.p)
        self.reset()

    def reset(self):
        """Clear controller memory (call after the car is reset)."""
        self.prev_error = None
        self.prev_branch = None
        self.prev_time = None
        self.prev_raw = None
        self.last = None
        self.integral = 0.0
        self.d_filtered = 0.0
        self.track_width = None
        self.prev_steer = 0.0
        self.v_estimate = 0.0

    def lateral_error(self, ranges, angle_min, angle_increment, range_max, lookahead,
                      dt=0.025):
        """Return ``(error, branch)``. Positive error means steer left.

        ``branch`` names the walls that were used ('both', 'left', 'right' or
        'none'); the derivative is reset when it changes.
        """
        p = self.p
        max_alpha = math.radians(p.max_wall_angle)
        left = wall_estimate(ranges, angle_min, angle_increment, +1,
                             p.beam_angle, range_max)
        right = wall_estimate(ranges, angle_min, angle_increment, -1,
                              p.beam_angle, range_max)
        d_left = d_right = None
        if left is not None:
            d_left = left[0] + lookahead * math.sin(np.clip(left[1], -max_alpha, max_alpha))
        if right is not None:
            d_right = right[0] + lookahead * math.sin(np.clip(right[1], -max_alpha, max_alpha))

        # Remember the track width, only from clean fits (both walls roughly parallel).
        if (left is not None and right is not None
                and abs(left[1]) < 0.25 and abs(right[1]) < 0.25):
            width = left[0] + right[0]
            k = dt / (0.5 + dt)  # 0.5 s time constant
            self.track_width = (width if self.track_width is None
                                else self.track_width + k * (width - self.track_width))

        if p.mode == 'left' and d_left is not None:
            return d_left - p.wall_distance, 'left'
        if p.mode == 'right' and d_right is not None:
            return p.wall_distance - d_right, 'right'

        # Centre mode, or fallback when the followed wall is missing.
        half = (self.track_width if self.track_width is not None
                else 2.0 * p.wall_distance) / 2.0
        if d_left is not None and d_right is not None:
            return (d_left - d_right) / 2.0, 'both'
        if d_left is not None:
            return d_left - half, 'left'
        if d_right is not None:
            return half - d_right, 'right'
        return 0.0, 'none'

    def step(self, ranges, angle_min, angle_increment, range_max, stamp=None,
             speed=None):
        """Compute commands for one LaserScan message.

        ``stamp`` is the message time in seconds and ``speed`` the measured
        forward speed in m/s (``None`` when odometry is not available).
        Returns ``(throttle, steering, info)``.
        """
        raw = np.asarray(ranges, dtype=float)
        if (self.last is not None and raw.shape == self.prev_raw.shape
                and np.array_equal(raw, self.prev_raw, equal_nan=True)):
            # Same scan republished: keep its steering, refresh throttle only.
            steer, v_target, info = self.last
            return self._throttle(v_target, speed), steer, info
        self.prev_raw = raw
        steer, v_target, info = self._new_scan(raw, angle_min, angle_increment,
                                               range_max, stamp, speed)
        self.last = (steer, v_target, info)
        return self._throttle(v_target, speed), steer, info

    def _throttle(self, v_target, speed):
        p = self.p
        throttle = v_target / p.v_full_throttle
        if speed is not None and p.speed_kp > 0.0:
            throttle += p.speed_kp * (v_target - speed)
        if speed is not None and p.traction_slip > 0.0:
            cap = (max(speed, 0.0) * (1.0 + p.traction_slip) + p.traction_offset)
            throttle = min(throttle, cap / p.v_full_throttle)
        return float(np.clip(throttle, p.min_throttle, p.max_throttle))

    def _new_scan(self, raw, angle_min, angle_increment, range_max, stamp, speed):
        p = self.p
        r = clean_ranges(raw, range_max)
        angles = angle_min + angle_increment * np.arange(len(r))

        # Time since the previous new scan. After a long gap, restart the derivative.
        dt = 0.025
        if stamp is not None and self.prev_time is not None:
            gap = stamp - self.prev_time
            if 1e-4 < gap < 0.5:
                dt = gap
            else:
                self.prev_error = None
        self.prev_time = stamp

        # Steering: PD on the lateral error projected ahead of the car.
        v_now = speed if speed is not None else self.v_estimate
        lookahead = p.lookahead + p.lookahead_per_speed * max(v_now, 0.0)
        if p.steer_law == 'pursuit_wall':
            lookahead = max(lookahead, p.lookahead_min)
        error, branch = self.lateral_error(r, angle_min, angle_increment, range_max,
                                           lookahead, dt)
        left_min = side_minimum(r, angles, +1)
        right_min = side_minimum(r, angles, -1)
        deepest = deepest_direction(r, angles, p.preview_fov)
        if p.preview_gain > 0.0:  # steer early towards where the track turns
            error += p.preview_gain * deepest
        e_ref = 0.0
        if p.inside_offset > 0.0:  # bounded target offset towards the inside of the turn
            e_ref = p.inside_offset * math.tanh(deepest / p.inside_alpha)
            d_inner = left_min if deepest > 0.0 else right_min
            room = max(d_inner - p.inner_clearance_min, 0.0)
            e_ref = float(np.clip(e_ref, -room, room))
            error += e_ref
        error = float(np.clip(error, -p.max_error, p.max_error))
        if self.prev_error is None or branch != self.prev_branch:
            self.d_filtered = 0.0  # no derivative across a change of reference wall
        else:
            raw_d = (error - self.prev_error) / dt
            k = dt / (p.d_filter_tau + dt)  # time-based, so the scan rate does not matter
            self.d_filtered += k * (raw_d - self.d_filtered)
        self.integral = float(np.clip(self.integral + error * dt, -1.0, 1.0))
        self.prev_error = error
        self.prev_branch = branch
        if p.steer_law == 'pursuit_wall':
            kappa = 2.0 * error / lookahead ** 2
            delta = p.steer_gain * math.atan(WHEELBASE * kappa)
        else:
            delta = p.kp * error
        delta += p.kd * self.d_filtered + p.ki * self.integral
        aim = None
        if p.steer_mode == 'pursuit':
            # Geometric feed-forward: the arc that reaches the aim point.
            L_d = p.pursuit_lookahead + p.pursuit_lookahead_per_speed * max(v_now, 0.0)
            r_ext = extend_disparities(r, angle_increment,
                                       p.car_half_width + p.disparity_margin,
                                       p.disparity_threshold)
            theta, L_used = pursuit_target(r_ext, angles, L_d, p.pursuit_min_lookahead,
                                           90.0, deepest)
            if p.apex_shift > 0.0 and abs(theta) > 0.05:
                theta = math.atan2(L_used * math.sin(theta) + math.copysign(p.apex_shift, theta),
                                   L_used * math.cos(theta))
            kappa = 2.0 * math.sin(theta) / L_used
            delta += math.atan(kappa * WHEELBASE)
            aim = (theta, L_used)

        # Side guard: push away from a wall that is too close.
        if p.use_side_guard:
            if left_min < p.side_clearance:
                delta -= p.side_gain * (p.side_clearance - left_min)
            if right_min < p.side_clearance:
                delta += p.side_gain * (p.side_clearance - right_min)

        delta = float(np.clip(delta, -MAX_STEER_RAD, MAX_STEER_RAD))
        steer = delta / MAX_STEER_RAD
        if p.steer_tau > 0.0:
            steer = self.prev_steer + dt / (p.steer_tau + dt) * (steer - self.prev_steer)
        steer = float(np.clip(steer, -1.0, 1.0))
        self.prev_steer = steer

        # Speed: braking-distance limit along the path and lateral-acceleration limit.
        curvature = math.tan(delta) / WHEELBASE
        if p.clearance_mode == 'arc':
            clearance = arc_clearance(r, angles, p.car_half_width, curvature)
        else:
            clearance = forward_clearance(r, angles, p.car_half_width)
        if p.speed_mode == 'adaptive':
            room = max(clearance - p.stop_margin, 0.0)
            v_brake = math.sqrt(p.v_corner ** 2 + 2.0 * p.brake_decel * room)
            k = abs(curvature)
            v_turn = math.sqrt(p.lat_accel / k) if k > 1e-6 else p.v_max
            v_target = max(min(p.v_max, v_brake, v_turn), p.v_min)
        elif p.speed_mode == 'steps':
            fast, mid, slow = (float(v) for v in p.step_speeds.split(','))
            angle = abs(math.degrees(delta))
            v_target = fast if angle < 10.0 else mid if angle < 20.0 else slow
        else:
            v_target = p.v_const
        self.v_estimate = v_target

        info = {'error': error, 'branch': branch, 'clearance': clearance, 'aim': aim,
                'e_ref': e_ref, 'lookahead': lookahead,
                'v_target': v_target, 'left_min': left_min, 'right_min': right_min,
                'track_width': self.track_width}
        return steer, v_target, info


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
        # Wakes the executor even when no data arrives, so Ctrl+C is always handled.
        self.create_timer(0.1, lambda: None)

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
        by_name = {f.name: f for f in fields(Params)}
        try:
            updates = {c.name: cast_param(by_name[c.name], c.value)
                       for c in changes if c.name in by_name}
            candidate = replace(self.controller.p, **updates)
            validate(candidate)
        except (TypeError, ValueError) as e:
            return SetParametersResult(successful=False, reason=str(e))
        self.controller.p = candidate
        return SetParametersResult(successful=True)

    def _on_odom(self, msg):
        self.speed = msg.twist.twist.linear.x  # body frame: forward speed

    def _on_scan(self, msg):
        if len(msg.ranges) < 3:  # malformed scan: keep the last command
            return
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
            for pub in (self.pub_throttle, self.pub_steering):
                pub.wait_for_all_acked(Duration(seconds=0.2))
        except Exception:  # context may already be shut down
            pass
        if self.log_file is not None:
            self.log_file.close()


def main(args=None):
    # Handle Ctrl+C (and SIGTERM, e.g. from launch or docker stop) ourselves, so
    # close() can still publish the stop command while the context is alive.
    signal.signal(signal.SIGTERM, signal.default_int_handler)
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
