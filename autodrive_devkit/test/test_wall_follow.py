"""Unit tests for the wall follower controller on synthetic LiDAR scans.

Run inside the container:  python3 -m pytest autodrive_devkit/test/test_wall_follow.py
"""

from dataclasses import fields
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from autodrive_roboracer.wall_follow_node import (  # noqa: E402
    MAX_STEER_RAD, Params, WallFollowController, arc_clearance, beam_index, cast_param,
    clean_ranges, deepest_direction, forward_clearance, validate, wall_estimate)

ANGLE_MIN = -2.35619
ANGLE_INC = 0.004363323
N_BEAMS = 1080
RANGE_MAX = 10.0
ANGLES = ANGLE_MIN + ANGLE_INC * np.arange(N_BEAMS)


def corridor_scan(left, right, yaw=0.0, front=None):
    """Scan inside a straight corridor.

    Walls at y = +left and y = -right (corridor frame), optional wall across at
    x = front. ``yaw`` is the car's heading relative to the corridor (+ = left).
    """
    phi = ANGLES + yaw
    s, c = np.sin(phi), np.cos(phi)
    r = np.full(N_BEAMS, np.inf)
    with np.errstate(divide='ignore'):
        r = np.where(s > 1e-9, np.minimum(r, left / s), r)
        r = np.where(s < -1e-9, np.minimum(r, right / -s), r)
        if front is not None:
            r = np.where(c > 1e-9, np.minimum(r, front / c), r)
    return np.minimum(r, RANGE_MAX)


def run(controller, scan, steps=5, speed=None):
    out = None
    for i in range(steps):
        out = controller.step(scan, ANGLE_MIN, ANGLE_INC, RANGE_MAX, stamp=0.025 * i,
                              speed=speed)
    return out


def test_beam_index_matches_scan_layout():
    assert beam_index(0.0, ANGLE_MIN, ANGLE_INC, N_BEAMS) == 540
    assert beam_index(math.pi / 2, ANGLE_MIN, ANGLE_INC, N_BEAMS) == 900
    assert beam_index(-math.pi / 2, ANGLE_MIN, ANGLE_INC, N_BEAMS) == 180
    assert beam_index(10.0, ANGLE_MIN, ANGLE_INC, N_BEAMS) == N_BEAMS - 1


def test_clean_ranges_replaces_invalid_beams():
    r = clean_ranges([1.0, float('nan'), float('inf'), -1.0, 0.0, 50.0], RANGE_MAX)
    assert list(r) == [1.0, RANGE_MAX, RANGE_MAX, RANGE_MAX, RANGE_MAX, RANGE_MAX]


def test_wall_estimate_recovers_distance_and_angle():
    scan = corridor_scan(0.8, 0.8, yaw=0.1)
    d, alpha = wall_estimate(scan, ANGLE_MIN, ANGLE_INC, +1, 50.0, RANGE_MAX)
    assert d == pytest.approx(0.8, abs=0.01)
    assert alpha == pytest.approx(-0.1, abs=0.01)  # car turned towards the left wall
    d, alpha = wall_estimate(scan, ANGLE_MIN, ANGLE_INC, -1, 50.0, RANGE_MAX)
    assert d == pytest.approx(0.8, abs=0.01)
    assert alpha == pytest.approx(0.1, abs=0.01)  # and away from the right wall


def test_wall_estimate_none_without_wall():
    scan = corridor_scan(50.0, 0.8)
    assert wall_estimate(scan, ANGLE_MIN, ANGLE_INC, +1, 50.0, RANGE_MAX) is None


def test_centered_and_aligned_goes_straight():
    throttle, steer, _ = run(WallFollowController(), corridor_scan(0.8, 0.8))
    assert abs(steer) < 0.02
    assert throttle > 0.0


@pytest.mark.parametrize('left,right,sign', [(0.4, 1.2, -1), (1.2, 0.4, +1)])
def test_offset_steers_back_to_center(left, right, sign):
    _, steer, _ = run(WallFollowController(), corridor_scan(left, right))
    assert sign * steer > 0.1


@pytest.mark.parametrize('yaw,sign', [(0.25, -1), (-0.25, +1)])
def test_heading_error_is_corrected(yaw, sign):
    _, steer, _ = run(WallFollowController(), corridor_scan(0.8, 0.8, yaw=yaw))
    assert sign * steer > 0.05


def test_single_wall_modes_hold_wall_distance():
    p = Params(mode='right', wall_distance=0.6, use_side_guard=False)
    _, steer, _ = run(WallFollowController(p), corridor_scan(1.0, 0.6))
    assert abs(steer) < 0.02
    _, steer, _ = run(WallFollowController(p), corridor_scan(1.2, 0.4))
    assert steer > 0.1  # too close to the right wall -> steer left
    p = Params(mode='left', wall_distance=0.6, use_side_guard=False)
    _, steer, _ = run(WallFollowController(p), corridor_scan(0.4, 1.2))
    assert steer < -0.1  # too close to the left wall -> steer right


def test_missing_wall_falls_back_to_remembered_width():
    c = WallFollowController(Params(preview_gain=0.0))  # test the fallback alone
    run(c, corridor_scan(0.8, 0.8), steps=20)  # learn the track width (1.6 m)
    _, steer, _ = run(c, corridor_scan(50.0, 0.8), steps=3)  # left wall gone
    assert abs(steer) < 0.05
    _, steer, _ = run(c, corridor_scan(50.0, 0.5), steps=3)
    assert steer > 0.1


def test_side_guard_pushes_away_from_close_wall():
    p_on = Params(mode='left', wall_distance=0.25)
    p_off = Params(mode='left', wall_distance=0.25, use_side_guard=False)
    scan = corridor_scan(0.2, 1.4)
    _, steer_on, _ = run(WallFollowController(p_on), scan)
    _, steer_off, _ = run(WallFollowController(p_off), scan)
    assert steer_on < steer_off


def test_forward_clearance_uses_car_corridor():
    scan = corridor_scan(0.8, 0.8, front=3.0)
    assert forward_clearance(scan, ANGLES, 0.25) == pytest.approx(3.0, abs=0.02)
    assert forward_clearance(corridor_scan(0.8, 0.8), ANGLES, 0.25) == pytest.approx(
        RANGE_MAX, abs=0.1)


def circle_scan(radius_in, radius_out, turn):
    """Scan from the middle of a circular corridor (ring), car tangent to it.

    ``turn`` = +1 for a left-hand bend, -1 for a right-hand bend.
    """
    rc = (radius_in + radius_out) / 2.0  # car sits on the middle circle
    cy = turn * rc  # centre of the ring in the car frame
    dy = np.sin(ANGLES)
    r = np.full(N_BEAMS, RANGE_MAX)
    for rad in (radius_in, radius_out):
        # |t*d - (0, cy)| = rad  ->  t^2 - 2 t dy cy + cy^2 - rad^2 = 0
        b = -2 * dy * cy
        c = cy ** 2 - rad ** 2
        disc = b ** 2 - 4 * c
        with np.errstate(invalid='ignore'):
            for t in ((-b - np.sqrt(disc)) / 2, (-b + np.sqrt(disc)) / 2):
                ok = (disc >= 0) & (t > 1e-6)
                r = np.where(ok & (t < r), t, r)
    return r


@pytest.mark.parametrize('turn', [+1, -1])
def test_arc_clearance_sees_through_a_bend(turn):
    scan = circle_scan(1.0, 2.0, turn)
    straight = forward_clearance(scan, ANGLES, 0.25)
    on_arc = arc_clearance(scan, ANGLES, 0.25, turn / 1.5)  # following the bend
    wrong_way = arc_clearance(scan, ANGLES, 0.25, -turn / 1.5)
    assert straight < 1.5  # the outer wall is close straight ahead
    assert on_arc == pytest.approx(1.5 * math.pi / 2, abs=0.05)  # whole checked arc free
    assert wrong_way < straight


def test_arc_clearance_matches_straight_for_zero_curvature():
    scan = corridor_scan(0.8, 0.8, front=3.0)
    assert arc_clearance(scan, ANGLES, 0.25, 0.0) == forward_clearance(scan, ANGLES, 0.25)


def test_adaptive_speed_brakes_for_wall_ahead():
    open_road, _, info_open = run(WallFollowController(), corridor_scan(0.8, 0.8))
    near_wall, _, info_near = run(WallFollowController(), corridor_scan(0.8, 0.8, front=1.5))
    assert info_open['v_target'] == pytest.approx(Params().v_max)
    assert near_wall < open_road
    assert open_road == pytest.approx(Params().v_max / Params().v_full_throttle)


def test_steps_mode_is_textbook_schedule():
    p = Params(speed_mode='steps')
    _, _, info = run(WallFollowController(p), corridor_scan(0.8, 0.8))
    assert info['v_target'] == 1.5
    _, _, info = run(WallFollowController(p), corridor_scan(0.3, 1.3))
    assert info['v_target'] == 0.5  # big correction -> slowest step


def test_const_mode_and_speed_feedback():
    p = Params(speed_mode='const', v_const=2.0, speed_kp=0.1, traction_slip=0.0)
    throttle, _, _ = run(WallFollowController(p), corridor_scan(0.8, 0.8), speed=1.0)
    assert throttle == pytest.approx(2.0 / 25.0 + 0.1 * (2.0 - 1.0))


def test_outputs_bounded_and_finite_on_garbage():
    rng = np.random.default_rng(0)
    c = WallFollowController()
    for i in range(300):
        scan = rng.uniform(0.0, 12.0, N_BEAMS)
        scan[rng.integers(0, N_BEAMS, 50)] = np.nan
        scan[rng.integers(0, N_BEAMS, 50)] = np.inf
        throttle, steer, _ = c.step(scan, ANGLE_MIN, ANGLE_INC, RANGE_MAX, 0.01 * i)
        assert math.isfinite(throttle) and math.isfinite(steer)
        assert -1.0 <= throttle <= 1.0 and -1.0 <= steer <= 1.0


def test_steering_command_scale():
    # A huge error saturates at exactly full lock (command 1.0 = MAX_STEER_RAD).
    p = Params(kp=100.0)
    _, steer, _ = run(WallFollowController(p), corridor_scan(1.5, 0.3))
    assert steer == pytest.approx(1.0)
    assert MAX_STEER_RAD == pytest.approx(0.52, abs=0.01)


def test_cast_param_accepts_ros_types():
    by_name = {f.name: f for f in fields(Params)}
    assert cast_param(by_name['v_max'], 6) == 6.0
    assert isinstance(cast_param(by_name['v_max'], 6), float)
    assert cast_param(by_name['use_side_guard'], 'false') is False
    assert cast_param(by_name['mode'], 'left') == 'left'


def test_republished_scan_keeps_steering_and_refreshes_throttle():
    p = Params(speed_kp=0.2, traction_slip=0.0)
    c = WallFollowController(p)
    scan = corridor_scan(0.5, 1.1)
    thr1, steer1, _ = c.step(scan, ANGLE_MIN, ANGLE_INC, RANGE_MAX, 0.000, speed=2.0)
    state = (c.prev_error, c.d_filtered, c.prev_time)
    thr2, steer2, _ = c.step(scan.copy(), ANGLE_MIN, ANGLE_INC, RANGE_MAX, 0.006, speed=3.0)
    assert steer2 == steer1  # same scan -> same steering, no filter update
    assert (c.prev_error, c.d_filtered, c.prev_time) == state
    assert thr2 == pytest.approx(thr1 - 0.2 * 1.0)  # throttle uses the new speed


def test_derivative_reacts_to_error_rate():
    # Car drifting towards the left wall: D adds steering to the right.
    def drift(kd):
        c = WallFollowController(Params(kd=kd, use_side_guard=False))
        out = None
        for i, left in enumerate(np.linspace(0.9, 0.7, 6)):
            out = c.step(corridor_scan(left, 1.6 - left), ANGLE_MIN, ANGLE_INC, RANGE_MAX,
                         0.025 * i)
        return out[1]
    assert drift(0.1) < drift(0.0) < 0.0


def test_long_gap_restarts_derivative():
    c = WallFollowController(Params(kd=0.1))
    c.step(corridor_scan(0.8, 0.8), ANGLE_MIN, ANGLE_INC, RANGE_MAX, 0.0)
    c.step(corridor_scan(0.4, 1.2), ANGLE_MIN, ANGLE_INC, RANGE_MAX, 5.0)  # 5 s later
    assert c.d_filtered == 0.0


def test_error_is_clamped():
    c = WallFollowController(Params(max_error=0.5))
    _, _, info = run(c, corridor_scan(3.0, 0.3))
    assert info['error'] == pytest.approx(0.5)


def test_right_side_guard_pushes_left():
    p_on = Params(mode='right', wall_distance=0.25)
    p_off = Params(mode='right', wall_distance=0.25, use_side_guard=False)
    scan = corridor_scan(1.4, 0.2)
    _, steer_on, _ = run(WallFollowController(p_on), scan)
    _, steer_off, _ = run(WallFollowController(p_off), scan)
    assert steer_on > steer_off


@pytest.mark.parametrize('bad', [{'mode': 'middle'}, {'speed_mode': 'fast'},
                                 {'step_speeds': '2'}, {'v_full_throttle': 0.0},
                                 {'traction_slip': 0.1, 'traction_offset': 0.0}])
def test_validate_rejects_bad_params(bad):
    with pytest.raises(ValueError):
        validate(Params(**bad))


@pytest.mark.parametrize('turn', [+1, -1])
def test_deepest_direction_points_into_the_bend(turn):
    scan = circle_scan(1.0, 2.0, turn)
    assert turn * deepest_direction(scan, ANGLES, 60.0) > 0.1
    assert abs(deepest_direction(corridor_scan(0.8, 0.8), ANGLES, 60.0)) < 0.02


@pytest.mark.parametrize('turn', [+1, -1])
def test_preview_steers_earlier_into_the_bend(turn):
    scan = circle_scan(1.0, 2.0, turn)
    _, steer_off, _ = run(WallFollowController(Params(preview_gain=0.0)), scan)
    _, steer_on, _ = run(WallFollowController(Params(preview_gain=0.4)), scan)
    assert turn * (steer_on - steer_off) > 0.05


def test_traction_control_caps_wheel_speed_at_low_speed():
    p = Params(traction_slip=0.1, traction_offset=0.5)
    throttle, _, _ = run(WallFollowController(p), corridor_scan(0.8, 0.8), speed=2.0)
    assert throttle == pytest.approx((2.0 * 1.1 + 0.5) / 25.0)
    throttle_off, _, _ = run(WallFollowController(Params(traction_slip=0.0)),
                             corridor_scan(0.8, 0.8), speed=2.0)
    assert throttle_off > throttle


def test_corner_speed_raises_braking_limit():
    scan = corridor_scan(0.8, 0.8, front=1.5)
    _, _, slow = run(WallFollowController(Params(v_corner=0.0)), scan)
    _, _, fast = run(WallFollowController(Params(v_corner=2.0)), scan)
    assert fast['v_target'] > slow['v_target']
