"""Closed-loop regression tests in the offline simulator (tools/sim2d.py).

These do not prove lap times (the real simulator does that); they catch
regressions: the default controller must finish laps without touching a wall,
in both directions and under a slower scan rate, LiDAR noise and latency.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'autodrive_devkit'))
from sim2d import simulate  # noqa: E402
from autodrive_roboracer.wall_follow_node import Params  # noqa: E402


@pytest.mark.parametrize('mirror', [False, True], ids=['clockwise', 'counter_clockwise'])
def test_default_controller_completes_three_laps(mirror):
    result = simulate(Params(), laps=3, mirror=mirror)
    assert result['status'] == 'ok', result
    assert len(result['laps']) == 3


@pytest.mark.parametrize('control_rate,noise,latency',
                         [(40.0, 0.0, 0), (160.0, 0.02, 0), (160.0, 0.0, 12)],
                         ids=['control_40Hz', 'noise_2cm', 'latency_75ms'])
def test_default_controller_is_robust(control_rate, noise, latency):
    result = simulate(Params(), laps=1, control_rate=control_rate, noise=noise,
                      latency=latency)
    assert result['status'] == 'ok', result


def test_simulator_can_fail():
    # Sanity check of the test bed: constant 4 m/s crashes in the real simulator
    # (bottom hairpin), so it must crash here too, or these tests prove nothing.
    p = Params(speed_mode='const', v_const=4.0, lookahead=1.0, lookahead_per_speed=0.0)
    assert simulate(p, laps=1)['status'] == 'collision'


def test_textbook_baseline_completes_a_lap():
    p = Params(mode='left', speed_mode='steps', lookahead=1.0, lookahead_per_speed=0.0,
               use_side_guard=False, wall_distance=0.7)
    result = simulate(p, laps=1, max_time=60.0)
    assert result['status'] == 'ok', result
