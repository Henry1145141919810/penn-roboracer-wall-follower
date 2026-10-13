#!/bin/bash
# Screening round 8: pure-pursuit steering geometry on the wall-centring error (steer_law=
# pursuit_wall) and the bounded inside offset, vs the free-sector pursuit and the PD default.
# 1 run x 3 laps each; speed parameters = tuned defaults.
source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash
cd /work
E="python3 tools/experiment.py --laps 3 --timeout 90"
W="-p steer_law=pursuit_wall -p preview_gain=0"
$E --label Z8_pd_default
$E --label Z8_pw_g0.9            $W -p steer_gain=0.9
$E --label Z8_pw_g0.9_in0.3      $W -p steer_gain=0.9 -p inside_offset=0.3
$E --label Z8_pw_g0.9_prev0.27   -p steer_law=pursuit_wall -p steer_gain=0.9
$E --label Z8_pw_g1.2_in0.3      $W -p steer_gain=1.2 -p inside_offset=0.3
$E --label Z8_pw_g0.7_in0.3      $W -p steer_gain=0.7 -p inside_offset=0.3
$E --label Z8_pw_g0.9_in0.3_lmin0.6 $W -p steer_gain=0.9 -p inside_offset=0.3 -p lookahead_min=0.6
$E --label Z8_pw_g0.9_in0.3_lps0.25 $W -p steer_gain=0.9 -p inside_offset=0.3 -p lookahead_per_speed=0.25
$E --label Z8_pursuit_L1.2_default -p steer_mode=pursuit -p preview_gain=0 -p kp=0.15   # lookahead left at the 1.2 m default
