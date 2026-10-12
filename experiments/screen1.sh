#!/bin/bash
# Screening round 1: textbook baselines vs centring vs adaptive speed (1 run x 3 laps each).
source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash
cd /work
E="python3 tools/experiment.py --laps 3 --timeout 150"
TEXTBOOK="-p speed_mode=steps -p lookahead=1.0 -p lookahead_per_speed=0 -p use_side_guard=false -p wall_distance=0.7"
$E --label B1_left_steps  -p mode=left  $TEXTBOOK
$E --label B2_right_steps -p mode=right $TEXTBOOK
$E --label C1_center_steps -p mode=center $TEXTBOOK
$E --label C2_center_adapt_v5 -p v_max=5
$E --label C2_center_adapt_v6 -p v_max=6
$E --label C2_center_adapt_v7 -p v_max=7
