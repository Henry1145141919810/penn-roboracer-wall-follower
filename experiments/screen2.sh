#!/bin/bash
# Screening round 2: arc clearance + cornering/braking limits (1 run x 3 laps each).
source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash
cd /work
E="python3 tools/experiment.py --laps 3 --timeout 120"
$E --label A1_arc_v7_lat6_br4   -p v_max=7 -p lat_accel=6  -p brake_decel=4
$E --label A2_arc_v7_lat8_br4   -p v_max=7 -p lat_accel=8  -p brake_decel=4
$E --label A3_arc_v7_lat10_br4  -p v_max=7 -p lat_accel=10 -p brake_decel=4
$E --label A4_arc_v7_lat8_br6   -p v_max=7 -p lat_accel=8  -p brake_decel=6
$E --label A5_arc_v7_lat10_br6  -p v_max=7 -p lat_accel=10 -p brake_decel=6
$E --label A6_arc_v8_lat10_br8  -p v_max=8 -p lat_accel=10 -p brake_decel=8
$E --label S1_str_v7_lat10_br6  -p v_max=7 -p lat_accel=10 -p brake_decel=6 -p clearance_mode=straight
