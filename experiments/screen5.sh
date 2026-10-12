#!/bin/bash
# Screening round 5, on the code after the red-team fixes (scan-repeat handling etc.).
# Base N = v_max 7, lat_accel 6, brake_decel 4, speed_kp 0.2. 1 run x 3 laps each.
source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash
cd /work
E="python3 tools/experiment.py --laps 3 --timeout 120"
N="-p v_max=7 -p lat_accel=6 -p brake_decel=4 -p speed_kp=0.2"
$E --label N0_kd0.06          $N -p kd=0.06
$E --label N1_kd0             $N -p kd=0.0
$E --label N2_kd0.03          $N -p kd=0.03
$E --label N3_kd0.12          $N -p kd=0.12
$E --label N4_kd0.03_lat6.5   $N -p kd=0.03 -p lat_accel=6.5
$E --label N5_kd0.03_skp0.4   $N -p kd=0.03 -p speed_kp=0.4
$E --label N6_kd0.03_look0.1  $N -p kd=0.03 -p lookahead_per_speed=0.1
$E --label N7_kd0.03_look0.2  $N -p kd=0.03 -p lookahead_per_speed=0.2
