#!/bin/bash
# Screening round 3: one-factor changes around the best setting so far (A1), 1 run x 3 laps.
source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash
cd /work
E="python3 tools/experiment.py --laps 3 --timeout 120"
A1="-p v_max=7 -p lat_accel=6 -p brake_decel=4"
$E --label R0_A1_repeat        $A1
$E --label R1_A1_speedkp0.2    $A1 -p speed_kp=0.2
$E --label R2_A1_look0.25      $A1 -p lookahead_per_speed=0.25
$E --label R3_A1_kd0.15        $A1 -p kd=0.15
$E --label R4_A1_kp1.2         $A1 -p kp=1.2
$E --label R5_A1_kp0.7         $A1 -p kp=0.7
$E --label R6_A1_lat5          $A1 -p lat_accel=5
$E --label R7_A1_vmax8         $A1 -p v_max=8
