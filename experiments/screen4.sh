#!/bin/bash
# Screening round 4: one-factor changes around R1 (A1 + speed_kp=0.2), 1 run x 3 laps.
source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash
cd /work
E="python3 tools/experiment.py --laps 3 --timeout 120"
R1="-p v_max=7 -p lat_accel=6 -p brake_decel=4 -p speed_kp=0.2"
$E --label Q1_R1_speedkp0.4   $R1 -p speed_kp=0.4
$E --label Q2_R1_kd0.03       $R1 -p kd=0.03
$E --label Q3_R1_kd0          $R1 -p kd=0.0
$E --label Q4_R1_lat7         $R1 -p lat_accel=7
$E --label Q5_R1_brake5       $R1 -p brake_decel=5
$E --label Q6_R1_kp0.75       $R1 -p kp=0.75
$E --label Q7_R1_halfw0.2     $R1 -p car_half_width=0.2
$E --label Q8_R1_margin0.3    $R1 -p stop_margin=0.3
