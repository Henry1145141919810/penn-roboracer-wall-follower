#!/bin/bash
# Screening round 6: corner preview gain on base P (v_max 7, lat 6.5, brake 4, speed_kp 0.2,
# kd 0.03). 1 run x 3 laps each.
source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash
cd /work
E="python3 tools/experiment.py --laps 3 --timeout 120"
P="-p v_max=7 -p lat_accel=6.5 -p brake_decel=4 -p speed_kp=0.2 -p kd=0.03"
for g in 0 0.2 0.3 0.4 0.5; do
  $E --label P_preview$g $P -p preview_gain=$g
done
