#!/bin/bash
# Steering-vs-speed-envelope separation (see the note at the end of final3.sh), plus a repeat
# of the final defaults, 3 runs x 4 laps interleaved.
source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash
cd /work
PD_STEER="-p steer_mode=pd -p kp=0.42 -p kd=0.03 -p lookahead=0.26 -p lookahead_per_speed=0.15 -p preview_gain=0.27 -p side_clearance=0.37"
PD_SPEED="-p v_max=7.34 -p lat_accel=7.8 -p brake_decel=6.0 -p v_corner=0.66 -p stop_margin=0.52 -p traction_slip=0.18"
for run in 0 1 2; do
  python3 tools/experiment.py --laps 4 --timeout 120 --start-index $run --label F14_final_repeat
  python3 tools/experiment.py --laps 4 --timeout 120 --start-index $run --label F12_pd_steer_pursuit_speed $PD_STEER
  python3 tools/experiment.py --laps 4 --timeout 120 --start-index $run --label F13_pursuit_steer_pd_speed $PD_SPEED
done
