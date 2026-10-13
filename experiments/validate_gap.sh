#!/bin/bash
# Re-measure the CMA-ES stage-3 winner (free-sector pursuit, gap mode) from scratch,
# interleaved with the PD defaults: 3 runs x 4 laps each.
source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash
cd /work
G="-p steer_mode=pursuit -p preview_gain=0 -p pursuit_lookahead=1.7238 -p pursuit_lookahead_per_speed=0.2619 \
-p pursuit_min_lookahead=1.0773 -p kp=0.1384 -p apex_shift=0.2022 -p disparity_margin=0.0719 -p v_max=6.9382 \
-p lat_accel=8.4333 -p brake_decel=5.6839 -p v_corner=0.8342 -p stop_margin=0.3255 -p traction_slip=0.1844"
for run in 0 1 2; do
  python3 tools/experiment.py --laps 4 --timeout 120 --start-index $run --label VG_pd_default
  python3 tools/experiment.py --laps 4 --timeout 120 --start-index $run --label VG_gap_stage3_best $G
done
