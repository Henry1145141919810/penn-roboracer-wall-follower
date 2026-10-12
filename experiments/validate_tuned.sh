#!/bin/bash
# Re-measure the CMA-ES winners from scratch (guards against the "winner's curse" of
# picking the best of many noisy evaluations). 3 runs x 4 laps each, interleaved.
source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash
cd /work
declare -a NAMES=("V0_hand_tuned" "V1_cma_stage1_best" "V2_cma_stage2_best" "V3_cma_stage2_second")
declare -a ARGS=(
  ""
  "-p v_max=7.5744 -p lat_accel=7.0283 -p brake_decel=4.8591 -p v_corner=0.2464 -p stop_margin=0.5579 -p preview_gain=0.3343 -p kp=0.61 -p lookahead=0.324 -p traction_slip=0.1642 -p side_clearance=0.3383"
  "-p v_max=7.3417 -p lat_accel=7.7957 -p brake_decel=5.965 -p v_corner=0.6556 -p stop_margin=0.5169 -p preview_gain=0.2665 -p kp=0.4189 -p lookahead=0.2603 -p traction_slip=0.1812 -p side_clearance=0.3679"
  "-p v_max=7.5082 -p lat_accel=7.7603 -p brake_decel=5.4742 -p v_corner=0.3812 -p stop_margin=0.5828 -p preview_gain=0.3255 -p kp=0.5065 -p lookahead=0.24 -p traction_slip=0.1284 -p side_clearance=0.3547")
for run in 0 1 2; do
  for i in "${!NAMES[@]}"; do
    python3 tools/experiment.py --laps 4 --timeout 120 --start-index $run --label "${NAMES[$i]}" ${ARGS[$i]}
  done
done
