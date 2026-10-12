#!/bin/bash
# Evaluation of the hand-tuned configuration (recorded at commit 6bf3960, before traction
# control and CMA-ES tuning): baselines, config F and one-component-removed ablations.
# Z03-Z10 used the defaults of that time; with today's defaults they measure the tuned config.
# 3 runs x 4 laps per variant (lap 1 dropped -> 9 flying laps), repeats interleaved
# round-robin so slow drift in the simulator cannot favour one variant.
source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash
cd /work
TEXTBOOK="-p speed_mode=steps -p lookahead=1.0 -p lookahead_per_speed=0 -p use_side_guard=false \
-p wall_distance=0.7 -p kd=0.06 -p speed_kp=0 -p preview_gain=0 -p traction_slip=0 \n-p v_corner=0"
declare -a NAMES=(
  "Z01_textbook_left"   "Z02_textbook_right"  "Z03_final"
  "Z04_no_preview"      "Z05_straight_clear"  "Z06_no_speed_fb"
  "Z07_no_side_guard"   "Z08_fixed_lookahead" "Z09_single_wall"
  "Z10_const_3ms")
declare -a ARGS=(
  "-p mode=left $TEXTBOOK"  "-p mode=right $TEXTBOOK"  ""
  "-p preview_gain=0"       "-p clearance_mode=straight" "-p speed_kp=0"
  "-p use_side_guard=false" "-p lookahead_per_speed=0 -p lookahead=1.0"
  "-p mode=right -p wall_distance=0.7"
  "-p speed_mode=const -p v_const=3.0")
for run in 0 1 2; do
  for i in "${!NAMES[@]}"; do
    python3 tools/experiment.py --laps 4 --timeout 150 --start-index $run \
      --label "${NAMES[$i]}" ${ARGS[$i]}
  done
done
