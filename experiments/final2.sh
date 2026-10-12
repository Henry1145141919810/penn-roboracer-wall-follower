#!/bin/bash
# Final evaluation of the CMA-ES-tuned defaults: one-component-removed ablation
# (3 runs x 4 laps per variant, interleaved), then a 31-lap run of the real node.
source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash
cd /work
declare -a NAMES=(
  "W01_final"          "W02_no_traction"   "W03_no_corner_speed" "W04_no_preview"
  "W05_straight_clear" "W06_no_speed_fb"   "W07_no_side_guard"   "W08_fixed_lookahead"
  "W09_single_wall"    "W10_const_3ms")
declare -a ARGS=(
  ""                         "-p traction_slip=0"         "-p v_corner=0"  "-p preview_gain=0"
  "-p clearance_mode=straight" "-p speed_kp=0"            "-p use_side_guard=false"
  "-p lookahead_per_speed=0 -p lookahead=1.0"
  "-p mode=right -p wall_distance=0.7"   "-p speed_mode=const -p v_const=3.0")
for run in 0 1 2; do
  for i in "${!NAMES[@]}"; do
    python3 tools/experiment.py --laps 4 --timeout 120 --start-index $run \
      --label "${NAMES[$i]}" ${ARGS[$i]}
  done
done
# Reliability: the real node (ros2 run, default parameters) drives 31 laps in a row.
ros2 run autodrive_roboracer wall_follow_node > /work/runs/node_long.log 2>&1 &
sleep 3
python3 tools/experiment.py --external --label L_ros2_run_31laps --laps 31 --timeout 400
# Background jobs of a non-interactive shell ignore SIGINT; the node maps SIGTERM to a
# clean stop (zero throttle). (During the recorded run this step was done by hand.)
pkill -TERM -f 'lib/autodrive_roboracer/wall_follow_node'
wait
