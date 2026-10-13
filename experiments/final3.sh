#!/bin/bash
# Final evaluation of the pursuit-mode defaults (after CMA-ES stage 4): one-component-removed
# ablation plus the two-wall alternative, 3 runs x 4 laps per variant, interleaved; then a
# 31-lap run of the real node with default parameters.
source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash
cd /work
TWO_WALL="-p steer_mode=pd -p kp=0.42 -p kd=0.03 -p lookahead=0.26 -p lookahead_per_speed=0.15 \
-p preview_gain=0.27 -p v_max=7.34 -p lat_accel=7.8 -p brake_decel=6.0 -p v_corner=0.66 \
-p stop_margin=0.52 -p traction_slip=0.18 -p side_clearance=0.37"
declare -a NAMES=(
  "F01_final"           "F02_two_wall_pd"      "F03_no_apex_shift"   "F04_no_disparity"
  "F05_no_wall_centring" "F06_no_traction"     "F07_no_corner_speed" "F08_no_side_guard"
  "F09_straight_clear"  "F10_no_speed_fb"      "F11_fixed_lookahead")
declare -a ARGS=(
  ""  "$TWO_WALL"  "-p apex_shift=0"  "-p disparity_threshold=99"
  "-p kp=0"  "-p traction_slip=0"  "-p v_corner=0"  "-p use_side_guard=false"
  "-p clearance_mode=straight"  "-p speed_kp=0"  "-p pursuit_lookahead_per_speed=0")
for run in 0 1 2; do
  for i in "${!NAMES[@]}"; do
    python3 tools/experiment.py --laps 4 --timeout 120 --start-index $run \
      --label "${NAMES[$i]}" ${ARGS[$i]}
  done
done
ros2 run autodrive_roboracer wall_follow_node > /work/runs/node_long2.log 2>&1 &
sleep 3
python3 tools/experiment.py --external --label L2_ros2_run_31laps --laps 31 --timeout 400
pkill -TERM -f 'lib/autodrive_roboracer/wall_follow_node'
wait
# Added after review: separate the steering change from the speed-envelope re-tune.
# F12 = two-wall PD steering with the pursuit speed envelope; F13 = pursuit steering with the
# PD-era speed envelope. 3 runs x 4 laps, interleaved with a repeat of the final defaults.
