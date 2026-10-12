#!/bin/bash
# Screening round 7: traction control and corner speed on the final config (1 run x 3 laps).
source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash
cd /work
E="python3 tools/experiment.py --laps 3 --timeout 120"
$E --label T0_final
$E --label T1_tc0.1         -p traction_slip=0.1
$E --label T2_tc0.05        -p traction_slip=0.05
$E --label T3_tc0.1_vmax8   -p traction_slip=0.1 -p v_max=8
$E --label T4_tc0.1_vc1.5   -p traction_slip=0.1 -p v_corner=1.5
