# Start headless simulator(s), each with its own container + bridge, for tools/tune.py.
# Use n = 1: several simulators on one PC change the simulator's behaviour (LiDAR at
# ~190 new scans/s instead of 40, and configs that are clean alone crash), see tools/tune.py.
# Worker k uses container penn-tune-k and host port 4566+k. The simulator must be started
# BEFORE its bridge: if the bridge is up first, the simulator's first message has no LiDAR
# data, the (unmodified) bridge raises KeyError and the connection stalls.
#   .\scripts\tune_up.ps1 1        # start one worker
#   .\scripts\tune_up.ps1 0        # stop all tuning containers and simulators
param([int]$n = 1)

$ErrorActionPreference = 'Continue'
$env:Path = "C:\Program Files\Docker\Docker\resources\bin;" + $env:Path
$repo = (Resolve-Path "$PSScriptRoot\..").Path
$sim = (Resolve-Path "$repo\..\simulator\autodrive_simulator\AutoDRIVE Simulator.exe").Path
$ros = 'source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash'

docker ps -a --format '{{.Names}}' | Where-Object { $_ -like 'penn-tune-*' } | ForEach-Object { docker rm -f $_ | Out-Null }
if ($n -eq 0) { Get-Process 'AutoDRIVE Simulator' -ErrorAction SilentlyContinue | Stop-Process -Force; exit }

for ($k = 1; $k -le $n; $k++) {
  $port = 4566 + $k
  docker run -d --platform linux/amd64 --name "penn-tune-$k" -p "${port}:4567" `
    -v "${repo}/autodrive_devkit:/home/autodrive_devkit/src/autodrive_devkit" -v "${repo}:/work" `
    --entrypoint /bin/bash penn-challenge -c 'sleep infinity' | Out-Null
  docker exec "penn-tune-$k" bash -c 'source /opt/ros/humble/setup.bash && cd /home/autodrive_devkit && colcon build > /dev/null 2>&1 && pip3 install -q cma==3.3.0 > /dev/null 2>&1'
  Start-Process -FilePath $sim -ArgumentList '-batchmode', '-nographics', '-ip', '127.0.0.1', '-port', "$port" | Out-Null
}
Start-Sleep -Seconds 25  # let the simulators finish loading before the bridges come up
for ($k = 1; $k -le $n; $k++) {
  docker exec -d "penn-tune-$k" bash -c "$ros && ros2 launch autodrive_roboracer bringup_headless.launch.py > /work/runs/tune_bridge_$k.log 2>&1"
}
Start-Sleep -Seconds 15
for ($k = 1; $k -le $n; $k++) {
  $hz = docker exec "penn-tune-$k" bash -c "$ros && timeout 4 ros2 topic hz /autodrive/roboracer_1/odom 2>&1 | grep average | tail -1"
  Write-Host "worker $k (port $(4566 + $k)): $hz"
}
