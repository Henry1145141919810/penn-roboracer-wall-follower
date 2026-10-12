# Dev helper for Windows PowerShell. Run from anywhere:
#   .\scripts\dev.ps1 up       # start container, colcon build, start bridge in background
#   .\scripts\dev.ps1 run      # start wall_follow_node (Ctrl+C stops it and zeroes the car)
#   .\scripts\dev.ps1 shell    # open a bash shell in the container
#   .\scripts\dev.ps1 test     # run unit tests and offline closed-loop tests
#   .\scripts\dev.ps1 down     # remove the container
# Extra args after 'run' are passed to the node, e.g.
#   .\scripts\dev.ps1 run -p v_max:=4.0 -p kp:=1.0
param([string]$cmd = 'up', [Parameter(ValueFromRemainingArguments = $true)]$rest)

$ErrorActionPreference = 'Stop'
$env:Path = "C:\Program Files\Docker\Docker\resources\bin;" + $env:Path
$repo = (Resolve-Path "$PSScriptRoot\..").Path
$name = 'penn-challenge'
$ros = 'source /opt/ros/humble/setup.bash && source /home/autodrive_devkit/install/setup.bash'

switch ($cmd) {
  'up' {
    docker rm -f $name 2>$null | Out-Null
    New-Item -ItemType Directory -Force "$repo\runs" | Out-Null
    docker run -d --platform linux/amd64 --name $name -p 4567:4567 `
      -v "${repo}/autodrive_devkit:/home/autodrive_devkit/src/autodrive_devkit" `
      -v "${repo}:/work" `
      --entrypoint /bin/bash penn-challenge -c 'sleep infinity' | Out-Null
    docker exec $name bash -c 'source /opt/ros/humble/setup.bash && cd /home/autodrive_devkit && colcon build 2>&1 | tail -3'
    docker exec -d $name bash -c "$ros && ros2 launch autodrive_roboracer bringup_headless.launch.py > /work/runs/bridge.log 2>&1"
    Write-Host 'Bridge started (log: runs/bridge.log). Open the simulator, connect to 127.0.0.1:4567.'
  }
  'run' {
    docker exec $name bash -c 'source /opt/ros/humble/setup.bash && cd /home/autodrive_devkit && colcon build 2>&1 | tail -1'
    $extra = if ($rest) { '--ros-args ' + ($rest -join ' ') } else { '' }
    docker exec -it $name bash -c "$ros && ros2 run autodrive_roboracer wall_follow_node $extra"
  }
  'shell' { docker exec -it $name bash -c "$ros; exec bash" }
  'test'  { docker exec $name bash -c "$ros && cd /work && python3 -m pytest -q tests autodrive_devkit/test/test_wall_follow.py" }
  'down'  { docker rm -f $name }
  default { Write-Host "unknown command: $cmd" }
}
