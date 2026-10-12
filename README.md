# Penn RoboRacer Technical Challenge: LiDAR wall follower

A ROS 2 (Humble) node that drives the AutoDRIVE RoboRacer practice track from LiDAR alone.

| | |
|---|---|
| **Fastest clean lap** | **RESULT_PENDING** (practice track, IROS 2026 practice simulator, no wall contact) |
| **Video** | VIDEO_PENDING (unlisted YouTube, Driver's Eye camera) |
| **Node** | [`autodrive_devkit/autodrive_roboracer/wall_follow_node.py`](autodrive_devkit/autodrive_roboracer/wall_follow_node.py) |

## How it works

Every LiDAR scan (1080 beams, 270°) produces one throttle and one steering command.

1. **Clean the scan.** NaN, inf and out-of-range beams are set to the 10 m maximum.
2. **Fit both side walls.** For each side, beam *b* points 90° to the side and beam *a* points
   50° further forward. For a straight wall, `tan(α) = (a·cos θ − b) / (a·sin θ)` gives the car's
   angle to the wall, and `D = b·cos α` gives its distance. The distance is projected ahead,
   `D + L·sin α`, with a look-ahead `L` that grows with speed. This lets the car react to where it
   will be, not only where it is.
3. **Steer to the centre line.** The error is `(D_left − D_right) / 2`, and a PD controller turns
   it into a steering angle. If one wall disappears (an opening), the node follows the other wall
   at half the remembered track width. A side guard adds extra steering away from any wall closer
   than 0.35 m. Single-wall following (`mode:=left|right`) is kept as the textbook baseline.
4. **Choose a speed.** The target speed is the lowest of three limits:
   - **top speed** `v_max`;
   - **braking:** `√(2·a_brake·d)`, where `d` is the free distance along the arc the car is
     currently steering on. Using the arc matters: mid-corner, the straight-ahead distance is
     short even though the path is clear;
   - **cornering:** `√(a_lat / κ)`, where `κ = tan(δ) / wheelbase` is the curvature of the
     current steering angle.
5. **Throttle.** In this simulator throttle behaves like a speed command (measured: steady
   speed ≈ 25 m/s × throttle, reached in under 0.5 s), so `throttle = v_target / 25`.

When the node stops (Ctrl+C), it publishes zero throttle and steering before shutting down. The
bridge keeps replaying the last command it received, so without this the car would keep driving.

Every tunable is a ROS parameter, for example:

```bash
ros2 run autodrive_roboracer wall_follow_node --ros-args -p v_max:=7.0 -p lat_accel:=10.0
ros2 param set /wall_follow_node kp 1.1     # live tuning while driving
```

## Results

RESULTS_PENDING

## Run it

Prerequisites: Docker Desktop, Git, the IROS 2026 **practice** simulator.

```powershell
docker build --platform linux/amd64 -t penn-challenge -f autodrive_devkit.Dockerfile .
.\scripts\dev.ps1 up     # container + colcon build + bridge in the background (Windows)
.\scripts\dev.ps1 run    # start wall_follow_node
```

Or use the commands from the challenge sheet: start the bridge with
`ros2 launch autodrive_roboracer bringup_headless.launch.py`, then in a second shell run
`ros2 run autodrive_roboracer wall_follow_node`. In the simulator, connect to `127.0.0.1:4567`
and switch Driving Mode to **Autonomous**.

## Testing

| What | Where | Runs in |
|---|---|---|
| Unit tests on synthetic scans: geometry, signs, invalid beams, missing walls, speed modes | `autodrive_devkit/test/test_wall_follow.py` | CI + container |
| Closed-loop tests in a small offline simulator on a replica of the practice track (also mirrored), with a 40 Hz scan rate, 2 cm LiDAR noise and 40 ms latency | `tests/test_closed_loop.py`, `tools/sim2d.py` | CI + container |
| Timed laps in the real simulator: resets the car, counts laps and wall contacts, logs a per-scan trace | `tools/experiment.py`, `tools/summarize.py` | container + simulator |

```powershell
.\scripts\dev.ps1 test
```

GitHub Actions runs flake8, `colcon build` and both test suites in a `ros:humble` container on
every push.

The offline simulator is a kinematic model with no tyre slip. It catches regressions but does not
predict lap times. All lap times in this README come from the AutoDRIVE simulator.

## Repository layout

```
autodrive_devkit/            ROS 2 package from AutoDRIVE + wall_follow_node.py (bridge unchanged)
autodrive_devkit.Dockerfile  unchanged from AutoDRIVE, so the image builds from this repo
tools/experiment.py          automated timed laps in the simulator
tools/summarize.py           results table (flying laps only)
tools/sim2d.py               offline simulator used by the closed-loop tests
tools/track_map.py           draws the track from logged scans
tests/                       closed-loop tests + track replica
experiments/                 the exact experiment scripts behind the results tables
scripts/dev.ps1              Windows helper (up / run / test / shell / down)
```

AutoDRIVE files are BSD-licensed, Copyright (c) AutoDRIVE / Tinker Twins (see `LICENSE.AutoDRIVE`).
