# Penn RoboRacer Technical Challenge: LiDAR wall follower

A ROS 2 (Humble) node that drives the AutoDRIVE RoboRacer practice track from LiDAR alone.

| | |
|---|---|
| **Fastest clean lap** | **7.039 s** (practice track, IROS 2026 practice simulator, no wall contact; 12 of 12 laps clean in the final check) |
| **Video** | VIDEO_LINK_HERE (unlisted YouTube, Driver's Eye camera) |
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
   **Corner preview:** the mean direction of the farthest returns within ±60° shows where the track
   turns. `preview_gain × that angle` is added to the error, so the car turns in a little earlier
   and takes a wider, faster arc through the hairpins.
4. **Choose a speed.** The target speed is the lowest of three limits:
   - **top speed** `v_max`;
   - **braking:** `√(2·a_brake·d)`, where `d` is the free distance along the arc the car is
     currently steering on. Using the arc matters: mid-corner, the straight-ahead distance is
     short even though the path is clear;
   - **cornering:** `√(a_lat / κ)`, where `κ = tan(δ) / wheelbase` is the curvature of the
     current steering angle.
5. **Throttle.** In this simulator throttle behaves like a speed command (measured: steady
   speed ≈ 25 m/s × throttle, reached in under 0.5 s), so `throttle = v_target / 25`.

The LiDAR produces about 40 new scans per second, but the bridge republishes each one 3–5 times.
Steering is recomputed only when a new scan arrives, so the derivative sees real changes. Throttle
is refreshed on every message, using the latest speed.

When the node stops (Ctrl+C or SIGTERM), it publishes zero throttle and steering before shutting
down. The
bridge keeps replaying the last command it received, so without this the car would keep driving.

Every tunable is a ROS parameter, for example:

```bash
ros2 run autodrive_roboracer wall_follow_node --ros-args -p v_max:=7.0 -p lat_accel:=10.0
ros2 param set /wall_follow_node kp 1.1     # live tuning while driving
```

## Results

All lap times below come from the simulator's own lap timer, recorded by `tools/experiment.py`.
They are **flying laps (lap 2 onward)**. A reset does not restart the simulator's lap timer, so
lap 1 also contains the idle time before the car starts.

**Final node.** The actual node (`ros2 run autodrive_roboracer wall_follow_node`, default
parameters) completed 3 runs × 4 laps. All 12 laps were free of wall contact (`collision_count`
unchanged). Best lap **7.039 s**, mean of the flying laps 7.097 ± 0.035 s.

![Final laps coloured by speed](docs/final_laps.png)

*Four laps of the final node on the practice track. Walls come from logged LiDAR scans.
Blue = slow, red = fast (6.5 m/s). The car runs clockwise from the green square.*

### Baselines and ablation

Protocol: each variant ran 3 times × 4 laps, which gives 9 flying laps per variant. The runs were
interleaved round-robin across variants, so slow drift in the simulator cannot favour one variant.
Each result records the git commit and a hash of the controller file
([`experiments/final.sh`](experiments/final.sh),
[`experiments/results_final.jsonl`](experiments/results_final.jsonl)).

| Variant | Clean runs | Best (s) | Mean ± std (s) | Δ vs final |
|---|---|---|---|---|
| Textbook wall follower, left wall (F1TENTH lab: PD + 1.5/1.0/0.5 m/s steps) | 3/3 | 24.036 | 24.080 ± 0.042 | +16.99 |
| Textbook wall follower, right (inner) wall | 3/3 | 20.897 | 21.015 ± 0.084 | +13.92 |
| **Final** | **3/3** | **7.060** | **7.095 ± 0.036** | – |
| − centring (follow the right wall only) | 3/3 | 7.758 | 7.919 ± 0.157 | +0.82 |
| − corner preview | 3/3 | 7.507 | 7.562 ± 0.041 | +0.47 |
| − arc clearance (straight-ahead corridor instead) | 3/3 | 7.272 | 7.292 ± 0.014 | +0.20 |
| − speed feedback (`speed_kp=0`) | 3/3 | 7.229 | 7.265 ± 0.036 | +0.17 |
| − speed-dependent look-ahead (fixed 1.0 m) | 3/3 | 7.018 | 7.078 ± 0.048 | −0.02 |
| − side guard | 3/3 | 6.979 | 7.034 ± 0.032 | −0.06 |
| − adaptive speed (constant 3 m/s) | 0/3 | – | crashed every run | – |

What the table says:

- **Speed control is the main gain.** The textbook follower is safe but crawls at 1.5 m/s.
  Braking on the free distance ahead and limiting cornering speed takes the lap from about 21 s to
  about 7 s. The final steering cannot run at a fixed speed: at a constant 3 m/s it crashed in all
  3 runs, because the corner preview cuts corners, and that only works when the car also slows for
  them.
- **Centring, the corner preview, arc clearance and speed feedback** each give a clear gain, well
  beyond the run-to-run noise (about ±0.04 s). Centring also makes the laps more repeatable
  (std 0.04 s vs 0.16 s).
- **The speed-dependent look-ahead made no measurable difference.** It stays in as the default,
  but the data does not support it.
- **The side guard costs about 0.06 s** (about 1.5 std). It stays, because it is the only term
  that reacts directly to a wall closer than 0.35 m.

### How the parameters were found

The parameters came from six screening rounds of one-factor-at-a-time changes, each run once
(`experiments/screen*.sh`, `experiments/results_screening.jsonl`). Then a red-team code review,
then the final evaluation above. Findings that shaped the design:

| Finding (measured) | Consequence |
|---|---|
| Throttle acts as a speed command: steady speed ≈ 25 m/s × throttle, reached in < 0.5 s | `throttle = v_target / 25`, plus a small speed-error term |
| Grip limit ≈ 6–7 m/s² lateral. Asking for 10 made the car slide wide (only ~50 % of the commanded curvature) and the lap got slower | `lat_accel = 6.5` |
| Constant 4–5 m/s crashes into the outer wall of the bottom hairpin | braking on free distance; arc instead of straight corridor |
| The LiDAR gives ~40 new scans/s, but the bridge republishes each one 3–5× (~160 msg/s) | steering is only recomputed on a new scan (found in the code review: before, the D term became a pulse train) |
| The D term never helped (kd 0 ≈ 0.03 < 0.06 < 0.12) | the projected error `D + L·sin α` already contains the heading; kd kept small |
| Braking ≈ 7.5 m/s², acceleration ≈ 6 m/s² | used in the offline simulator, which now also reproduces the 4 m/s crash |

**Robustness checks** (offline simulator, `tests/test_closed_loop.py`): the final controller
finishes on the mirrored, counter-clockwise track. It also finishes with a 40 Hz control rate,
with 2 cm LiDAR noise, and with up to 75 ms of extra command latency (laps get slower, no crash).
**Shutdown check** (real simulator): Ctrl+C on the node mid-lap at 4.4 m/s brought the car to
0.0 m/s, and the bridge reported throttle 0.

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
