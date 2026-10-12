# Penn RoboRacer Technical Challenge: LiDAR wall follower

A ROS 2 (Humble) node that drives the AutoDRIVE RoboRacer practice track (IROS 2026 practice
simulator) from its LiDAR scan.

| | |
|---|---|
| **Fastest clean lap** | **6.299 s** |
| **Consecutive clean laps** | **31 of 31** in one run of the real node (`ros2 run`, default parameters). Best 6.324 s, slowest flying lap 6.426 s |
| **Video** | VIDEO_LINK_HERE (unlisted YouTube, Driver's Eye camera) |
| **Node** | [`autodrive_devkit/autodrive_roboracer/wall_follow_node.py`](autodrive_devkit/autodrive_roboracer/wall_follow_node.py) |

All times come from the simulator's own lap timer, which is the same clock the HUD shows.

![Final configuration, laps coloured by speed](docs/final_laps.png)

*Final configuration on the practice track. Blue is slow, red is fast (7.3 m/s). The car runs
clockwise from the green square. Walls are drawn from logged LiDAR scans.*

## How it works

Each LiDAR scan (1080 beams over 270°) produces one steering command and one throttle command.

**Steering**

1. **Wall fit.** For each side wall, beam *b* points 90° to the side and beam *a* points 50°
   further forward. For a straight wall, `tan α = (a·cos θ − b) / (a·sin θ)` gives the car's angle
   to the wall, and `D = b·cos α` its distance. The distance is projected ahead,
   `D + L·sin α`, with a look-ahead `L` that grows with speed.
2. **Centring.** The error is `(D_left − D_right) / 2`, and a PD controller turns it into a
   steering angle. If one wall disappears, the node follows the other wall at half the remembered
   track width. Wall-angle and error clamps reject bad fits. A side guard pushes away from any wall
   closer than 0.37 m.
3. **Corner preview.** The mean direction of the farthest returns within ±60° shows where the
   track turns. A small bias towards it (`preview_gain`) makes the car turn in earlier and take a
   wider, faster arc. This term borrows from follow-the-gap; everything else is wall following.
   With `preview_gain:=0` the node is a pure two-wall follower, which is also clean but slower
   (7.33 s, see the ablation).

**Speed**

4. **Target speed** is the lowest of three limits:
   - the top speed `v_max`;
   - braking: `√(v_corner² + 2·a_brake·d)`, where `d` is the free distance along the arc the car is
     steering on. Mid-corner, the straight-ahead distance is short even when the path is clear,
     so the arc matters;
   - cornering: `√(a_lat / κ)`, with `κ = tan(δ) / wheelbase`.
5. **Throttle.** In this simulator, throttle sets wheel speed (measured: steady speed ≈ 25 m/s ×
   throttle). The throttle is therefore `v_target / 25` plus a speed-error term from odometry.
   **Traction control** caps the commanded wheel speed at 18 % (+0.5 m/s) above the car's speed. Full throttle
   from low speed spins the wheels: in the measurement, 5.4 m/s after 1.2 s instead of 7.4 m/s.

**Plumbing**

- The LiDAR produces about 40 new scans per second, and the bridge republishes each one 3–5
  times. Steering is recomputed only on a new scan, so the derivative sees real changes. Throttle
  is refreshed on every message.
- When the node stops (Ctrl+C or SIGTERM), it publishes zero throttle and steering. The bridge
  keeps replaying the last command it received, so without this the car would keep driving.
- Every tunable is a ROS parameter, and parameters can be changed live
  (`ros2 param set /wall_follow_node v_max 6.0`).

## Results

### From baseline to final

Lap times are flying laps (lap 2 onward). A reset does not restart the simulator's lap timer, so
lap 1 also contains the idle time before the car starts. Each row is 3 runs × 4 laps (9 flying
laps), with runs interleaved across variants.

| Version | Mean flying lap (s) | Best (s) | Wall contacts |
|---|---|---|---|
| Textbook wall follower, left wall (F1TENTH lab: PD, 1.5/1.0/0.5 m/s by steering angle) | 24.080 ± 0.042 | 24.036 | 0 |
| Textbook wall follower, right (inner) wall | 21.015 ± 0.084 | 20.897 | 0 |
| Hand-tuned: centring, arc braking, cornering limit, corner preview, speed feedback | 7.104 ± 0.043 | 7.052 | 0 |
| **Final: + traction control, parameters tuned with CMA-ES** | **6.374 ± 0.055** | **6.299** | **0** |

Every run of the final configuration (4 runs, 43 laps) was clean. Across all 39 flying laps:
mean 6.367 s, std 0.036 s, best 6.299 s, worst 6.466 s.

### Ablation: remove one component from the final configuration

| Variant | Mean flying lap (s) | Δ vs final (s) | Clean runs |
|---|---|---|---|
| **Final** | **6.374 ± 0.055** | – | 3/3 |
| − centring (follow the right wall only) | 7.869 ± 0.099 | +1.50 | 3/3 |
| − arc clearance (straight-ahead corridor) | 7.370 ± 0.009 | +1.00 | 3/3 |
| − corner preview (pure wall follower) | 7.327 ± 0.039 | +0.95 | 3/3 |
| − speed feedback | 7.135 ± 0.235 | +0.76 | 3/3 |
| − traction control | 6.506 ± 0.260 | +0.13 | 3/3 |
| − speed-dependent look-ahead (fixed 1.0 m) | 6.485 ± 0.280 | +0.11 | 3/3 |
| − corner-entry speed (`v_corner=0`) | 6.351 ± 0.013 | −0.02 | 3/3 |
| − side guard | 6.363 ± 0.025 | −0.01 | 3/3 |
| − adaptive speed (constant 3 m/s) | 10.206 ± 0.001 | +3.83 | 3/3 |

Centring, arc clearance, the corner preview and speed feedback each carry close to a second.
Traction control and the speed-dependent look-ahead give smaller gains, and removing them also
makes laps less consistent (std 0.26–0.28 s). The corner-entry speed and the side guard show no
measurable effect on lap time. The side guard stays as a safety margin.

### Parameter tuning (CMA-ES)

Ten parameters were tuned with CMA-ES ([`tools/tune.py`](tools/tune.py)) directly in the
simulator. The ten: top speed, cornering limit, braking, corner-entry speed, stop margin, preview
gain, kp, look-ahead, traction slip and side clearance.

- One evaluation is a reset plus 3 laps. The score is the mean of laps 2–3, and any wall contact
  scores 12 s.
- Stage 1 started from the hand-tuned configuration plus traction control, and ran 60
  evaluations. `kp` and the
  look-ahead ended on the edge of their ranges.
- Stage 2 widened those two ranges and ran 30 more evaluations.
- All 90 configurations were clean.
- The best one was re-measured from scratch: 3 runs × 4 laps, interleaved with the hand-tuned
  configuration and the runners-up. This guards against picking a lucky evaluation. The result was
  6.363 s, against 6.32 s during the search. Its values, rounded, are the node's defaults.

What the optimiser changed:
- gentler steering (`kp` 0.9 → 0.42, base look-ahead 0.5 → 0.26 m);
- braking planned at the measured 6 m/s² instead of 4;
- a higher cornering limit (7.8 m/s²);
- traction control at 18 % slip.

Full history: [`experiments/tuning_history.jsonl`](experiments/tuning_history.jsonl).

### Measurements that shaped the design

| Measured in the simulator | Design consequence |
|---|---|
| Throttle sets wheel speed: steady speed ≈ 25 m/s × throttle | `throttle = v_target / 25` + speed feedback |
| Full throttle from low speed spins the wheels (5.4 vs 7.4 m/s after 1.2 s) | traction control |
| Braking is ≈ 6 m/s² for any throttle below the current speed (0, −0.3 and −1 are the same) | no special braking logic; plan with 6 m/s² |
| Constant 4–5 m/s crashes into the outer wall of the bottom hairpin; 3 m/s is clean | brake on the free distance ahead, limit cornering speed |
| The LiDAR gives ~40 new scans/s; the bridge republishes each 3–5× (~160 messages/s) | steering only on new scans; D term computed between real scans |
| The D term never helped (kd 0 ≈ 0.03 < 0.06 < 0.12) | `D + L·sin α` already contains the heading; kd kept small |
| A reset does not restart the lap timer | compare flying laps only |

[`tools/sysid.py`](tools/sysid.py) reproduces the acceleration and braking measurements.

## Run it

Prerequisites: Docker Desktop, Git, and the IROS 2026 **practice** simulator.

```powershell
docker build --platform linux/amd64 -t penn-challenge -f autodrive_devkit.Dockerfile .
.\scripts\dev.ps1 up     # container + colcon build + bridge in the background (Windows)
.\scripts\dev.ps1 run    # start wall_follow_node with the default (final) parameters
```

Alternatively, use the commands from the challenge sheet. Start the bridge with
`ros2 launch autodrive_roboracer bringup_headless.launch.py`, then run
`ros2 run autodrive_roboracer wall_follow_node`. In the simulator, connect to `127.0.0.1:4567` and
set Driving Mode to **Autonomous**.

Options:

```bash
# pure two-wall follower (no corner preview)
ros2 run autodrive_roboracer wall_follow_node --ros-args -p preview_gain:=0.0
# hand-tuned configuration: slower (7.1 s), more steering margin
ros2 run autodrive_roboracer wall_follow_node --ros-args --params-file \
  /home/autodrive_devkit/src/autodrive_devkit/config/hand_tuned.yaml
```

## Testing

| What | Where | Runs in |
|---|---|---|
| Unit tests on synthetic scans: geometry, signs, invalid beams, missing walls, repeated scans, speed modes, traction control, parameter validation | `autodrive_devkit/test/test_wall_follow.py` | CI + container |
| Closed-loop tests in a small offline simulator on a replica of the track. Covers both directions (mirrored track), 40 Hz control, 2 cm LiDAR noise and 75 ms command latency. Also checks that a constant 4 m/s *does* crash, as it does in the real simulator | `tests/test_closed_loop.py`, `tools/sim2d.py` | CI + container |
| Timed laps in the real simulator: reset, lap count, wall contacts, per-scan trace, code hash per result | `tools/experiment.py`, `tools/summarize.py` | container + simulator |

GitHub Actions runs flake8, `colcon build` and both test suites in a `ros:humble` container on
every push. The offline simulator is calibrated to measured acceleration, braking and grip, but it
is a kinematic model: it catches regressions and does not predict lap times.

## Reproducing the experiments

Each results table above has a script and a raw results file in [`experiments/`](experiments/):

| Experiment | Script | Results |
|---|---|---|
| Baselines and hand-tuned ablation | `final.sh` | `results_final.jsonl` |
| Tuned candidates, final ablation, 31-lap run | `validate_tuned.sh`, `final2.sh` | `results_tuned_final.jsonl` |
| Screening rounds | `screen*.sh` | `results_screening*.jsonl` |
| CMA-ES search | `tools/tune.py` | `tuning_history.jsonl` |

`tools/summarize.py <file>` prints any of these as a table. Runs need the simulator in headless
batch mode (`-batchmode -nographics -ip 127.0.0.1 -port 4567`). Start the simulator *before* the
bridge. Otherwise the simulator's first message has no LiDAR data, the bridge raises a `KeyError`,
and the connection stalls.

## Notes and limitations

- **Where the results were measured.** All results were measured with the simulator in headless
  batch mode on one PC. The GUI mode used for the video also renders, which can change message
  timing. The offline tests cover 40 Hz control and 75 ms latency. If the car misbehaves in GUI
  mode, the hand-tuned parameter file above is the conservative fallback.
- **Run one ROS container at a time.** Containers on the same Docker network discover each
  other's ROS 2 topics, so two controllers would drive the same car. To run several, give each its
  own `ROS_DOMAIN_ID`.
- **Tuned for this track.** The parameters were tuned on the practice track. The algorithm is
  symmetric and passes the mirrored-track tests, but the exact numbers are specific to this
  track's geometry.

## Repository layout

```
autodrive_devkit/            ROS 2 package from AutoDRIVE + wall_follow_node.py (bridge unchanged)
  config/hand_tuned.yaml     fallback parameter file
autodrive_devkit.Dockerfile  unchanged from AutoDRIVE, so the image builds from this repo
tools/experiment.py          automated timed laps in the simulator
tools/tune.py                CMA-ES parameter search
tools/sysid.py               acceleration / braking measurements
tools/sim2d.py               offline simulator used by the closed-loop tests
tools/summarize.py           results tables (flying laps only)
tools/track_map.py           track map from logged scans
tools/lap_bound.py           point-mass reference lap time along a fixed path
tests/                       closed-loop tests + track replica
experiments/                 experiment scripts and raw results
scripts/                     Windows helpers: dev.ps1 (up/run/test/shell/down), tune_up.ps1
```

AutoDRIVE files are BSD-licensed, Copyright (c) AutoDRIVE / Tinker Twins (see `LICENSE.AutoDRIVE`).
