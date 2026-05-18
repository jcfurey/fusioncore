# FusionCore

**ROS 2 UKF sensor fusion. IMU + wheel encoders + GPS + GPS velocity + radar Doppler → one position estimate. No manual noise tuning. Apache 2.0.**

[![CI](https://github.com/manankharwar/fusioncore/actions/workflows/ci.yml/badge.svg)](https://github.com/manankharwar/fusioncore/actions/workflows/ci.yml)
[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.19834991.svg)](https://doi.org/10.5281/zenodo.19834991)

---

## What it does

FusionCore is a 23-state UKF that fuses IMU, wheel encoders, GPS position, GPS velocity, and radar Doppler ego-velocity into a single clean odometry output at 100 Hz. It runs as a single ROS 2 lifecycle node: no `navsat_transform`, no coordinate projection node, no feedback loop between two filters.

It publishes `/fusion/odom` and the full `odom → base_link` TF. Nav2 consumes it directly.

GPS is optional. FusionCore runs fine on IMU + wheel odometry alone for indoor robots. GPS velocity and radar velocity are each independently optional: enable whichever sensors you have.

---

## How FusionCore differs from robot_localization

robot_localization is a solid, well-maintained package used on tens of thousands of robots. FusionCore makes different architectural choices:

| Capability | robot_localization | FusionCore |
|---|---|---|
| GPS fusion | navsat_transform node required; ECEF TF frame added in rolling-devel | Filter state runs natively in ECEF: no UTM projection |
| IMU bias estimation | Not in state vector | Gyro + accel bias as filter states |
| Outlier rejection | Mahalanobis threshold (manual scalars, no DOF guidance) | Mahalanobis chi-squared gate (pre-calibrated to sensor DOF) |
| Adaptive noise | Fixed config values | Auto from innovation sequence |
| ZUPT | Not built-in | Auto when stationary |
| Delay compensation | `smooth_lagged_data` + `history_length` | IMU ring buffer replay |
| GPS fix quality gating | Not built-in | HDOP, satellite count, fix type |
| Dual antenna heading | Not built-in | Yes |
| Inertial coast mode | Not built-in | Auto on sustained GPS dropout |
| GPS velocity fusion (wheel slip detection) | Not built-in | Yes (Doppler vs wheel innovation reveals slip) |
| Radar Doppler velocity fusion | Not built-in | Yes (works indoors, all weather, slip detection) |
| ROS 2 Jazzy / Humble | Ported from ROS 1 | Native, from scratch |

---

## Install

!!! note "Humble users"
    Replace `jazzy` with `humble` in the commands below (Ubuntu 22.04).

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/manankharwar/fusioncore.git
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

!!! tip "Headless / Raspberry Pi users"
    Add a `COLCON_IGNORE` file before building to skip the Gazebo package:
    ```bash
    touch ~/ros2_ws/src/fusioncore/fusioncore_gazebo/COLCON_IGNORE
    ```

---

## Quick start

```bash
ros2 launch fusioncore_ros fusioncore_nav2.launch.py \
  fusioncore_config:=/path/to/your_robot.yaml
```

That's the full stack: FusionCore + Nav2, lifecycle managed automatically.

**No Nav2?** Use `fusioncore.launch.py` instead:

```bash
ros2 launch fusioncore_ros fusioncore.launch.py \
  fusioncore_config:=/path/to/your_robot.yaml
```

---

## Benchmark: 9 NCLT sequences, same config, no per-sequence tuning

Evaluated against robot_localization EKF on the [NCLT dataset](http://robots.engin.umich.edu/nclt/) (University of Michigan). Same IMU, wheel odometry, and GPS inputs. SE3-aligned ATE against RTK ground truth.

| Sequence | FC ATE | RL-EKF ATE | Winner |
|---|---|---|---|
| 2012-01-08 (92 min) | **18.6 m** | 41.2 m | FC +55% |
| 2012-02-04 (77 min) | **49.7 m** | 265.5 m | FC +81% |
| 2012-03-31 (87 min) | **22.0 m** | 156.5 m | FC +86% |
| 2012-05-11 (84 min) | **9.7 m** | 11.5 m | FC +16% |
| 2012-06-15 (55 min) | 49.2 m | **18.2 m** | RL +63% |
| 2012-08-20 (83 min) | 98.3 m | **10.6 m** | RL +89% |
| 2012-09-28 (77 min) | **10.8 m** | 55.7 m | FC +81% |
| 2012-10-28 (85 min) | **29.9 m** | 60.0 m | FC +50% |
| 2012-11-04 (79 min) | **60.1 m** | 122.0 m | FC +51% |

**7/9 FC wins.** RL-EKF's losses on 2012-02-04, 2012-03-31, and 2012-09-28 trace to a single cause: NCLT's GPS receiver reports tighter covariances than its actual noise, causing RL's Mahalanobis gate to reject valid fixes for long stretches. FusionCore's `gnss.base_noise_xy` floors measurement noise to match real sensor behavior.

The two FC losses (2012-06-15 and 2012-08-20) both involve GPS blackouts longer than 2 minutes paired with GPS outlier data quality issues at specific sequence boundaries. Full root-cause analysis in the [benchmark reference](reference/benchmark.md).

RL-UKF diverged with NaN on all nine sequences (known numerical instability under sim-time playback).

---

## Where to go next

- **New user** → [Getting Started](getting-started.md)
- **Configuring your robot** → [Configuration](configuration.md)
- **Using with Nav2** → [Nav2 Integration](nav2.md)
- **Coming from robot_localization** → [Migration Guide](migration_from_robot_localization.md)
- **Simulation / testing without hardware** → [Simulation](simulation.md)
- **What all the parameters mean** → [Configuration](configuration.md)
- **How the filter actually works** → [How It Works](how-it-works.md)
