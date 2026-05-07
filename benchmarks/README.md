# FusionCore Benchmarks

Benchmarks comparing FusionCore against robot_localization EKF on public datasets.

## Structure

```
benchmarks/
  nclt/
    2012-01-08/         ← one folder per sequence
      ground_truth.tum  ← RTK GPS reference (gitignored, regenerate with step 2)
      fusioncore.tum    ← FusionCore trajectory (gitignored, regenerate with step 3)
      rl_ekf.tum        ← RL-EKF trajectory (gitignored, regenerate with step 3)
      bag/              ← ROS2 bag from the run (gitignored, large)
      results/
        BENCHMARK.md    ← results table (committed)
        README.md       ← sequence-specific notes
    2012-02-04/         ← add siblings the same way
    2012-03-31/
```

> `*.tum`, `*.mcap`, and `bag/` folders are gitignored: they are large generated
> files. Only `results/BENCHMARK.md` is committed.

---

## Results Summary

RL-EKF gating: `odom0_twist_rejection_threshold: 4.03` (chi2(3,0.999)), `odom1_pose_rejection_threshold: 3.72` (chi2(2,0.999)).
RL-UKF: NaN divergence on all sequences (confirmed by RL maintainer as known numerical instability).

| Dataset | Sequence | FC ATE RMSE | RL-EKF ATE RMSE | RL-UKF | Winner |
|---------|----------|-------------|-----------------|--------|--------|
| NCLT (Univ. of Michigan) | 2012-01-08 | **5.6 m** | 13.0 m | NaN divergence at t=31s | FusionCore (2.3×) |
| NCLT (Univ. of Michigan) | 2012-02-04 | **9.7 m** | 19.1 m | NaN divergence at t=22s | FusionCore (2.0×) |
| NCLT (Univ. of Michigan) | 2012-03-31 | **4.2 m** | 54.3 m | NaN divergence at t=18s | FusionCore (12.9×) |
| NCLT (Univ. of Michigan) | 2012-08-20 | **7.5 m** | 24.1 m | NaN divergence | FusionCore (3.2×) |
| NCLT (Univ. of Michigan) | 2012-11-04 | 28.6 m | **9.6 m** | NaN divergence | RL-EKF (3.0×) |
| NCLT (Univ. of Michigan) | 2013-02-23 | **4.1 m** | 11.0 m | NaN divergence | FusionCore (2.7×) |

---

## Running a Benchmark (any sequence)

Replace `<SEQ>` with the sequence date (e.g. `2012-01-08`) everywhere below.

### Prerequisites

- NCLT data downloaded: `http://robots.engin.umich.edu/nclt/`
- ROS2 Jazzy sourced
- `evo` installed: `python3 -m pip install evo --break-system-packages`

---

### Step 1: Kill any leftover ROS processes

```bash
pkill -9 -f "fusioncore_node|nclt_player|ekf_node|navsat_transform|ros2 bag record"
sleep 3
```

Always do this first. Leftover processes from a previous run contaminate the bag.

---

### Step 2: Create the output folder

```bash
SEQ=2012-01-08    # change this to your sequence
mkdir -p benchmarks/nclt/$SEQ/results
```

---

### Step 3: Run the benchmark (~200s at 3× speed)

```bash
source install/setup.bash

ros2 launch fusioncore_datasets nclt_benchmark.launch.py \
  data_dir:=/path/to/nclt/$SEQ \
  output_bag:=./benchmarks/nclt/$SEQ/bag \
  playback_rate:=3.0 \
  duration_s:=600.0
```

Wait for `Playback complete.` to print, then **Ctrl+C immediately**.

---

### Step 4: Convert ground truth (RTK GPS → TUM)

```bash
python3 tools/nclt_rtk_to_tum.py \
  --rtk /path/to/nclt/$SEQ/gps_rtk.csv \
  --out benchmarks/nclt/$SEQ/ground_truth.tum
```

---

### Step 5: Extract filter trajectories from the bag

```bash
python3 tools/odom_to_tum.py \
  --bag   benchmarks/nclt/$SEQ/bag \
  --topic /fusion/odom \
  --out   benchmarks/nclt/$SEQ/fusioncore.tum

python3 tools/odom_to_tum.py \
  --bag   benchmarks/nclt/$SEQ/bag \
  --topic /rl/odometry \
  --out   benchmarks/nclt/$SEQ/rl_ekf.tum
```

---

### Step 6: Sort the trajectory files

```bash
sort -n benchmarks/nclt/$SEQ/fusioncore.tum > /tmp/fc_sorted.tum
sort -n benchmarks/nclt/$SEQ/rl_ekf.tum    > /tmp/rl_sorted.tum
cp /tmp/fc_sorted.tum benchmarks/nclt/$SEQ/fusioncore.tum
cp /tmp/rl_sorted.tum benchmarks/nclt/$SEQ/rl_ekf.tum
```

---

### Step 7: Evaluate

```bash
python3 tools/evaluate.py \
  --gt         benchmarks/nclt/$SEQ/ground_truth.tum \
  --fusioncore benchmarks/nclt/$SEQ/fusioncore.tum \
  --rl         benchmarks/nclt/$SEQ/rl_ekf.tum \
  --sequence   $SEQ \
  --out_dir    benchmarks/nclt/$SEQ/results
```

Results print to terminal and are saved to `benchmarks/nclt/$SEQ/results/BENCHMARK.md`.

---

### Step 8: Commit only the results

```bash
git add benchmarks/nclt/$SEQ/results/BENCHMARK.md
git commit -m "benchmark(nclt): add results for $SEQ"
```

The large files (`.tum`, bag) are gitignored automatically.

---

## Adding a New Sequence

```bash
# Download the sequence from http://robots.engin.umich.edu/nclt/
# Then just repeat Steps 1–8 with the new SEQ value.
SEQ=2012-02-04
mkdir -p benchmarks/nclt/$SEQ/results
# ... same steps as above
```

No code changes needed: the pipeline is fully parameterized by `data_dir` and `$SEQ`.

---

## Oxford Robotcar Dataset

The [Oxford Robotcar Dataset](https://robotcar-dataset.robots.ox.ac.uk/) contains 100+ traversals of a 10 km route through Oxford, collected across a full year (2014-2015). Ground truth is from a NovAtel SPAN-CPT RTK GPS/INS unit (~10 cm accuracy).

Sensors used by FusionCore: NovAtel SPAN-CPT GPS/INS (GPS + synthesized IMU + synthesized wheel odometry from INS derivatives).

### Structure

```
benchmarks/
  robotcar/
    2014-11-18-13-20-12/     ← sequence folder (date-time)
      ground_truth.tum        ← from gps/ins.csv (gitignored, regenerate with step 2)
      fusioncore.tum          ← FusionCore trajectory (gitignored)
      rl_ekf.tum              ← RL-EKF trajectory (gitignored)
      bag/                    ← ROS2 bag (gitignored, large)
      results/
        BENCHMARK.md
```

### Recommended sequences

| Sequence | Duration | Conditions | Notes |
|----------|----------|------------|-------|
| `2014-11-18-13-20-12` | ~15 min | Overcast, clear roads | Good starter sequence |
| `2014-12-09-13-21-02` | ~15 min | Winter, wet roads | Weather robustness |
| `2015-02-03-08-45-10` | ~15 min | Low sun, morning | Lighting challenge |
| `2015-08-28-09-29-42` | ~15 min | Summer, busy streets | Dense traffic |

### Download

```bash
# Register at https://robotcar-dataset.robots.ox.ac.uk/ (free academic access)
# Download a sequence using the RobotCar SDK or direct URL:

SEQ=2014-11-18-13-20-12
mkdir -p /path/to/robotcar/$SEQ

# Required files per sequence (download from dataset portal):
#   gps/ins.csv        — RTK GPS/INS ground truth + synthesized IMU source
#   gps/gps.csv        — raw GPS fixes (optional; used if present)
```

### Step 1: Kill leftover ROS processes

```bash
pkill -9 -f "fusioncore_node|robotcar_player|ekf_node|navsat_transform|ros2 bag record"
sleep 3
```

### Step 2: Create output folder and ground truth

```bash
SEQ=2014-11-18-13-20-12
mkdir -p benchmarks/robotcar/$SEQ/results

python3 tools/robotcar_ins_to_tum.py \
  --ins /path/to/robotcar/$SEQ/gps/ins.csv \
  --out benchmarks/robotcar/$SEQ/ground_truth.tum
```

### Step 3: Run the benchmark (~300s at 3× speed for 15 min sequence)

```bash
source install/setup.bash

ros2 launch fusioncore_datasets robotcar_benchmark.launch.py \
  data_dir:=/path/to/robotcar/$SEQ \
  output_bag:=./benchmarks/robotcar/$SEQ/bag \
  playback_rate:=3.0 \
  duration_s:=900.0
```

Wait for `Playback complete.` then **Ctrl+C immediately**.

Optional fault injection:

```bash
# GPS spike at t=120s, 500m magnitude
ros2 launch fusioncore_datasets robotcar_benchmark.launch.py \
  data_dir:=/path/to/robotcar/$SEQ \
  output_bag:=./benchmarks/robotcar/$SEQ/bag_spike \
  playback_rate:=3.0 duration_s:=300.0 \
  gps_spike_time_s:=120.0 gps_spike_magnitude_m:=500.0

# GPS outage 120–165s
ros2 launch fusioncore_datasets robotcar_benchmark.launch.py \
  data_dir:=/path/to/robotcar/$SEQ \
  output_bag:=./benchmarks/robotcar/$SEQ/bag_outage \
  playback_rate:=3.0 duration_s:=300.0 \
  gps_outage_start_s:=120.0 gps_outage_duration_s:=45.0
```

### Step 4: Extract trajectories

```bash
python3 tools/odom_to_tum.py \
  --bag   benchmarks/robotcar/$SEQ/bag \
  --topic /fusion/odom \
  --out   benchmarks/robotcar/$SEQ/fusioncore.tum

python3 tools/odom_to_tum.py \
  --bag   benchmarks/robotcar/$SEQ/bag \
  --topic /rl/odometry \
  --out   benchmarks/robotcar/$SEQ/rl_ekf.tum
```

### Step 5: Sort and evaluate

```bash
sort -n benchmarks/robotcar/$SEQ/fusioncore.tum > /tmp/fc_sorted.tum
sort -n benchmarks/robotcar/$SEQ/rl_ekf.tum    > /tmp/rl_sorted.tum
cp /tmp/fc_sorted.tum benchmarks/robotcar/$SEQ/fusioncore.tum
cp /tmp/rl_sorted.tum benchmarks/robotcar/$SEQ/rl_ekf.tum

python3 tools/evaluate.py \
  --gt         benchmarks/robotcar/$SEQ/ground_truth.tum \
  --fusioncore benchmarks/robotcar/$SEQ/fusioncore.tum \
  --rl         benchmarks/robotcar/$SEQ/rl_ekf.tum \
  --sequence   $SEQ \
  --out_dir    benchmarks/robotcar/$SEQ/results
```

### Step 6: Commit results

```bash
git add benchmarks/robotcar/$SEQ/results/BENCHMARK.md
git commit -m "benchmark(robotcar): add results for $SEQ"
```

### Notes on IMU synthesis

The Robotcar INS solution does not export a raw IMU stream. `robotcar_player.py` synthesizes IMU data by differentiating the INS orientation and velocity estimates (NED frame) and converting to ENU body frame. This produces noisier IMU than hardware-native, so `imu.accel_noise` and `imu.gyro_noise` in `robotcar_fusioncore.yaml` are set conservatively.

Because the synthesized IMU already includes gravity in the acceleration terms, `imu.remove_gravitational_acceleration: false` is set in the config.
