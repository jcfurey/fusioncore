#!/bin/bash
# Run all 12 NCLT benchmark sequences with FusionCore DifferentialDrive motion model.
# Each sequence: ~230s wall clock at 3x speed (600s data).
# Total runtime: ~50 minutes.

set -e

export PATH="/home/manankharwar/.local/bin:$PATH"
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/Admin/ROS/ROS/fusioncore/install/setup.bash

REPO=/mnt/c/Users/Admin/ROS/ROS/fusioncore
RATE=3.0
DURATION=600.0
WALL_TIME=220   # 600/3 + 20s startup buffer

SEQUENCES=(
    2012-01-08
    2012-02-04
    2012-03-31
    2012-05-11
    2012-06-15
    2012-08-20
    2012-09-28
    2012-10-28
    2012-11-04
    2012-12-01
    2013-02-23
    2013-04-05
)

for SEQ in "${SEQUENCES[@]}"; do
    echo ""
    echo "========================================"
    echo "SEQ: $SEQ  ($(date))"
    echo "========================================"

    DATA_DIR="$REPO/benchmarks/nclt/$SEQ"
    BAG_DIR="$DATA_DIR/bag"

    # Kill any leftover ROS processes
    pkill -9 -f "fusioncore_node|nclt_player|ekf_node|navsat_transform_node|ros2 bag record" 2>/dev/null || true
    sleep 3

    # Remove old bag so recorder starts clean
    rm -rf "$BAG_DIR"

    # Launch benchmark in background
    ros2 launch fusioncore_datasets nclt_benchmark.launch.py \
        data_dir:="$DATA_DIR" \
        output_bag:="$BAG_DIR" \
        playback_rate:=$RATE \
        duration_s:=$DURATION \
        > "$DATA_DIR/results/launch.log" 2>&1 &
    LAUNCH_PID=$!

    echo "Launch PID: $LAUNCH_PID — waiting ${WALL_TIME}s for playback..."
    sleep $WALL_TIME

    # Graceful shutdown: SIGTERM lets rosbag2 flush its index
    kill -TERM $LAUNCH_PID 2>/dev/null || true
    sleep 6

    # Force-kill anything still running
    pkill -9 -f "fusioncore_node|nclt_player|ekf_node|navsat_transform_node|ros2 bag record" 2>/dev/null || true
    sleep 2

    if [ ! -d "$BAG_DIR" ]; then
        echo "ERROR: bag not created for $SEQ — skipping evaluation"
        continue
    fi

    echo "Bag ready. Running evaluation pipeline..."

    # Ground truth
    python3 "$REPO/tools/nclt_rtk_to_tum.py" \
        --rtk "$DATA_DIR/raw files/gps_rtk.csv" \
        --out "$DATA_DIR/ground_truth.tum"

    # Extract trajectories
    python3 "$REPO/tools/odom_to_tum.py" \
        --bag "$BAG_DIR" \
        --topic /fusion/odom \
        --out "$DATA_DIR/fusioncore.tum"

    python3 "$REPO/tools/odom_to_tum.py" \
        --bag "$BAG_DIR" \
        --topic /rl/odometry \
        --out "$DATA_DIR/rl_ekf.tum"

    # Sort by timestamp
    sort -n "$DATA_DIR/fusioncore.tum" > /tmp/fc_sorted.tum && cp /tmp/fc_sorted.tum "$DATA_DIR/fusioncore.tum"
    sort -n "$DATA_DIR/rl_ekf.tum"    > /tmp/rl_sorted.tum && cp /tmp/rl_sorted.tum "$DATA_DIR/rl_ekf.tum"

    # Evaluate
    python3 "$REPO/tools/evaluate.py" \
        --gt         "$DATA_DIR/ground_truth.tum" \
        --fusioncore "$DATA_DIR/fusioncore.tum" \
        --rl         "$DATA_DIR/rl_ekf.tum" \
        --sequence   "$SEQ" \
        --out_dir    "$DATA_DIR/results"

    echo "DONE: $SEQ"
done

echo ""
echo "All 12 sequences complete. $(date)"
