#!/bin/bash
# Run a single NCLT benchmark sequence.
# Usage: bash benchmarks/run_one.sh 2012-01-08

SEQ=${1:-2012-01-08}

export PATH="/home/manankharwar/.local/bin:$PATH"
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/Admin/ROS/ROS/fusioncore/install/setup.bash

REPO=/mnt/c/Users/Admin/ROS/ROS/fusioncore
RATE=3.0
DURATION=600.0
WALL_TIME=220

DATA_DIR="$REPO/benchmarks/nclt/$SEQ"
BAG_DIR="$DATA_DIR/bag"

echo "========================================"
echo "SEQ: $SEQ  ($(date))"
echo "========================================"

pkill -9 -f "fusioncore_node|nclt_player|ekf_node|navsat_transform_node|ros2 bag record|ros2 launch|component_container" 2>/dev/null || true
sleep 8

rm -rf "$BAG_DIR"
rm -f "$DATA_DIR/fusioncore.tum" "$DATA_DIR/rl_ekf.tum" "$DATA_DIR/ground_truth.tum"

echo "Launching benchmark — will run for ${WALL_TIME}s then stop..."
timeout --signal=SIGINT $WALL_TIME ros2 launch fusioncore_datasets nclt_benchmark.launch.py \
    data_dir:="$DATA_DIR/raw files" \
    output_bag:="$BAG_DIR" \
    playback_rate:=$RATE \
    duration_s:=$DURATION 2>&1 | tee "$DATA_DIR/results/launch.log" || true

sleep 8
pkill -9 -f "fusioncore_node|nclt_player|ekf_node|navsat_transform_node|ros2 bag record|component_container" 2>/dev/null || true
sleep 3

if [ ! -d "$BAG_DIR" ]; then
    echo "ERROR: bag not created for $SEQ"
    exit 1
fi

echo "Bag ready. Running evaluation..."

python3 "$REPO/tools/nclt_rtk_to_tum.py" \
    --rtk "$DATA_DIR/raw files/gps_rtk.csv" \
    --out "$DATA_DIR/ground_truth.tum"

python3 "$REPO/tools/odom_to_tum.py" \
    --bag "$BAG_DIR" --topic /fusion/odom \
    --out "$DATA_DIR/fusioncore.tum"

python3 "$REPO/tools/odom_to_tum.py" \
    --bag "$BAG_DIR" --topic /rl/odometry \
    --out "$DATA_DIR/rl_ekf.tum"

sort -n "$DATA_DIR/fusioncore.tum" > /tmp/fc_s.tum && cp /tmp/fc_s.tum "$DATA_DIR/fusioncore.tum"
sort -n "$DATA_DIR/rl_ekf.tum"    > /tmp/rl_s.tum && cp /tmp/rl_s.tum "$DATA_DIR/rl_ekf.tum"

python3 "$REPO/tools/evaluate.py" \
    --gt         "$DATA_DIR/ground_truth.tum" \
    --fusioncore "$DATA_DIR/fusioncore.tum" \
    --rl         "$DATA_DIR/rl_ekf.tum" \
    --sequence   "$SEQ" \
    --out_dir    "$DATA_DIR/results"

echo "DONE: $SEQ  ($(date))"
