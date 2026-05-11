#!/bin/bash
# FusionCore quick verification test.
#
# Starts the filter with fake sensors and checks that all expected outputs
# are publishing. Replaces the 4-terminal manual test in Getting Started.
#
# Usage:
#   bash tools/quick_test.sh
#
# Requirements:
#   FusionCore built in this workspace:
#     colcon build --packages-up-to fusioncore_ros
#   OR use the Docker container:
#     docker run --rm ghcr.io/manankharwar/fusioncore:latest bash tools/quick_test.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"
PIDS=()
FAIL=0

GREEN='\033[0;32m'; RED='\033[0;31m'; BLUE='\033[0;34m'; YELLOW='\033[1;33m'; NC='\033[0m'
pass() { echo -e "  ${GREEN}[PASS]${NC} $*"; }
fail() { echo -e "  ${RED}[FAIL]${NC} $*"; FAIL=1; }
info() { echo -e "  ${BLUE}[....]${NC} $*"; }

cleanup() {
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
}
trap cleanup EXIT

echo ""
echo "  FusionCore Quick Test"
echo "  ====================="
echo ""

# ── 1. Source ROS ──────────────────────────────────────────────────────────────
info "Sourcing ROS environment..."
if [[ -z "${AMENT_PREFIX_PATH:-}" ]]; then
    if   [[ -f /opt/ros/jazzy/setup.bash  ]]; then source /opt/ros/jazzy/setup.bash
    elif [[ -f /opt/ros/humble/setup.bash ]]; then source /opt/ros/humble/setup.bash
    else
        fail "ROS not found. Run:  source /opt/ros/<distro>/setup.bash"
        exit 1
    fi
fi

if [[ -f "${WORKSPACE}/install/setup.bash" ]]; then
    source "${WORKSPACE}/install/setup.bash"
    pass "ROS environment sourced"
else
    fail "FusionCore not built. Run:  cd ${WORKSPACE} && colcon build --packages-up-to fusioncore_ros"
    exit 1
fi

# ── 2. TF publishers ──────────────────────────────────────────────────────────
info "Starting TF publishers..."
ros2 run tf2_ros static_transform_publisher \
    --frame-id base_link --child-frame-id imu_link >/dev/null 2>&1 &
PIDS+=($!)
ros2 run tf2_ros static_transform_publisher \
    --frame-id odom --child-frame-id base_link >/dev/null 2>&1 &
PIDS+=($!)
sleep 1

# ── 3. Launch FusionCore ──────────────────────────────────────────────────────
info "Launching FusionCore..."
ros2 launch fusioncore_ros fusioncore.launch.py >/dev/null 2>&1 &
PIDS+=($!)
sleep 3

# ── 4. Lifecycle (retry for DDS discovery latency on WSL2 / slow machines) ────
info "Configuring lifecycle node..."
for i in 1 2 3 4 5; do
    if ros2 lifecycle set /fusioncore configure >/dev/null 2>&1; then
        break
    fi
    if [[ $i -eq 5 ]]; then
        fail "Could not configure /fusioncore (node not found after 5 s)"
        echo "       Check: ros2 node list"
        exit 1
    fi
    sleep 1
done
sleep 1

info "Activating lifecycle node..."
if ! ros2 lifecycle set /fusioncore activate >/dev/null 2>&1; then
    fail "Could not activate /fusioncore"
    exit 1
fi
pass "Lifecycle: configure → activate"
sleep 1

# ── 5. Fake sensors ───────────────────────────────────────────────────────────
info "Publishing fake IMU at 100 Hz (stationary, gravity pointing up)..."
ros2 topic pub /imu/data sensor_msgs/msg/Imu "{
  header: {frame_id: 'imu_link'},
  angular_velocity: {x: 0.0, y: 0.0, z: 0.0},
  linear_acceleration: {x: 0.0, y: 0.0, z: 9.81},
  orientation_covariance: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}" --rate 100 >/dev/null 2>&1 &
PIDS+=($!)

info "Publishing fake wheel odometry at 50 Hz (stationary)..."
ros2 topic pub /odom/wheels nav_msgs/msg/Odometry "{
  header: {frame_id: 'odom'},
  twist: {twist: {linear: {x: 0.0}, angular: {z: 0.0}}}
}" --rate 50 >/dev/null 2>&1 &
PIDS+=($!)

info "Waiting 6 s for filter to initialize..."
sleep 6

# ── 6. Checks ─────────────────────────────────────────────────────────────────
echo ""
echo "  Checks:"
echo "  -------"

check_topic() {
    local topic="$1" label="$2"
    if ros2 topic echo "${topic}" --once --timeout-seconds 3 >/dev/null 2>&1; then
        pass "${label}"
    else
        fail "${label}  (topic: ${topic})"
    fi
}

check_topic /fusion/odom  "/fusion/odom publishing (main output)"
check_topic /fusion/pose  "/fusion/pose publishing"
check_topic /diagnostics  "/diagnostics publishing"

if ros2 service call /fusioncore/reset std_srvs/srv/Trigger '{}' >/dev/null 2>&1; then
    pass "/fusioncore/reset service responds"
else
    fail "/fusioncore/reset service not found"
fi

# ── 7. Result ─────────────────────────────────────────────────────────────────
echo ""
if [[ $FAIL -eq 0 ]]; then
    echo -e "  ${GREEN}All checks passed.${NC} FusionCore is working correctly."
    echo ""
    echo "  Next: point it at your robot config:"
    echo "    ros2 launch fusioncore_ros fusioncore.launch.py \\"
    echo "      fusioncore_config:=/path/to/your_robot.yaml"
    echo ""
    exit 0
else
    echo -e "  ${RED}Some checks failed.${NC} Run for diagnostics:"
    echo "    ros2 topic echo /diagnostics --once"
    echo "    ros2 lifecycle get /fusioncore"
    echo ""
    exit 1
fi
