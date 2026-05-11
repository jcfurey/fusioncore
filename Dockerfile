FROM ros:jazzy-ros-base AS builder

# Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /fusioncore_ws/src/fusioncore
COPY . .

# Skip Gazebo (pulls in hundreds of MB of GUI deps not needed in a container)
RUN touch /fusioncore_ws/src/fusioncore/fusioncore_gazebo/COLCON_IGNORE

# Resolve and install ROS deps
RUN . /opt/ros/jazzy/setup.sh \
    && cd /fusioncore_ws \
    && rosdep update --rosdistro jazzy \
    && rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy

# Build (core + ROS wrapper only, no Gazebo)
RUN . /opt/ros/jazzy/setup.sh \
    && cd /fusioncore_ws \
    && colcon build \
        --packages-up-to compass_msgs fusioncore_core fusioncore_ros \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log

# ── Runtime stage ─────────────────────────────────────────────────────────────
FROM ros:jazzy-ros-base AS runtime

# Copy only the install tree
COPY --from=builder /fusioncore_ws/install /fusioncore_ws/install

# Copy source for tools/quick_test.sh and benchmarks (demo_quick.py data)
COPY --from=builder /fusioncore_ws/src/fusioncore/tools /fusioncore_ws/src/fusioncore/tools
COPY --from=builder /fusioncore_ws/src/fusioncore/benchmarks /fusioncore_ws/src/fusioncore/benchmarks
COPY --from=builder /fusioncore_ws/src/fusioncore/demo /fusioncore_ws/src/fusioncore/demo

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-numpy \
    python3-matplotlib \
    && rm -rf /var/lib/apt/lists/*

# Source both ROS and workspace on every bash session
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc \
    && echo "source /fusioncore_ws/install/setup.bash" >> /etc/bash.bashrc

ENV AMENT_PREFIX_PATH=/fusioncore_ws/install/fusioncore_ros:/fusioncore_ws/install/fusioncore_core:/fusioncore_ws/install/compass_msgs:/opt/ros/jazzy

WORKDIR /fusioncore_ws/src/fusioncore

SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/bin/bash", "-c", \
    "source /opt/ros/jazzy/setup.bash && source /fusioncore_ws/install/setup.bash && exec \"$@\"", \
    "--"]
CMD ["bash"]
