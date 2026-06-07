FROM ros:jazzy-ros-base

# System dependencies
# - liblsl-dev: required by pylsl at install time and runtime
# - python3-pip: not included in ros-base
# - python3-numpy, python3-yaml: faster via apt (pre-compiled)
# - ros-jazzy-rmw-cyclonedds-cpp: CycloneDDS RMW for reliable Docker bridge networking
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-numpy \
    python3-yaml \
    ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Python packages — pinned to match the dev environment
RUN pip3 install --break-system-packages \
    pylsl==1.18.2 \
    websockets==16.0

WORKDIR /ros2_ws

# Copy workspace source and Docker config
# (.dockerignore excludes build/, install/, log/, .venv/, encoder_data_publisher/, micro_ros_setup/)
COPY src/ src/
COPY docker/cyclone_bridge.xml docker/cyclone_bridge.xml

# Build — interface package must come first (claude_visualizer depends on it)
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build \
        --packages-select claude_visualizer_interface claude_visualizer \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --event-handlers console_cohesion+

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

EXPOSE 8000 9090

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "claude_visualizer", "bringup.launch.py"]
