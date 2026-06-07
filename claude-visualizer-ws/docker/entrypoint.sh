#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}

# Activate CycloneDDS config if provided (used in bridge network / mock mode)
if [ -n "${CYCLONEDDS_URI}" ]; then
    export CYCLONEDDS_URI
fi

exec "$@"
