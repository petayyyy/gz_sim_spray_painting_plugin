#!/bin/bash
# Entrypoint: sources ROS 2 + colcon install, then runs the given command.
set -e

# Source ROS 2 Humble
. /opt/ros/humble/setup.bash

# Source colcon install if available
if [ -f /ws/install/setup.bash ]; then
    . /ws/install/setup.bash
fi

# Export plugin path
export GZ_SIM_SYSTEM_PLUGIN_PATH=/ws/install/gz_sim_spray_painting_plugin/lib/gz_sim_spray_painting_plugin:${GZ_SIM_SYSTEM_PLUGIN_PATH}

exec "$@"
