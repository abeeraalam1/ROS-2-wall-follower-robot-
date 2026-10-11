#!/bin/bash
# Source ROS 2 and the workspace, then run whatever command was given.
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${WS}/install/setup.bash"
exec "$@"
