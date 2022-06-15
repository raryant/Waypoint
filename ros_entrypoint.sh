#!/bin/bash
set -e
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/foxy/share/turtlebot3_gazebo/models
export TURTLEBOT3_MODEL=waffle
export DISPLAY=host.docker.internal:0.0
export LIBGL_ALWAYS_INDIRECT=0
# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
exec "$@"
