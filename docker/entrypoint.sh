#!/bin/bash
set -e

source /opt/ros/$ROS_DISTRO/setup.sh
source /home/$USER/ros_ws/install/setup.bash

exec "$@"