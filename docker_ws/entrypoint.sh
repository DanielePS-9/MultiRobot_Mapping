#!/bin/bash
 
source /opt/ros/jazzy/setup.bash
[ -f /ros_ws/install/setup.bash ] && source /ros_ws/install/setup.bash
 
export ROS_DOMAIN_ID=0
export TURTLEBOT3_MODEL=waffle
export GZ_SIM_RESOURCE_PATH=/ros_ws/src
export DISPLAY=:0
exec "$@"
 