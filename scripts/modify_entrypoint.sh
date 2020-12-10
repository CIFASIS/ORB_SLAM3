#!/bin/bash
sed -i '/exec "$@"/i export ROS_PACKAGE_PATH="/opt/ros/melodic/share:${CATKIN_WS}/src/ORB_SLAM3/Examples/ROS"' /ros_entrypoint.sh