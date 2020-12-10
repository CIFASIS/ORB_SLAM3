#!/bin/bash

chmod +x build_orbslam3.sh
sync
./build_orbslam3.sh
chmod +x build_ros.sh
sync
/ros_entrypoint.sh ./build_ros.sh
