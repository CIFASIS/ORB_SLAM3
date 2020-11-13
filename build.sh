#!/bin/bash

CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

cd $CURRENT_DIR

chmod +x build_orbslam3.sh
./build_orbslam3.sh
chmod +x build_ros.sh
/ros_entrypoint.sh ./build_ros.sh