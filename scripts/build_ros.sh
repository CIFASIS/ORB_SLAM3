echo "Building ROS nodes"

cd $ORBSLAM3_ROOT
cd Examples/ROS/ORB_SLAM3
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release -DLOG_TRACKING_TIMESTAMPS=ON
make -j
