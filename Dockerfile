# Define parent image
FROM ros:melodic-perception

# Set environment and working directory
ENV CATKIN_WS=/root/catkin_ws
WORKDIR $CATKIN_WS
ENV DEBIAN_FRONTEND noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends apt-utils && \
    apt-get install -y \
    python-pip \
    libpython2.7-dev \
    libglew-dev && \
    rm -rf /var/lib/apt/lists/* && \
    mkdir src && cd src && \
    git clone https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    git checkout 86eb4975fc4fc8b5d92148c2e370045ae9bf9f5d && \
    mkdir build && \
    cd build && \
    cmake .. && \
    cmake --build .

# Copy files
COPY ./ ./src/ORB_SLAM3

# Build SLAM system
RUN cd ./src/ORB_SLAM3 && \
    chmod +x build.sh && \
    ./build.sh && \
    echo "export ROS_PACKAGE_PATH=/opt/ros/melodic/share:${CATKIN_WS}/src/ORB_SLAM3/Examples/ROS" >> ~/.bashrc && \
    chmod +x build_ros.sh && \
    /bin/bash -c "source /opt/ros/melodic/setup.bash && export ROS_PACKAGE_PATH=/opt/ros/melodic/share:${CATKIN_WS}/src/ORB_SLAM3/Examples/ROS && ./build_ros.sh"

# Define CMD
CMD /bin/bash -c "export ROS_PACKAGE_PATH=/opt/ros/melodic/share:${CATKIN_WS}/src/ORB_SLAM3/Examples/ROS && roslaunch ORB_SLAM3 orbslam3.launch"
