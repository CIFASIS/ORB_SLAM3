FROM ros:melodic-perception

ENV CATKIN_WS=/root/catkin_ws \
    ORBSLAM3_ROOT=/root/catkin_ws/src/ORB_SLAM3/ \
    DEBIAN_FRONTEND=noninteractive

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

COPY . $ORBSLAM3_ROOT
COPY ./scripts $CATKIN_WS

WORKDIR $CATKIN_WS

RUN /bin/bash -c "chmod +x build.sh && chmod +x modify_entrypoint.sh && sync && ./modify_entrypoint.sh && ./build.sh"

#CMD roslaunch ORB_SLAM3 orbslam3.launch
