FROM amd64/ros:noetic-perception-focal

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=noetic

#
# install ORBSLAM3 ROS package
#

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        software-properties-common \
        git \
        build-essential \
        cmake \
        libeigen3-dev \
        libepoxy-dev \
        libgl1-mesa-dev \
        libglew-dev \
        ros-${ROS_DISTRO}-hector-trajectory-server \
        python3-catkin-tools \
        libopencv-dev && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean

WORKDIR /root

# Pin Pangolin to a release that builds cleanly against Ubuntu 20.04 / OpenEXR 2.x
# (master may fail on half.h deprecated-copy when treating warnings as errors).
RUN git clone --branch v0.8 --depth 1 https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DBUILD_PANGOLIN_PYTHON=OFF && \
    make -j$(nproc) && \
    make install && \
    ldconfig

RUN mkdir -p catkin_ws/src && \
    cd catkin_ws/src && \
    git clone https://github.com/thien94/orb_slam3_ros.git && \
    cd .. && \
    catkin config \
      --extend /opt/ros/noetic && \
    catkin build

RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

#
# install RealSenseSDK / RealSense ROS wrapper
#

RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -sc) main"

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        libssl-dev \
        libudev-dev \
        libusb-1.0-0-dev \
        librealsense2-dev \
        librealsense2-utils \
        ros-${ROS_DISTRO}-realsense2-camera &&  \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean
