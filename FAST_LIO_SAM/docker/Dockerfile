# 基于ROS Noetic的Dockerfile for FAST_LIO_SAM
FROM osrf/ros:noetic-desktop-full

# 设置环境变量
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic

# 设置工作目录
WORKDIR /root

# 安装系统依赖和工具
RUN apt-get update && apt-get install -y \
    git \
    wget \
    curl \
    vim \
    build-essential \
    cmake \
    libpcl-dev \
    libeigen3-dev \
    libgeographic-dev \
    libpython3-dev \
    python3-pip \
    libomp-dev \
    libgoogle-glog-dev \
    && rm -rf /var/lib/apt/lists/*

# 安装ROS依赖包
RUN apt-get update && apt-get install -y \
    ros-noetic-geometry-msgs \
    ros-noetic-nav-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-roscpp \
    ros-noetic-rospy \
    ros-noetic-std-msgs \
    ros-noetic-pcl-ros \
    ros-noetic-tf \
    ros-noetic-message-generation \
    ros-noetic-eigen-conversions \
    ros-noetic-tf2-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

# 安装GTSAM（使用PPA，避免自带Eigen与系统Eigen冲突）
# 如果ros-noetic-gtsam版本满足要求，可以改用：ros-noetic-gtsam
RUN apt-get update && apt-get install -y \
    software-properties-common \
    && apt-get update && apt-get install -y \
    ros-noetic-gtsam \
    && rm -rf /var/lib/apt/lists/*

# 创建catkin工作空间
RUN mkdir -p /root/catkin_ws/src

# 设置工作空间环境
WORKDIR /root/catkin_ws

# 初始化catkin工作空间
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# 安装GeographicLib（从源码编译）
# 注意：将CMake最低版本要求从3.17.0降低到3.16.0以兼容系统CMake版本
RUN cd /tmp && \
    git clone https://github.com/geographiclib/geographiclib.git && \
    cd geographiclib && \
    find . -name "CMakeLists.txt" -type f -exec sed -i 's/cmake_minimum_required\([[:space:]]*\)(VERSION[[:space:]]*3\.17\.[0-9]*)/cmake_minimum_required\1(VERSION 3.16.0)/g' {} \; && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    cd / && \
    rm -rf /tmp/geographiclib

# 安装Livox SDK（livox_ros_driver的依赖）
RUN cd /tmp && \
    git clone https://github.com/Livox-SDK/Livox-SDK.git && \
    cd Livox-SDK && \
    mkdir build2 && \
    cd build2 && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    cd / && \
    rm -rf /tmp/Livox-SDK

# 复制项目文件到工作空间
# 构建方式（推荐）：从项目根目录执行构建脚本
# cd /home/fg/Codes/FAST_LIO_SAM && ./FAST_LIO_SAM/docker/build.sh
# 或手动构建：
# cd /home/fg/Codes/FAST_LIO_SAM && docker build -f FAST_LIO_SAM/docker/Dockerfile -t fast_lio_sam .
COPY FAST_LIO_SAM/ /root/catkin_ws/src/FAST_LIO_SAM/

# 克隆livox_ros_driver到工作空间
RUN cd /root/catkin_ws/src && \
    git clone https://github.com/Livox-SDK/livox_ros_driver.git

# 编译项目（包括fast_lio_sam和livox_ros_driver）
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd /root/catkin_ws && \
    catkin_make"

# 设置环境变量
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

# 设置工作目录
WORKDIR /root/catkin_ws

# 默认命令
CMD ["/bin/bash"]

