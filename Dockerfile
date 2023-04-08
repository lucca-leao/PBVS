# Compilation phase -------------------------------------------
FROM ros:noetic-perception-focal
ENV DEBIAN_FRONTEND noninteractive
RUN
# python3-catkin-tools \
# install ros package
RUN apt-get update && apt-get install -y \
    iputils-ping \
    nano \
    byobu \
    build-essential \
    g++ \
    cmake \
    locales \
    python3-pip \
    git \
    qt5-default \
    qtwebengine5-dev \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-pykdl \
    libeigen3-dev \
    # ros-${ROS_DISTRO}-rviz \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-joint-limits-interface \
    ros-${ROS_DISTRO}-realtime-tools \
    ros-${ROS_DISTRO}-industrial-robot-client \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-joint-trajectory-controller \
    ros-${ROS_DISTRO}-rqt-joint-trajectory-controller \
    ros-${ROS_DISTRO}-joint-state-controller \
    ros-${ROS_DISTRO}-urdf-parser-plugin \
    ros-${ROS_DISTRO}-urdfdom-py \
    ros-${ROS_DISTRO}-kdl-parser-py \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-camera-calibration \
    ros-${ROS_DISTRO}-aruco-ros \
    ros-${ROS_DISTRO}-moveit-visual-tools \
    ros-${ROS_DISTRO}-handeye \
    ros-${ROS_DISTRO}-tf-conversions && \
    rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c "echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc"
RUN /bin/bash -c "echo 'source /catkin_ws/devel/setup.bash' >> /root/.bashrc"

# Booststrap workspace.
ENV CATKIN_DIR=/catkin_ws

COPY ./catkin_ws/src $CATKIN_DIR/src

WORKDIR $CATKIN_DIR

RUN /bin/bash -c '. /opt/ros/noetic/setup.bash && cd /catkin_ws/src/ && catkin_init_workspace && cd /catkin_ws/ && catkin_make' && \
    /bin/bash -c '. /catkin_ws/devel/setup.bash'

RUN /bin/bash -c "pip3 install scipy && pip3 install machinevision-toolbox-python"
