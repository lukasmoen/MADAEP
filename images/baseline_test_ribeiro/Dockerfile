# Use Ubuntu 20.04 as base
FROM ubuntu:20.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Europe/Stockholm \
    ROS_DISTRO=noetic \
    CATKIN_WS=/home/user/catkin_ws

# Install basic dependencies
RUN apt-get update && apt-get install -y \
        build-essential \
        cmake \
        libsparsehash-dev \
        python3-pip \
        libboost-dev \
        libboost-all-dev \
        net-tools \
        nano \
        wget \
        sudo \
        unzip \
        curl \
        rsync \
        git \
        lsb-release \
        && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install ROS Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' 

# Install MRS UAV System dependencies
RUN apt-get update && apt-get install -y \
    && apt-get install -q -y --no-install-recommends \
        ros-noetic-desktop-full \
        ros-noetic-mavros \
        ros-noetic-mavros-extras \
        ros-noetic-octomap-ros \
        ros-noetic-octomap-msgs \
        ros-noetic-pcl-ros \
        ros-noetic-tf2-sensor-msgs \
        ros-noetic-tf2-geometry-msgs \
        ros-noetic-move-base-msgs \
        ros-noetic-joy \
        ros-noetic-geographic-msgs \
        ros-noetic-geodesy \
        ros-noetic-velodyne-gazebo-plugins \
        ros-noetic-velodyne-simulator \
        ros-noetic-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/* 

    # Configure ROS environment
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc \
&& /bin/bash -c "source ~/.bashrc"

# Add MRS stable PPA
RUN curl https://ctu-mrs.github.io/ppa-stable/add_ros_ppa.sh | bash

RUN curl https://ctu-mrs.github.io/ppa-stable/add_ppa.sh | bash

# Install MRS UAV System
RUN apt-get update && apt-get install -y ros-noetic-mrs-uav-system-full

# Ribeiro planner
RUN apt-get update && apt-get install -y python3-catkin-tools
    
RUN mkdir -p ~/exploration_ws/src
RUN cd ~/exploration_ws/src
RUN git clone https://github.com/andremribeiro/thesis.git
RUN git clone https://github.com/andremribeiro/mean_shift_clustering.git
RUN cd 
# RUN source devel/setup.bash

RUN mkdir -p ~/mapping_ws/src
RUN cd ~/mapping_ws/src
RUN git clone https://github.com/andremribeiro/octomap_mapping.git
RUN git clone https://github.com/andremribeiro/mrs_octomap_planner.git
RUN cd ..
# RUN source devel/setup.bash

# cd ~/exploration_ws/src/thesis/tmux
# ./start.sh

RUN sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
RUN sudo apt-get install --reinstall gazebo11 libgazebo11 libgazebo11-dev
RUN sudo apt-get install ros-noetic-octomap-rviz-plugins

RUN mkdir -p /usr/share/gazebo-11/media/materials/scripts/ && \
    touch /usr/share/gazebo-11/media/materials/scripts/Gazebo.material

ENV GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11




