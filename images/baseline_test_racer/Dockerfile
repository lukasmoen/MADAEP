# # Use Ubuntu 20.04 as base
# FROM ubuntu:20.04

# # Set environment variables
# ENV DEBIAN_FRONTEND=noninteractive \
#     TZ=Europe/Stockholm \
#     ROS_DISTRO=noetic \
#     CATKIN_WS=/home/user/catkin_ws

# # Install basic dependencies
# RUN apt-get update && apt-get install -y \
#         build-essential \
#         cmake \
#         libsparsehash-dev \
#         python3-pip \
#         libboost-dev \
#         libboost-all-dev \
#         net-tools \
#         nano \
#         wget \
#         sudo \
#         unzip \
#         curl \
#         rsync \
#         git \
#         lsb-release \
#         && rm -rf /var/lib/apt/lists/*

# # setup keys
# RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# # Install ROS Noetic
# RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' 

# # Install MRS UAV System dependencies
# RUN apt-get update && apt-get install -y \
#     && apt-get install -q -y --no-install-recommends \
#         ros-noetic-desktop-full \
#         ros-noetic-mavros \
#         ros-noetic-mavros-extras \
#         ros-noetic-octomap-ros \
#         ros-noetic-octomap-msgs \
#         ros-noetic-pcl-ros \
#         ros-noetic-tf2-sensor-msgs \
#         ros-noetic-tf2-geometry-msgs \
#         ros-noetic-move-base-msgs \
#         ros-noetic-joy \
#         ros-noetic-geographic-msgs \
#         ros-noetic-geodesy \
#         ros-noetic-velodyne-gazebo-plugins \
#         ros-noetic-velodyne-simulator \
#         ros-noetic-teleop-twist-keyboard \
#     && rm -rf /var/lib/apt/lists/* 



# # Clone and install nlopt v2.7.1
# RUN git clone -b v2.7.1 https://github.com/stevengj/nlopt.git /nlopt \
#     && cd /nlopt \
#     && mkdir build \
#     && cd build \
#     && cmake .. \
#     && make \
#     && sudo make install

# # Install LKH-3.0.6
# RUN wget http://akira.ruc.dk/~keld/research/LKH-3/LKH-3.0.6.tgz \
#     && tar xvfz LKH-3.0.6.tgz \
#     && cd LKH-3.0.6 \
#     && make \
#     && sudo cp LKH /usr/local/bin

# # Clone and compile RACER package
# RUN mkdir -p ros_ws/src
# RUN cd /ros_ws/src \
#     && git clone https://github.com/SYSU-STAR/RACER.git 
   
# Use Ubuntu 18.04 as the base
FROM ubuntu:18.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Europe/Stockholm \
    ROS_DISTRO=melodic \
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
        gnupg2 \
        && rm -rf /var/lib/apt/lists/*

# Set up ROS Melodic repository keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install ROS Melodic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' 
RUN apt-get update && apt-get install -y \
        ros-melodic-desktop-full \
        ros-melodic-mavros \
        ros-melodic-mavros-extras \
        ros-melodic-octomap-ros \
        ros-melodic-octomap-msgs \
        ros-melodic-pcl-ros \
        ros-melodic-tf2-sensor-msgs \
        ros-melodic-tf2-geometry-msgs \
        ros-melodic-move-base-msgs \
        ros-melodic-joy \
        ros-melodic-geographic-msgs \
        ros-melodic-geodesy \
        ros-melodic-velodyne-gazebo-plugins \
        ros-melodic-velodyne-simulator \
        ros-melodic-teleop-twist-keyboard \
        && rm -rf /var/lib/apt/lists/* 

# Install dependencies for building
RUN apt-get update && apt-get install -y \
        libarmadillo-dev \
        && rm -rf /var/lib/apt/lists/*

# Clone and install nlopt v2.7.1
RUN git clone -b v2.7.1 https://github.com/stevengj/nlopt.git /nlopt \
    && cd /nlopt \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make \
    && sudo make install

# Install LKH-3.0.6
RUN wget http://akira.ruc.dk/~keld/research/LKH-3/LKH-3.0.6.tgz \
    && tar xvfz LKH-3.0.6.tgz \
    && cd LKH-3.0.6 \
    && make \
    && sudo cp LKH /usr/local/bin

# Clone and compile RACER package
RUN mkdir -p /ros_ws/src
RUN cd /ros_ws/src \
    && git clone https://github.com/SYSU-STAR/RACER.git



