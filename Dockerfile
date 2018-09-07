FROM osrf/ros:kinetic-desktop-full-jessie
SHELL ["/bin/bash", "-c"]

# Install cartographer
RUN source /ros_entrypoint.sh \
  && apt-get update && apt-get install -y sudo && rm -rf /var/lib/apt/lists/* \
  && sudo apt-get update && sudo apt-get install -y python-wstool python-rosdep ninja-build \
  && mkdir ~/catkin_ws \
  && cd ~/catkin_ws \
  && rm -rf /etc/ros/rosdep/sources.list.d/20-default.list \
  && wstool init src \
  && wstool merge -t src https://raw.githubusercontent.com/googlecartographer/cartographer_ros/master/cartographer_ros.rosinstall \
  && wstool update -t src \
  && src/cartographer/scripts/install_proto3.sh \
  && sudo rosdep init \
  && rosdep update \
  && rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y \
  && catkin_make_isolated --install --use-ninja

# Install tools
RUN sudo apt update \
  && sudo apt install -y vim-gtk \
  && sudo apt install -y qtcreator \
  && sudo apt install -y ros-kinetic-navigation

# Install dependencies
RUN sudo apt update \
  && sudo apt install -y ros-kinetic-controller-manager \
  && sudo apt install -y ros-kinetic-gazebo-ros-control \
  && sudo apt install -y ros-kinetic-diff-drive-controller \
  && sudo apt install -y ros-kinetic-joint-state-controller

# Install rplidar_ros and boat_slam
RUN source /ros_entrypoint.sh \
  && source ~/catkin_ws/install_isolated/setup.bash \
  && mkdir -p ~/ros_ws/src && cd ~/ros_ws/src \
  && git clone https://github.com/robopeak/rplidar_ros.git \
  && git clone https://github.com/jhlenes/boat_slam.git \
  && git clone https://github.com/jhlenes/otter_ros.git \
  && cd .. \
  && rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y \
  && catkin_make

# Setup environment
RUN echo "source /opt/ros/kinetic/setup.bash" >> setup.bash
RUN echo "source ~/catkin_ws/install_isolated/setup.bash" >> setup.bash
RUN echo "source ~/ros_ws/devel/setup.bash" >> setup.bash
RUN chmod +x setup.bash

RUN echo "alias ls='ls --color=auto'" >> ~/.bashrc
RUN echo "alias ll='ls --color=auto -l'" >> ~/.bashrc

COPY office_map_complete.bag /office_map_complete.bag
