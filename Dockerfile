# Base image with GUI and Gazebo support
FROM osrf/ros:humble-desktop
# Change the default shell to Bash
SHELL ["/bin/bash","-c"]

# Install Gazebo + ROS2 deps (added git for cloning)
RUN apt-get update && apt-get install -y \
  git \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher \
  ros-humble-robot-localization \
  ros-humble-xacro \
  ros-humble-tf2-ros \
  ros-humble-tf2-tools \
  ros-humble-rmw-cyclonedds-cpp \
  python3-colcon-common-extensions \
  libx11-dev \
  libxtst-dev \
  libxrender-dev \
  libxext-dev \
  libxkbcommon-dev \
  libwayland-dev \
  libegl1-mesa-dev \
  libgl1-mesa-glx \
  libglib2.0-dev \
  libqt5gui5 \
  libqt5x11extras5-dev \
  qtwayland5 \
  x11-utils \
  xterm \
  mesa-utils \
  ros-humble-controller-manager \
  ros-humble-gazebo-ros2-control-demos \
  ros-humble-rviz2 \
  ros-humble-gazebo-ros \
  ros-humble-gazebo-plugins \
  ros-humble-teleop-twist-keyboard \
  python3-rosdep \
  python3-rosinstall \
  python3-rosinstall-generator \
  && rm -rf /var/lib/apt/lists/*

# Qt / X11 tweaks for headless Gazebo
ENV QT_XCB_GLUE_ALWAYS_IN_USE=1
ENV QT_X11_NO_MITSHM=1
ENV DISPLAY=:1
ENV GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}

# Workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# --- Clone your repo and keep only the two packages under /ros2_ws/src ---
# Resulting tree:
#   /ros2_ws/src/fastbot
#   /ros2_ws/src/fastbot_waypoints
ARG REPO_URL="https://github.com/mathrosas/ros2_testing.git"
ARG REPO_BRANCH="main"
RUN source /opt/ros/humble/setup.bash \
 && git clone --depth 1 --branch "${REPO_BRANCH}" "${REPO_URL}" /tmp/ros2_testing \
 && mv /tmp/ros2_testing/fastbot /ros2_ws/src/fastbot \
 && mv /tmp/ros2_testing/fastbot_waypoints /ros2_ws/src/fastbot_waypoints \
 && rm -rf /tmp/ros2_testing

COPY entrypoint.sh /ros2_ws/entrypoint.sh
RUN chmod +x /ros2_ws/entrypoint.sh

# Build the Colcon workspace and ensure it's sourced
RUN source /opt/ros/humble/setup.bash \
 && colcon build --symlink-install \
 && echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# ROS 2 middleware defaults
ENV ROS_DOMAIN_ID=1
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Default workdir
WORKDIR /ros2_ws

# Use your entrypoint (it should launch sim, server, and tests)
# CMD is handled by entrypoint.sh; keep commented if you run via `docker run ...`
# CMD ["bash"]

ENTRYPOINT ["/ros2_ws/entrypoint.sh"]
