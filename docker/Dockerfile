FROM osrf/ros:humble-desktop

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosbridge-suite \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-rviz2 \
    ros-humble-rclcpp-action \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    git \
    cmake \
    build-essential \
    curl \
    nano \
    locales \
    && rm -rf /var/lib/apt/lists/*

# Install acados
WORKDIR /opt/acados
RUN git clone https://github.com/acados/acados.git . && \
    git submodule update --recursive --init && \
    mkdir build && cd build && \
    cmake -DACADOS_WITH_QPOASES=ON -DACADOS_WITH_OSQP=ON .. && \
    make install -j$(nproc)

ENV ACADOS_SOURCE_DIR=/opt/acados
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/acados/lib

# Install acados python interface
RUN pip3 install -e /opt/acados/interfaces/acados_template

# Set locale
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Initialize rosdep
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi
RUN rosdep update

# Set DDS middleware
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Create workspace directory
WORKDIR /ros2_ws

# Setup entrypoint to source ROS 2
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> ~/.bashrc
