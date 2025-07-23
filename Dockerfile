ARG ROS_DISTRO
FROM ros:${ROS_DISTRO}

ENV COLCON_WS=/root/colcon_ws
ENV COLCON_WS_SRC=/root/colcon_ws/src
ENV SAYE_DESC=/root/RGLGazeboPlugin
ENV PYTHONWARNINGS="ignore:setup.py install is deprecated::setuptools.command.install"
ENV DEBIAN_FRONTEND=noninteractive

ARG GZ_VERSION

RUN apt-get update -qq \
    && apt-get install -y \
        wget git \
    && rm -rf /var/lib/apt/lists/*

RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list

RUN apt-get update -qq \
    && apt-get install -y \
        gz-${GZ_VERSION} \
        build-essential \
        python3-pip \
        ros-${ROS_DISTRO}-rcl-interfaces \
        ros-${ROS_DISTRO}-rclcpp \
        ros-${ROS_DISTRO}-builtin-interfaces \
        ros-${ROS_DISTRO}-ros-gz \
        ros-${ROS_DISTRO}-sdformat-urdf \
        ros-${ROS_DISTRO}-vision-msgs \
        ros-${ROS_DISTRO}-actuator-msgs \
        ros-${ROS_DISTRO}-image-transport \
        ros-${ROS_DISTRO}-nav2* \
        ros-${ROS_DISTRO}-behaviortree-cpp-v3 \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p ${COLCON_WS_SRC}\
    && git clone https://github.com/HangyoCho/ackermann-vehicle-gzsim-ros2.git ${COLCON_WS_SRC}/ackermann-vehicle-gzsim-ros2 \
    && cd ${COLCON_WS}\
    && . /opt/ros/${ROS_DISTRO}/setup.sh\
    && colcon build

Run git clone https://github.com/RobotecAI/RGLGazeboPlugin.git /root/RGLGazeboPlugin \
    && cd /root/RGLGazeboPlugin \
    && mkdir build \ 
    && cd build \
    && cmake .. \
    && make -j24 \ 
    && make install

# Set environment variables for resource and plugin paths
ENV GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:${COLCON_WS_SRC}/ackermann-vehicle-gzsim-ros2
ENV ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${COLCON_WS_SRC}/ackermann-vehicle-gzsim-ros2
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/root/RGLGazeboPlugin/install/RGLServerPlugin/:$GZ_SIM_SYSTEM_PLUGIN_PATH
ENV GZ_GUI_PLUGIN_PATH=/root/RGLGazeboPlugin/install/RGLVisualize:$GZ_GUI_PLUGIN_PATH
ENV RGL_PATTERNS_DIR=/root/RGLGazeboPlugin/lidar_patterns

# Source ROS and workspace on shell startup
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /root/colcon_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash", "-c", "cd /root/colcon_ws && source install/setup.bash && exec bash"]
