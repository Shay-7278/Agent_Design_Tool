FROM ubuntu:24.04 AS ros_base

ARG DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

RUN apt-get update && apt-get install -y \
        locales curl gnupg software-properties-common \
    && add-apt-repository universe \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
       > /etc/apt/sources.list.d/ros2.list \
    && curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
       | gpg --dearmor -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
       > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y \
        ros-jazzy-desktop-full \
        ros-jazzy-gz-ros2-control \
        ros-jazzy-moveit \
        ros-jazzy-ros2-control \
        ros-jazzy-ros2-controllers \
        ros-jazzy-rmw-cyclonedds-cpp \
        ros-jazzy-ros-gz \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-pip \
        xvfb x11vnc novnc websockify openbox xterm dbus-x11 \
        libgl1-mesa-dri mesa-utils \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init && rosdep update
RUN python3 -m pip install --no-cache-dir --break-system-packages scipy

FROM ros_base AS workspace
WORKDIR /ros2_ws
COPY src/ /ros2_ws/src/
COPY scripts/ /ros2_ws/scripts/
COPY tests/ /ros2_ws/tests/
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y \
      --skip-keys='warehouse_ros_mongo ament_python joint_state_publisher joint_state_publisher_gui' && \
    colcon build"

FROM ros_base AS runtime
WORKDIR /ros2_ws
COPY --from=workspace /ros2_ws /ros2_ws
COPY docker/cyclonedds.xml /cyclonedds.xml
COPY docker/entrypoint.sh /entrypoint.sh

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=/cyclonedds.xml
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV GALLIUM_DRIVER=llvmpipe
ENV QT_X11_NO_MITSHM=1
ENV DISPLAY=:1
ENV GZ_SIM_RESOURCE_PATH=/ros2_ws/install/abb_irb120_description/share:/ros2_ws/install/abb_irb120_gazebo/share/abb_irb120_gazebo/models

RUN sed -i 's/\r$//' /entrypoint.sh && chmod +x /entrypoint.sh
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /etc/bash.bashrc && \
    echo 'source /ros2_ws/install/setup.bash' >> /etc/bash.bashrc
EXPOSE 6080 5900
ENTRYPOINT ["/entrypoint.sh"]
