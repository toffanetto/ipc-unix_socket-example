FROM ros:humble-ros-core-jammy

RUN apt update 

# Installing colcon build
RUN apt install ros-dev-tools -y

WORKDIR /ros2_ws

COPY ros2_ws/src src/

# Build package
RUN . "/opt/ros/$ROS_DISTRO/setup.sh" && \
    colcon build --cmake-args=-DCMAKE_BUILD_TYPE=Release

# Cleaning up
WORKDIR /ros2_ws

RUN rm -rf build/ log/ src/ && apt remove ros-dev-tools -y && rm -rf /var/cache/apt/archives /var/lib/apt/lists/*

# Setup entrypoint
COPY --chmod=755 ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]