# ROS2 humble Base
FROM ros:humble

RUN apt-get update && apt-get upgrade -y

# https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/docker-specialized.html
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

RUN apt-get install --no-install-recommends -y \
    software-properties-common \
    vim \
    python3-pip \
    python3-tk

# Added updated mesa drivers for integration with cpu - https://github.com/ros2/rviz/issues/948#issuecomment-1428979499
RUN add-apt-repository ppa:kisak/kisak-mesa && \
    apt-get update && apt-get upgrade -y

# Cyclone DDS
RUN apt-get install --no-install-recommends -y \
    ros-$ROS_DISTRO-cyclonedds \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp

# Install packages via pip with exact DARP-compatible versions
RUN pip3 install --no-cache-dir \
    numba==0.55.0 \
    scikit-learn==1.1.3 \
    pygame==2.1.0

# Use cyclone DDS by default
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Source by default
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

ENV WORKSPACE_PATH=/root/workspace

COPY workspace/ $WORKSPACE_PATH/src/

RUN rosdep update && cd $WORKSPACE_PATH && \
    rosdep install --from-paths src -y --ignore-src --skip-keys="ament_python"

COPY scripts/setup/ /root/scripts/setup
RUN /root/scripts/setup/workspace.sh