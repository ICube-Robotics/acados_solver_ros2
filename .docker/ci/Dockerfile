ARG ROS_DISTRO="humble"
FROM ros:${ROS_DISTRO}

# COPY . /ros2_dev/src/acados_controllers_ros2
RUN apt update && apt upgrade -y
RUN apt install -y python3-pip git tree
# RUN apt update && \
#     cd ros2_dev/src/acados_controllers_ros2 && \
#     git submodule update --init --recursive \
