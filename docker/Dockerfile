FROM ros:jazzy-ros-base

RUN apt update && apt install -y \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    ros-jazzy-ament-cmake \
    ros-jazzy-ament-cmake-test \
    git \
    nano \
    vim \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros_ws

CMD ["bash"]
