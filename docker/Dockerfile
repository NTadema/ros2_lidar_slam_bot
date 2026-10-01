FROM ros:jazzy-ros-base

RUN apt update && apt install -y \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    ros-jazzy-ament-cmake \
    ros-jazzy-ament-cmake-test \
    ros-jazzy-ament-cmake-gtest \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-ros-gz \
    git \
    nano \
    vim \
    xauth \
    x11-utils \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros_ws

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

CMD ["bash"]
