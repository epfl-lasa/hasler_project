FROM osrf/ros:melodic-desktop-full

WORKDIR /root/catkin_ws

SHELL ["/bin/bash", "-c"]


RUN mkdir /root/catkin_ws/src \
    && apt-get update \
    && apt-get install -y \
    apt-utils \
    wget \
    ssh \
    python-catkin-tools

COPY . /root/catkin_ws/src/rokubimini_sdk

RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc \
    && echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

RUN cd /root/catkin_ws/src \
    && git clone https://github.com/ANYbotics/any_node.git \
    && git clone https://github.com/ANYbotics/kindr.git \
    && git clone https://github.com/ANYbotics/kindr_ros.git \
    && git clone https://github.com/ANYbotics/message_logger.git \
    && git clone https://github.com/ANYbotics/soem.git \
    && git clone https://github.com/ANYbotics/signal_logger.git \
    && source /opt/ros/melodic/setup.bash \
    && cd /root/catkin_ws/ \
    && rosdep update \
    && rosdep install --from-path src --ignore-src -y -r --skip-keys="cmake_clang_tools cmake_code_coverage" \
    && apt-get clean \
    && catkin build rokubimini_sdk
