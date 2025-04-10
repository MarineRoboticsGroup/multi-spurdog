FROM ros:noetic-ros-core

RUN apt-get update \
    && apt-get upgrade -y \
    && apt install -y \
        python3-pip \
        git \
        ros-noetic-dynamic-reconfigure \
        socat
RUN pip install pytest pytest-cov