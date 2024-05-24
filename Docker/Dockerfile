FROM osrf/ros:noetic-desktop-full
# Set the environment variable to non-interactive, this is to avoid the tzdata configuration during the installation of the packages
ENV DEBIAN_FRONTEND=noninteractive

RUN sed -i 's#http://archive.ubuntu.com/#http://mirrors.tuna.tsinghua.edu.cn/#' /etc/apt/sources.list

# System layer installation
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y wget && \
    apt-get install -y iputils-ping && \
    apt-get -y install build-essential && \
    apt install -y cmake && \
    apt install -y tree && \
    apt install -y gdb && \
    apt-get install -y git && \
    apt-get install -y python3-pip && \
    apt-get install -y xauth x11-apps # This is for GUI testing

# Dependencies layer # PoseEstimation
RUN apt-get update -y && \
    apt-get upgrade -y && \
    python3 -m pip install -i https://pypi.tuna.tsinghua.edu.cn/simple --upgrade pip && \
    pip3 config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple && \
    pip3 install opencv-python && \
    pip3 install numpy  &&\
    pip3 install transforms3d && \
    pip3 install scipy

# Use rosdep to install all dependencies according to the package.xml files in all src folders
# And compile the workspace
RUN mkdir -p /catkin_ws/src && \
    cd /catkin_ws && \
    rosdep install --from-paths src --ignore-src -r -y && \
    /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make'

# Automatically source
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

RUN echo "ALL Done"
