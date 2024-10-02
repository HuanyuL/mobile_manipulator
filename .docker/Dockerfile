ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base as base

ARG ROS_DISTRO

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive
ENV TERM xterm-256color


RUN apt-get update && apt-get install -y --no-install-recommends\
    wget \
    curl \
    terminator \
    iputils-ping \
    openssh-server \
    iproute2 \
    python3-pip \
    nano \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-flake8 \
    python3-setuptools \
    python3-vcstool \
    freeglut3-dev \
    lsb-release gnupg \
    libglew-dev \
    libqt5opengl5-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir --upgrade pip\
    argcomplete \
    flake8-blind-except \
    flake8-builtins \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-return \
    flake8-length \
    flake8-todo \
    flake8-quotes \
    black \
    mypy \
    prospector \
    pylint \
    numpy \
    opencv-python \
    pydantic==1.10.14 \
    panda \
    matplotlib \
    rosbags     


# RUN wget https://packages.clearpathrobotics.com/public.key -O - | apt-key add - \
#     && echo "deb https://packages.clearpathrobotics.com/stable/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/clearpath-latest.list

# RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
#     echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null



RUN apt-get update && apt-get install -y --no-install-recommends\
    ignition-fortress \
    ros-$ROS_DISTRO-rqt \
    ros-$ROS_DISTRO-rqt-common-plugins \
    ros-$ROS_DISTRO-rqt-robot-dashboard \
    ros-$ROS_DISTRO-rqt-robot-steering \
    ros-$ROS_DISTRO-rqt-tf-tree \
    ros-$ROS_DISTRO-rqt-tf-tree \
    ros-$ROS_DISTRO-image-transport-plugins \
    ros-$ROS_DISTRO-compressed-image-transport \
    ros-$ROS_DISTRO-clearpath-desktop \
    ros-$ROS_DISTRO-cv-bridge\
    ros-$ROS_DISTRO-rosbag2-py\
    ros-$ROS_DISTRO-usb-cam\
    ros-$ROS_DISTRO-ur\
    ros-$ROS_DISTRO-rviz2\
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp\
    ros-$ROS_DISTRO-ros-testing\
    ros-$ROS_DISTRO-ros2bag\
    ros-$ROS_DISTRO-moveit\
    ros-$ROS_DISTRO-ign-ros2-control\
    ros-$ROS_DISTRO-moveit-servo\
    ros-$ROS_DISTRO-ros-ign-bridge\
    ros-$ROS_DISTRO-ros-ign-gazebo\
    ros-$ROS_DISTRO-ament-clang-format\
    ros-$ROS_DISTRO-moveit-visual-tools\
    && apt-get clean && rm -rf /var/lib/apt/lists/*


RUN mkdir -p /dev_ws/src


WORKDIR /dev_ws/src

# RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && colcon mixin update default

# RUN git clone --branch humble https://github.com/ros-planning/moveit2_tutorials \
#     vcs import < moveit2_tutorials/moveit2_tutorials.repos \
#     cd /dev_ws && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y


# RUN git clone https://github.com/AndrejOrsula/panda_ign_moveit2.git
    # vcs import < panda_ign_moveit2/panda_ign_moveit2.repos

# RUN git clone https://github.com/AndrejOrsula/pymoveit2.git

# RUN git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git -b $ROS_DISTRO 
# RUN git clone https://github.com/moveit/moveit2.git -b $ROS_DISTRO \
#     && cd moveit2 \
#     && for repo in moveit2.repos $(f="moveit2_${ROS_DISTRO}.repos"; test -r $f && echo $f); do vcs import < "$repo"; done \
#     && cd /dev_ws && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

WORKDIR /dev_ws


COPY . /dev_ws/src
RUN apt-get update && rosdep install --ignore-src --from-paths src -y\
    && apt-get clean && rm -rf /var/lib/apt/lists/*


# RUN ["/bin/bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash &&\
#     colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release"]

RUN ["/bin/bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash && \
     colcon build --merge-install --symlink-install --cmake-args '-DCMAKE_BUILD_TYPE=Release'"]


RUN ["/bin/bash", "-c", "cp /dev_ws/src/.docker/setup.bash /dev_ws/setup.bash && chmod 777 /dev_ws/setup.bash"]
RUN ["/bin/bash", "-c", "cp /dev_ws/src/.docker/entrypoint.bash / && chmod 777 /entrypoint.bash"]

ENTRYPOINT ["bash", "/entrypoint.bash"]

CMD ["bash"]