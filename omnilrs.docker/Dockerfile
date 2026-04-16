FROM nvcr.io/nvidia/isaac-sim:2023.1.1

RUN /isaac-sim/python.sh -m pip install --upgrade pip
RUN /isaac-sim/python.sh -m pip install \
  opencv-python \ 
  omegaconf \ 
  hydra-core

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
  ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
  apt-get update && \
  apt-get install -q -y --no-install-recommends tzdata apt-utils && \
  rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y \
  dirmngr \
  gnupg2 \
  curl \
  libglib2.0-0 \
  tmux \
  vim \
  nano \
  && rm -rf /var/lib/apt/lists/*


# setup keys
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# setup environment
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

ENV ROS_DISTRO=humble

# install ros2 packages
RUN apt-get update && apt-get install -y \
  ros-${ROS_DISTRO}-ros-core=0.10.0-1* \
  gdal-bin \
  libgdal-dev \
  ros-${ROS_DISTRO}-vision-msgs \
  ros-${ROS_DISTRO}-tf2-msgs \  
  ros-${ROS_DISTRO}-teleop-twist-keyboard \
  g++ \
  wget \
  && rm -rf /var/lib/apt/lists/*
 
RUN /isaac-sim/python.sh -m pip install \
  skyfield \
  zfpy \
  numba \
  gdown \
  em \
  lark \
  pandas

RUN /isaac-sim/python.sh -m pip install gdal==$(gdal-config --version)

# isaac ros2_bridge config
ENV LD_LIBRARY_PATH=/isaac-sim/exts/omni.isaac.ros2_bridge/${ROS_DISTRO}/lib
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

## For convenience:
RUN echo "alias kk='clear'" >> /root/.bashrc
RUN echo "alias tt='tmux new'" >> /root/.bashrc
RUN echo "alias isaacpy='/isaac-sim/python.sh'" >> /root/.bashrc
RUN echo "alias isaacapp='/isaac-sim/runapp.sh'" >> /root/.bashrc
# Save the bash history immediately on each execution
RUN echo "export PROMPT_COMMAND='history -a' " >> "/root/.bashrc"
RUN echo "export HISTFILE=/commandhistory/.bash_history" >> "/root/.bashrc"

# =============================================================================
# NEW: Fix ROS2 library path for interactive shells (fixes spdlog conflict)
# =============================================================================
RUN echo '# Fix ROS2 spdlog library conflict' >> /root/.bashrc && \
    echo 'export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH' >> /root/.bashrc && \
    echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc

# =============================================================================
# NEW: Add convenient teleop alias
# =============================================================================
RUN echo "alias teleop='ros2 run teleop_twist_keyboard teleop_twist_keyboard'" >> /root/.bashrc

# =============================================================================
# NEW: Copy TurtleBot controller script
# =============================================================================
COPY ./scripts/turtlebot_controller.py /opt/turtlebot/turtlebot_controller.py
COPY ./scripts/jetauto_controller.py /opt/jetauto/jetauto_controller.py

# setup entrypoint
COPY ./entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
