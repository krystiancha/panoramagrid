FROM osrf/ros:melodic-desktop-full


# Install dependencies

RUN set -ex && \
  apt-get update && apt-get install -y \
    gdb \
    wget \
    unzip \
    libglfw3-dev \
    libglm-dev \
#    linux-tools-generic \
    mercurial \
    git \
    vim \
  && rm -rf /var/lib/apt/lists/*

# Install gazebo models
RUN set -ex && \
  mkdir /root/.gazebo && \
  cd /root/.gazebo && \
  git clone https://github.com/osrf/gazebo_models.git models

# Install rcprg_gazebo_utils
RUN ["/bin/bash", "-c", "set -ex && \
  source /opt/ros/melodic/setup.bash && \
  mkdir -p /root/catkin_ws/src && \
  cd /root/catkin_ws && \
  git clone https://github.com/RCPRG-ros-pkg/rcprg_gazebo_utils.git src/rcprg_gazebo_utils && \
  git clone https://github.com/RCPRG-ros-pkg/tiago_sim_integration.git src/tiago_sim_integration && \
  catkin_make \
"]

# Install panoramagrid

COPY . /usr/local/src/panoramagrid

RUN set -ex && \
  cd /usr/local/src/panoramagrid && \
  mkdir build && \
  cd build && \
  cmake -DCMAKE_BUILD_TYPE=Release .. && \
  make install && \
  cd ../ && \
  rm -r build

ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# Install panoramagrid ros package

RUN ["/bin/bash", "-c", "set -ex && \
  source /opt/ros/melodic/setup.bash && \
  cd /root/catkin_ws && \
  cp -r /usr/local/src/panoramagrid/ros src/panoramagrid && \
  catkin_make \
"]

RUN printf "\n# Setup ROS environment\nsource /root/catkin_ws/devel/setup.bash\n" >> /root/.bashrc
