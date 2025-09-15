ARG HTTP_PROXY=""
ARG HTTPS_PROXY=""
ARG NO_PROXY=""

FROM osrf/ros:noetic-desktop-full@sha256:7dbfb9576d8e6d226c31e06129a82aaab8702695f38eca2116918cb9b9308797

# set up environment variables
# for visualization
ENV QT_X11_NO_MITSHM=1
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all
RUN echo "source /opt/ros/noetic/setup.bash" >> /etc/bash.bashrc

# Set environment variables ONLY if the build args are not empty
# This is a shell "or" (||) operation: if $HTTP_PROXY is empty, set http_proxy to empty.
RUN if [ -n "$HTTP_PROXY" ]; then export http_proxy=$HTTP_PROXY; fi && \
    if [ -n "$HTTPS_PROXY" ]; then export https_proxy=$HTTPS_PROXY; fi && \
    if [ -n "$NO_PROXY" ]; then export no_proxy=$NO_PROXY; fi && \
    apt-get update && \
    apt-get install -y \
    git \
    vim \
    wget \
    ros-noetic-rosfmt \
    ros-noetic-ackermann-msgs \
    ros-noetic-pybind11-catkin \
    ros-noetic-serial \
    ros-noetic-rqt-multiplot \
    ros-noetic-vrpn-client-ros \
    ros-noetic-ompl \
    ros-noetic-plotjuggler-ros \
    libasio-dev \
    libompl-dev \
    libeigen3-dev \
    libglfw3-dev \
    libglew-dev \
    libdw-dev \
    python3-pip \
    python3-tk \
    xarclock mesa-utils \
    libx11-dev libxcursor-dev libxrandr-dev libxinerama-dev libxi-dev \
    vulkan-utils mesa-vulkan-drivers pigz libegl1

WORKDIR /home/topay
COPY . ./

# install osqp
WORKDIR /home
RUN wget https://github.com/osqp/osqp/releases/download/v0.6.2/complete_sources.tar.gz
RUN tar -xzf complete_sources.tar.gz
RUN rm complete_sources.tar.gz
WORKDIR /home/osqp/build
RUN cmake .. && make -j && make install

# install osqp-eigen
WORKDIR /home
RUN wget https://github.com/robotology/osqp-eigen/archive/refs/tags/v0.8.0.tar.gz
RUN tar -xzf v0.8.0.tar.gz
RUN rm v0.8.0.tar.gz
WORKDIR /home/osqp-eigen-0.8.0/build
RUN cmake .. && make -j && make install

# install casadi
WORKDIR /home
RUN wget https://github.com/casadi/casadi/archive/main.tar.gz
RUN tar -xzf main.tar.gz
RUN rm main.tar.gz
RUN sed -i -e 's/WITH_LAPACK_DEF\sOFF/WITH_LAPACK_DEF ON/g' /home/casadi-main/CMakeLists.txt
RUN sed -i -e 's/WITH_QPOASES_DEF\sOFF/WITH_QPOASES_DEF ON/g' /home/casadi-main/CMakeLists.txt
WORKDIR /home/casadi-main/build
RUN cmake .. && make -j && make install

# install livox for rog map
WORKDIR /home
RUN wget https://github.com/Livox-SDK/Livox-SDK2/archive/master.tar.gz
RUN tar -xzf master.tar.gz
RUN rm master.tar.gz
WORKDIR /home/Livox-SDK2-master/build
RUN cmake .. && make -j && make install

WORKDIR /home/topay
