FROM ubuntu:18.04

LABEL maintainer="ay20-5-1994@hotmail.com"

RUN apt-get update
RUN apt-get install -y \
    cmake \
    git \
    libopenmpi-dev \
    python3-dev \
    python3-pip \
    zlib1g-dev \
    wget \
    libglib2.0-0 \
    libgl1-mesa-glx \
    xcb \
    "^libxcb.*" \
    libx11-xcb-dev \
    libglu1-mesa-dev \
    libxrender-dev \
    libxi6 \
    libdbus-1-3 \
    libfontconfig1 \
    xvfb \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --upgrade pip
RUN pip3 install tensorflow-gpu==1.14.0
RUN pip3 install \
    enum34~=1.1.6 \
    gpflow==1.5.1 \
    stable-baselines

RUN wget http://coppeliarobotics.com/files/V-REP_PRO_EDU_V3_5_0_Linux.tar.gz
RUN tar -xf V-REP_PRO_EDU_V3_5_0_Linux.tar.gz

RUN git clone https://github.com/Alonso94/Vrep_server

RUN echo 'export QT_DEBUG_PLUGINS=1' >> ~/.bashrc
RUN echo 'export PATH=/V-REP_PRO_EDU_V3_5_0_Linux/:$PATH' >> ~/.bashrc#
RUN echo "HELLO!"