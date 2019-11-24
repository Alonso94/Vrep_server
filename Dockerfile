FROM ubuntu:18.04

LABEL maintainer="ay20-5-1994@hotmail.com"

RUN apt update && apt install -y \
    cmake \
    git \
    python3-dev \
    python3-pip \
    zlib1g-dev \
    xterm \
    ca-certificates \
    libopenmpi-dev \
    curl \
    wget \
    libglib2.0-0 \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    xcb \
    "^libxcb.*" \
    libx11-xcb-dev \
    libglu1-mesa-dev \
    libxrender-dev \
    libxi6 \
    libdbus-1-3 \
    libfontconfig1 \
    xvfb \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

RUN add-apt-repository ppa:ubuntu-x-swat/updates
RUN apt-get dist-upgrade -y
RUN apt install -y mesa-utils

RUN pip3 install --upgrade pip
RUN pip3 install tensorflow-gpu==1.14.0
RUN pip3 install \
    enum34~=1.1.6 \
    gpflow==1.5.1 \
    stable-baselines \
    jupyter

RUN curl -O -J -L https://sourceforge.net/projects/virtualgl/files/2.6.3/virtualgl_2.6.3_amd64.deb/download
RUN dpkg -i virtualgl_2.6.3_amd64.deb
RUN apt install -f
RUN /opt/VirtualGL/bin/vglserver_config -config +s +f -t
RUN rm virtualgl_2.6.3_amd64.deb

# nvidia-docker links
LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:/opt/VirtualGL/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

RUN wget http://coppeliarobotics.com/files/V-REP_PRO_EDU_V3_5_0_Linux.tar.gz
RUN tar -xf V-REP_PRO_EDU_V3_5_0_Linux.tar.gz
RUN rm V-REP_PRO_EDU_V3_5_0_Linux.tar.gz

RUN git clone https://github.com/Alonso94/Vrep_server
RUN echo 'export QT_DEBUG_PLUGINS=1' >> ~/.bashrc
RUN echo 'export PATH=/V-REP_PRO_EDU_V3_5_0_Linux/:$PATH' >> ~/.bashrc#

EXPOSE 22

COPY entrypoint.sh /
RUN ["chmod","+x","/entrypoint.sh"]
ENTRYPOINT ["sh","/entrypoint.sh"]

CMD ["/bin/bash"]
