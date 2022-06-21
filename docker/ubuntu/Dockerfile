FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y apt-transport-https
RUN apt-get update && apt-get install build-essential software-properties-common -y \
    sudo \
    cmake \
    libboost-all-dev \
    graphviz \
    doxygen \
    libgl1-mesa-dev \
    libglew-dev \
    git-lfs \
    libprotobuf-dev \
    protobuf-compiler \
    libhdf5-dev \
    libfreetype6-dev \
    libtbb-dev \
    openexr \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libavutil-dev \
    clang \
    sed \
    gawk \
    clang-10 \
    zlib1g \
    clang-format-10 \
    ninja-build \
    wget \
    ffmpeg \
    python \
    python3 \
    python3-pip \
    libeigen3-dev \
    libsuitesparse-dev \
    git \
    && pip3 install --upgrade pip \
    && pip3 install cmake-format \
    && pip3 install conan \
    && pip3 install pydot \
    && pip3 install pydotplus \
    && pip3 install networkx \
    && pip3 install pyyaml \
    && pip3 install sympy \
    && pip3 install scipy \
    && pip3 install opencv-contrib-python \
    && pip3 install yapf==0.31.0 \
    && pip3 install protobuf \
    && pip3 install typing_extensions \
    && pip3 install dataclasses \
    && pip3 install laspy \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

RUN add-apt-repository ppa:ubuntu-toolchain-r/test -y \
    && apt-get update \
    && apt -y install g++-10

RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-10 100
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 100
RUN update-alternatives --install /usr/bin/gcov gcov /usr/bin/gcov-10 100
RUN update-alternatives --install /usr/bin/clang++ clang++ /usr/lib/llvm-10/bin/clang++ 100
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 100

RUN wget https://cmake.org/files/v3.17/cmake-3.17.0-Linux-x86_64.tar.gz \
    && tar xvf cmake-3.17.0-Linux-x86_64.tar.gz \
    && cd cmake-3.17.0-Linux-x86_64 \
    && cp -r bin /usr/ \
    && cp -r share /usr/ \
    && cp -r doc /usr/share/ \
    && cp -r man /usr/share/ \
    && cd .. \
    && rm -rf cmake*

RUN wget http://ftp.de.debian.org/debian/pool/main/l/lcov/lcov_1.15.orig.tar.xz \
    && tar xvf lcov_1.15.orig.tar.xz \
    && cd lcov-1.15 \
    && make install \
    && cd .. \
    && rm -rf lcov*


RUN cd /usr/include && ln -sf eigen3/Eigen Eigen &&  ln -sf eigen3/unsupported unsupported


RUN mkdir /app
WORKDIR /app

CMD ["/bin/bash"]
