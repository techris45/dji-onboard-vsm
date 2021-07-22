FROM ubuntu:18.04

RUN apt-get update

ENV BUILD_FOLDER /build

ENV PATH="${PATH}:/pitools/arm-unknown-linux-gnueabihf/"
ENV PATH="${PATH}:/pitools/arm-unknown-linux-gnueabihf/bin"
ENV PATH="${PATH}:/pitools/arm-unknown-linux-gnueabihf/libexec/gcc/arm-unknown-linux-gnueabihf/7.4.1/"
ENV PATH="${PATH}:/usr/local/bin"

ENV PITOOLS_HOME="/pitools"
ENV PI_HOME="/raspberry"

# Add armhf cross-compiler toolchain

WORKDIR /pitools

RUN apt-get update && apt-get install -y wget
RUN wget https://industrial.ugcs.com/dl/deps/arm-unknown-linux-gnueabihf.tar.gz && \
  tar xzvf arm-unknown-linux-gnueabihf.tar.gz && \
  rm arm-unknown-linux-gnueabihf.tar.gz
ADD pi.cmake ${PI_HOME}/

# Install dependencies

RUN apt-get update && apt-get install -y git build-essential cmake libc6-i386 lib32z1 lib32stdc++6
RUN apt-get update && apt-get install -y python
RUN apt-get update && apt-get install -y autoconf automake libtool curl make g++ unzip

# Build and install Protobuf

WORKDIR /build
RUN git clone --branch=3.9.x https://github.com/protocolbuffers/protobuf --depth=1
WORKDIR /build/protobuf
RUN git submodule update --init --recursive
RUN ./autogen.sh && ./configure && make
# RUN make check
RUN make install && ldconfig
RUN git clean -f -X -d

WORKDIR /pitools
RUN apt-get update && apt-get install -y g++-aarch64-linux-gnu
ADD toolchain-arm64.cmake ${PI_HOME}/

