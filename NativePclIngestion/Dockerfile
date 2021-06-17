# Copyright (c) 2021 Intel Corporation.

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# Dockerfile for PCL UDF

ARG EII_VERSION
ARG DOCKER_REGISTRY
ARG ARTIFACTS="/artifacts"

FROM ia_video_common:$EII_VERSION as video_common
FROM ia_openvino_base:$EII_VERSION as openvino_base
FROM ${DOCKER_REGISTRY}openedgeinsights/ia_video_ingestion:$EII_VERSION as video_ingestion
FROM ia_eiibase:$EII_VERSION as builder

LABEL description="C++ based PCL UDF"

WORKDIR /app

ARG ARTIFACTS
RUN mkdir $ARTIFACTS \
          $ARTIFACTS/rs2pcl \
          $ARTIFACTS/lib

ARG CMAKE_INSTALL_PREFIX

ENV DEBIAN_FRONTEND="noninteractive"

RUN apt-get update && \
    apt-get install -y libpcl-dev

COPY --from=video_common ${CMAKE_INSTALL_PREFIX}/include ${CMAKE_INSTALL_PREFIX}/include
COPY --from=video_common ${CMAKE_INSTALL_PREFIX}/lib ${CMAKE_INSTALL_PREFIX}/lib
COPY --from=openvino_base /opt/intel /opt/intel

# Copy more than one UDFs here
# Both C++ & Python are allowed in a container.
COPY ./rs2pcl/ ./rs2pcl
RUN cp -r ./rs2pcl $ARTIFACTS/rs2pcl

# Build native UDF samples
RUN /bin/bash -c "source /opt/intel/openvino/bin/setupvars.sh && \
    cd ./rs2pcl && \
    rm -rf build && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_INSTALL_INCLUDEDIR=${CMAKE_INSTALL_PREFIX}/include -DCMAKE_INSTALL_PREFIX=$CMAKE_INSTALL_PREFIX -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} .. && \
    make"

RUN cp ./rs2pcl/build/librs2pcl.so $ARTIFACTS/lib

RUN ls /usr/lib/libOpenNI2.so

FROM video_ingestion as runtime

HEALTHCHECK NONE

WORKDIR /app

ARG ARTIFACTS
ARG CMAKE_INSTALL_PREFIX

# PCL libraries are required in both build stage for compiling udf and
# runtime stage for loading the udf. Since PCL has a lot of dependency
# packages it was feasible to copy the required files from build stage.
# Hence libpcl-dev package is installed again in runtime stage

ENV DEBIAN_FRONTEND="noninteractive"

RUN apt-get update && \
    apt-get install -y libpcl-dev

COPY --from=builder $ARTIFACTS/lib ${CMAKE_INSTALL_PREFIX}/lib
COPY --from=builder $ARTIFACTS/rs2pcl .
