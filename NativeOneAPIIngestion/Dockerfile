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

ARG EII_VERSION
ARG DOCKER_REGISTRY
ARG ARTIFACTS="/artifacts"

FROM ia_video_common:$EII_VERSION as video_common
FROM ia_openvino_base:$EII_VERSION as openvino_base
FROM ${DOCKER_REGISTRY}openedgeinsights/ia_video_ingestion:$EII_VERSION as video_ingestion
FROM ia_eiibase:$EII_VERSION as builder

LABEL description="DPCPP based ia_native_oneapi_ingestion UDF"

WORKDIR /app

ARG ARTIFACTS
RUN mkdir $ARTIFACTS \
          $ARTIFACTS/lib

ARG CMAKE_INSTALL_PREFIX

ENV DEBIAN_FRONTEND="noninteractive"

RUN apt-get update

COPY --from=video_common ${CMAKE_INSTALL_PREFIX}/include ${CMAKE_INSTALL_PREFIX}/include
COPY --from=video_common ${CMAKE_INSTALL_PREFIX}/lib ${CMAKE_INSTALL_PREFIX}/lib
COPY --from=openvino_base /opt/intel /opt/intel

# Add package repository to install GPGPU drivers
RUN apt update && apt install -y gpg-agent wget && \
    wget -qO - https://repositories.intel.com/graphics/intel-graphics.key | apt-key add -  && \
    echo 'deb [arch=amd64] https://repositories.intel.com/graphics/ubuntu focal main' | tee /etc/apt/sources.list.d/igpu_driver.list && \
    apt update

# Install runtime packages, developed packages and clinfo tool
RUN apt install -y --no-install-recommends \
    intel-opencl-icd \
    intel-level-zero-gpu \
    level-zero \
    intel-media-va-driver-non-free \
    libmfx1 \
    libigc-dev \
    intel-igc-cm \
    libigdfcl-dev \
    libigfxcmrt-dev \
    level-zero-dev && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update
# Add package repository to install intel-basekit and install oneAPI intel-basekit
RUN wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB -O - | apt-key add - && \
    echo "deb https://apt.repos.intel.com/oneapi all main" | tee /etc/apt/sources.list.d/oneAPI.list && \
    apt update && \
    apt install -y intel-basekit

COPY ./dpcpp_blur_udf/ ./dpcpp_blur_udf
RUN cp -r ./dpcpp_blur_udf $ARTIFACTS/dpcpp_blur_udf

RUN /bin/bash -c "source /opt/intel/oneapi/setvars.sh &&\
    source /opt/intel/openvino/bin/setupvars.sh && \
    cd ./dpcpp_blur_udf && \
    rm -rf build && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_INSTALL_INCLUDEDIR=${CMAKE_INSTALL_PREFIX}/include -DCMAKE_INSTALL_PREFIX=$CMAKE_INSTALL_PREFIX -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} .. && \
    make"

RUN cp ./dpcpp_blur_udf/build/libdpcpp_blur_udf.so $ARTIFACTS/lib

FROM video_ingestion as runtime
USER root

HEALTHCHECK NONE

WORKDIR /app

ARG ARTIFACTS
ARG CMAKE_INSTALL_PREFIX

ENV DEBIAN_FRONTEND="noninteractive"

RUN apt-get update && \
    apt-get install -y libpcl-dev

COPY --from=builder $ARTIFACTS/lib ${CMAKE_INSTALL_PREFIX}/lib
COPY --from=builder $ARTIFACTS/dpcpp_blur_udf $ARTIFACTS/dpcpp_blur_udf

RUN apt-get update
# Add package repository to install intel-basekit and install oneAPI intel-basekit
RUN wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB -O - | apt-key add - && \
    echo "deb https://apt.repos.intel.com/oneapi all main" | tee /etc/apt/sources.list.d/oneAPI.list && \
    apt update && \
    apt install -y intel-basekit

COPY --from=builder /opt/intel/oneapi/vpl/2021.6.0/lib/cmake/* ${CMAKE_INSTALL_PREFIX}/lib/cmake/
COPY --from=builder /opt/intel/oneapi/vpl/2021.6.0/lib/pkgconfig/* ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig/
COPY --from=builder /opt/intel/oneapi/vpl/2021.6.0/lib/lib* ${CMAKE_INSTALL_PREFIX}/lib/
COPY --from=builder /opt/intel/oneapi/intelpython/python3.7/envs/2021.4.0/lib/libsvml.so ${CMAKE_INSTALL_PREFIX}/lib/
COPY --from=builder /opt/intel/oneapi/intelpython/python3.7/lib/* ${CMAKE_INSTALL_PREFIX}/lib/

ARG EII_USER_NAME
RUN chown ${EII_USER_NAME}:${EII_USER_NAME} /app /var/tmp
RUN usermod -a -G users ${EII_USER_NAME}