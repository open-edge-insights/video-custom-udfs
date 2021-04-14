
# Copyright (c) 2020 Intel Corporation.

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

# Dockerfile for Native Safety Gear UDF

ARG EII_VERSION
ARG DOCKER_REGISTRY
ARG ARTIFACTS="/artifacts"
FROM ${DOCKER_REGISTRY}ia_video_common:$EII_VERSION as video_common
FROM ${DOCKER_REGISTRY}ia_openvino_base:$EII_VERSION as openvino_base
FROM ${DOCKER_REGISTRY}ia_video_analytics:$EII_VERSION as video_analytics
FROM ${DOCKER_REGISTRY}ia_eiibase:$EII_VERSION as builder
LABEL description="C++ based Safety Gear UDF Image"

WORKDIR /app

ARG ARTIFACTS
RUN mkdir $ARTIFACTS \
          $ARTIFACTS/safety_gear_demo \
          $ARTIFACTS/lib
ARG CMAKE_INSTALL_PREFIX
COPY --from=video_common ${CMAKE_INSTALL_PREFIX}/include ${CMAKE_INSTALL_PREFIX}/include
COPY --from=video_common ${CMAKE_INSTALL_PREFIX}/lib ${CMAKE_INSTALL_PREFIX}/lib
COPY --from=openvino_base /opt/intel /opt/intel

# Copy more than one UDFs here
# Both C++ & Python are allowed in a container.
COPY ./safety_gear_demo/ ./safety_gear_demo
RUN cp -r ./safety_gear_demo $ARTIFACTS/safety_gear_demo

# Build native UDF samples
RUN /bin/bash -c "source /opt/intel/openvino/bin/setupvars.sh && \
    cd ./safety_gear_demo && \
    rm -rf build && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_INSTALL_INCLUDEDIR=${CMAKE_INSTALL_PREFIX}/include -DCMAKE_INSTALL_PREFIX=$CMAKE_INSTALL_PREFIX -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} .. && \
    make"

RUN cp ./safety_gear_demo/build/libsafety_gear_demo.so $ARTIFACTS/lib

FROM video_analytics as runtime
HEALTHCHECK NONE
WORKDIR /app
ARG ARTIFACTS
ARG CMAKE_INSTALL_PREFIX
COPY --from=builder $ARTIFACTS/lib ${CMAKE_INSTALL_PREFIX}/lib
COPY --from=builder $ARTIFACTS/safety_gear_demo .
