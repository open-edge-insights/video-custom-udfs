
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

ARG EIS_VERSION
ARG DOCKER_REGISTRY
FROM ${DOCKER_REGISTRY}ia_video_analytics:$EIS_VERSION
LABEL description="C++ based Safety Gear UDF Image"

WORKDIR ${GO_WORK_DIR}

# Copy more than one UDFs here
# Both C++ & Python are allowed in a container.
COPY ./safety_gear_demo/ ./safety_gear_demo

# Build native UDF samples
RUN /bin/bash -c "source /opt/intel/openvino/bin/setupvars.sh && \
    cd ./safety_gear_demo && \
    rm -rf build && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} .. && \
    make && \
    make install"
