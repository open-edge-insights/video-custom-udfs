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

# Dockerfile for python based Safety Gear Analytics container

ARG EII_VERSION
ARG DOCKER_REGISTRY
FROM ${DOCKER_REGISTRY}openedgeinsights/ia_video_analytics:$EII_VERSION
LABEL description="Python based safety gear analytics image"
HEALTHCHECK NONE

WORKDIR /app

# Added /app to python path as copying the UDF code block under /app
# else user can add the path to which it copies its udf code.
ENV PYTHONPATH ${PYTHONPATH}:/app

# User can mention about all UDF directories here, both py and C++.
COPY ./safety_gear ./safety_gear
