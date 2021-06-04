# Copyright (c) 2021 Intel Corporation.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to
# deal in the Software without restriction, including without limitation the
# rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
# sell copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM,OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
# IN THE SOFTWARE.

cmake_minimum_required(VERSION 3.12)
project(eii-udf-rs2pcl VERSION 1.0.0 LANGUAGES C CXX)

# Globals
set(EII_COMMON_CMAKE "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake")

# Set C++ standard to C11
set(CMAKE_CXX_STANDARD 11)

# Set CFLAGS
set(CMAKE_CXX_FLAGS "-fPIE -fPIC -O2 -Wall -fstack-protector-strong -D_FORTIFY_SOURCE=2")

# Set LDFLAGS
set(CMAKE_SHARED_LINKER_FLAGS "-z noexecstack -z relro -z now")

# Add cmake module path
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" "${EII_COMMON_CMAKE}")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(EXISTS "${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json")
    EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json
        ${CMAKE_CURRENT_SOURCE_DIR}/compile_commands.json
    )
endif()

# Finding libraries
find_package(EIIUtils REQUIRED)
find_package(EIIMsgEnv REQUIRED)
find_package(UDFLoader REQUIRED)
find_package(OpenCV REQUIRED)
find_package(IntelSafeString REQUIRED)
find_package(InferenceEngine 2.1 REQUIRED)
find_package(realsense2 REQUIRED)
find_package(PCL REQUIRED)
if (PCL_FOUND)
    add_definitions(${PCL_DEFINITIONS})
endif()

# Set include directories
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${EIIMsgEnv_INCLUDE}
    ${EIIUtils_INCLUDE}
    ${UDFLoader_INCLUDE}
    ${IntelSafeString_INCLUDE})

set(CMAKE_INSTALL_LIBDIR "${CMAKE_INSTALL_PREFIX}/lib/udfs")

add_library(rs2pcl SHARED "rs2pcl.cpp")

target_include_directories(rs2pcl
    PRIVATE
        rs2pcl
        ${PCL_INCLUDE_DIRS}
        ${EIIMsgEnv_INCLUDE}
        ${EIIUtils_INCLUDE}
        ${InferenceEngine_INCLUDE_DIRS}
        ${InferenceEngine_INCLUDE_DIRS}/../src/extension
        ${IE_EXTERNAL_DIR}
        ${IntelSafeString_INCLUDE}
    )

target_link_libraries(rs2pcl
    PUBLIC
        ${UDFLoader_LIBRARIES}
        ${CPU_LIB}
        ${EIIMsgEnv_LIBRARIES}
        ${EIIUtils_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${InferenceEngine_LIBRARIES}
        ${PCL_LIBRARIES}
        ${realsense2_LIBRARY}
    PRIVATE
        ${IntelSafeString_LIBRARIES}
    )

install(
    TARGETS rs2pcl
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

