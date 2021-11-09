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

# This sample must be compiled and executed within the oneAPI product enviroment
# with the dpcpp compiler to use dpcpp features
set(DPCPP_COMPILER_NAME "dpcpp")
set(CMAKE_CXX_STANDARD 17)
set(TARGET dpcpp_blur_udf)

find_program(
  DPCPP_COMPILER
  NAMES ${DPCPP_COMPILER_NAME}
  HINTS /opt/intel/oneapi)

if(DPCPP_COMPILER)
  set(CMAKE_CXX_COMPILER ${DPCPP_COMPILER})
  message(STATUS "DPCPP compiler found, sample enabled")

  set(CMAKE_CXX_FLAGS "-fPIE -fPIC -O2 -Wall -fstack-protector-strong -D_FORTIFY_SOURCE=2")
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  if (NOT CMAKE_BUILD_TYPE)
      set(CMAKE_BUILD_TYPE RelWithDebInfo)
  endif()
  if(CMAKE_BUILD_TYPE EQUAL "Release")
    message(STATUS "Create Release Build.")
    set(CMAKE_CXX_FLAGS "-O2 ${CMAKE_CXX_FLAGS}")
  else()
    message(STATUS "Create Debug Build.")
  endif()

  project(dpcpp-blur LANGUAGES CXX)
  project(eii-udf-dpcpp-blur VERSION 1.0.0 LANGUAGES C CXX)
  set(EII_COMMON_CMAKE "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake")
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

  find_package(OpenCV REQUIRED)
  find_package(EIIUtils REQUIRED)
  find_package(EIIMsgEnv REQUIRED)
  find_package(UDFLoader REQUIRED)
  find_package(IntelSafeString REQUIRED)

  include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${EIIMsgEnv_INCLUDE}
    ${EIIUtils_INCLUDE}
    ${UDFLoader_INCLUDE}
    ${IntelSafeString_INCLUDE})

  set(CMAKE_INSTALL_LIBDIR "${CMAKE_INSTALL_PREFIX}/lib/udfs")
  add_library(dpcpp_blur_udf SHARED "dpcpp_blur_udf.cpp")

  target_link_libraries(dpcpp_blur_udf
    PUBLIC
        ${UDFLoader_LIBRARIES}
        ${EIIMsgEnv_LIBRARIES}
        ${EIIUtils_LIBRARIES}
        ${OpenCV_LIBRARIES}
    PRIVATE
        ${IntelSafeString_LIBRARIES}
    )
else()
  message(STATUS "DPCPP compiler not found, dpcpp-blur build disabled")
endif()
