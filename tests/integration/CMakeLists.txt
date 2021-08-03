# This software is distributed under the terms of the MIT License.
# Copyright (c) 2021 Zubax Robotics.
# Author: Pavel Kirienko <pavel.kirienko@zubax.com>

cmake_minimum_required(VERSION 3.19)

project(kocherga_integration_test CXX)

set(library_dir "${CMAKE_CURRENT_SOURCE_DIR}/../../kocherga")

# C++ options
set(CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror -pedantic -fstrict-aliasing")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wdouble-promotion -Wswitch-enum -Wfloat-equal -Wundef")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wconversion -Wsign-promo")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wsign-conversion -Wcast-align -Wmissing-declarations")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wtype-limits -Wzero-as-null-pointer-constant -Wnon-virtual-dtor")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Woverloaded-virtual -Wsign-promo -Wold-style-cast")
# Attribute warning is useless in GCC: https://stackoverflow.com/questions/50646334
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-attributes")

include_directories(${library_dir})
include_directories(../util)

file(GLOB bootloader_sources ${CMAKE_CURRENT_SOURCE_DIR}/bootloader/*.cpp)
add_executable(bootloader ${bootloader_sources})

execute_process(COMMAND
        python -m pip --disable-pip-version-check pyuavcan[transport_serial]~=1.2
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
