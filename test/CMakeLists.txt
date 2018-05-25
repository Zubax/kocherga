#
# The MIT License (MIT)
#
# Copyright (c) 2018 Zubax Robotics
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of
# this software and associated documentation files (the "Software"), to deal in
# the Software without restriction, including without limitation the rights to
# use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
# the Software, and to permit persons to whom the Software is furnished to do so,
# subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
# FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
# COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
# IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#
# Author: Pavel Kirienko <pavel.kirienko@zubax.com>
#

cmake_minimum_required(VERSION 3.1)

project(kocherga_test)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
        "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
        FORCE)
endif()

include_directories(..
                    .
                    senoval
                    popcop/c++)

# Using 32-bit mode for compatibility with libcanard
# Enabling all warnings; make them fatal for C++ and non-fatal for C (because C is only needed for dependencies)
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -m32 -std=c11   -Wall -Wextra         -pedantic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m32 -std=c++17 -Wall -Wextra -Werror -pedantic")

# C++ warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wdouble-promotion -Wswitch-enum -Wfloat-equal -Wundef")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wconversion -Wsign-promo")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wsign-conversion -Wcast-align -Wmissing-declarations")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wtype-limits -Wzero-as-null-pointer-constant -Wnon-virtual-dtor")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Woverloaded-virtual -Wsign-promo -Wold-style-cast")

# Adding library headers explicitly to the project in order to let the IDE handle things properly.
file(GLOB KOCHERGA_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "../kocherga*.hpp")

# Adding all tests
file(GLOB KOCHERGA_TEST_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.cpp")
file(GLOB KOCHERGA_TEST_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.hpp")

# Main test executable
add_executable(kocherga_test
               ${KOCHERGA_HEADERS}
               ${KOCHERGA_TEST_SOURCES}
               ${KOCHERGA_TEST_HEADERS})

# Location of the test source directory is needed for some tests
add_definitions(-DKOCHERGA_TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

# Libcanard
include_directories(SYSTEM                          # Using system include mode to squelch C++ compatibility warnings
                    libcanard)
add_library(canard
            libcanard/canard.c
            libcanard/drivers/socketcan/socketcan.c)

# Dependencies
target_link_libraries(kocherga_test
                      canard
                      pthread)
