# This software is distributed under the terms of the MIT License.
# Copyright (c) 2020 Zubax Robotics.
# Author: Pavel Kirienko <pavel.kirienko@zubax.com>

cmake_minimum_required(VERSION 3.19)
enable_testing()

project(kocherga CXX)

set(CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
list(APPEND CMAKE_CTEST_ARGUMENTS "--output-on-failure")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
        "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
        FORCE)
endif()

set(LIBRARY_DIR "${CMAKE_SOURCE_DIR}/../kocherga")

# Use -DNO_STATIC_ANALYSIS=1 to suppress static analysis if the tools are not available.
if (NOT NO_STATIC_ANALYSIS)
    find_program(clang_tidy NAMES clang-tidy)
    if (NOT clang_tidy)
        message(FATAL_ERROR "Could not locate clang-tidy")
    endif ()
    message(STATUS "Using clang-tidy: ${clang_tidy}")
    set(CMAKE_CXX_CLANG_TIDY ${clang_tidy})
endif ()

# clang-format
find_program(clang_format NAMES clang-format)
if (NOT clang_format)
    message(STATUS "Could not locate clang-format")
else ()
    file(GLOB_RECURSE FORMAT_FILES
            ${LIBRARY_DIR}/*.hpp
            ${CMAKE_SOURCE_DIR}/unit/*.[ch]pp)
    message(STATUS "Using clang-format: ${clang_format}; files: ${FORMAT_FILES}")
    add_custom_target(format COMMAND ${clang_format} -i -fallback-style=none -style=file --verbose ${FORMAT_FILES})
endif ()

add_subdirectory(unit)
add_subdirectory(integration)
