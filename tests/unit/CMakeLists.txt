# This software is distributed under the terms of the MIT License.
# Copyright (c) 2020 Zubax Robotics.
# Author: Pavel Kirienko <pavel.kirienko@zubax.com>

cmake_minimum_required(VERSION 3.19)
enable_testing()

project(kocherga_unit_test CXX)

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
include_directories(SYSTEM ../3rd_party)
add_definitions(-DCATCH_CONFIG_ENABLE_ALL_STRINGMAKERS=1)

# Test targets
function(gen_test name files compile_definitions compile_flags link_flags)
    add_executable(${name} ${files})
    target_compile_definitions(${name} PUBLIC ${compile_definitions})
    target_link_libraries(${name} pthread)
    set_target_properties(${name} PROPERTIES COMPILE_FLAGS "${compile_flags}" LINK_FLAGS "${link_flags}")

    add_test("run_${name}" "${name}" --rng-seed time)
    set_tests_properties("run_${name}" PROPERTIES ENVIRONMENT SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR})
endfunction()

file(GLOB test_sources
        ${CMAKE_CURRENT_SOURCE_DIR}/test_*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/serial/test_*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/can/test_*.cpp)
gen_test("test_x64" "${test_sources}" "" "-m64" "-m64")
gen_test("test_x32" "${test_sources}" "" "-m32" "-m32")
if ((CMAKE_CXX_COMPILER_ID STREQUAL "GNU") AND (CMAKE_BUILD_TYPE STREQUAL "Debug"))
    gen_test("test_cov" "${test_sources}" "" "-g -O0 --coverage" "--coverage")
endif ()
