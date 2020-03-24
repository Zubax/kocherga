# This software is distributed under the terms of the MIT License.
# Copyright (c) 2020 Zubax Robotics.
# Author: Pavel Kirienko <pavel.kirienko@zubax.com>

cmake_minimum_required(VERSION 3.12)
enable_testing()

project(kocherga_test CXX)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
        "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
        FORCE)
endif()

set(library_dir "${CMAKE_SOURCE_DIR}/../../kocherga")

file(GLOB FORMAT_FILES
     ${library_dir}/*.hpp
     ${CMAKE_SOURCE_DIR}/*.[ch]pp)

# Use -DNO_STATIC_ANALYSIS=1 to suppress static analysis.
# If not suppressed, the tools used here shall be available, otherwise the build will fail.
if (NOT NO_STATIC_ANALYSIS)
    # clang-tidy (separate config files per directory)
    find_program(clang_tidy NAMES clang-tidy-10 clang-tidy-9 clang-tidy)
    if (NOT clang_tidy)
        message(FATAL_ERROR "Could not locate clang-tidy")
    endif ()
    message(STATUS "Using clang-tidy: ${clang_tidy}")
    set(CMAKE_CXX_CLANG_TIDY ${clang_tidy})

    # clang-format
    find_program(clang_format NAMES clang-format-10 clang-format-9 clang-format)
    if (NOT clang_format)
        message(FATAL_ERROR "Could not locate clang-format")
    endif ()
    file(GLOB FORMAT_FILES
         ${library_dir}/*.hpp
         ${CMAKE_SOURCE_DIR}/*.[ch]pp)
    message(STATUS "Using clang-format: ${clang_format}; files: ${FORMAT_FILES}")
    add_custom_target(format COMMAND ${clang_format} -i -fallback-style=none -style=file --verbose ${FORMAT_FILES})
endif ()

# C++ options
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror -pedantic -fstrict-aliasing")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wdouble-promotion -Wswitch-enum -Wfloat-equal -Wundef")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wconversion -Wsign-promo")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wsign-conversion -Wcast-align -Wmissing-declarations")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wtype-limits -Wzero-as-null-pointer-constant -Wnon-virtual-dtor")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Woverloaded-virtual -Wsign-promo -Wold-style-cast")

include_directories(../3rd_party ${library_dir})

# Test targets
function(gen_test name files compile_definitions compile_flags link_flags)
    add_executable(${name} ${files})
    target_compile_definitions(${name} PUBLIC ${compile_definitions})
    target_link_libraries(${name} pthread)
    set_target_properties(${name} PROPERTIES COMPILE_FLAGS "${compile_flags}" LINK_FLAGS "${link_flags}")

    add_test("run_${name}" "${name}" --rng-seed time)
    set_tests_properties("run_${name}" PROPERTIES ENVIRONMENT SOURCE_DIR=${CMAKE_SOURCE_DIR})
endfunction()

file(GLOB test_sources ${CMAKE_SOURCE_DIR}/test_*.cpp)
gen_test("test_x64" "${test_sources}" "" "-m64" "-m64")
gen_test("test_x32" "${test_sources}" "" "-m32" "-m32")
if ((CMAKE_CXX_COMPILER_ID STREQUAL "GNU") AND (CMAKE_BUILD_TYPE STREQUAL "Debug"))
    gen_test("test_cov" "${test_sources}" "" "-g -O0 --coverage" "--coverage")
endif ()
