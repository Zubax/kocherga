/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Zubax Robotics
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

// Skipping in case of Clang because it can't compile Popcop - the compiler is broken.
// See the bug report here: https://bugs.llvm.org/show_bug.cgi?id=31852
// TODO: This test should be re-enabled when a fixed version of Clang is available.
#ifndef __clang__

// We want to ensure that assertion checks are enabled when tests are run, for extra safety
#ifdef NDEBUG
# undef NDEBUG
#endif

#define KOCHERGA_TRACE          std::printf

// The library headers must be included first to make sure that they don't have any hidden include dependencies.
#include <kocherga_popcop.hpp>

#include "catch.hpp"
#include "mocks.hpp"
#include "images.hpp"

#include <thread>
#include <numeric>
#include <functional>
#include <iostream>
#include <utility>
#include <sys/prctl.h>
#include <csignal>


namespace
{

}  // namespace


TEST_CASE("Popcop-Basic")
{

}

#endif // __clang__
